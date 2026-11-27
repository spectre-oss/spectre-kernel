// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020, Stephan Gerhold

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/rwsem.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/soc/qcom/apr.h>
#include "q6voice-common.h"

#define APRV2_IBASIC_CMD_DESTROY_SESSION	0x0001003C

#define TIMEOUT_MS	300

struct q6voice_service {
	struct apr_device *adev;
	struct kref refcount;
	struct rw_semaphore io_lock;
	bool removed;
	enum q6voice_service_type type;

	/* Protect sessions array */
	spinlock_t lock;
	struct q6voice_session *sessions[Q6VOICE_PATH_COUNT];
};

/* Protect q6voice_services */
static DEFINE_SPINLOCK(q6voice_services_lock);
static atomic_t q6voice_token = ATOMIC_INIT(0);
static struct q6voice_service *q6voice_services[Q6VOICE_SERVICE_COUNT] = {0};

static void q6voice_service_free(struct kref *ref)
{
	struct q6voice_service *svc = container_of(ref, struct q6voice_service, refcount);

	put_device(&svc->adev->dev);
	kfree(svc);
}

int q6voice_common_probe(struct apr_device *adev, enum q6voice_service_type type)
{
	struct device *dev = &adev->dev;
	struct q6voice_service *svc, *current_svc;
	unsigned long flags;

	if (type >= Q6VOICE_SERVICE_COUNT)
		return -EINVAL;

	svc = kzalloc(sizeof(*svc), GFP_KERNEL);
	if (!svc)
		return -ENOMEM;

	svc->adev = adev;
	get_device(dev);
	kref_init(&svc->refcount);
	init_rwsem(&svc->io_lock);
	svc->type = type;
	spin_lock_init(&svc->lock);

	dev_set_drvdata(dev, svc);

	spin_lock_irqsave(&q6voice_services_lock, flags);
	current_svc = q6voice_services[type];
	if (!current_svc)
		q6voice_services[type] = svc;
	spin_unlock_irqrestore(&q6voice_services_lock, flags);

	if (current_svc) {
		kref_put(&svc->refcount, q6voice_service_free);
		return -EEXIST;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(q6voice_common_probe);

void q6voice_common_remove(struct apr_device *adev)
{
	struct q6voice_service *svc = dev_get_drvdata(&adev->dev);
	enum q6voice_service_type type = svc->type;
	unsigned long flags;

	spin_lock_irqsave(&q6voice_services_lock, flags);
	if (q6voice_services[type] == svc)
		q6voice_services[type] = NULL;
	spin_unlock_irqrestore(&q6voice_services_lock, flags);

	/* Session owners retain the allocation, but cannot use the removed APR device. */
	down_write(&svc->io_lock);
	svc->removed = true;
	up_write(&svc->io_lock);
	kref_put(&svc->refcount, q6voice_service_free);
}
EXPORT_SYMBOL_GPL(q6voice_common_remove);

static void q6voice_session_free(struct kref *ref)
{
	struct q6voice_session *s = container_of(ref, struct q6voice_session,
						 refcount);

	kref_put(&s->svc->refcount, q6voice_service_free);
	kfree(s);
}

static int q6voice_session_destroy(struct q6voice_session *s)
{
	struct apr_pkt cmd = {};

	cmd.hdr.pkt_size = APR_HDR_SIZE;
	cmd.hdr.opcode = APRV2_IBASIC_CMD_DESTROY_SESSION;

	return q6voice_common_send(s, &cmd.hdr);
}

void q6voice_session_release(struct q6voice_session *s)
{
	struct q6voice_service *svc = s->svc;
	unsigned long flags;

	if (s->handle)
		q6voice_session_destroy(s);

	spin_lock_irqsave(&svc->lock, flags);
	if (svc->sessions[s->port] == s)
		svc->sessions[s->port] = NULL;
	spin_unlock_irqrestore(&svc->lock, flags);

	kref_put(&s->refcount, q6voice_session_free);
}
EXPORT_SYMBOL_GPL(q6voice_session_release);

struct q6voice_session *
q6voice_session_create(enum q6voice_service_type type,
		       enum q6voice_path_type path, struct apr_hdr *hdr)
{
	struct q6voice_service *svc;
	struct q6voice_session *s;
	unsigned long flags;
	int ret;

	if (type >= Q6VOICE_SERVICE_COUNT || path >= Q6VOICE_PATH_COUNT)
		return ERR_PTR(-EINVAL);

	spin_lock_irqsave(&q6voice_services_lock, flags);
	svc = q6voice_services[type];
	if (svc)
		kref_get(&svc->refcount);
	spin_unlock_irqrestore(&q6voice_services_lock, flags);
	if (!svc)
		return ERR_PTR(-ENODEV);

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s) {
		kref_put(&svc->refcount, q6voice_service_free);
		return ERR_PTR(-ENOMEM);
	}

	s->dev = &svc->adev->dev;
	s->svc = svc;
	s->port = path;

	kref_init(&s->refcount);
	spin_lock_init(&s->lock);
	init_waitqueue_head(&s->wait);

	spin_lock_irqsave(&svc->lock, flags);
	if (svc->sessions[path]) {
		spin_unlock_irqrestore(&svc->lock, flags);
		kref_put(&s->refcount, q6voice_session_free);
		return ERR_PTR(-EBUSY);
	}
	svc->sessions[path] = s;
	spin_unlock_irqrestore(&svc->lock, flags);

	dev_dbg(s->dev, "create session\n");

	if (type == Q6VOICE_SERVICE_MVM) {
		struct apr_hdr query = {};

		query.pkt_size = APR_HDR_SIZE;
		query.opcode = 0x00011378; /* VSS_IVERSION_CMD_GET */
		ret = q6voice_common_send(s, &query);
		if (ret)
			dev_warn(s->dev, "CVD version query failed: %d\n", ret);
	}
	ret = q6voice_common_send(s, hdr);
	if (ret)
		goto err;

	if (!s->handle) {
		dev_warn(s->dev, "failed to receive handle\n");
		ret = -EIO;
		goto err;
	}

	dev_dbg(s->dev, "handle: %d\n", s->handle);

	return s;

err:
	q6voice_session_release(s);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(q6voice_session_create);

static void q6voice_session_callback(struct q6voice_session *s,
				     const struct apr_resp_pkt *data)
{
	struct aprv2_ibasic_rsp_result_t *result = data->payload;
	unsigned long flags;

	/* Stock CVS event: two little-endian u16 frequencies, not PCM data. */
	if (data->hdr.opcode == 0x0001101a) {
		if (s->svc->type != Q6VOICE_SERVICE_CVS || data->payload_size != 4) {
			dev_warn_ratelimited(s->dev, "invalid RX DTMF event size %u\n",
					     data->payload_size);
			return;
		}
		dev_info_ratelimited(s->dev,
			"voice path %u RX DTMF detected %u/%u Hz (remote %#x)\n",
			s->port, get_unaligned_le16(data->payload),
			get_unaligned_le16((const u8 *)data->payload + 2),
			data->hdr.src_port);
		return;
	}

	/* Passive stream creation alone does not prove modem media is flowing. */
	if (data->hdr.opcode == 0x000110fc || data->hdr.opcode == 0x000110fd) {
		dev_info(s->dev, "voice path %u stream %s (remote %#x)\n",
			 s->port, data->hdr.opcode == 0x000110fc ? "READY" : "NOT_READY",
			 data->hdr.src_port);
		return;
	}

	if (data->hdr.opcode == 0x00011379 && data->payload_size > 0) {
		size_t len = min_t(size_t, data->payload_size, sizeof(s->version) - 1);

		spin_lock_irqsave(&s->lock, flags);
		if (s->expected_opcode == 0x00011378 && data->hdr.token == s->token) {
			memcpy(s->version, data->payload, len);
			s->version[len] = '\0';
			s->result = 0;
			s->expected_opcode = 0;
		}
		spin_unlock_irqrestore(&s->lock, flags);
		dev_info(s->dev, "CVD version %s\n", s->version);
		wake_up(&s->wait);
		return;
	}

	if (data->payload_size < sizeof(*result) ||
	    data->hdr.opcode != APR_BASIC_RSP_RESULT)
		return; /* Not handled here */

	dev_dbg(s->dev, "basic result: opcode %#x, status: %#x\n",
		result->opcode, result->status);

	spin_lock_irqsave(&s->lock, flags);
	if (result->opcode != s->expected_opcode || data->hdr.token != s->token) {
		spin_unlock_irqrestore(&s->lock, flags);
		dev_warn(s->dev, "unexpected reply for opcode %#x (status: %#x)\n",
			 result->opcode, result->status);
		return;
	}

	if (!s->handle) {
		if (!result->status)
			s->handle = data->hdr.src_port;
	} else if (s->handle != data->hdr.src_port) {
		spin_unlock_irqrestore(&s->lock, flags);
		dev_warn(s->dev, "unexpected reply for session %#x (!= %#x)\n",
			 data->hdr.src_port, s->handle);
		return;
	}

	s->result = result->status;
	s->expected_opcode = 0;
	spin_unlock_irqrestore(&s->lock, flags);

	wake_up(&s->wait);
}

int q6voice_common_callback(struct apr_device *adev, const struct apr_resp_pkt *data)
{
	struct device *dev = &adev->dev;
	struct q6voice_service *v = dev_get_drvdata(dev);
	struct q6voice_session *s;
	unsigned long flags;

	dev_dbg(dev, "callback: %#x\n", data->hdr.opcode);

	if (data->hdr.dest_port >= Q6VOICE_PATH_COUNT) {
		dev_warn(dev, "callback() called for unhandled/invalid path: %d\n",
			 data->hdr.dest_port);
		return 0;
	}

	spin_lock_irqsave(&v->lock, flags);
	s = v->sessions[data->hdr.dest_port];
	if (s)
		kref_get(&s->refcount);
	spin_unlock_irqrestore(&v->lock, flags);

	if (s) {
		q6voice_session_callback(s, data);
		kref_put(&s->refcount, q6voice_session_free);
	} else {
		dev_warn(dev, "callback() called for inactive path: %d\n",
			 data->hdr.dest_port);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(q6voice_common_callback);

int q6voice_common_send(struct q6voice_session *s, struct apr_hdr *hdr)
{
	struct q6voice_service *svc = s->svc;
	unsigned long flags;
	int ret;

	down_read(&svc->io_lock);
	if (svc->removed) {
		ret = -ENODEV;
		goto out;
	}
	hdr->hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				       APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	hdr->src_port = s->port;
	hdr->dest_port = s->handle;
	hdr->token = atomic_inc_return(&q6voice_token);

	spin_lock_irqsave(&s->lock, flags);
	s->expected_opcode = hdr->opcode;
	s->token = hdr->token;
	s->result = 0;
	spin_unlock_irqrestore(&s->lock, flags);

	ret = apr_send_pkt(svc->adev, (struct apr_pkt *)hdr);
	if (ret < 0)
		goto clear;
	ret = wait_event_timeout(s->wait, !READ_ONCE(s->expected_opcode),
				 msecs_to_jiffies(TIMEOUT_MS));
	if (!ret) {
		dev_err(s->dev, "command %#x timed out\n", hdr->opcode);
		ret = -ETIMEDOUT;
		goto clear;
	}
	if (s->result) {
		dev_err(s->dev, "command %#x failed with error %u\n",
			hdr->opcode, s->result);
		ret = -EIO;
	} else {
		ret = 0;
	}
clear:
	spin_lock_irqsave(&s->lock, flags);
	s->expected_opcode = 0;
	spin_unlock_irqrestore(&s->lock, flags);
out:
	up_read(&svc->io_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(q6voice_common_send);

MODULE_AUTHOR("Stephan Gerhold <stephan@gerhold.net>");
MODULE_DESCRIPTION("Q6Voice common session management");
MODULE_LICENSE("GPL v2");
