// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2012-2017, The Linux Foundation. All rights reserved.
// Copyright (c) 2020, Stephan Gerhold

#include <linux/module.h>
#include <linux/of.h>
#include <linux/soc/qcom/apr.h>
#include "q6cvs.h"
#include "q6voice-common.h"

/* Stock sunfish q6voice: passive stream shares the modem VSID session. */
struct q6voice_session *q6cvs_session_create(enum q6voice_path_type path)
{
	struct {
		struct apr_hdr hdr;
		char name[20];
	} __packed cmd = {};

	if (path != Q6VOICE_PATH_VOICEMMODE1 && path != Q6VOICE_PATH_VOICE)
		return ERR_PTR(-EINVAL);
	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.opcode = 0x00011140;
	strscpy(cmd.name, path == Q6VOICE_PATH_VOICEMMODE1 ?
		"11C05000" : "default modem voice", sizeof(cmd.name));
	return q6voice_session_create(Q6VOICE_SERVICE_CVS, path, &cmd.hdr);
}
EXPORT_SYMBOL_GPL(q6cvs_session_create);

int q6cvs_unmute(struct q6voice_session *cvs)
{
	struct {
		struct apr_hdr hdr;
		u16 direction;
		u16 mute;
		u16 ramp_ms;
	} __packed cmd = {};
	int ret;

	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.opcode = 0x0001138b; /* VSS_IVOLUME_CMD_MUTE_V2 */
	/* IVOLUME uses TX=0/RX=1, unlike IVOCPROC's RX=0/TX=1. */
	cmd.direction = 0;
	cmd.ramp_ms = 20;
	ret = q6voice_common_send(cvs, &cmd.hdr);
	if (ret)
		return ret;
	cmd.direction = 1;
	return q6voice_common_send(cvs, &cmd.hdr);
}
EXPORT_SYMBOL_GPL(q6cvs_unmute);

int q6cvs_set_rx_dtmf_detection(struct q6voice_session *cvs, bool enable)
{
	struct {
		struct apr_hdr hdr;
		u32 enable;
	} __packed cmd = {};

	/* Stock sunfish VSS_ISTREAM_CMD_SET_RX_DTMF_DETECTION. */
	cmd.hdr.pkt_size = sizeof(cmd);
	cmd.hdr.opcode = 0x00011027;
	cmd.enable = enable;
	return q6voice_common_send(cvs, &cmd.hdr);
}
EXPORT_SYMBOL_GPL(q6cvs_set_rx_dtmf_detection);

static int q6cvs_probe(struct apr_device *adev)
{
	return q6voice_common_probe(adev, Q6VOICE_SERVICE_CVS);
}

static const struct of_device_id q6cvs_device_id[]  = {
	{ .compatible = "qcom,q6cvs" },
	{},
};
MODULE_DEVICE_TABLE(of, q6cvs_device_id);

static struct apr_driver qcom_q6cvs_driver = {
	.probe = q6cvs_probe,
	.remove = q6voice_common_remove,
	.callback = q6voice_common_callback,
	.driver = {
		.name = "qcom-q6cvs",
		.of_match_table = of_match_ptr(q6cvs_device_id),
	},
};

module_apr_driver(qcom_q6cvs_driver);

MODULE_AUTHOR("Stephan Gerhold <stephan@gerhold.net>");
MODULE_DESCRIPTION("Q6 Core Voice Stream");
MODULE_LICENSE("GPL v2");
