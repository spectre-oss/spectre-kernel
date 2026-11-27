// SPDX-License-Identifier: GPL-2.0-only
/* Pixel 4a signed FPC app lab. Enrollment/matching lab; no raw images exported. */
#include <linux/delay.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/firmware/qcom/qcom_scm.h>
#include <linux/firmware/qcom/sunfish_fpc.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/hex.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/sysfs.h>

#define FPC_APP "fpctzappfingerprint"
#define FPC_PAYLOAD SZ_4K
#define FPC_LIST (2 * SZ_4K)
#define FPC_IMAGE SZ_64K
#define FPC_MAX_IMAGE SZ_8M
#define FPC_LOG SZ_16M
#define FPC_LOG_SIZE SZ_32K

#include "sunfish_fpc_storage.h"

struct sunfish_fpc {
	struct device *dev;
	void __iomem *mem;
	phys_addr_t phys;
	struct gpio_desc *reset, *irq;
	struct mutex lock;
	u32 app_id;
	int result, sensor_status;
	bool poisoned, initialized, enrolling, enrollment_ready, auth_key;
	u32 remaining, finger_id, finger_count, finger_ids[5];
	u64 challenge;
	u8 key_reply[SZ_4K];
	bool key_reply_valid, log_registered;
	bool navigating;
	u32 nav_code, nav_wait, nav_active;
	struct fpc_storage storage;
};

static int fpc_transfer(struct sunfish_fpc *f, u32 group, u32 op,
			const void *data, size_t size)
{
	int ret;

	if (size > SZ_4K - 12)
		return -E2BIG;
	if (!f->app_id || f->poisoned)
		return -EIO;
	memset_io(f->mem, 0, 128);
	memset_io(f->mem + FPC_PAYLOAD, 0, SZ_4K);
	memset_io(f->mem + FPC_LIST, 0, 32);
	writel(max_t(size_t, 88, 12 + size), f->mem);
	writel(f->phys + FPC_PAYLOAD, f->mem + 4);
	writel(group, f->mem + FPC_PAYLOAD);
	writel(op, f->mem + FPC_PAYLOAD + 4);
	if (group == 10 && op >= 1 && op <= 4)
		writel(1, f->mem + FPC_PAYLOAD + 12);
	if (size)
		memcpy_toio(f->mem + FPC_PAYLOAD + 12, data, size);
	/* One contiguous 32-bit payload address, patched at request offset 4. */
	writel(BIT(31) | 4, f->mem + FPC_LIST);
	writel(SZ_4K, f->mem + FPC_LIST + 4);
	wmb();
	ret = qcom_scm_fpc_send(f->app_id, f->phys, f->phys + 64,
			       f->phys + FPC_LIST);
	if (ret) {
		f->poisoned = true;
		return ret;
	}
	rmb();
	ret = (s32)readl(f->mem + 64);
	f->sensor_status = (s32)readl(f->mem + FPC_PAYLOAD + 8);
	dev_info(f->dev, "group=%u op=%u transport=%d sensor=%d\n", group, op, ret,
		 f->sensor_status);
	return ret ?: f->sensor_status;
}

static int fpc_command(struct sunfish_fpc *f, u32 op)
{
	return fpc_transfer(f, 10, op, NULL, 0);
}

/* Commands/offsets from the stock HAL and signed TA, documented in fpc protocol notes. */
static int fpc_auth_init(struct sunfish_fpc *f)
{
	u32 id, offset, length;
	u8 *payload;
	int ret;

	if (!f->app_id)
		return -ENODEV;
	ret = qcom_scm_qseecom_app_get_id("keymaster64", &id);
	if (ret)
		return ret;
	/* Stock HAL: request {0x205, 2}; reply {status, offset, length}. */
	memset_io(f->mem + 0x3000, 0, 1024);
	writel(0x205, f->mem + 0x3000);
	writel(2, f->mem + 0x3004);
	wmb();
	ret = qcom_scm_fpc_key_request(id, f->phys + 0x3000, f->phys + 0x3040, 64, 960);
	if (ret) {
		f->poisoned = true;
		return ret;
	}
	rmb();
	ret = (s32)readl(f->mem + 0x3040);
	if (ret)
		return ret;
	offset = readl(f->mem + 0x3044);
	length = readl(f->mem + 0x3048);
	if (offset < 12 || offset > 960 || !length || length > 960 - offset)
		return -EPROTO;
	payload = kzalloc(length + 4, GFP_KERNEL);
	if (!payload)
		return -ENOMEM;
	*(__le32 *)payload = cpu_to_le32(length);
	memcpy_fromio(payload + 4, f->mem + 0x3040 + offset, length);
	ret = fpc_transfer(f, 3, 5, payload, length + 4);
	f->auth_key = !ret;
	kfree_sensitive(payload);
	/* Retain shared memory untouched on an incomplete secure operation. */
	if (!f->poisoned) {
		memset_io(f->mem + FPC_PAYLOAD, 0, SZ_4K);
		memset_io(f->mem + 0x3000, 0, 1024);
	}
	return ret;
}

static int fpc_bio(struct sunfish_fpc *f, u32 op, u32 argument)
{
	__le32 arg = cpu_to_le32(argument);
	int ret = fpc_transfer(f, 11, op, &arg, sizeof(arg));

	if (ret >= 0) {
		u32 output = readl(f->mem + FPC_PAYLOAD + 12);
		if (op == 1)
			f->remaining = output;
		if (op == 2 || op == 3)
			f->finger_id = output;
		if (op == 7 && !ret) {
			if (output > ARRAY_SIZE(f->finger_ids))
				return -EOVERFLOW;
			f->finger_count = output;
			for (u32 i = 0; i < output; i++)
				f->finger_ids[i] = readl(f->mem + FPC_PAYLOAD + 16 + i * 4);
		}
	}
	return ret;
}

static int fpc_load(struct sunfish_fpc *f)
{
	const struct firmware *mdt, *segment;
	Elf64_Ehdr eh;
	Elf64_Phdr ph;
	char name[80];
	size_t used;
	u32 existing;
	int ret, i;

	if (f->app_id)
		return -EALREADY;
	ret = qcom_scm_qseecom_app_get_id(FPC_APP, &existing);
	if (!ret)
		return -EBUSY; /* Never take over another client's app. */
	if (ret != -ENOENT)
		return ret;
	ret = request_firmware(&mdt, "qcom/sunfish/" FPC_APP ".mdt", f->dev);
	if (ret)
		return ret;
	ret = -EINVAL;
	if (mdt->size < sizeof(eh) || mdt->size > FPC_MAX_IMAGE)
		goto release;
	memcpy(&eh, mdt->data, sizeof(eh));
	if (memcmp(eh.e_ident, ELFMAG, SELFMAG) ||
	    eh.e_ident[EI_CLASS] != ELFCLASS64 ||
	    eh.e_ident[EI_DATA] != ELFDATA2LSB || eh.e_machine != EM_AARCH64 ||
	    eh.e_phnum != 8 || eh.e_phentsize != sizeof(ph) ||
	    eh.e_phoff > mdt->size ||
	    8 * sizeof(ph) > mdt->size - eh.e_phoff)
		goto release;
	used = mdt->size;
	memcpy_toio(f->mem + FPC_IMAGE, mdt->data, used);
	for (i = 0; i < 8; i++) {
		memcpy(&ph, mdt->data + eh.e_phoff + i * sizeof(ph), sizeof(ph));
		snprintf(name, sizeof(name), "qcom/sunfish/" FPC_APP ".b%02d", i);
		ret = request_firmware(&segment, name, f->dev);
		if (ret)
			goto release;
		if (segment->size != ph.p_filesz || segment->size > FPC_MAX_IMAGE - used) {
			release_firmware(segment);
			ret = -EINVAL;
			goto release;
		}
		memcpy_toio(f->mem + FPC_IMAGE + used, segment->data, segment->size);
		used += segment->size;
		release_firmware(segment);
	}
	wmb();
	ret = qcom_scm_fpc_load(f->phys + FPC_IMAGE, mdt->size, used, &f->app_id);
	if (ret)
		f->poisoned = true;
release:
	release_firmware(mdt);
	return ret;
}

static int fpc_initialize(struct sunfish_fpc *f)
{
	int ret;

	if (!f->app_id)
		return -ENODEV;
	ret = gpiod_direction_output(f->reset, 1);
	if (ret)
		return ret;
	usleep_range(100, 200);
	gpiod_set_value_cansleep(f->reset, 0);
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(f->reset, 1);
	usleep_range(5000, 5100);
	ret = fpc_command(f, 0);
	f->initialized = !ret;
	return ret;
}

static int fpc_wait_irq(struct sunfish_fpc *f)
{
	unsigned long end = jiffies + msecs_to_jiffies(15000);
	int value;

	do {
		value = gpiod_get_value_cansleep(f->irq);
		if (value < 0)
			return value;
		if (value)
			return 0;
		if (msleep_interruptible(10))
			return -EINTR;
	} while (time_before(jiffies, end));
	return -ETIMEDOUT;
}

static int fpc_capture(struct sunfish_fpc *f)
{
	int ret, end_wait, cleanup;

	if (!f->initialized)
		return -EINVAL;
	ret = fpc_command(f, 2);
	if (!ret) {
		ret = fpc_wait_irq(f);
		end_wait = fpc_command(f, 1);
		if (!ret && end_wait < 0)
			ret = end_wait;
	}
	if (!ret)
		ret = fpc_command(f, 3);
	if (!ret)
		ret = fpc_wait_irq(f);
	if (!ret)
		ret = fpc_command(f, 4);
	if (!f->poisoned) {
		cleanup = fpc_command(f, 5);
		if (!ret)
			ret = cleanup;
	}
	return ret;
}

/* Stock module 8 navigation. Lab metadata only; no input injection yet. */
static int fpc_navigation(struct sunfish_fpc *f, u32 op)
{
	int ret, cleanup, level;

	f->nav_code = 0;
	if (!f->initialized || f->enrolling)
		return -EBUSY;
	if (op == 1) {
		if (f->navigating)
			return -EBUSY;
		ret = fpc_transfer(f, 8, 1, NULL, 0);
		if (ret)
			return ret;
		f->navigating = true;
		f->nav_wait = 0;
		f->nav_active = 0;
		ret = fpc_transfer(f, 8, 5, NULL, 0);
		if (ret && !f->poisoned) {
			cleanup = fpc_transfer(f, 8, 2, NULL, 0);
			if (!cleanup)
				f->navigating = false;
		}
		return ret;
	}
	if (!f->navigating)
		return -EINVAL;
	if (op == 3 && f->nav_wait) {
		level = gpiod_get_value_cansleep(f->irq);
		if (level < 0)
			return level;
		if (level != (f->nav_wait == 1))
			return -EAGAIN;
	}
	ret = fpc_transfer(f, 8, op, NULL, 0);
	if (ret)
		return ret;
	if (op == 2) {
		f->navigating = false;
		f->nav_wait = 0;
		f->nav_active = 0;
	} else {
		f->nav_code = readl(f->mem + FPC_PAYLOAD + 12);
		f->nav_active = readl(f->mem + FPC_PAYLOAD + 20);
		f->nav_wait = readl(f->mem + FPC_PAYLOAD + 24);
		if (f->nav_wait > 2) {
			f->nav_code = 0;
			return -EPROTO;
		}
	}
	return 0;
}

static ssize_t control_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct sunfish_fpc *f = dev_get_drvdata(dev);
	ssize_t n;

	mutex_lock(&f->lock);
	n = sysfs_emit(buf, "app_id=%u initialized=%u poisoned=%u result=%d sensor=%d enrolling=%u remaining=%u finger_id=%u count=%u ids=%u,%u,%u,%u,%u challenge=%llu auth_key=%u\n",
		       f->app_id, f->initialized, f->poisoned, f->result, f->sensor_status,
		       f->enrolling, f->remaining, f->finger_id, f->finger_count,
		       f->finger_ids[0], f->finger_ids[1], f->finger_ids[2],
		       f->finger_ids[3], f->finger_ids[4], f->challenge, f->auth_key);
	n += sysfs_emit_at(buf, n, "navigating=%u nav_code=%u nav_wait=%u nav_active=%u\n",
			  f->navigating, f->nav_code, f->nav_wait, f->nav_active);
	n += sysfs_emit_at(buf, n, "storage_online=%u storage_registered=%u\n",
			  READ_ONCE(f->storage.online), f->storage.registered);
	mutex_unlock(&f->lock);
	return n;
}

static ssize_t control_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct sunfish_fpc *f = dev_get_drvdata(dev);
	int ret;

	if (mutex_lock_interruptible(&f->lock))
		return -EINTR;
	if (f->poisoned) {
		ret = -EIO;
	} else if (sysfs_streq(buf, "nav-start")) {
		ret = fpc_navigation(f, 1);
	} else if (sysfs_streq(buf, "nav-poll")) {
		ret = fpc_navigation(f, 3);
	} else if (sysfs_streq(buf, "nav-stop")) {
		ret = fpc_navigation(f, 2);
	} else if (f->navigating) {
		ret = -EBUSY;
	} else if (sysfs_streq(buf, "storage-init")) {
		ret = fpc_storage_register(&f->storage, f->phys);
		if (ret && ret != -ENOTCONN)
			f->poisoned = true;
	} else if (sysfs_streq(buf, "log-init")) {
		if (f->log_registered) {
			ret = 0;
		} else {
			/* Dedicated reserved memory remains valid until reboot. */
			memset_io(f->mem + FPC_LOG, 0, FPC_LOG_SIZE);
			wmb();
			ret = qcom_scm_fpc_log(f->phys + FPC_LOG, FPC_LOG_SIZE);
			f->log_registered = !ret;
			if (ret)
				f->poisoned = true;
		}
	} else if (sysfs_streq(buf, "load")) {
		ret = fpc_load(f);
	} else if (sysfs_streq(buf, "init")) {
		ret = fpc_initialize(f);
	} else if (sysfs_streq(buf, "capture")) {
		ret = f->enrolling ? -EBUSY : fpc_capture(f);
	} else if (sysfs_streq(buf, "auth-init")) {
		ret = fpc_auth_init(f);
	} else if (sysfs_streq(buf, "challenge")) {
		ret = fpc_transfer(f, 3, 2, NULL, 0);
		if (!ret)
			f->challenge = readq(f->mem + FPC_PAYLOAD + 16);
	} else if (!strncmp(buf, "authorize ", 10)) {
		/* Packed Android hardware_auth_token_t, never log the token. */
		u8 payload[73] = { 69, 0, 0, 0 };
		if ((count != 148 && count != 149) ||
		    (count == 149 && buf[148] != '\n') ||
		    hex2bin(payload + 4, buf + 10, 69)) {
			ret = -EINVAL;
		} else {
			ret = fpc_transfer(f, 3, 3, payload, sizeof(payload));
		}
		memzero_explicit(payload, sizeof(payload));
		if (!f->poisoned)
			memset_io(f->mem + FPC_PAYLOAD, 0, SZ_4K);
	} else if (sysfs_streq(buf, "fingers")) {
		memset(f->finger_ids, 0, sizeof(f->finger_ids));
		f->finger_count = 0;
		ret = fpc_bio(f, 7, 5);
	} else if (sysfs_streq(buf, "enroll-begin")) {
		if (!f->initialized || f->enrolling) {
			ret = -EBUSY;
		} else {
			ret = fpc_bio(f, 0, 0);
			f->enrolling = !ret;
			f->remaining = 0;
			f->enrollment_ready = false;
			f->finger_id = 0;
		}
	} else if (sysfs_streq(buf, "enroll-sample")) {
		ret = f->enrolling ? fpc_capture(f) : -EINVAL;
		if (!ret) {
			ret = fpc_bio(f, 1, 0);
			f->enrollment_ready = ret == 0 && f->remaining == 0;
		}
		/* Positive biometric results describe retry/progress, not errno. */
		if (ret > 0)
			ret = 0;
	} else if (sysfs_streq(buf, "enroll-end")) {
		if (!f->enrolling || !f->enrollment_ready) {
			ret = -EINVAL;
		} else {
			f->finger_id = 0;
			ret = fpc_bio(f, 2, 0);
			f->enrolling = false;
		}
	} else if (sysfs_streq(buf, "match")) {
		f->finger_id = 0;
		ret = f->enrolling ? -EBUSY : fpc_capture(f);
		if (!ret)
			ret = fpc_bio(f, 3, 0);
		if (ret > 0) {
			f->finger_id = 0;
			ret = 0;
		}
	} else if (sysfs_streq(buf, "sleep")) {
		ret = fpc_command(f, 5);
	} else if (sysfs_streq(buf, "unload") && f->app_id) {
		ret = qcom_scm_fpc_unload(f->app_id);
		if (ret) {
			f->poisoned = true;
		} else {
			f->app_id = 0;
			f->initialized = false;
			f->enrolling = false;
			f->finger_id = 0;
			f->challenge = 0;
			f->auth_key = false;
			f->finger_count = 0;
			f->remaining = 0;
			f->enrollment_ready = false;
			memset(f->finger_ids, 0, sizeof(f->finger_ids));
			ret = gpiod_direction_input(f->reset);
		}
	} else {
		ret = -EINVAL;
	}
	f->result = ret;
	mutex_unlock(&f->lock);
	return ret ? (ret < 0 ? ret : -EREMOTEIO) : count;
}
/* Root-only bounded Gatekeeper transport. No general Keymaster command proxy. */
static ssize_t keymaster_write(struct file *file, struct kobject *kobj,
		const struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct sunfish_fpc *f = dev_get_drvdata(kobj_to_dev(kobj));
	u32 command, id;
	int ret;

	if (off || count < 4 || count > SZ_4K)
		return -EINVAL;
	memcpy(&command, buf, 4);
	command = le32_to_cpu((__force __le32)command);
	if (command != 0x200 && command != 0x1001 && command != 0x1002)
		return -EOPNOTSUPP;
	if ((command == 0x200 && count != 4) || (command != 0x200 && count < 32))
		return -EINVAL;
	mutex_lock(&f->lock);
	f->key_reply_valid = false;
	memzero_explicit(f->key_reply, sizeof(f->key_reply));
	if (f->navigating) {
		ret = -EBUSY;
		goto out;
	}
	if (f->poisoned) {
		ret = -EIO;
		goto out;
	}
	ret = qcom_scm_qseecom_app_get_id("keymaster64", &id);
	if (ret)
		goto out;
	memset_io(f->mem + 0x4000, 0, 2 * SZ_4K);
	memcpy_toio(f->mem + 0x4000, buf, count);
	wmb();
	ret = qcom_scm_fpc_key_request(id, f->phys + 0x4000,
			f->phys + 0x5000, count, SZ_4K);
	if (ret) {
		f->poisoned = true;
		goto out;
	}
	rmb();
	memcpy_fromio(f->key_reply, f->mem + 0x5000, SZ_4K);
	f->key_reply_valid = true;
	memset_io(f->mem + 0x4000, 0, 2 * SZ_4K);
out:
	mutex_unlock(&f->lock);
	return ret ? ret : count;
}

static ssize_t keymaster_read(struct file *file, struct kobject *kobj,
		const struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct sunfish_fpc *f = dev_get_drvdata(kobj_to_dev(kobj));
	ssize_t ret;

	mutex_lock(&f->lock);
	ret = f->key_reply_valid ? memory_read_from_buffer(buf, count, &off,
			f->key_reply, sizeof(f->key_reply)) : -ENODATA;
	mutex_unlock(&f->lock);
	return ret;
}
static const struct bin_attribute bin_attr_keymaster = {
	.attr = { .name = "keymaster", .mode = 0600 },
	.size = SZ_4K,
	.read = keymaster_read,
	.write = keymaster_write,
};
static ssize_t firmware_log_read(struct file *file, struct kobject *kobj,
		const struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	struct sunfish_fpc *f = dev_get_drvdata(kobj_to_dev(kobj));

	if (!f->log_registered)
		return -ENODATA;
	if (off < 0 || off >= FPC_LOG_SIZE)
		return 0;
	count = min_t(size_t, count, FPC_LOG_SIZE - off);
	memcpy_fromio(buf, f->mem + FPC_LOG + off, count);
	return count;
}
static const struct bin_attribute bin_attr_firmware_log = {
	.attr = { .name = "firmware_log", .mode = 0400 },
	.size = FPC_LOG_SIZE,
	.read = firmware_log_read,
};
static const struct bin_attribute * const fpc_bin_attrs[] = {
	&bin_attr_keymaster, &bin_attr_firmware_log, NULL
};
static DEVICE_ATTR(control, 0600, control_show, control_store);
static struct attribute *fpc_attrs[] = { &dev_attr_control.attr, NULL };
static const struct attribute_group fpc_group = { .attrs = fpc_attrs, .bin_attrs = fpc_bin_attrs };
static const struct attribute_group *fpc_groups[] = { &fpc_group, NULL };

static int sunfish_fpc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node;
	struct reserved_mem *mem;
	struct sunfish_fpc *f;
	int ret;

	if (!of_machine_is_compatible("google,sunfish"))
		return -ENODEV;
	if (!qcom_scm_is_available())
		return -EPROBE_DEFER;
	f = devm_kzalloc(dev, sizeof(*f), GFP_KERNEL);
	if (!f)
		return -ENOMEM;
	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	mem = node ? of_reserved_mem_lookup(node) : NULL;
	of_node_put(node);
	if (!mem || mem->base != 0x9e400000 || mem->size != SZ_16M + SZ_4M)
		return -EINVAL;
	if (!devm_request_mem_region(dev, mem->base, mem->size, dev_name(dev)))
		return -EBUSY;
	f->mem = devm_ioremap(dev, mem->base, mem->size);
	if (!f->mem)
		return -ENOMEM;
	f->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(f->reset))
		return dev_err_probe(dev, PTR_ERR(f->reset), "reset GPIO\n");
	f->irq = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(f->irq))
		return dev_err_probe(dev, PTR_ERR(f->irq), "IRQ GPIO\n");
	f->dev = dev;
	f->phys = mem->base;
	mutex_init(&f->lock);
	platform_set_drvdata(pdev, f);
	ret = fpc_storage_setup(dev, &f->storage, f->mem);
	if (ret)
		return ret;
	dev_info(dev, "FPC lab ready; root control required, no boot-time SCM calls\n");
	return 0;
}

static const struct of_device_id sunfish_fpc_match[] = {
	{ .compatible = "google,sunfish-fpc-lab" },
	{ }
};
static struct platform_driver sunfish_fpc_driver = {
	.probe = sunfish_fpc_probe,
	.driver = {
		.name = "sunfish-fpc-lab",
		.of_match_table = sunfish_fpc_match,
		.dev_groups = fpc_groups,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(sunfish_fpc_driver);
MODULE_LICENSE("GPL");
