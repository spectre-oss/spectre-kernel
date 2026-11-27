/* SPDX-License-Identifier: GPL-2.0-only */
/* Private, single-owner storage mailbox. Included only by sunfish_fpc.c.
 * Frames: LE32 generation, listener ID, payload size, reserved zero; payload.
 * No ioctl or physical-address interface. Reserved RAM survives all failures.
 */
#include <linux/capability.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>

#define FPC_FS_OFFSET (SZ_16M + SZ_1M)
#define FPC_GPFS_OFFSET (FPC_FS_OFFSET + SZ_64K)
#define FPC_FS_SIZE 0x5000
#define FPC_GPFS_SIZE 0x7e000
#define FPC_RPMB_OFFSET (SZ_16M + 2 * SZ_1M)
#define FPC_RPMB_SIZE 0x6400

struct fpc_storage {
	struct miscdevice misc;
	struct mutex mutex;
	wait_queue_head_t queue;
	void __iomem *memory, *request;
	u32 generation, id, size;
	bool online, pending, replied;
	unsigned int registered;
};

static int fpc_storage_callback(void *context, u32 id)
{
	struct fpc_storage *s = context;
	int ret = 0;
	long waited;

	mutex_lock(&s->mutex);
	if (!s->online || s->pending || s->generation == U32_MAX ||
	    (id != 0xa && id != 0x7000 && id != 0x2000)) {
		mutex_unlock(&s->mutex);
		return -ENOTCONN;
	}
	s->id = id;
	s->size = id == 0xa ? FPC_FS_SIZE : FPC_GPFS_SIZE;
	s->request = s->memory + (id == 0xa ? FPC_FS_OFFSET : FPC_GPFS_OFFSET);
	if (id == 0x2000) {
		s->size = FPC_RPMB_SIZE;
		s->request = s->memory + FPC_RPMB_OFFSET;
	}
	s->generation++;
	s->replied = false;
	rmb();
	s->pending = true;
	mutex_unlock(&s->mutex);
	wake_up_interruptible(&s->queue);
	waited = wait_event_timeout(s->queue,
		READ_ONCE(s->replied) || !READ_ONCE(s->online), 30 * HZ);
	mutex_lock(&s->mutex);
	if (!waited || !s->replied)
		ret = waited ? -ECONNRESET : -ETIMEDOUT;
	s->pending = false;
	mutex_unlock(&s->mutex);
	return ret;
}

static int fpc_storage_open(struct inode *inode, struct file *file)
{
	struct fpc_storage *s = container_of(file->private_data, struct fpc_storage, misc);
	int ret = 0;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;
	mutex_lock(&s->mutex);
	if (s->online || s->pending)
		ret = -EBUSY;
	else {
		s->online = true;
		file->private_data = s;
	}
	mutex_unlock(&s->mutex);
	return ret;
}

static int fpc_storage_release(struct inode *inode, struct file *file)
{
	struct fpc_storage *s = file->private_data;

	mutex_lock(&s->mutex);
	s->online = false;
	mutex_unlock(&s->mutex);
	wake_up_all(&s->queue);
	return 0;
}

static ssize_t fpc_storage_read(struct file *file, char __user *buf,
			       size_t count, loff_t *pos)
{
	struct fpc_storage *s = file->private_data;
	__le32 *frame;
	ssize_t ret;

	ret = wait_event_interruptible(s->queue,
		READ_ONCE(s->pending) && !READ_ONCE(s->replied));
	if (ret)
		return ret;
	mutex_lock(&s->mutex);
	if (!s->pending || s->replied) {
		ret = -EAGAIN;
		goto unlock;
	}
	if (count < s->size + 16) {
		ret = -EMSGSIZE;
		goto unlock;
	}
	frame = kvzalloc(s->size + 16, GFP_KERNEL);
	if (!frame) {
		ret = -ENOMEM;
		goto unlock;
	}
	frame[0] = cpu_to_le32(s->generation);
	frame[1] = cpu_to_le32(s->id);
	frame[2] = cpu_to_le32(s->size);
	memcpy_fromio(frame + 4, s->request, s->size);
	ret = copy_to_user(buf, frame, s->size + 16) ? -EFAULT : s->size + 16;
	kvfree_sensitive(frame, s->size + 16);
unlock:
	mutex_unlock(&s->mutex);
	return ret;
}

static ssize_t fpc_storage_write(struct file *file, const char __user *buf,
				size_t count, loff_t *pos)
{
	struct fpc_storage *s = file->private_data;
	__le32 *frame;
	ssize_t ret = -ESTALE;

	if (count < 20 || count > FPC_GPFS_SIZE + 16)
		return -EMSGSIZE;
	frame = kvmalloc(count, GFP_KERNEL);
	if (!frame)
		return -ENOMEM;
	if (copy_from_user(frame, buf, count)) {
		ret = -EFAULT;
		goto free;
	}
	mutex_lock(&s->mutex);
	if (!s->pending || s->replied || le32_to_cpu(frame[0]) != s->generation ||
	    le32_to_cpu(frame[1]) != s->id)
		goto unlock;
	if (le32_to_cpu(frame[2]) != count - 16 || frame[3] || count - 16 > s->size) {
		ret = -EINVAL;
		goto unlock;
	}
	memset_io(s->request, 0, s->size);
	memcpy_toio(s->request, frame + 4, count - 16);
	wmb();
	s->replied = true;
	ret = count;
unlock:
	mutex_unlock(&s->mutex);
	wake_up_all(&s->queue);
free:
	kvfree_sensitive(frame, count);
	return ret;
}

static const struct file_operations fpc_storage_fops = {
	.owner = THIS_MODULE,
	.open = fpc_storage_open, .release = fpc_storage_release,
	.read = fpc_storage_read, .write = fpc_storage_write,
};

static void fpc_storage_deregister(void *data)
{
	struct fpc_storage *s = data;
	misc_deregister(&s->misc);
}

static int fpc_storage_setup(struct device *dev, struct fpc_storage *s,
			     void __iomem *memory)
{
	int ret;

	s->memory = memory;
	mutex_init(&s->mutex);
	init_waitqueue_head(&s->queue);
	s->misc = (struct miscdevice) {
		.minor = MISC_DYNAMIC_MINOR, .name = "sunfish-fpc-storage",
		.fops = &fpc_storage_fops, .mode = 0600, .parent = dev,
	};
	ret = misc_register(&s->misc);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, fpc_storage_deregister, s);
	if (!ret)
		qcom_scm_fpc_listener_set(fpc_storage_callback, s);
	return ret;
}

static int fpc_storage_register(struct fpc_storage *s, phys_addr_t phys)
{
	int ret;
	unsigned int i;
	const u32 ids[] = { 0xa, 0x7000, 0x2000 };
	const u32 offsets[] = { FPC_FS_OFFSET, FPC_GPFS_OFFSET, FPC_RPMB_OFFSET };
	const u32 sizes[] = { FPC_FS_SIZE, FPC_GPFS_SIZE, FPC_RPMB_SIZE };

	if (!READ_ONCE(s->online))
		return -ENOTCONN;
	for (i = s->registered; i < ARRAY_SIZE(ids); i++) {
		/* Stock registers the mapped DMA allocation size, not the
		 * userspace protocol length (RPMB 0x6400 -> 0x7000).
		 */
		u32 allocation = ALIGN(sizes[i], SZ_4K);

		memset_io(s->memory + offsets[i], 0, allocation);
		wmb();
		ret = qcom_scm_fpc_listener_register(ids[i], phys + offsets[i], allocation);
		if (ret)
			return ret;
		s->registered++;
	}
	return 0;
}
