// SPDX-License-Identifier: GPL-2.0-only
/*
 * Pixel 4a (sunfish) DIAG rpmsg sniffer — Lead 3l Stage A.
 *
 * Binds MPSS DIAG_CTRL / DIAG_DATA / DIAG_CMD. Logs probe (channel +
 * parent of_node; expect edge "modem"). Copies RX payloads to debugfs.
 *
 * No diagchar, no memshare, no MSG-mask send (Stage C). Do not mmap
 * the 5 MiB DIAG CMA (Lead 3k: MSS_MSA-only while MPSS runs).
 */
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/rpmsg.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#define SUNFISH_DIAG_PKTS	64
#define SUNFISH_DIAG_PKT_MAX	4096

enum {
	CH_CTRL = 0,
	CH_DATA = 1,
	CH_CMD  = 2,
	CH_N    = 3,
};

static const char * const ch_names[CH_N] = {
	"DIAG_CTRL", "DIAG_DATA", "DIAG_CMD",
};

struct sunfish_diag_pkt {
	u16 ch;
	u16 len;
	u32 seq;
	u8 data[SUNFISH_DIAG_PKT_MAX];
};

struct sunfish_diag {
	struct rpmsg_device *rpdev[CH_N];
	u32 rx_count[CH_N];
	u32 drop_count;
	u32 seq;
	u32 dump_left[CH_N];
	unsigned int head;
	unsigned int nstored;
	struct sunfish_diag_pkt pkts[SUNFISH_DIAG_PKTS];
	spinlock_t lock;
	struct dentry *dbg;
};

static struct sunfish_diag *g_diag;

static int ch_from_name(const char *name)
{
	int i;

	for (i = 0; i < CH_N; i++)
		if (!strcmp(name, ch_names[i]))
			return i;
	return -EINVAL;
}

static const char *edge_name(struct rpmsg_device *rpdev)
{
	struct device *p;

	for (p = rpdev->dev.parent; p; p = p->parent) {
		if (p->of_node)
			return p->of_node->name;
	}
	if (rpdev->dev.parent)
		return dev_name(rpdev->dev.parent);
	return "(no-parent)";
}

static int sunfish_diag_probe(struct rpmsg_device *rpdev)
{
	struct sunfish_diag *d = g_diag;
	const char *name = rpdev->id.name;
	int ch;

	ch = ch_from_name(name);
	if (ch < 0) {
		dev_warn(&rpdev->dev, "sunfish-diag: unexpected channel %s\n",
			 name);
		return -ENODEV;
	}

	dev_info(&rpdev->dev,
		 "sunfish-diag: probe %s edge=%s src=0x%x dst=0x%x\n",
		 name, edge_name(rpdev), rpdev->src, rpdev->dst);

	spin_lock(&d->lock);
	d->rpdev[ch] = rpdev;
	spin_unlock(&d->lock);
	dev_set_drvdata(&rpdev->dev, d);
	if (rpdev->ept)
		rpdev->ept->priv = (void *)(uintptr_t)ch;

	return 0;
}

static void sunfish_diag_remove(struct rpmsg_device *rpdev)
{
	struct sunfish_diag *d = dev_get_drvdata(&rpdev->dev);
	int ch = ch_from_name(rpdev->id.name);

	dev_info(&rpdev->dev, "sunfish-diag: remove %s\n", rpdev->id.name);
	if (!d || ch < 0)
		return;
	spin_lock(&d->lock);
	if (d->rpdev[ch] == rpdev)
		d->rpdev[ch] = NULL;
	spin_unlock(&d->lock);
}

static int sunfish_diag_cb(struct rpmsg_device *rpdev, void *data, int len,
			   void *priv, u32 src)
{
	struct sunfish_diag *d = g_diag;
	struct sunfish_diag_pkt *pkt;
	int ch = (int)(uintptr_t)priv;
	unsigned int slot;
	unsigned long flags;
	u32 seq;

	if (ch < 0 || ch >= CH_N)
		ch = ch_from_name(rpdev->id.name);
	if (ch < 0 || len < 0)
		return 0;

	spin_lock_irqsave(&d->lock, flags);
	seq = ++d->seq;
	d->rx_count[ch]++;
	if ((u32)len > SUNFISH_DIAG_PKT_MAX) {
		d->drop_count++;
		len = SUNFISH_DIAG_PKT_MAX;
	}
	slot = d->head;
	d->head = (d->head + 1) % SUNFISH_DIAG_PKTS;
	if (d->nstored < SUNFISH_DIAG_PKTS)
		d->nstored++;
	pkt = &d->pkts[slot];
	pkt->ch = ch;
	pkt->len = len;
	pkt->seq = seq;
	memcpy(pkt->data, data, len);
	spin_unlock_irqrestore(&d->lock, flags);

	if (d->dump_left[ch]) {
		d->dump_left[ch]--;
		print_hex_dump(KERN_INFO, "sunfish-diag: ", DUMP_PREFIX_OFFSET,
			       16, 1, data, min(len, 64), true);
	}

	return 0;
}

static int bound_show(struct seq_file *m, void *v)
{
	struct sunfish_diag *d = g_diag;
	int i;

	spin_lock(&d->lock);
	for (i = 0; i < CH_N; i++)
		seq_printf(m, "%s %s edge=%s rx=%u\n",
			   ch_names[i],
			   d->rpdev[i] ? "bound" : "unbound",
			   d->rpdev[i] ? edge_name(d->rpdev[i]) : "-",
			   d->rx_count[i]);
	seq_printf(m, "seq=%u stored=%u drops=%u\n",
		   d->seq, d->nstored, d->drop_count);
	spin_unlock(&d->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(bound);

static int log_show(struct seq_file *m, void *v)
{
	struct sunfish_diag *d = g_diag;
	struct sunfish_diag_pkt *tmp;
	unsigned int i, idx, n;
	int off, j, chunk;

	tmp = kmalloc(sizeof(*tmp), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	spin_lock(&d->lock);
	n = d->nstored;
	for (i = 0; i < n; i++) {
		if (n == SUNFISH_DIAG_PKTS)
			idx = (d->head + i) % SUNFISH_DIAG_PKTS;
		else
			idx = i;
		*tmp = d->pkts[idx];
		spin_unlock(&d->lock);
		seq_printf(m, "#%u %s len=%u\n", tmp->seq,
			   ch_names[tmp->ch], tmp->len);
		for (off = 0; off < tmp->len; off += 16) {
			chunk = min(16, tmp->len - off);
			seq_printf(m, "  %04x:", off);
			for (j = 0; j < chunk; j++)
				seq_printf(m, " %02x", tmp->data[off + j]);
			seq_putc(m, '\n');
		}
		spin_lock(&d->lock);
		n = d->nstored;
	}
	spin_unlock(&d->lock);
	kfree(tmp);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(log);

static ssize_t data_read(struct file *file, char __user *buf, size_t count,
			 loff_t *ppos)
{
	struct sunfish_diag *d = g_diag;
	struct sunfish_diag_pkt *copy;
	unsigned int i, idx, n, total = 0;
	char *out;
	ssize_t ret;

	copy = kcalloc(SUNFISH_DIAG_PKTS, sizeof(*copy), GFP_KERNEL);
	if (!copy)
		return -ENOMEM;

	spin_lock(&d->lock);
	n = d->nstored;
	for (i = 0; i < n; i++) {
		if (n == SUNFISH_DIAG_PKTS)
			idx = (d->head + i) % SUNFISH_DIAG_PKTS;
		else
			idx = i;
		copy[i] = d->pkts[idx];
		total += copy[i].len;
	}
	spin_unlock(&d->lock);

	out = kvmalloc(total ? total : 1, GFP_KERNEL);
	if (!out) {
		kfree(copy);
		return -ENOMEM;
	}
	total = 0;
	for (i = 0; i < n; i++) {
		memcpy(out + total, copy[i].data, copy[i].len);
		total += copy[i].len;
	}
	kfree(copy);
	ret = simple_read_from_buffer(buf, count, ppos, out, total);
	kvfree(out);
	return ret;
}

static const struct file_operations data_fops = {
	.owner = THIS_MODULE,
	.read = data_read,
	.llseek = default_llseek,
};

static const struct rpmsg_device_id sunfish_diag_id_table[] = {
	{ .name = "DIAG_CTRL" },
	{ .name = "DIAG_DATA" },
	{ .name = "DIAG_CMD" },
	{}
};
MODULE_DEVICE_TABLE(rpmsg, sunfish_diag_id_table);

static struct rpmsg_driver sunfish_diag_driver = {
	.probe = sunfish_diag_probe,
	.remove = sunfish_diag_remove,
	.callback = sunfish_diag_cb,
	.id_table = sunfish_diag_id_table,
	.drv = {
		.name = "sunfish_diag",
	},
};

static int __init sunfish_diag_init(void)
{
	struct sunfish_diag *d;
	int ret, i;

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	spin_lock_init(&d->lock);
	for (i = 0; i < CH_N; i++)
		d->dump_left[i] = 8;
	g_diag = d;

	d->dbg = debugfs_create_dir("sunfish_diag", NULL);
	debugfs_create_file("bound", 0444, d->dbg, NULL, &bound_fops);
	debugfs_create_file("log", 0444, d->dbg, NULL, &log_fops);
	debugfs_create_file("data", 0444, d->dbg, NULL, &data_fops);

	ret = register_rpmsg_driver(&sunfish_diag_driver);
	if (ret) {
		debugfs_remove_recursive(d->dbg);
		kfree(d);
		g_diag = NULL;
		return ret;
	}
	pr_info("sunfish-diag: waiting for DIAG_CTRL/DATA/CMD on edge modem\n");
	return 0;
}

static void __exit sunfish_diag_exit(void)
{
	unregister_rpmsg_driver(&sunfish_diag_driver);
	if (g_diag) {
		debugfs_remove_recursive(g_diag->dbg);
		kfree(g_diag);
		g_diag = NULL;
	}
}

module_init(sunfish_diag_init);
module_exit(sunfish_diag_exit);

MODULE_DESCRIPTION("sunfish DIAG rpmsg sniffer (Lead 3l Stage A)");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("pixel-4a bring-up");
