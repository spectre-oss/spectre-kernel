// SPDX-License-Identifier: GPL-2.0-only
/*
 * rt5514-spi.c  --  RT5514 SPI driver
 *
 * Copyright 2015 Realtek Semiconductor Corp.
 * Author: Oder Chiou <oder_chiou@realtek.com>
 */

#include <linux/cleanup.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_qos.h>
#include <linux/sysfs.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "rt5514-spi.h"

#define DRV_NAME "rt5514-spi"
#define RT5514_SPI_HOLD_PERIOD 4096

static struct spi_device *rt5514_spi;
static struct rt5514_dsp *rt5514_spi_dsp;
static u8 rt5514_spi_discard[RT5514_SPI_HOLD_PERIOD];

struct rt5514_dsp {
	struct device *dev;
	struct delayed_work copy_work;
	struct mutex dma_lock;
	struct snd_pcm_substream *substream;
	bool hold_stream;
	bool v_p;
	unsigned int buf_base, buf_limit, buf_rp;
	size_t buf_size, get_size, dma_offset;
	int irq;
};

static const struct snd_pcm_hardware rt5514_spi_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.period_bytes_min	= PAGE_SIZE,
	.period_bytes_max	= 0x20000 / 8,
	.periods_min		= 8,
	.periods_max		= 8,
	.channels_min		= 1,
	.channels_max		= 1,
	.buffer_bytes_max	= 0x20000,
};

static struct snd_soc_dai_driver rt5514_spi_dai = {
	.name = "rt5514-dsp-cpu-dai",
	.id = 0,
	.capture = {
		.stream_name = "DSP Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static bool rt5514_spi_read_pointer(unsigned int reg, unsigned int *value)
{
	u8 buf[8];

	if (!rt5514_spi_burst_read(reg, buf, sizeof(buf)))
		return false;

	*value = (u32)buf[0] | (u32)buf[1] << 8 |
		 (u32)buf[2] << 16 | (u32)buf[3] << 24;
	return true;
}

static bool rt5514_spi_ring_valid(bool v_p, unsigned int base,
				 unsigned int limit, unsigned int wp)
{
	if (!base || limit <= base || !IS_ALIGNED(base, 8) ||
	    !IS_ALIGNED(limit, 8) || !IS_ALIGNED(wp, 8) ||
	    wp < base || wp > limit)
		return false;

	if (v_p && ((base & 0xffe00000) != 0x4fe00000 ||
		    ((limit - 1) & 0xffe00000) != 0x4fe00000))
		return false;

	return true;
}

static bool rt5514_spi_period_valid(struct rt5514_dsp *dsp,
				   size_t period_bytes)
{
	return rt5514_spi_ring_valid(dsp->v_p, dsp->buf_base,
				    dsp->buf_limit, dsp->buf_rp) &&
	       dsp->buf_rp < dsp->buf_limit && period_bytes &&
	       IS_ALIGNED(period_bytes, 8) &&
	       period_bytes <= dsp->buf_limit - dsp->buf_base;
}

static bool rt5514_spi_voice_ready(struct rt5514_dsp *rt5514_dsp,
				   size_t period_bytes)
{
	unsigned int cur_wp, remain_data;
	unsigned int wp_reg = rt5514_dsp->v_p ? RT5514P_BUFFER_VOICE_WP :
					      RT5514_BUFFER_VOICE_WP;

	if (!rt5514_spi_period_valid(rt5514_dsp, period_bytes) ||
	    !rt5514_spi_read_pointer(wp_reg, &cur_wp) ||
	    !rt5514_spi_ring_valid(rt5514_dsp->v_p, rt5514_dsp->buf_base,
				  rt5514_dsp->buf_limit, cur_wp))
		return false;

	if (cur_wp == rt5514_dsp->buf_limit)
		cur_wp = rt5514_dsp->buf_base;

	if (cur_wp >= rt5514_dsp->buf_rp)
		remain_data = cur_wp - rt5514_dsp->buf_rp;
	else
		remain_data = (rt5514_dsp->buf_limit - rt5514_dsp->buf_rp) +
			      (cur_wp - rt5514_dsp->buf_base);

	return remain_data >= period_bytes;
}

static bool rt5514_spi_advance_rp(struct rt5514_dsp *rt5514_dsp,
				  u8 *dest, size_t period_bytes)
{
	size_t truncated_bytes;

	if (!rt5514_spi_period_valid(rt5514_dsp, period_bytes))
		return false;

	truncated_bytes = rt5514_dsp->buf_limit - rt5514_dsp->buf_rp;
	if (period_bytes <= truncated_bytes) {
		if (!rt5514_spi_burst_read(rt5514_dsp->buf_rp, dest,
					  period_bytes))
			return false;

		if (period_bytes == truncated_bytes)
			rt5514_dsp->buf_rp = rt5514_dsp->buf_base;
		else
			rt5514_dsp->buf_rp += period_bytes;
	} else {
		if (!rt5514_spi_burst_read(rt5514_dsp->buf_rp, dest,
					  truncated_bytes) ||
		    !rt5514_spi_burst_read(rt5514_dsp->buf_base,
					  dest + truncated_bytes,
					  period_bytes - truncated_bytes))
			return false;

		rt5514_dsp->buf_rp = rt5514_dsp->buf_base + period_bytes -
			truncated_bytes;
	}

	return true;
}

static void rt5514_spi_copy_work(struct work_struct *work)
{
	struct rt5514_dsp *rt5514_dsp =
		container_of(work, struct rt5514_dsp, copy_work.work);
	struct snd_pcm_runtime *runtime;
	size_t period_bytes;
	u8 *dest;

	mutex_lock(&rt5514_dsp->dma_lock);
	if (!rt5514_dsp->substream && !rt5514_dsp->hold_stream)
		goto done;

	if (rt5514_dsp->substream) {
		runtime = rt5514_dsp->substream->runtime;
		period_bytes = snd_pcm_lib_period_bytes(rt5514_dsp->substream);
		dest = runtime->dma_area + rt5514_dsp->dma_offset;
		if (!period_bytes) {
			schedule_delayed_work(&rt5514_dsp->copy_work, 5);
			goto done;
		}

		if (rt5514_dsp->buf_size % period_bytes)
			rt5514_dsp->buf_size =
				(rt5514_dsp->buf_size / period_bytes) *
				period_bytes;

		if (rt5514_dsp->get_size >= rt5514_dsp->buf_size &&
		    !rt5514_spi_voice_ready(rt5514_dsp, period_bytes)) {
			schedule_delayed_work(&rt5514_dsp->copy_work, 5);
			goto done;
		}
	} else {
		period_bytes = RT5514_SPI_HOLD_PERIOD;
		dest = rt5514_spi_discard;
		if (!rt5514_spi_voice_ready(rt5514_dsp, period_bytes)) {
			schedule_delayed_work(&rt5514_dsp->copy_work, 5);
			goto done;
		}
	}

	if (!rt5514_spi_advance_rp(rt5514_dsp, dest, period_bytes)) {
		schedule_delayed_work(&rt5514_dsp->copy_work, 5);
		goto done;
	}

	if (rt5514_dsp->substream) {
		rt5514_dsp->get_size += period_bytes;
		rt5514_dsp->dma_offset += period_bytes;
		if (rt5514_dsp->dma_offset >= runtime->dma_bytes)
			rt5514_dsp->dma_offset = 0;
		snd_pcm_period_elapsed(rt5514_dsp->substream);
	}

	schedule_delayed_work(&rt5514_dsp->copy_work, 5);
done:
	mutex_unlock(&rt5514_dsp->dma_lock);
}

static bool rt5514_schedule_copy(struct rt5514_dsp *rt5514_dsp)
{
	unsigned int base, limit, wp;
	unsigned int base_reg = rt5514_dsp->v_p ? RT5514P_BUFFER_VOICE_BASE :
						RT5514_BUFFER_VOICE_BASE;
	unsigned int limit_reg = rt5514_dsp->v_p ? RT5514P_BUFFER_VOICE_LIMIT :
						 RT5514_BUFFER_VOICE_LIMIT;
	unsigned int wp_reg = rt5514_dsp->v_p ? RT5514P_BUFFER_VOICE_WP :
					      RT5514_BUFFER_VOICE_WP;

	if (!rt5514_dsp->substream && !rt5514_dsp->hold_stream)
		return false;

	rt5514_dsp->get_size = 0;
	rt5514_dsp->buf_base = 0;
	rt5514_dsp->buf_limit = 0;
	rt5514_dsp->buf_rp = 0;
	rt5514_dsp->buf_size = 0;

	/* Register mailboxes need separate eight-byte SPI reads. */
	if (!rt5514_spi_read_pointer(base_reg, &base) ||
	    !rt5514_spi_read_pointer(limit_reg, &limit) ||
	    !rt5514_spi_read_pointer(wp_reg, &wp)) {
		dev_warn_ratelimited(rt5514_dsp->dev,
				     "voice ring mailbox read failed\n");
		return false;
	}

	if (!rt5514_spi_ring_valid(rt5514_dsp->v_p, base, limit, wp) ||
	    (rt5514_dsp->hold_stream &&
	     limit - base < RT5514_SPI_HOLD_PERIOD)) {
		dev_warn_ratelimited(rt5514_dsp->dev,
				     "invalid voice ring %08x/%08x wp %08x\n",
				     base, limit, wp);
		return false;
	}

	rt5514_dsp->buf_base = base;
	rt5514_dsp->buf_limit = limit;
	rt5514_dsp->buf_rp = wp == limit ? base : wp;
	rt5514_dsp->buf_size = limit - base;
	schedule_delayed_work(&rt5514_dsp->copy_work, 0);
	return true;
}

static irqreturn_t rt5514_spi_irq(int irq, void *data)
{
	struct rt5514_dsp *rt5514_dsp = data;

	mutex_lock(&rt5514_dsp->dma_lock);
	rt5514_schedule_copy(rt5514_dsp);
	mutex_unlock(&rt5514_dsp->dma_lock);

	return IRQ_HANDLED;
}

/* PCM for streaming audio from the DSP buffer */
static int rt5514_spi_pcm_open(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &rt5514_spi_pcm_hardware);

	return 0;
}

static int rt5514_spi_hw_params(struct snd_soc_component *component,
				struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *hw_params)
{
	struct rt5514_dsp *rt5514_dsp =
		snd_soc_component_get_drvdata(component);
	u8 buf[8];

	guard(mutex)(&rt5514_dsp->dma_lock);
	rt5514_dsp->substream = substream;
	rt5514_dsp->dma_offset = 0;

	/* Read IRQ status and schedule copy accordingly. */
	if (rt5514_spi_burst_read(RT5514_IRQ_CTRL, buf, sizeof(buf)) &&
	    (buf[0] & RT5514_IRQ_STATUS_BIT))
		rt5514_schedule_copy(rt5514_dsp);

	return 0;
}

static int rt5514_spi_hw_free(struct snd_soc_component *component,
			      struct snd_pcm_substream *substream)
{
	struct rt5514_dsp *rt5514_dsp =
		snd_soc_component_get_drvdata(component);

	scoped_guard(mutex, &rt5514_dsp->dma_lock)
		rt5514_dsp->substream = NULL;

	cancel_delayed_work_sync(&rt5514_dsp->copy_work);

	return 0;
}

static snd_pcm_uframes_t rt5514_spi_pcm_pointer(
		struct snd_soc_component *component,
		struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct rt5514_dsp *rt5514_dsp =
		snd_soc_component_get_drvdata(component);

	return bytes_to_frames(runtime, rt5514_dsp->dma_offset);
}


static int rt5514_spi_pcm_probe(struct snd_soc_component *component)
{
	struct rt5514_dsp *rt5514_dsp = dev_get_drvdata(component->dev);

	if (!rt5514_dsp)
		return -ENODEV;

	snd_soc_component_set_drvdata(component, rt5514_dsp);
	return 0;
}

static void rt5514_spi_pcm_remove(struct snd_soc_component *component)
{
	struct rt5514_dsp *rt5514_dsp =
		snd_soc_component_get_drvdata(component);

	snd_soc_component_set_drvdata(component, NULL);

	if (rt5514_dsp->irq) {
		free_irq(rt5514_dsp->irq, rt5514_dsp);
		device_init_wakeup(rt5514_dsp->dev, false);
	}

	cancel_delayed_work_sync(&rt5514_dsp->copy_work);

	mutex_destroy(&rt5514_dsp->dma_lock);
	kfree(rt5514_dsp);
}

static int rt5514_spi_pcm_new(struct snd_soc_component *component,
			      struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_VMALLOC,
				       NULL, 0, 0);
	return 0;
}

static const struct snd_soc_component_driver rt5514_spi_component = {
	.name			= DRV_NAME,
	.probe			= rt5514_spi_pcm_probe,
	.remove			= rt5514_spi_pcm_remove,
	.open			= rt5514_spi_pcm_open,
	.hw_params		= rt5514_spi_hw_params,
	.hw_free		= rt5514_spi_hw_free,
	.pointer		= rt5514_spi_pcm_pointer,
	.pcm_new		= rt5514_spi_pcm_new,
	.legacy_dai_naming	= 1,
};

/**
 * rt5514_spi_burst_read - Read data from SPI by rt5514 address.
 * @addr: Start address.
 * @rxbuf: Data Buffer for reading.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5514_spi_burst_read(unsigned int addr, u8 *rxbuf, size_t len)
{
	u8 spi_cmd = RT5514_SPI_CMD_BURST_READ;
	int status;
	u8 write_buf[8];
	unsigned int i, end, offset = 0;

	struct spi_message message;
	struct spi_transfer x[3];

	while (offset < len) {
		if (offset + RT5514_SPI_BUF_LEN <= len)
			end = RT5514_SPI_BUF_LEN;
		else
			end = len % RT5514_SPI_BUF_LEN;

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		spi_message_init(&message);
		memset(x, 0, sizeof(x));

		x[0].len = 5;
		x[0].tx_buf = write_buf;
		spi_message_add_tail(&x[0], &message);

		x[1].len = 4;
		x[1].tx_buf = write_buf;
		spi_message_add_tail(&x[1], &message);

		x[2].len = end;
		x[2].rx_buf = rxbuf + offset;
		spi_message_add_tail(&x[2], &message);

		status = spi_sync(rt5514_spi, &message);

		if (status)
			return false;

		offset += RT5514_SPI_BUF_LEN;
	}

	for (i = 0; i < len; i += 8) {
		write_buf[0] = rxbuf[i + 0];
		write_buf[1] = rxbuf[i + 1];
		write_buf[2] = rxbuf[i + 2];
		write_buf[3] = rxbuf[i + 3];
		write_buf[4] = rxbuf[i + 4];
		write_buf[5] = rxbuf[i + 5];
		write_buf[6] = rxbuf[i + 6];
		write_buf[7] = rxbuf[i + 7];

		rxbuf[i + 0] = write_buf[7];
		rxbuf[i + 1] = write_buf[6];
		rxbuf[i + 2] = write_buf[5];
		rxbuf[i + 3] = write_buf[4];
		rxbuf[i + 4] = write_buf[3];
		rxbuf[i + 5] = write_buf[2];
		rxbuf[i + 6] = write_buf[1];
		rxbuf[i + 7] = write_buf[0];
	}

	return true;
}
EXPORT_SYMBOL_GPL(rt5514_spi_burst_read);

/**
 * rt5514_spi_burst_write - Write data to SPI by rt5514 address.
 * @addr: Start address.
 * @txbuf: Data Buffer for writng.
 * @len: Data length, it must be a multiple of 8.
 *
 *
 * Returns true for success.
 */
int rt5514_spi_burst_write(u32 addr, const u8 *txbuf, size_t len)
{
	u8 spi_cmd = RT5514_SPI_CMD_BURST_WRITE;
	u8 *write_buf;
	unsigned int i, end, offset = 0;

	write_buf = kmalloc(RT5514_SPI_BUF_LEN + 6, GFP_KERNEL);

	if (write_buf == NULL)
		return -ENOMEM;

	while (offset < len) {
		if (offset + RT5514_SPI_BUF_LEN <= len)
			end = RT5514_SPI_BUF_LEN;
		else
			end = len % RT5514_SPI_BUF_LEN;

		write_buf[0] = spi_cmd;
		write_buf[1] = ((addr + offset) & 0xff000000) >> 24;
		write_buf[2] = ((addr + offset) & 0x00ff0000) >> 16;
		write_buf[3] = ((addr + offset) & 0x0000ff00) >> 8;
		write_buf[4] = ((addr + offset) & 0x000000ff) >> 0;

		for (i = 0; i < end; i += 8) {
			write_buf[i + 12] = txbuf[offset + i + 0];
			write_buf[i + 11] = txbuf[offset + i + 1];
			write_buf[i + 10] = txbuf[offset + i + 2];
			write_buf[i +  9] = txbuf[offset + i + 3];
			write_buf[i +  8] = txbuf[offset + i + 4];
			write_buf[i +  7] = txbuf[offset + i + 5];
			write_buf[i +  6] = txbuf[offset + i + 6];
			write_buf[i +  5] = txbuf[offset + i + 7];
		}

		write_buf[end + 5] = spi_cmd;

		spi_write(rt5514_spi, write_buf, end + 6);

		offset += RT5514_SPI_BUF_LEN;
	}

	kfree(write_buf);

	return 0;
}
EXPORT_SYMBOL_GPL(rt5514_spi_burst_write);

void rt5514_spi_stream_hold(bool enable, bool v_p)
{
	struct rt5514_dsp *dsp = rt5514_spi_dsp;
	bool cancel = false;

	if (!dsp) {
		pr_warn("rt5514-spi: stream hold %d but dsp not ready\n", enable);
		return;
	}

	mutex_lock(&dsp->dma_lock);
	if (enable) {
		if (dsp->v_p != v_p && (dsp->hold_stream || dsp->substream)) {
			dev_warn(dsp->dev, "cannot change active voice mailbox layout\n");
			goto unlock;
		}
		if (!dsp->hold_stream) {
			dsp->v_p = v_p;
			dsp->hold_stream = true;
			if (!rt5514_schedule_copy(dsp)) {
				dsp->hold_stream = false;
				cancel = !dsp->substream;
				goto unlock;
			}
			dev_info(dsp->dev, "stream hold on (%s voice ring)\n",
				 v_p ? "P" : "legacy");
		}
	} else if (dsp->hold_stream) {
		dsp->hold_stream = false;
		cancel = !dsp->substream;
		dev_info(dsp->dev, "rt5514-spi: stream hold off\n");
	}
unlock:
	mutex_unlock(&dsp->dma_lock);

	if (cancel)
		cancel_delayed_work_sync(&dsp->copy_work);
}
EXPORT_SYMBOL_GPL(rt5514_spi_stream_hold);

static int rt5514_spi_probe(struct spi_device *spi)
{
	struct rt5514_dsp *rt5514_dsp;
	int ret;

	rt5514_spi = spi;

	rt5514_dsp = devm_kzalloc(&spi->dev, sizeof(*rt5514_dsp), GFP_KERNEL);
	if (!rt5514_dsp)
		return -ENOMEM;

	rt5514_dsp->dev = &spi->dev;
	mutex_init(&rt5514_dsp->dma_lock);
	INIT_DELAYED_WORK(&rt5514_dsp->copy_work, rt5514_spi_copy_work);
	rt5514_spi_dsp = rt5514_dsp;
	dev_set_drvdata(&spi->dev, rt5514_dsp);

	if (spi->irq) {
		ret = devm_request_threaded_irq(&spi->dev, spi->irq, NULL,
						rt5514_spi_irq,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						"rt5514-spi", rt5514_dsp);
		if (ret)
			dev_err(&spi->dev, "Failed to request IRQ: %d\n", ret);
		else
			device_init_wakeup(rt5514_dsp->dev, true);
	}

	ret = devm_snd_soc_register_component(&spi->dev,
					      &rt5514_spi_component,
					      &rt5514_spi_dai, 1);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register component.\n");
		rt5514_spi_dsp = NULL;
		return ret;
	}

	return 0;
}

static int rt5514_suspend(struct device *dev)
{
	int irq = to_spi_device(dev)->irq;

	if (device_may_wakeup(dev))
		enable_irq_wake(irq);

	return 0;
}

static int rt5514_resume(struct device *dev)
{
	struct rt5514_dsp *rt5514_dsp = dev_get_drvdata(dev);
	int irq = to_spi_device(dev)->irq;
	u8 buf[8];

	if (device_may_wakeup(dev))
		disable_irq_wake(irq);

	if (rt5514_dsp) {
		mutex_lock(&rt5514_dsp->dma_lock);
		if (rt5514_dsp->substream || rt5514_dsp->hold_stream) {
			if (rt5514_spi_burst_read(RT5514_IRQ_CTRL, buf, sizeof(buf)) &&
			    (buf[0] & RT5514_IRQ_STATUS_BIT))
				rt5514_schedule_copy(rt5514_dsp);
		}
		mutex_unlock(&rt5514_dsp->dma_lock);
	}

	return 0;
}

static const struct dev_pm_ops rt5514_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(rt5514_suspend, rt5514_resume)
};

static const struct of_device_id rt5514_of_match[] = {
	{ .compatible = "realtek,rt5514", },
	{},
};
MODULE_DEVICE_TABLE(of, rt5514_of_match);

static struct spi_driver rt5514_spi_driver = {
	.driver = {
		.name = "rt5514",
		.pm = pm_ptr(&rt5514_pm_ops),
		.of_match_table = of_match_ptr(rt5514_of_match),
	},
	.probe = rt5514_spi_probe,
};
module_spi_driver(rt5514_spi_driver);

MODULE_DESCRIPTION("RT5514 SPI driver");
MODULE_AUTHOR("Oder Chiou <oder_chiou@realtek.com>");
MODULE_LICENSE("GPL v2");
