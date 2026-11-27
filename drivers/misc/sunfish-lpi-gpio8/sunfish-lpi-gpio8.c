// SPDX-License-Identifier: GPL-2.0
/* Throwaway: gpio8–11 CFG only (0x62b48000, 16K). No gpiochip.
 * Full 0x62b40000/0x20000 ioremap hung USB. Do not mmap from userspace.
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/printk.h>

#define GPIO8_CFG	0x62b48000UL
#define PAGE		0x1000UL
#define NPINS		4
#define MAP_LEN		(PAGE * NPINS)

#define CFG_OFF		0x00
#define VAL_OFF		0x04
#define PULL_MASK	GENMASK(1, 0)
#define FUNC_MASK	GENMASK(5, 2)
#define DRV_MASK	GENMASK(8, 6)
#define OE_BIT		BIT(9)
#define VAL_OUT		BIT(1)

/* Stock active: func3/3/4/2, 8 mA, bias-disable, output-high */
static const u32 funcs[NPINS] = { 3, 3, 4, 2 };
static const char * const names[NPINS] = { "gpio8", "gpio9", "gpio10", "gpio11" };

static int __init sunfish_lpi_gpio8_init(void)
{
	void __iomem *base;
	int i;

	pr_emerg("sunfish-lpi-gpio8: ioremap 0x%lx 16K (gpio8-11)\n", GPIO8_CFG);
	base = ioremap(GPIO8_CFG, MAP_LEN);
	if (!base) {
		pr_emerg("sunfish-lpi-gpio8: ioremap failed\n");
		return -ENOMEM;
	}
	pr_emerg("sunfish-lpi-gpio8: map ok\n");

	for (i = 0; i < NPINS; i++) {
		void __iomem *cfg = base + i * PAGE + CFG_OFF;
		void __iomem *val = base + i * PAGE + VAL_OFF;
		u32 old, new, ds = 3; /* 8 mA: v/2-1 */

		old = readl(cfg);
		new = old;
		new &= ~(PULL_MASK | FUNC_MASK | DRV_MASK | OE_BIT);
		new |= FIELD_PREP(FUNC_MASK, funcs[i]);
		new |= FIELD_PREP(DRV_MASK, ds);
		new |= OE_BIT;
		writel(new, cfg);
		writel(readl(val) | VAL_OUT, val);
		pr_emerg("sunfish-lpi-gpio8: %s CFG 0x%08x -> 0x%08x (rb 0x%08x) VAL=0x%08x\n",
			 names[i], old, new, readl(cfg), readl(val));
	}

	iounmap(base);
	return 0;
}

static void __exit sunfish_lpi_gpio8_exit(void)
{
}

module_init(sunfish_lpi_gpio8_init);
module_exit(sunfish_lpi_gpio8_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sunfish LPI gpio8-11 mux write");
