// SPDX-License-Identifier: GPL-2.0-only
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

struct sunfish_cam_scan {
	struct clk *mclk0;
	struct clk *mclk2;
};

static int sunfish_cam_scan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunfish_cam_scan *scan;
	int ret;

	scan = devm_kzalloc(dev, sizeof(*scan), GFP_KERNEL);
	if (!scan)
		return -ENOMEM;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;

	scan->mclk0 = devm_clk_get(dev, "mclk0");
	if (IS_ERR(scan->mclk0)) {
		ret = PTR_ERR(scan->mclk0);
		goto err_pm;
	}

	scan->mclk2 = devm_clk_get(dev, "mclk2");
	if (IS_ERR(scan->mclk2)) {
		ret = PTR_ERR(scan->mclk2);
		goto err_pm;
	}

	ret = clk_set_rate(scan->mclk0, 24000000);
	if (ret)
		goto err_pm;
	ret = clk_set_rate(scan->mclk2, 24000000);
	if (ret)
		goto err_pm;

	ret = clk_prepare_enable(scan->mclk0);
	if (ret)
		goto err_pm;
	ret = clk_prepare_enable(scan->mclk2);
	if (ret) {
		clk_disable_unprepare(scan->mclk0);
		goto err_pm;
	}

	platform_set_drvdata(pdev, scan);
	dev_info(dev, "mclk0=%lu mclk2=%lu\n",
		 clk_get_rate(scan->mclk0), clk_get_rate(scan->mclk2));
	return 0;

err_pm:
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
	return ret;
}

static void sunfish_cam_scan_remove(struct platform_device *pdev)
{
	struct sunfish_cam_scan *scan = platform_get_drvdata(pdev);

	clk_disable_unprepare(scan->mclk2);
	clk_disable_unprepare(scan->mclk0);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}

static const struct of_device_id sunfish_cam_scan_of_match[] = {
	{ .compatible = "google,sunfish-cam-scan" },
	{}
};
MODULE_DEVICE_TABLE(of, sunfish_cam_scan_of_match);

static struct platform_driver sunfish_cam_scan_driver = {
	.probe = sunfish_cam_scan_probe,
	.remove = sunfish_cam_scan_remove,
	.driver = {
		.name = "sunfish-cam-scan",
		.of_match_table = sunfish_cam_scan_of_match,
	},
};
module_platform_driver(sunfish_cam_scan_driver);

MODULE_DESCRIPTION("Pixel 4a camera MCLK hold for CCI scan");
MODULE_LICENSE("GPL");
