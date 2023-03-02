// SPDX-License-Identifier: GPL-2.0+
/*
 * StarFive DWMAC platform driver
 *
 * Copyright (C) 2022 StarFive Technology Co., Ltd.
 * Copyright (C) 2022 Emil Renner Berthing <kernel@esmil.dk>
 *
 */

#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include "stmmac_platform.h"

#define MACPHYC_PHY_INFT_RMII	0x4
#define MACPHYC_PHY_INFT_RGMII	0x1

struct starfive_dwmac {
	struct device *dev;
	struct clk *clk_tx;
	struct clk *clk_gtx;
	bool tx_use_rgmii_rxin_clk;
};

static void starfive_eth_fix_mac_speed(void *priv, unsigned int speed)
{
	struct starfive_dwmac *dwmac = priv;
	unsigned long rate;
	int err;

	/* Generally, the rgmii_tx clock is provided by the internal clock,
	 * which needs to match the corresponding clock frequency according
	 * to different speeds. If the rgmii_tx clock is provided by the
	 * external rgmii_rxin, there is no need to configure the clock
	 * internally, because rgmii_rxin will be adaptively adjusted.
	 */
	if (dwmac->tx_use_rgmii_rxin_clk)
		return;

	switch (speed) {
	case SPEED_1000:
		rate = 125000000;
		break;
	case SPEED_100:
		rate = 25000000;
		break;
	case SPEED_10:
		rate = 2500000;
		break;
	default:
		dev_err(dwmac->dev, "invalid speed %u\n", speed);
		break;
	}

	err = clk_set_rate(dwmac->clk_tx, rate);
	if (err)
		dev_err(dwmac->dev, "failed to set tx rate %lu\n", rate);
}

static int starfive_dwmac_set_mode(struct plat_stmmacenet_data *plat_dat)
{
	struct starfive_dwmac *dwmac = plat_dat->bsp_priv;
	struct of_phandle_args args;
	struct regmap *regmap;
	unsigned int reg, mask, mode;
	int err;

	switch (plat_dat->interface) {
	case PHY_INTERFACE_MODE_RMII:
		mode = MACPHYC_PHY_INFT_RMII;
		break;

	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
		mode = MACPHYC_PHY_INFT_RGMII;
		break;

	default:
		dev_err(dwmac->dev, "Unsupported interface %d\n",
			plat_dat->interface);
	}

	err = of_parse_phandle_with_fixed_args(dwmac->dev->of_node,
					       "starfive,syscon", 2, 0, &args);
	if (err) {
		dev_dbg(dwmac->dev, "syscon reg not found\n");
		return -EINVAL;
	}

	reg = args.args[0];
	mask = args.args[1];
	regmap = syscon_node_to_regmap(args.np);
	of_node_put(args.np);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return regmap_update_bits(regmap, reg, mask, mode << __ffs(mask));
}

static int starfive_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct starfive_dwmac *dwmac;
	int err;

	err = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (err)
		return err;

	plat_dat = stmmac_probe_config_dt(pdev, stmmac_res.mac);
	if (IS_ERR(plat_dat)) {
		dev_err(&pdev->dev, "dt configuration failed\n");
		return PTR_ERR(plat_dat);
	}

	dwmac = devm_kzalloc(&pdev->dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac)
		return -ENOMEM;

	dwmac->clk_tx = devm_clk_get_enabled(&pdev->dev, "tx");
	if (IS_ERR(dwmac->clk_tx))
		return dev_err_probe(&pdev->dev, PTR_ERR(dwmac->clk_tx),
				    "error getting tx clock\n");

	dwmac->clk_gtx = devm_clk_get_enabled(&pdev->dev, "gtx");
	if (IS_ERR(dwmac->clk_gtx))
		return dev_err_probe(&pdev->dev, PTR_ERR(dwmac->clk_gtx),
				    "error getting gtx clock\n");

	if (device_property_read_bool(&pdev->dev, "starfive,tx-use-rgmii-clk"))
		dwmac->tx_use_rgmii_rxin_clk = true;

	dwmac->dev = &pdev->dev;
	plat_dat->fix_mac_speed = starfive_eth_fix_mac_speed;
	plat_dat->init = NULL;
	plat_dat->bsp_priv = dwmac;
	plat_dat->dma_cfg->dche = true;

	starfive_dwmac_set_mode(plat_dat);
	err = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (err) {
		stmmac_remove_config_dt(pdev, plat_dat);
		return err;
	}

	return 0;
}

static const struct of_device_id starfive_dwmac_match[] = {
	{ .compatible = "starfive,jh7110-dwmac"	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, starfive_dwmac_match);

static struct platform_driver starfive_dwmac_driver = {
	.probe  = starfive_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name = "starfive-dwmac",
		.pm = &stmmac_pltfr_pm_ops,
		.of_match_table = starfive_dwmac_match,
	},
};
module_platform_driver(starfive_dwmac_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("StarFive DWMAC platform driver");
MODULE_AUTHOR("Emil Renner Berthing <kernel@esmil.dk>");
MODULE_AUTHOR("Samin Guo <samin.guo@starfivetech.com>");
