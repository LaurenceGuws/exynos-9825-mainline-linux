// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos9820/9825 G3D (Mali-G76) power domain.
 *
 * The G3D block consists of the Mali GPU core ("embedded_g3d") inside the
 * outer G3D power domain.  The CMU_G3D clock controller (0x18400000) lives
 * in the same domain, so its muxes, dividers and gates are programmed here
 * after the PMU sequence brings the domain up.
 *
 * The GPU core clock is routed from the always-running CMU_TOP G3D switch
 * clock (PLL_SHARED2).  PLL_G3D is enabled as required by the vendor power-on
 * sequence, but is not selected for the core clock path:
 *
 *   PLL_SHARED2 -> gout_cmu_g3d_switch -> dout_cmu_g3d_switch
 *     -> mout_cmu_g3d_switch_user -> mout_clk_g3d_busd
 *     -> dout_clk_g3d_busd -> mout_cmu_embedded_g3d_user -> GPU leaf gate
 *
 * PMU and CMU register sequences are taken from the Samsung SM-N975F
 * (exynos9825) OSS pmucal/cmucal tables, which are identical to exynos9820.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/arm-smccc.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>

/* PMU (0x15860000) register offsets */
#define PMU_EMBEDDED_G3D_CONFIGURATION		0x2100
#define PMU_EMBEDDED_G3D_STATUS			0x2104
#define PMU_EMBEDDED_G3D_OUT			0x2120
#define PMU_G3D_CONFIGURATION			0x2380
#define PMU_G3D_STATUS				0x2384

/*
 * PMU_ALIVE registers that are shared with other masters (CP etc.) are
 * modified through the atomic set/clear aliases (offset | 0xc000 / 0x8000).
 */
#define PMU_ATOMIC_SET(off)			((off) | 0xc000)

/*
 * DTZPC unlock: after powering the domain, the secure monitor must open the
 * G3D block's TZPC so the non-secure world can program CMU_G3D.  Mirrors the
 * vendor exynos_pd_tz_restore() call (SMC_CMD_PREAPRE_PD_ONOFF with the
 * need_smc value from the vendor pd-embedded_g3d DT node).
 */
#define SMC_CMD_PREAPRE_PD_ONOFF	0x82000410
#define EXYNOS_WAKEUP_PD_DOWN		1
#define RUNTIME_PM_TZPC_GROUP		2
#define EMBEDDED_G3D_TZPC_ADDR		0x18410204

/* CMU_G3D (0x18400000) register offsets */
#define CMU_MUX_EMBEDDED_G3D_USER		0x0100
#define CMU_MUX_G3D_BUS_USER			0x0120
#define CMU_MUX_G3D_SWITCH_USER			0x0140
#define CMU_MUX_CLK_G3D_BUSD			0x1000
#define CMU_GATE_G3D_CMU_G3D_PCLK		0x2000
#define CMU_GATE_GPU_CLK			0x2004
#define CMU_GATE_HPM_G3D0			0x2008
#define CMU_GATE_RSTNSYNC_G3D_OSCCLK		0x200c
#define CMU_GATE_ASB_QE_G3D0			0x2014
#define CMU_GATE_ASB_QE_G3D1			0x2018
#define CMU_GATE_ASB_QE_G3D2			0x201c
#define CMU_GATE_ASB_QE_G3D3			0x2020
#define CMU_GATE_ASB_PPMU_G3D0			0x2024
#define CMU_GATE_ASB_PPMU_G3D1			0x2028
#define CMU_GATE_ASB_PPMU_G3D2			0x202c
#define CMU_GATE_ASB_PPMU_G3D3			0x2030
#define CMU_GATE_AXI2APB_G3D			0x2034
#define CMU_GATE_BUSIF_HPMG3D			0x2038
#define CMU_GATE_D_TZPC_G3D			0x203c
#define CMU_GATE_GRAY2BIN_G3D			0x2040
#define CMU_GATE_LHM_AXI_G3DSFR			0x2044
#define CMU_GATE_LHM_AXI_P_G3D			0x2048
#define CMU_GATE_LHS_AXI_G3DSFR			0x204c
#define CMU_GATE_RSTNSYNC_G3D_BUSP		0x2050
#define CMU_GATE_SYSREG_G3D			0x2054
#define CMU_GATE_VGEN_LITE_G3D			0x2058
#define CMU_GATE_XIU_P_G3D			0x205c
#define CMU_QCH_BUSIF_HPMG3D			0x303c

#define GATE_ENABLE				BIT(21)
#define GATE_MANUAL				BIT(20)
#define MUX_SEL				BIT(4)
#define MUX_BUSD_SEL				BIT(0)
/*
 * CMU user muxes report busy on bit 7 (cmucal-sfr.c *_USER_BUSY); the
 * CLK_CON_MUX_*_BUSD mux and the dividers use bit 16.
 */
#define MUX_USER_BUSY				BIT(7)
#define BUSY				BIT(16)

/*
 * PLL_G3D lives in the always-on CMU_TOP block (0x1a240000); the vendor
 * pmucal enables it first in the G3D power-on sequence even though the
 * core clock normally runs from the G3D_SWITCH path.
 */
#define PLL_G3D_BASE				0x1a240000
#define PLL_CON0_PLL_G3D			0x140
#define PLL_CON2_PLL_G3D			0x148
#define PLL_G3D_ON_CON2				0x30000003
#define PLL_CON0_MUX_SEL			BIT(4)

struct exynos9820_g3d_pd {
	struct generic_pm_domain genpd;
	struct device *dev;
	struct regmap *pmu;
	void __iomem *cmu;
};

static void exynos9820_g3d_cmu_gates(struct exynos9820_g3d_pd *pd, bool on)
{
	static const u32 gates[] = {
		CMU_GATE_G3D_CMU_G3D_PCLK,
		CMU_GATE_GPU_CLK,
		CMU_GATE_HPM_G3D0,
		CMU_GATE_RSTNSYNC_G3D_OSCCLK,
		CMU_GATE_ASB_QE_G3D0,
		CMU_GATE_ASB_QE_G3D1,
		CMU_GATE_ASB_QE_G3D2,
		CMU_GATE_ASB_QE_G3D3,
		CMU_GATE_ASB_PPMU_G3D0,
		CMU_GATE_ASB_PPMU_G3D1,
		CMU_GATE_ASB_PPMU_G3D2,
		CMU_GATE_ASB_PPMU_G3D3,
		CMU_GATE_AXI2APB_G3D,
		CMU_GATE_BUSIF_HPMG3D,
		CMU_GATE_D_TZPC_G3D,
		CMU_GATE_GRAY2BIN_G3D,
		CMU_GATE_LHM_AXI_G3DSFR,
		CMU_GATE_LHM_AXI_P_G3D,
		CMU_GATE_LHS_AXI_G3DSFR,
		CMU_GATE_RSTNSYNC_G3D_BUSP,
		CMU_GATE_SYSREG_G3D,
		CMU_GATE_VGEN_LITE_G3D,
		CMU_GATE_XIU_P_G3D,
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(gates); i++) {
		u32 val = readl_relaxed(pd->cmu + gates[i]);

		if (on)
			val |= GATE_MANUAL | GATE_ENABLE;
		else
			val &= ~(GATE_MANUAL | GATE_ENABLE);
		writel_relaxed(val, pd->cmu + gates[i]);
	}
}

/*
 * Mirror the vendor pmucal G3D ON sequence: PLL_CON2 first, then the
 * CON0 mux-select bit.  PLL ratios (CON0/CON1 fields) are left as the
 * bootloader configured them; the core clock uses the G3D_SWITCH path
 * anyway, so this is belt-and-braces.
 */
static int exynos9820_g3d_pll_enable(struct exynos9820_g3d_pd *pd)
{
	void __iomem *pll = ioremap(PLL_G3D_BASE, 0x1000);
	u32 val;

	if (!pll) {
		dev_err(pd->dev, "failed to map PLL_G3D registers\n");
		return -ENOMEM;
	}
	writel_relaxed(PLL_G3D_ON_CON2, pll + PLL_CON2_PLL_G3D);
	val = readl_relaxed(pll + PLL_CON0_PLL_G3D);
	val |= PLL_CON0_MUX_SEL;
	writel_relaxed(val, pll + PLL_CON0_PLL_G3D);
	iounmap(pll);

	return 0;
}

static void exynos9820_g3d_dump(struct exynos9820_g3d_pd *pd)
{
	void __iomem *pll = ioremap(PLL_G3D_BASE, 0x1000);

	pr_info("exynos9820-g3d: cmu mux 0x100=%08x 0x120=%08x 0x140=%08x 0x1000=%08x\n",
		readl_relaxed(pd->cmu + CMU_MUX_EMBEDDED_G3D_USER),
		readl_relaxed(pd->cmu + CMU_MUX_G3D_BUS_USER),
		readl_relaxed(pd->cmu + CMU_MUX_G3D_SWITCH_USER),
		readl_relaxed(pd->cmu + CMU_MUX_CLK_G3D_BUSD));
	pr_info("exynos9820-g3d: cmu gate 0x2000=%08x 0x2004=%08x div 0x1800=%08x 0x1804=%08x\n",
		readl_relaxed(pd->cmu + CMU_GATE_G3D_CMU_G3D_PCLK),
		readl_relaxed(pd->cmu + CMU_GATE_GPU_CLK),
		readl_relaxed(pd->cmu + 0x1800),
		readl_relaxed(pd->cmu + 0x1804));
	if (pll) {
		pr_info("exynos9820-g3d: pll_g3d con0=%08x con1=%08x con2=%08x\n",
			readl_relaxed(pll + PLL_CON0_PLL_G3D),
			readl_relaxed(pll + 0x144),
			readl_relaxed(pll + PLL_CON2_PLL_G3D));
		iounmap(pll);
	}
}

/*
 * Route the GPU core clock from the CMU_TOP G3D switch clock and open the
 * CMU_G3D leaf gates.  Must be called with the G3D power domain up.
 */
static int exynos9820_g3d_cmu_setup(struct exynos9820_g3d_pd *pd)
{
	u32 val;
	int ret;

	/* Open the CMU_G3D gates first (CMU pclk included) */
	exynos9820_g3d_cmu_gates(pd, true);

	/* mout_cmu_g3d_switch_user: CLKCMU_G3D_SWITCH */
	val = readl_relaxed(pd->cmu + CMU_MUX_G3D_SWITCH_USER);
	val |= MUX_SEL;
	writel_relaxed(val, pd->cmu + CMU_MUX_G3D_SWITCH_USER);
	ret = readl_relaxed_poll_timeout(pd->cmu + CMU_MUX_G3D_SWITCH_USER,
					 val, !(val & MUX_USER_BUSY), 10, 10000);
	if (ret)
		return dev_err_probe(pd->dev, ret,
				     "G3D switch mux timed out\n");

	/* mout_clk_g3d_busd: MUX_CLKCMU_G3D_SWITCH_USER */
	val = readl_relaxed(pd->cmu + CMU_MUX_CLK_G3D_BUSD);
	val |= MUX_BUSD_SEL;
	writel_relaxed(val, pd->cmu + CMU_MUX_CLK_G3D_BUSD);
	ret = readl_relaxed_poll_timeout(pd->cmu + CMU_MUX_CLK_G3D_BUSD,
					 val, !(val & BUSY), 10, 10000);
	if (ret)
		return dev_err_probe(pd->dev, ret,
				     "G3D bus mux timed out\n");

	/*
	 * The G3D dividers (CLK_CON_DIV_DIV_CLK_G3D_BUSD/BUSP) are left at
	 * their bootloader defaults: the vendor cmucal only SAVE_RESTOREs
	 * them and marks DIV_CLK_G3D_BUSD as EMPTY_CAL_ID (fixed ratio), so
	 * reprogramming them here could alter the GPU clock rate.  The
	 * mout_cmu_g3d_bus_user mux also stays on OSCCLK (PLL_G3D unused).
	 */

	/* mout_cmu_embedded_g3d_user: DIV_CLK_G3D_BUSD, enables GPU core clock */
	val = readl_relaxed(pd->cmu + CMU_MUX_EMBEDDED_G3D_USER);
	val |= MUX_SEL;
	writel_relaxed(val, pd->cmu + CMU_MUX_EMBEDDED_G3D_USER);
	ret = readl_relaxed_poll_timeout(pd->cmu + CMU_MUX_EMBEDDED_G3D_USER,
					 val, !(val & MUX_USER_BUSY), 10, 10000);
	if (ret)
		return dev_err_probe(pd->dev, ret,
				     "embedded G3D mux timed out\n");

	return 0;
}

static int exynos9820_g3d_dtzpc_unlock(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMC_CMD_PREAPRE_PD_ONOFF, EXYNOS_WAKEUP_PD_DOWN,
		      EMBEDDED_G3D_TZPC_ADDR, RUNTIME_PM_TZPC_GROUP,
		      0, 0, 0, 0, &res);

	return res.a0;
}

static int exynos9820_g3d_pd_power_off(struct generic_pm_domain *genpd);

static int exynos9820_g3d_pd_power_on(struct generic_pm_domain *genpd)
{
	struct exynos9820_g3d_pd *pd = container_of(genpd,
						    struct exynos9820_g3d_pd,
						    genpd);
	u32 val;
	int ret;

	/* Vendor order: PLL_G3D first, then the PMU sequence */
	ret = exynos9820_g3d_pll_enable(pd);
	if (ret)
		return ret;

	/* Force the outer G3D domain into its off state first */
	ret = regmap_update_bits(pd->pmu, PMU_G3D_CONFIGURATION, 1, 0);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(pd->pmu, PMU_G3D_STATUS, val,
				       !(val & 1), 10, 10000);
	if (ret)
		return ret;

	/* Request power for the G3D domain, then the embedded Mali core */
	ret = regmap_write(pd->pmu, PMU_ATOMIC_SET(PMU_EMBEDDED_G3D_OUT), 2);
	if (ret)
		return ret;
	usleep_range(250, 300);
	ret = regmap_update_bits(pd->pmu, PMU_G3D_CONFIGURATION, 1, 1);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(pd->pmu, PMU_G3D_STATUS, val,
				       val & 1, 10, 10000);
	if (ret) {
		if (regmap_update_bits(pd->pmu, PMU_G3D_CONFIGURATION, 1, 0))
			dev_err(pd->dev,
				"failed to cancel timed-out outer power-on\n");
		return ret;
	}
	usleep_range(250, 300);

	ret = regmap_update_bits(pd->pmu, PMU_EMBEDDED_G3D_CONFIGURATION, 1, 1);
	if (ret)
		goto rollback;
	ret = regmap_read_poll_timeout(pd->pmu, PMU_EMBEDDED_G3D_STATUS, val,
				       val & 1, 10, 10000);
	if (ret)
		goto rollback;

	ret = exynos9820_g3d_dtzpc_unlock();
	if (ret) {
		dev_err(pd->dev, "DTZPC unlock failed: %d, skipping CMU_G3D setup\n",
			ret);
		ret = -EIO;
		goto rollback;
	}

	ret = exynos9820_g3d_cmu_setup(pd);
	if (ret)
		goto rollback;
	exynos9820_g3d_dump(pd);

	pr_info("exynos9820-g3d: pd-embedded_g3d powered on, GPU clock routed via G3D_SWITCH\n");

	return 0;

rollback:
	if (exynos9820_g3d_pd_power_off(genpd))
		dev_err(pd->dev, "failed to roll back G3D power-on\n");
	return ret;
}

static int exynos9820_g3d_pd_power_off(struct generic_pm_domain *genpd)
{
	struct exynos9820_g3d_pd *pd = container_of(genpd,
						    struct exynos9820_g3d_pd,
						    genpd);
	u32 val;
	int ret;

	/* Close the HPM QCH, then drop the embedded Mali core */
	val = readl_relaxed(pd->cmu + CMU_QCH_BUSIF_HPMG3D);
	val &= ~BIT(2);
	writel_relaxed(val, pd->cmu + CMU_QCH_BUSIF_HPMG3D);

	exynos9820_g3d_cmu_gates(pd, false);

	ret = regmap_update_bits(pd->pmu, PMU_EMBEDDED_G3D_CONFIGURATION, 1, 0);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(pd->pmu, PMU_EMBEDDED_G3D_STATUS, val,
				       !(val & 1), 10, 10000);
	if (ret)
		return ret;
	usleep_range(130, 160);

	ret = regmap_update_bits(pd->pmu, PMU_G3D_CONFIGURATION, 1, 0);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(pd->pmu, PMU_G3D_STATUS, val,
				       !(val & 1), 10, 10000);
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id exynos9820_g3d_pd_of_match[] = {
	{ .compatible = "samsung,exynos9820-g3d-pd" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos9820_g3d_pd_of_match);

static int exynos9820_g3d_pd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos9820_g3d_pd *pd;
	int ret;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->dev = dev;
	pd->pmu = syscon_regmap_lookup_by_phandle(dev->of_node,
						  "samsung,pmu-syscon");
	if (IS_ERR(pd->pmu))
		return dev_err_probe(dev, PTR_ERR(pd->pmu),
				     "failed to get PMU syscon\n");

	pd->cmu = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pd->cmu))
		return PTR_ERR(pd->cmu);

	pd->genpd.name = dev_name(dev);
	pd->genpd.power_off = exynos9820_g3d_pd_power_off;
	pd->genpd.power_on = exynos9820_g3d_pd_power_on;
	/*
	 * Keep the domain always on for the initial bring-up: panfrost does
	 * aggressive runtime-PM autosuspend and power-cycling the Mali block
	 * is only enabled after the first boot is proven.  Because always-on
	 * domains are considered powered on at init, run the PMU power-on
	 * sequence and CMU_G3D clock setup explicitly here.
	 */
	pd->genpd.flags |= GENPD_FLAG_ALWAYS_ON;

	ret = pm_genpd_init(&pd->genpd, NULL, false);
	if (ret)
		return ret;

	ret = exynos9820_g3d_pd_power_on(&pd->genpd);
	if (ret) {
		dev_err(dev, "failed to power on G3D domain: %d\n", ret);
		if (pm_genpd_remove(&pd->genpd))
			dev_warn(dev, "failed to remove unregistered power domain\n");
		return ret;
	}

	ret = of_genpd_add_provider_simple(dev->of_node, &pd->genpd);
	if (ret) {
		int cleanup_ret;

		cleanup_ret = exynos9820_g3d_pd_power_off(&pd->genpd);
		if (cleanup_ret)
			dev_warn(dev, "failed to power off G3D after probe error: %d\n",
				 cleanup_ret);
		cleanup_ret = pm_genpd_remove(&pd->genpd);
		if (cleanup_ret)
			dev_warn(dev, "failed to remove power domain: %d\n",
				 cleanup_ret);
		return ret;
	}
	platform_set_drvdata(pdev, pd);

	return 0;
}

static void exynos9820_g3d_pd_remove(struct platform_device *pdev)
{
	struct exynos9820_g3d_pd *pd = platform_get_drvdata(pdev);
	int ret;

	of_genpd_del_provider(pdev->dev.of_node);
	ret = pm_genpd_remove(&pd->genpd);
	if (ret) {
		dev_warn(&pdev->dev, "failed to remove power domain: %d\n", ret);
		return;
	}

	ret = exynos9820_g3d_pd_power_off(&pd->genpd);
	if (ret)
		dev_warn(&pdev->dev, "failed to power off domain: %d\n", ret);
}

static struct platform_driver exynos9820_g3d_pd_driver = {
	.driver = {
		.name = "exynos9820-g3d-pd",
		.of_match_table = exynos9820_g3d_pd_of_match,
	},
	.probe = exynos9820_g3d_pd_probe,
	.remove = exynos9820_g3d_pd_remove,
};
module_platform_driver(exynos9820_g3d_pd_driver);

MODULE_AUTHOR("exynos-9825-mainline");
MODULE_DESCRIPTION("Samsung Exynos9820/9825 G3D power domain driver");
MODULE_LICENSE("GPL");
