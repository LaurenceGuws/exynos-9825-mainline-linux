// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung Exynos9825 USB 3.1 DRD PHY driver
 *
 * Port of the Samsung vendor USB3P1 (USBCON) PHY bring-up sequence
 * (drivers/phy/samsung/phy-exynos-usb3p1.c) to the mainline generic PHY
 * framework, for the SM-N975F (d2s) mainline kernel bring-up.
 *
 * The Exynos9825 embeds a "USB 3.1 PHY gen1" (USB3P1) block at 0x10b00000
 * which provides the USB2 (UTMI+) transceiver used by the dwc3 controller
 * in peripheral mode.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

/* USB3P1 (USBCON) register map */
#define EXYNOS9825_USBCON_LINK_CTRL		0x04
#define EXYNOS9825_USBCON_LINK_PORT		0x08
#define EXYNOS9825_USBCON_CLKRST		0x20
#define EXYNOS9825_USBCON_UTMI			0x50
#define EXYNOS9825_USBCON_HSP			0x54
#define EXYNOS9825_USBCON_HSP_TEST		0x5c
#define EXYNOS9825_USBCON_COMBO_PMA_CTRL	0x48

/* PMU (system controller @0x15860000) USB PHY isolation controls.
 * Matches the vendor 9825 DTS: pmu_offset=0x72c (USB2/UTMI),
 * pmu_offset_dp=0x704 (USB3/pipe3), pmu_mask=bit0. Writing bit0=1
 * de-isolates (powers on) the PHY; 0 isolates it.
 */
#define EXYNOS9825_PMU_USB20_PHY_CTRL		0x72c
#define EXYNOS9825_PMU_USBDRD_PHY_CTRL		0x704
#define EXYNOS9825_PMU_PHY_ENABLE		BIT(0)

/* CMU_FSYS0A Q-channel clock gates for the USB31DRD block */
#define EXYNOS9825_CMU_FSYS0A			0x10a00000
#define QCH_CON_USB31DRD_QCH_SLV_LINK		0x3030
#define QCH_CON_USB31DRD_QCH_SLV_CTRL		0x302c
#define QCH_CON_USB31DRD_QCH_PCS		0x3028
#define QCH_CON_USB31DRD_QCH_APB		0x3020
#define QCH_CON_LHS_AXI_D_USB_QCH		0x3018
#define QCH_CON_LHM_AXI_P_USB_QCH		0x3014
#define DMYQCH_CON_USB31DRD_QCH_REF		0x3000

/* LINK_CTRL */
#define LINKCTRL_DIS_QACT_LINKGATE		BIT(12)
#define LINKCTRL_DIS_QACT_ID0			BIT(11)
#define LINKCTRL_DIS_QACT_VBUS_VALID		BIT(10)
#define LINKCTRL_DIS_QACT_BVALID		BIT(9)
#define LINKCTRL_FORCE_QACT			BIT(8)
#define LINKCTRL_BUS_FILTER_BYPASS(_x)		(((_x) & 0xf) << 4)
#define LINKCTRL_PIPE3_FORCE_RX_ELEC_IDLE	BIT(18)
#define LINKCTRL_PIPE3_FORCE_PHY_STATUS		BIT(17)
#define LINKCTRL_PIPE3_FORCE_EN			BIT(16)

/* LINK_PORT */
#define LINKPORT_HUB_PORT_SEL_OCD_U3		BIT(3)
#define LINKPORT_HUB_PORT_SEL_OCD_U2		BIT(2)

/* CLKRST */
#define CLKRST_PHY20_SW_RST			BIT(13)
#define CLKRST_PHY20_RST_SEL			BIT(12)
#define CLKRST_PHY30_SW_RST			BIT(3)
#define CLKRST_PHY30_RST_SEL			BIT(2)
#define CLKRST_PORT_RST				BIT(1)

/* UTMI */
#define UTMI_FORCE_VBUSVALID			BIT(5)
#define UTMI_FORCE_BVALID			BIT(4)
#define UTMI_DP_PULLDOWN			BIT(3)
#define UTMI_DM_PULLDOWN			BIT(2)
#define UTMI_FORCE_SUSPEND			BIT(1)
#define UTMI_FORCE_SLEEP			BIT(0)

/* HSP */
#define HSP_VBUSVLDEXTSEL			BIT(13)
#define HSP_VBUSVLDEXT				BIT(12)
#define HSP_EN_UTMISUSPEND			BIT(9)
#define HSP_COMMONONN				BIT(8)

/* HSP_TUNE (0x58): d2s board values (vendor exynos9820-d2_common.dtsi
 * usb_hs_tune, device column): tx_vref=0xf tx_pre_emp=0x3 rx_sqrx=0x5
 * compdis=0x7 tx_res=0x3 -> HSP_TUNE = 0xF06C0507
 */
#define EXYNOS9825_USBCON_HSP_TUNE		0x58
#define EXYNOS9825_HSP_TUNE_D2S			0xF06C0507

/* HSP_TEST */
#define HSP_TEST_SIDDQ				BIT(24)
#define PMA_LOW_PWRN				BIT(4)

/* ReWA (HS remote wakeup) block: vendor phy_exynos_usb3p1_rewa_ready() */
#define EXYNOS9825_USBCON_REWA_ENABLE		0x100
#define EXYNOS9825_USBCON_HSREWA_INTR		0x104
#define EXYNOS9825_USBCON_HSREWA_CTRL		0x108
#define EXYNOS9825_USBCON_HSREWA_REFTO		0x10c
#define EXYNOS9825_USBCON_HSREWA_HSTK		0x110
#define EXYNOS9825_USBCON_HSREWA_INT1_MASK	0x11c
#define REWA_ENABLE_HS_REWA_EN			BIT(0)
#define HSREWA_CTRL_DPDM_MON_SEL		BIT(24)
#define HSREWA_CTRL_DIG_BYPASS_CON_EN		BIT(28)
#define HSREWA_INT1_EVT_MASK			(BIT(18) | BIT(17) | BIT(16) | \
						 BIT(2) | BIT(1) | BIT(0))

/* HSP_TEST line-state/datarx bits (read-only diagnostics) */
#define HSP_TEST_LINESTATE_SHIFT		20
#define HSP_TEST_LINESTATE_MASK			(0x3 << HSP_TEST_LINESTATE_SHIFT)
#define HSP_TEST_HS_RXDAT			BIT(26)
#define HSP_TEST_HS_SQUELCH			BIT(25)

struct exynos9825_usbdrd_phy {
	struct device *dev;
	void __iomem *regs;
	void __iomem *cmu_regs;
	struct regmap *pmu;
	struct clk_bulk_data *clks;
	int num_clks;
	bool clocks_enabled;
	struct delayed_work wire_work;
	int wire_checks;
};

static void exynos9825_usbdrd_phy_wire_check(struct work_struct *work)
{
	struct exynos9825_usbdrd_phy *phy_drd =
		container_of(work, struct exynos9825_usbdrd_phy,
			     wire_work.work);
	u32 hsp_test = readl(phy_drd->regs + EXYNOS9825_USBCON_HSP_TEST);

	phy_drd->wire_checks++;

	dev_dbg(phy_drd->dev, "d2s-usbphy: ls=%u rxd=%u sq=%u hsp_test=%08x\n",
		(hsp_test & HSP_TEST_LINESTATE_MASK) >> HSP_TEST_LINESTATE_SHIFT,
		(hsp_test & HSP_TEST_HS_RXDAT) ? 1 : 0,
		(hsp_test & HSP_TEST_HS_SQUELCH) ? 1 : 0,
		hsp_test);

	if (phy_drd->wire_checks < 2)
		schedule_delayed_work(&phy_drd->wire_work, 50 * HZ);
}

static int exynos9825_usbdrd_phy_pmu_isolate(struct exynos9825_usbdrd_phy *phy,
					     bool isolate)
{
	u32 val = isolate ? 0 : EXYNOS9825_PMU_PHY_ENABLE;
	u32 old_usb20, old_usbdr;
	u32 usb20, usbdr;
	int ret;

	ret = regmap_read(phy->pmu, EXYNOS9825_PMU_USB20_PHY_CTRL,
			  &old_usb20);
	if (ret)
		return ret;
	ret = regmap_read(phy->pmu, EXYNOS9825_PMU_USBDRD_PHY_CTRL,
			  &old_usbdr);
	if (ret)
		return ret;

	ret = regmap_update_bits(phy->pmu, EXYNOS9825_PMU_USB20_PHY_CTRL,
				 EXYNOS9825_PMU_PHY_ENABLE, val);
	if (ret)
		return ret;
	ret = regmap_update_bits(phy->pmu, EXYNOS9825_PMU_USBDRD_PHY_CTRL,
				 EXYNOS9825_PMU_PHY_ENABLE, val);
	if (ret) {
		int rollback_ret;

		rollback_ret = regmap_update_bits(phy->pmu,
			EXYNOS9825_PMU_USB20_PHY_CTRL,
			EXYNOS9825_PMU_PHY_ENABLE,
			old_usb20 & EXYNOS9825_PMU_PHY_ENABLE);
		if (rollback_ret)
			dev_err(phy->dev,
				"failed to roll back partial PHY isolation: %d\n",
				rollback_ret);
		return ret;
	}

	ret = regmap_read(phy->pmu, EXYNOS9825_PMU_USB20_PHY_CTRL, &usb20);
	if (ret)
		goto debug_read_failed;
	ret = regmap_read(phy->pmu, EXYNOS9825_PMU_USBDRD_PHY_CTRL, &usbdr);
	if (ret)
		goto debug_read_failed;
	dev_dbg(phy->dev, "d2s-usbphy: pmu isol=%d usb20=%08x usbdr=%08x\n",
		isolate, usb20, usbdr);

	return 0;

debug_read_failed:
	dev_dbg(phy->dev,
		"PHY isolation applied but status read failed: %d (old %08x/%08x)\n",
		ret, old_usb20, old_usbdr);
	return 0;
}

static void exynos9825_usbdrd_phy_sw_rst(struct exynos9825_usbdrd_phy *phy,
					 bool assert)
{
	u32 clkrst;

	clkrst = readl(phy->regs + EXYNOS9825_USBCON_CLKRST);
	if (assert) {
		clkrst |= CLKRST_PHY20_SW_RST | CLKRST_PHY20_RST_SEL;
		clkrst |= CLKRST_PHY30_SW_RST | CLKRST_PHY30_RST_SEL;
	} else {
		clkrst |= CLKRST_PHY20_RST_SEL;
		clkrst &= ~CLKRST_PHY20_SW_RST;
		clkrst &= ~CLKRST_PHY30_SW_RST;
		clkrst &= ~CLKRST_PORT_RST;
	}
	writel(clkrst, phy->regs + EXYNOS9825_USBCON_CLKRST);
}

/* HS ReWA configuration, mirrors vendor phy_exynos_usb3p1_rewa_ready() */
static void exynos9825_usbdrd_phy_rewa_ready(struct exynos9825_usbdrd_phy *phy)
{
	void __iomem *regs = phy->regs;
	u32 reg;

	/* Disable ReWA */
	reg = readl(regs + EXYNOS9825_USBCON_REWA_ENABLE);
	reg &= ~REWA_ENABLE_HS_REWA_EN;
	writel(reg, regs + EXYNOS9825_USBCON_REWA_ENABLE);

	/* Line-state check circuit (0 = FSVPLUS/FSMINUS), bypass drive-K */
	reg = readl(regs + EXYNOS9825_USBCON_HSREWA_CTRL);
	reg &= ~HSREWA_CTRL_DPDM_MON_SEL;
	reg |= HSREWA_CTRL_DIG_BYPASS_CON_EN;
	writel(reg, regs + EXYNOS9825_USBCON_HSREWA_CTRL);

	/* Drive-K time and timeout counter */
	writel(0x1, regs + EXYNOS9825_USBCON_HSREWA_HSTK);
	writel(0xff00, regs + EXYNOS9825_USBCON_HSREWA_REFTO);

	/* Disable all abnormal-event sources */
	reg = readl(regs + EXYNOS9825_USBCON_HSREWA_INT1_MASK);
	reg |= HSREWA_INT1_EVT_MASK;
	writel(reg, regs + EXYNOS9825_USBCON_HSREWA_INT1_MASK);

	dev_dbg(phy->dev, "d2s-usbphy: rewa %08x %08x %08x %08x %08x\n",
		readl(regs + EXYNOS9825_USBCON_REWA_ENABLE),
		readl(regs + EXYNOS9825_USBCON_HSREWA_INTR),
		readl(regs + EXYNOS9825_USBCON_HSREWA_CTRL),
		readl(regs + EXYNOS9825_USBCON_HSREWA_REFTO),
		readl(regs + EXYNOS9825_USBCON_HSREWA_HSTK));
}

static int exynos9825_usbdrd_phy_init(struct phy *phy)
{
	struct exynos9825_usbdrd_phy *phy_drd = phy_get_drvdata(phy);
	void __iomem *regs = phy_drd->regs;
	u32 reg, reg_hsp;
	u32 hsp_test;
	int ret;

	ret = clk_bulk_prepare_enable(phy_drd->num_clks, phy_drd->clks);
	if (ret)
		return dev_err_probe(phy_drd->dev, ret,
				     "failed to enable PHY clocks\n");
	phy_drd->clocks_enabled = true;

	/* De-isolate the UTMI and pipe3 PHYs via PMU before touching them */
	ret = exynos9825_usbdrd_phy_pmu_isolate(phy_drd, false);
	if (ret) {
		dev_err_probe(phy_drd->dev, ret, "failed to de-isolate PHY\n");
		clk_bulk_disable_unprepare(phy_drd->num_clks, phy_drd->clks);
		phy_drd->clocks_enabled = false;
		return ret;
	}

	/*
	 * Bring the USB2 (UTMI+) transceiver out of reset and force the
	 * VBUS/B-valid signals, as the dwc3 peripheral mode requires them.
	 * Sequence mirrors the vendor USB3P1 PHY enable path for
	 * USBCON version 3.0.1 (Exynos9825).
	 */

	/* Force q-channel: disable all q-act sources from the USB link */
	reg = readl(regs + EXYNOS9825_USBCON_LINK_CTRL);
	reg |= LINKCTRL_DIS_QACT_ID0 | LINKCTRL_DIS_QACT_VBUS_VALID;
	reg |= LINKCTRL_DIS_QACT_BVALID | LINKCTRL_DIS_QACT_LINKGATE;
	reg &= ~LINKCTRL_FORCE_QACT;
	usleep_range(500, 600);
	writel(reg, regs + EXYNOS9825_USBCON_LINK_CTRL);
	usleep_range(500, 600);
	reg = readl(regs + EXYNOS9825_USBCON_LINK_CTRL);
	reg |= LINKCTRL_FORCE_QACT;
	usleep_range(500, 600);
	writel(reg, regs + EXYNOS9825_USBCON_LINK_CTRL);

	/* PHY POR high */
	exynos9825_usbdrd_phy_sw_rst(phy_drd, true);

	/* Clear UTMI suspend/sleep and D+/D- pulldowns */
	reg = readl(regs + EXYNOS9825_USBCON_UTMI);
	reg &= ~UTMI_FORCE_SUSPEND;
	reg &= ~UTMI_FORCE_SLEEP;
	reg &= ~UTMI_DP_PULLDOWN;
	reg &= ~UTMI_DM_PULLDOWN;
	writel(reg, regs + EXYNOS9825_USBCON_UTMI);

	/* Enable UTMI suspend + common-on in the HS PHY control */
	reg = readl(regs + EXYNOS9825_USBCON_HSP);
	reg |= HSP_EN_UTMISUSPEND;
	reg |= HSP_COMMONONN;
	writel(reg, regs + EXYNOS9825_USBCON_HSP);

	usleep_range(100, 150);

	/* Bypass the VBUS filter */
	reg = readl(regs + EXYNOS9825_USBCON_LINK_CTRL);
	reg |= LINKCTRL_BUS_FILTER_BYPASS(0xf);
	writel(reg, regs + EXYNOS9825_USBCON_LINK_CTRL);

	/* Force VBUS-valid and B-valid */
	reg = readl(regs + EXYNOS9825_USBCON_UTMI);
	reg_hsp = readl(regs + EXYNOS9825_USBCON_HSP);
	reg |= UTMI_FORCE_BVALID | UTMI_FORCE_VBUSVALID;
	reg_hsp |= HSP_VBUSVLDEXTSEL | HSP_VBUSVLDEXT;
	writel(reg, regs + EXYNOS9825_USBCON_UTMI);
	writel(reg_hsp, regs + EXYNOS9825_USBCON_HSP);

	/* Board HS PHY tune (d2s usb_hs_tune, device values) */
	writel(EXYNOS9825_HSP_TUNE_D2S, regs + EXYNOS9825_USBCON_HSP_TUNE);

	/* Take the HS PHY out of power-down (SIDDQ off) */
	reg = readl(regs + EXYNOS9825_USBCON_HSP_TEST);
	reg &= ~HSP_TEST_SIDDQ;
	writel(reg, regs + EXYNOS9825_USBCON_HSP_TEST);

	usleep_range(10, 20);

	/* PHY POR low */
	exynos9825_usbdrd_phy_sw_rst(phy_drd, false);

	usleep_range(75, 100);

	/* OVC IO usage: route the OCD signals from the link */
	reg = readl(regs + EXYNOS9825_USBCON_LINK_PORT);
	reg |= LINKPORT_HUB_PORT_SEL_OCD_U3;
	reg |= LINKPORT_HUB_PORT_SEL_OCD_U2;
	writel(reg, regs + EXYNOS9825_USBCON_LINK_PORT);

	/* Force the pipe3 signals for the link and keep the combo PMA
	 * (USB3 SuperSpeed block) in low-power mode.
	 */
	reg = readl(regs + EXYNOS9825_USBCON_LINK_CTRL);
	reg |= LINKCTRL_PIPE3_FORCE_EN;
	reg &= ~LINKCTRL_PIPE3_FORCE_PHY_STATUS;
	reg |= LINKCTRL_PIPE3_FORCE_RX_ELEC_IDLE;
	writel(reg, regs + EXYNOS9825_USBCON_LINK_CTRL);

	reg = readl(regs + EXYNOS9825_USBCON_COMBO_PMA_CTRL);
	reg |= PMA_LOW_PWRN;
	writel(reg, regs + EXYNOS9825_USBCON_COMBO_PMA_CTRL);

	/* HS ReWA configuration (vendor does this at the end of enable) */
	exynos9825_usbdrd_phy_rewa_ready(phy_drd);

	hsp_test = readl(regs + EXYNOS9825_USBCON_HSP_TEST);

	dev_dbg(phy_drd->dev,
		"d2s-usbphy: ver=%08x clkrst=%08x utmi=%08x hsp=%08x hsp_tune=%08x hsp_test=%08x\n",
		readl(regs + 0x00),
		readl(regs + EXYNOS9825_USBCON_CLKRST),
		readl(regs + EXYNOS9825_USBCON_UTMI),
		readl(regs + EXYNOS9825_USBCON_HSP),
		readl(regs + EXYNOS9825_USBCON_HSP_TUNE),
		hsp_test);
	dev_dbg(phy_drd->dev, "d2s-usbphy: ls=%u rxd=%u sq=%u\n",
		(hsp_test & HSP_TEST_LINESTATE_MASK) >> HSP_TEST_LINESTATE_SHIFT,
		(hsp_test & HSP_TEST_HS_RXDAT) ? 1 : 0,
		(hsp_test & HSP_TEST_HS_SQUELCH) ? 1 : 0);

	/* Re-check the wire state after the host connects (10s, 60s) */
	phy_drd->wire_checks = 0;
	mod_delayed_work(system_wq, &phy_drd->wire_work, 10 * HZ);

	return 0;
}

static int exynos9825_usbdrd_phy_exit(struct phy *phy)
{
	struct exynos9825_usbdrd_phy *phy_drd = phy_get_drvdata(phy);

	cancel_delayed_work_sync(&phy_drd->wire_work);
	if (phy_drd->clocks_enabled) {
		clk_bulk_disable_unprepare(phy_drd->num_clks, phy_drd->clks);
		phy_drd->clocks_enabled = false;
	}

	return 0;
}

static int exynos9825_usbdrd_phy_power_on(struct phy *phy)
{
	struct exynos9825_usbdrd_phy *phy_drd = phy_get_drvdata(phy);

	return exynos9825_usbdrd_phy_pmu_isolate(phy_drd, false);
}

static int exynos9825_usbdrd_phy_power_off(struct phy *phy)
{
	struct exynos9825_usbdrd_phy *phy_drd = phy_get_drvdata(phy);
	u32 reg;

	reg = readl(phy_drd->regs + EXYNOS9825_USBCON_HSP_TEST);
	reg |= HSP_TEST_SIDDQ;
	writel(reg, phy_drd->regs + EXYNOS9825_USBCON_HSP_TEST);

	exynos9825_usbdrd_phy_sw_rst(phy_drd, true);

	return exynos9825_usbdrd_phy_pmu_isolate(phy_drd, true);
}

static const struct phy_ops exynos9825_usbdrd_phy_ops = {
	.init		= exynos9825_usbdrd_phy_init,
	.exit		= exynos9825_usbdrd_phy_exit,
	.power_on	= exynos9825_usbdrd_phy_power_on,
	.power_off	= exynos9825_usbdrd_phy_power_off,
	.owner		= THIS_MODULE,
};

static void exynos9825_usbdrd_dump_qch(struct exynos9825_usbdrd_phy *phy)
{
	/*
	 * Read-only dump of the CMU_FSYS0A QCH gates and GOUT mux/gate
	 * registers that the USB31DRD block depends on. The vendor power
	 * domain code enables the QCH gates during pd_on; mainline has no
	 * PM domains for FSYS0A so we only inspect them here (do NOT write:
	 * the 9825 register map for these offsets has not been verified).
	 */
	static const u32 qch_regs[] = {
		QCH_CON_USB31DRD_QCH_SLV_LINK,
		QCH_CON_USB31DRD_QCH_SLV_CTRL,
		QCH_CON_USB31DRD_QCH_PCS,
		QCH_CON_USB31DRD_QCH_APB,
		QCH_CON_LHS_AXI_D_USB_QCH,
		QCH_CON_LHM_AXI_P_USB_QCH,
		DMYQCH_CON_USB31DRD_QCH_REF,
	};
	u32 qch[ARRAY_SIZE(qch_regs)];
	int i;

	for (i = 0; i < ARRAY_SIZE(qch_regs); i++)
		qch[i] = readl(phy->cmu_regs + qch_regs[i]);

	dev_dbg(phy->dev, "d2s-usbphy: QCH %08x %08x %08x %08x %08x %08x %08x\n",
		qch[0], qch[1], qch[2], qch[3], qch[4], qch[5], qch[6]);
	dev_dbg(phy->dev,
		"d2s-usbphy: cmu mux120=%08x gates %08x %08x %08x %08x %08x %08x %08x %08x\n",
		readl(phy->cmu_regs + 0x120),
		readl(phy->cmu_regs + 0x2014),
		readl(phy->cmu_regs + 0x2018),
		readl(phy->cmu_regs + 0x201c),
		readl(phy->cmu_regs + 0x2020),
		readl(phy->cmu_regs + 0x2024),
		readl(phy->cmu_regs + 0x2028),
		readl(phy->cmu_regs + 0x202c),
		readl(phy->cmu_regs + 0x2030));
}

static int exynos9825_usbdrd_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos9825_usbdrd_phy *phy_drd;
	struct phy_provider *phy_provider;
	struct phy *phy;
	int ret;

	phy_drd = devm_kzalloc(dev, sizeof(*phy_drd), GFP_KERNEL);
	if (!phy_drd)
		return -ENOMEM;

	phy_drd->dev = dev;
	phy_drd->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(phy_drd->regs))
		return PTR_ERR(phy_drd->regs);

	phy_drd->cmu_regs = devm_ioremap(dev, EXYNOS9825_CMU_FSYS0A, 0x8000);
	if (!phy_drd->cmu_regs)
		return -ENOMEM;

	phy_drd->pmu =
		syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pmu-syscon");
	if (IS_ERR(phy_drd->pmu))
		return dev_err_probe(dev, PTR_ERR(phy_drd->pmu),
				     "failed to get PMU syscon\n");
	dev_dbg(dev, "d2s-usbphy: pmu regmap ok\n");

	exynos9825_usbdrd_dump_qch(phy_drd);

	ret = devm_clk_bulk_get_all(dev, &phy_drd->clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get clocks\n");
	phy_drd->num_clks = ret;
	dev_dbg(dev, "d2s-usbphy: got %d clocks\n", phy_drd->num_clks);

	phy = devm_phy_create(dev, NULL, &exynos9825_usbdrd_phy_ops);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy),
				     "failed to create USB DRD PHY\n");
	dev_dbg(dev, "d2s-usbphy: created phy\n");

	phy_set_drvdata(phy, phy_drd);
	platform_set_drvdata(pdev, phy_drd);

	INIT_DELAYED_WORK(&phy_drd->wire_work,
			  exynos9825_usbdrd_phy_wire_check);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return dev_err_probe(dev, PTR_ERR(phy_provider),
				     "failed to register PHY provider\n");
	dev_dbg(dev, "d2s-usbphy: provider registered\n");

	dev_info(dev, "Exynos9825 USB DRD PHY ready\n");

	return 0;
}

static void exynos9825_usbdrd_phy_remove(struct platform_device *pdev)
{
	struct exynos9825_usbdrd_phy *phy_drd =
		platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&phy_drd->wire_work);
	if (phy_drd->clocks_enabled) {
		clk_bulk_disable_unprepare(phy_drd->num_clks, phy_drd->clks);
		phy_drd->clocks_enabled = false;
	}
}

static const struct of_device_id exynos9825_usbdrd_phy_of_match[] = {
	{ .compatible = "samsung,exynos9825-usbdrd-phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, exynos9825_usbdrd_phy_of_match);

static struct platform_driver exynos9825_usbdrd_phy_driver = {
	.probe	= exynos9825_usbdrd_phy_probe,
	.remove	= exynos9825_usbdrd_phy_remove,
	.driver	= {
		.name	= "exynos9825-usbdrd-phy",
		.of_match_table	= exynos9825_usbdrd_phy_of_match,
	},
};
module_platform_driver(exynos9825_usbdrd_phy_driver);

MODULE_AUTHOR("d2s mainline bring-up");
MODULE_DESCRIPTION("Samsung Exynos9825 USB DRD PHY driver");
MODULE_LICENSE("GPL");
