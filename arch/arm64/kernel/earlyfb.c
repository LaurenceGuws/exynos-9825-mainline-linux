// SPDX-License-Identifier: GPL-2.0-only
/*
 * Early framebuffer boot console for the Samsung Galaxy Note 10+ (d2s)
 * mainline bring-up.
 *
 * The bootloader (uniLoader) leaves the panel showing the simple-framebuffer
 * at EARLYFB_PHYS. Render the kernel log there so boot progress is visible
 * on the device screen before the real console drivers come up.
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/font.h>
#include <linux/timer.h>

#include <asm/mmu.h>
#include <asm/setup.h>

#define EARLYFB_PHYS	0xca000000UL
#define EARLYFB_WIDTH	1440
#define EARLYFB_HEIGHT	3040
#define EARLYFB_STRIDE	(EARLYFB_WIDTH * 4)
#define EARLYFB_SIZE	(EARLYFB_STRIDE * EARLYFB_HEIGHT)

#define EARLYFB_COLS	(EARLYFB_WIDTH / 8)
#define EARLYFB_ROWS	(EARLYFB_HEIGHT / 8)

#define EARLYFB_WHITE	0xffffffffU
#define EARLYFB_BLACK	0xff000000U

static int d2s_wdt_disable;
static void __iomem *d2s_wdt_base;
static struct timer_list d2s_wdt_timer;

static int __init d2s_wdt_param(char *str)
{
	if (str && !strcmp(str, "0"))
		d2s_wdt_disable = 1;
	return 0;
}
early_param("d2s_wdt", d2s_wdt_param);

static void __iomem *earlyfb_map;
static unsigned char earlyfb_shadow[EARLYFB_COLS * EARLYFB_ROWS];
static int earlyfb_x;
static int earlyfb_y;

static const unsigned char *earlyfb_font(void)
{
	return font_data_buf(font_vga_8x8.data);
}

static void earlyfb_draw_glyph(int col, int row, unsigned char c)
{
	const unsigned char *font = earlyfb_font();
	void __iomem *dst = earlyfb_map + row * 8 * EARLYFB_STRIDE + col * 8 * 4;
	u32 pixels[8];
	int x, y;

	if (c < 0x20 || c > 0x7e)
		c = ' ';

	for (y = 0; y < 8; y++) {
		unsigned char bits = font[c * 8 + y];

		for (x = 0; x < 8; x++)
			pixels[x] = bits & BIT(7 - x) ?
				EARLYFB_WHITE : EARLYFB_BLACK;
		memcpy_toio(dst + y * EARLYFB_STRIDE, pixels, sizeof(pixels));
	}
}

static void earlyfb_scroll(void)
{
	int col, row;

	memmove(earlyfb_shadow, earlyfb_shadow + EARLYFB_COLS,
		(EARLYFB_ROWS - 1) * EARLYFB_COLS);
	memset(earlyfb_shadow + (EARLYFB_ROWS - 1) * EARLYFB_COLS, ' ',
	       EARLYFB_COLS);

	for (row = 0; row < EARLYFB_ROWS; row++)
		for (col = 0; col < EARLYFB_COLS; col++)
			earlyfb_draw_glyph(col, row,
					   earlyfb_shadow[row * EARLYFB_COLS + col]);
}

static void earlyfb_putc(char c)
{
	if (c == '\r') {
		earlyfb_x = 0;
		return;
	}
	if (c == '\n') {
		earlyfb_x = 0;
		earlyfb_y++;
		goto done;
	}
	if (c < 0x20 || c > 0x7e)
		return;

	if (earlyfb_x >= EARLYFB_COLS) {
		earlyfb_x = 0;
		earlyfb_y++;
	}
	if (earlyfb_y >= EARLYFB_ROWS) {
		earlyfb_scroll();
		earlyfb_y = EARLYFB_ROWS - 1;
	}

	earlyfb_shadow[earlyfb_y * EARLYFB_COLS + earlyfb_x] = c;
	earlyfb_draw_glyph(earlyfb_x, earlyfb_y, c);
	earlyfb_x++;

done:
	if (earlyfb_y >= EARLYFB_ROWS) {
		earlyfb_scroll();
		earlyfb_y = EARLYFB_ROWS - 1;
	}
}

static void earlyfb_write(struct console *co, const char *s, unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++)
		earlyfb_putc(s[i]);
}

static struct console earlyfb_console = {
	.name = "earlyfb",
	.write = earlyfb_write,
	.flags = CON_PRINTBUFFER | CON_ENABLED | CON_BOOT,
	.index = -1,
};

static void d2s_wdt_setup(const char *tag, bool arm)
{
	void __iomem *wdt, *pmu;
	int i;

	/* Samsung Exynos 9825 watchdog timers: WTCON at +0x0 (bit 5 = WDTEN),
	 * WTDAT at +0x4, WTCNT at +0x8. The cluster0 timer counts down at the
	 * 26 MHz oscclk / (prescaler+1) / divisor. With prescaler 255 and
	 * divisor 128 that is ~793 Hz, so a 16-bit count of 47610 gives ~60 s.
	 * Arming it at early boot (mirroring s3c2410_wdt.c on exynos850,
	 * including the PMU reset-unmask/counter-enable bits) turns the boot
	 * watchdog into a safety net: any kernel hardlock resets the SoC back
	 * through the bootloader, while the initramfs services it during
	 * normal operation.
	 */
	for (i = 0; i < 2; i++) {
		phys_addr_t base = i ? 0x10060000UL : 0x10050000UL;

		wdt = ioremap(base, 0x100);
		if (!wdt)
			continue;
		if (arm && i == 0) {
			pmu = ioremap(0x15860000UL, 0x10000);
			if (pmu) {
				/* NONCPU_INT_EN bit2: unmask WDTRESET */
				writel(readl(pmu + 0x1244) | BIT(2),
				       pmu + 0x1244);
				/* NONCPU_OUT bit7: enable WDT counter */
				writel(readl(pmu + 0x1220) | BIT(7),
				       pmu + 0x1220);
				iounmap(pmu);
			}
			writel(47610, wdt + 0x4);
			writel(47610, wdt + 0x8);
			/* RSTEN | DIV128 | ENABLE | PRESCALE(255) */
			writel(0xFF39, wdt + 0x0);
			d2s_wdt_base = wdt;
			pr_info("d2s-wdt: %s cl%d WTCON=%08x WTDAT=%08x WTCNT=%08x (armed)\n",
				tag, i,
				readl(wdt + 0x0), readl(wdt + 0x4), readl(wdt + 0x8));
			/* keep the mapping; the feed timer uses it */
			continue;
		} else if (!arm && i == 1) {
			/* keep the multistage cluster2 watchdog disabled */
			writel(0, wdt);
		}
		pr_info("d2s-wdt: %s cl%d WTCON=%08x WTDAT=%08x WTCNT=%08x\n",
			tag, i,
			readl(wdt + 0x0), readl(wdt + 0x4), readl(wdt + 0x8));
		iounmap(wdt);
	}
}

static void d2s_wdt_feed(struct timer_list *unused)
{
	if (d2s_wdt_base)
		writel(47610, d2s_wdt_base + 0x8);
	mod_timer(&d2s_wdt_timer, jiffies + msecs_to_jiffies(20000));
}

static int __init d2s_wdt_late_probe(void)
{
	d2s_wdt_setup("late", false);

	/*
	 * Start the kernel keepalive for the early-armed cluster0 watchdog.
	 * (The s3c2410_wdt driver would take over when its DT node is
	 * enabled; with the inherited DT it never probes, so feed here.)
	 */
	if (d2s_wdt_base && !d2s_wdt_disable) {
		timer_setup(&d2s_wdt_timer, d2s_wdt_feed, 0);
		mod_timer(&d2s_wdt_timer, jiffies + msecs_to_jiffies(20000));
		pr_info("d2s-wdt: kernel feed timer started (20s interval)\n");
	}
	return 0;
}
late_initcall(d2s_wdt_late_probe);

void __init earlyfb_console_init(void)
{
	void *bridge;

	if (d2s_wdt_disable)
		pr_info("d2s-wdt: disabled via d2s_wdt=0\n");
	else
		d2s_wdt_setup("early", true);

	bridge = READ_ONCE(note10_paging_bridge);

	/*
	 * Note10 bring-up diagnostic: the selected early watchdog setup branch
	 * returned. Repaint the already-proven high-TTBR1 bridge amber and hold
	 * before inspecting or creating earlyfb_map.
	 */
	{
		register unsigned long bridge_reg asm("x9") = (unsigned long)bridge;

		asm volatile("mov x10, %0\n\t"
			"movz x11, #0xa000\n\t"
			"movk x11, #0xffff, lsl #16\n\t"
			"movk x11, #0xa000, lsl #32\n\t"
			"movk x11, #0xffff, lsl #48\n\t"
			"movz x12, #0x0002, lsl #16\n\t"
			"movk x12, #0xd000\n\t"
			"add x12, x10, x12\n\t"
			"1:\n\t"
			"str x11, [x10], #8\n\t"
			"cmp x10, x12\n\t"
			"b.lo 1b\n\t"
			"dsb sy\n\t"
			:
			: "r" (bridge_reg)
			: "x0", "x1", "x8", "x10", "x11", "x12", "x13",
			  "x14", "cc", "memory");
	}

	if (earlyfb_map)
		return;

	if (!slab_is_available())
		return;

	earlyfb_map = ioremap_wc(EARLYFB_PHYS, EARLYFB_SIZE);
	if (!earlyfb_map)
		return;

	memset_io(earlyfb_map, 0, EARLYFB_SIZE);
	register_console(&earlyfb_console);
	pr_info("earlyfb: boot console on simple-framebuffer at 0x%lx\n",
		(unsigned long)EARLYFB_PHYS);
}
