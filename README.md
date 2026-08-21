# exynos-9825-mainline-linux

This project is a port of the exynos 9825 mainline kernel by [Eithan Asulin](https://github.com/EithanAsulin). Please keep in mind that this is still highly experimental and is mostly developed by one person!

## Info

- **Kernel:** Linux v7.2.0-rc7
- **Processor:** Exynos 9825
- **Device identifier:** d2s/d2x/d1s/d1x
- **Telnet/SSH:** Works
- **Display:** framebuffer works somewhat, no touchscreen yet
- **UFS:** link and device detection work, but block reads are not working
- **Power management:** still experimental, some UFS power-saving paths are disabled for stability

## This project

This project aims to bring modern Linux kernels to the Exynos 9825. That means
making systems such as [postmarketOS](https://postmarketos.org/),
[Mobian](https://mobian-project.org/), [Droidian](https://droidian.org/), and
[Ubuntu Touch](https://www.ubuntu-touch.io/) possible on these devices.

Right now the kernel can boot far enough to reach a postmarketOS shell over USB
networking. A lot is still missing: touchscreen support, a proper display
stack, reliable UFS storage I/O, and several drivers.

## Building

You need an AArch64 cross compiler:

```sh
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- exynos9825-d2s_defconfig
make -j"$(nproc)" ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image dtbs
```

The d2s device tree is at `arch/arm64/boot/dts/exynos/exynos9825-d2s.dtb`.

## Help and testing
> [!NOTE]
> Do you want to help this project?
> If you have an Exynos 9825 device and want to help, open an issue or pull request!

## Credits

- **[Exynos 9820 Mainline](https://github.com/chiffathefox/exynos-9820-mainline-linux)** - this project started as a fork
- **Samsung D2S OSS** - useful bits of vendor source
- **[Android Kernel Samsung D2S by LineageOS](https://github.com/LineageOS/android_kernel_samsung_d2)** - very helpful reference
- **[Linux by Linus Torvalds](https://github.com/torvalds/linux)** - the source code
- **[UniLoader](https://github.com/ivoszbg/uniLoader)** - makes booting a mainline kernel on unsupported hardware easier

> [!WARNING]
> Research in this project has been assisted by AI tools. Changes still need
> testing and review.
