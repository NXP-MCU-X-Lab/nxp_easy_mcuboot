# NXP EASY MCU bootlaoder 

这是NXP官方 MCUBOOT 的 简化版C实现，砍掉了大部分功能，只保留串口下载功能。可以配合 官方的命令行工具blhost 或者 GUI工具： Kinetis Flash Tool 来实现一个基于MCU串口的bootloader.  非常容易移植到任何MCU上



## 简介

* 基础实现和具体硬件完全解耦。 只有2个.c .h文件:kptl.c  mcuboot.c。 除Systick中断外 不使用任何中断

* 所有与硬件相关的必要操作全部由回调函数显示。 kptl 和  mcuboot 本身没有任何硬件依赖。非常容易移植到任何ARM Cortex MCU上



## 文件结构

```
nxp_easy_mcuboot
├─ Libraries - 存放所有库文件，bootloader源文件
│    ├─ drivers_k64 - NXP KinetisK64 驱动文件
│    ├─ drivers_lpc800  - NXP LPC800驱动文件
│    ├─ startup - 所有MCU启动文件
│    └─ utilities - 存放bootloader源代码
├─ Project - example 工程
│    ├─ frdm_k64 基于FRDM-K64的 bootloader示例工程
│    └─ lpc802 基于LPC802的 bootloader示例工程
├─ pc_tool - PC工具
│    ├─ KinetisFlashTool.exe  GUI工具，直接使用这个下载程序
│    ├─ KinetisFlashTool.ini
│    ├─ blfwkdll.dll
│    └─ bootloader.log
└─ readme.md
```



## 示例

project 文件夹：

```
Project
├─ frdm_k64
│    ├─ frdm_k64_bl  - bootloader工程，Keil编译
│    └─ frdm_k64_example_app  - app示例工程
└─ lpc802
       ├─ lpc802_bl - bootloader工程，Keil编译
       └─ lpc802_example_app - app示例工程
```



下面以FRDM-K64为例子说明：

1. 编译 frdm_k64_bl 工程，并使用JLINK下载到 FRDM-K64板子上。
2. 打开KinetisFlashTool.exe 。 再MCU复位后300ms内点击连接，连接成功后，上位机会显示MCU信息。
3. 编译frdm_k64_example_app 得到 frdm_k64_example_app.hex文件，选择hex文件，点击"UPDATE"即开始升级



## 移植





