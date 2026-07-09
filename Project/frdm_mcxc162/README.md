# FRDM-MCXC162 备份升级 Bootloader 说明

## 项目概述

本目录提供面向 FRDM-MCXC162 的 UART bootloader 方案。bootloader 保持 NXP KBOOT/MCUBOOT 串口协议，PC 端可使用 `blhost` 下载应用固件。MCU 端在写 Flash 时将主应用区地址映射到备份区，启动时再根据镜像完整性和版本号决定是否升级到主应用区。

该方案的目标是提高现场升级可靠性：固件下载阶段不会直接覆盖当前正在运行的 A 面应用。新的 image 先写入 B 面备份区，只有在 B 面 CRC 校验通过且版本更高，或 A 面无有效 image 时，bootloader 才会把 B 面复制到 A 面并启动。

## 目录结构

| 路径 | 说明 |
| --- | --- |
| `mcxc162_bl` | FRDM-MCXC162 bootloader 工程，使用 MCUXpresso IDE 构建并烧录到 `0x00000000`。 |
| `mcxc162_example_app` | 示例应用工程，链接到 A 面地址 `0x00004000`，用于验证升级和跳转流程。 |
| `tools/mcxc162_patch_image.py` | 应用镜像后处理脚本，用于向 raw Intel HEX 写入 image header、size、version 和 CRC。 |

## Flash 分区

FRDM-MCXC162 当前按 64 KB P-Flash 进行三分区：

| 区域 | 地址范围 | 大小 | 用途 |
| --- | --- | --- | --- |
| Bootloader | `0x00000000-0x00003FFF` | 16 KB | bootloader 固件和 KBOOT 串口协议处理。 |
| App A | `0x00004000-0x00009FFF` | 24 KB | 实际运行的应用程序区域。 |
| Backup B | `0x0000A000-0x0000FFFF` | 24 KB | 升级下载暂存区。 |

PC 端升级时仍按 A 面地址下载，即擦写和写入地址使用 `0x00004000`。bootloader 内部会把 A 面地址映射到 B 面，所以下载阶段实际写入 `0x0000A000` 起始的备份区。

## Image 格式

应用程序必须链接到 A 面地址 `0x00004000`。image header 固定放在 `slot_base + 0x200`，也就是 `0x00004200`。示例 app 的 linker script 已经保留该区域。

当前 image ABI 如下：

| 字段 | 值或说明 |
| --- | --- |
| Magic | `0x4D435831` |
| Header offset | `0x200` |
| Header address | `0x00004200` |
| Header version | `1` |
| Header size | `64` bytes |
| Image version | 由 `source/app_version.h` 中的 `APP_IMAGE_VERSION` 写入 |
| Image size | 由 `mcxc162_patch_image.py` 根据 HEX 内容计算，按 16 字节对齐 |
| CRC32 | 软件 CRC32，计算时 CRC 字段自身按 0 参与计算 |

bootloader 校验 image 时会检查 vector table、header、image size 和 CRC。只有校验通过的 image 才会被认为有效。

## 启动行为

上电或复位后，bootloader 按以下顺序运行：

1. 初始化时钟、UART 和 Flash 驱动。
2. 校验 A 面 image 和 B 面 image。
3. 如果 B 面有效，并且 A 面无效或 B 面版本号更高，则擦除 A 面并把 B 面复制到 A 面。
4. 复制完成后重新校验 A 面。
5. 如果 A 面有效，bootloader 等待约 300 ms 的 KBOOT 连接窗口；窗口内没有收到主机连接数据，则跳转到 A 面应用。
6. 如果 A 面无效，则停留在 bootloader，等待 `blhost` 下载新的固件。

版本号只用于判断 B 面是否应该覆盖 A 面。CRC 和 vector/header 校验用于判断 image 是否完整、可启动。

## 启动日志

当前 bootloader 默认打开启动日志，开关为 `BL_ENABLE_BOOT_LOG=1`。UART 参数为 `115200 8N1`。

典型日志格式如下：

```text
MCXC162 BL
layout boot base=0x00000000 size=0x00004000
layout app base=0x00004000 size=0x00006000
layout backup base=0x0000A000 size=0x00006000
layout header offset=0x00000200 addr=0x00004200
A valid ver=0x00010002 size=0x00001C50 crc=0xE18EA444
B invalid
Keep A
Jump A
```

日志含义：

| 日志 | 含义 |
| --- | --- |
| `Flash init failed` | Flash 驱动初始化失败。成功时不打印该行。 |
| `layout boot ...` | bootloader 分区起始地址和大小。 |
| `layout app ...` | A 面应用分区起始地址和大小。 |
| `layout backup ...` | B 面备份分区起始地址和大小。 |
| `layout header ...` | image header 在 app slot 内的偏移和 A 面绝对地址。 |
| `A valid ...` | A 面 image 校验通过，并打印 version、size、crc。 |
| `B valid ...` | B 面 image 校验通过，并打印 version、size、crc。 |
| `A invalid` / `B invalid` | 对应 image 无效，可能是空白、header 错误、size 错误、CRC 错误或 vector table 不合法。 |
| `Promote B to A` | B 面满足升级条件，开始复制到 A 面。 |
| `Promote ok` | B 面复制到 A 面并重新校验成功。 |
| `Keep A` | 不需要升级，继续保留当前 A 面。 |
| `Jump A` | 跳转到 A 面应用。 |

当 image 无效时，bootloader 不打印该 image 的 size/crc，因为这些字段不能作为可信信息。

## 构建应用固件

应用工程必须满足以下要求：

1. 链接地址为 `0x00004000`，slot size 为 `0x00006000`。
2. `0x00004200-0x0000423F` 必须保留给 image header。
3. 构建后先生成 raw Intel HEX，再用 `mcxc162_patch_image.py` 生成升级文件。
4. 交付给 `blhost` 的文件必须是版本化的 `*_vXXXXXXXX_upgrade.hex`，不是 raw HEX，也不是 AXF。

`mcxc162_example_app` 的 Debug 和 Release 配置已经包含 post-build 步骤：

```sh
arm-none-eabi-size "${BuildArtifactFileName}";
arm-none-eabi-objcopy -O ihex "${BuildArtifactFileName}" "${BuildArtifactFileBaseName}.raw.hex";
py -3 "${ProjDirPath}/../tools/mcxc162_patch_image.py" --input "${BuildArtifactFileBaseName}.raw.hex" --output "${BuildArtifactFileBaseName}_v{version}_upgrade.hex" --slot-base 0x4000 --slot-size 0x6000 --header-offset 0x200 --version-header "${ProjDirPath}/source/app_version.h" --version-macro APP_IMAGE_VERSION
```

生成结果中：

| 文件 | 用途 |
| --- | --- |
| `mcxc162_example_app.axf` | MCUXpresso 调试用文件。 |
| `mcxc162_example_app.raw.hex` | 从 AXF 直接导出的原始 HEX，仅用于后处理输入。 |
| `mcxc162_example_app_v00010002_upgrade.hex` | 已写入 header 和 CRC 的升级文件，供 `blhost` 下载。 |

## 修改 app slot 大小

当前 64 KB P-Flash 分区下，Bootloader 固定占用 16 KB，A/B 两个 app slot 各占用 24 KB。也就是说，在保持 16 KB bootloader 和双 slot 备份升级机制不变的前提下，`0x6000` 已经是单个 app slot 的最大大小：

```text
0x4000 + 2 * 0x6000 = 0x10000
```

如果未来需要修改 app slot 大小，不能只在 app 工程里修改。分区参数必须在 bootloader、app linker、MCUXpresso 工程配置、post-build 和升级命令中保持一致。

建议按以下顺序修改：

1. 先重新规划 Flash 分区，确认满足 `BL_BOOT_SIZE + 2 * BL_SLOT_SIZE <= 0x10000`。
2. 修改 bootloader 工程 `mcxc162_bl/source/bl_config.h` 中的 `BL_APP_BASE`、`BL_SLOT_SIZE`、`BL_BACKUP_BASE`，必要时同步调整 `BL_BOOT_SIZE`。
3. 修改 app 工程 linker script：`mcxc162_example_app/linker/mcxc162_example_app.ld` 中 `PROGRAM_FLASH ORIGIN` 和 `LENGTH`。
4. 在 MCUXpresso IDE GUI 中打开 app 工程属性：右键 `mcxc162_example_app` -> `Properties` -> `C/C++ Build` -> `MCU settings`，把 `PROGRAM_FLASH` 的 `Location` 和 `Size` 改成与 linker script 一致。
5. 在 MCUXpresso IDE GUI 中打开 app post-build：右键 `mcxc162_example_app` -> `Properties` -> `C/C++ Build` -> `Settings` -> 选择对应配置 `Debug` 或 `Release` -> `Build Steps` -> `Post-build steps`，同步修改 `--slot-base` 和 `--slot-size`。
6. 更新 `blhost` 擦除命令中的长度，例如当前为 `flash-erase-region 0x4000 0x6000`。
7. 重新构建 bootloader 和 app，并使用新的 `*_vXXXXXXXX_upgrade.hex` 验证升级流程。

`0x200` header offset 建议保持不变。只有在 bootloader 和 patch 脚本 ABI 同时更新时，才应修改 header offset。

## 修改固件版本号

版本号由 app 工程中的 `source/app_version.h` 指定：

```c
#define APP_IMAGE_VERSION (0x00010002u)
```

修改版本号时只改这个宏。构建时 `mcxc162_patch_image.py` 会从该头文件读取 `APP_IMAGE_VERSION`，并写入 image header。bootloader 升级判断仍只信任 image header，不直接读取 app C 宏。

在 MCUXpresso IDE GUI 中打开版本文件：

1. 展开 `mcxc162_example_app`。
2. 打开 `source/app_version.h`。
3. 修改 `APP_IMAGE_VERSION`。
4. 重新构建 app，并使用新生成的版本化 HEX 升级。

Debug 和 Release 的 post-build command 都从同一个 `source/app_version.h` 读取版本号，因此不需要分别维护两个 `--version` 参数。

post-build 命令中的输出文件名使用 `{version}` 占位符：

```sh
--output "${BuildArtifactFileBaseName}_v{version}_upgrade.hex"
```

`mcxc162_patch_image.py` 会把 `{version}` 自动展开为 8 位大写十六进制版本号。例如 `APP_IMAGE_VERSION` 为 `0x00010002u` 时，会生成 `mcxc162_example_app_v00010002_upgrade.hex`。

建议按 32 位十六进制管理版本号，例如：

| 版本 | 建议编码 |
| --- | --- |
| 1.0.0 | `0x00010000` |
| 1.0.1 | `0x00010001` |
| 1.1.0 | `0x00010100` |
| 2.0.0 | `0x00020000` |

升级判断规则是数值比较。只有 B 面 image 有效，并且版本号大于 A 面版本号时，bootloader 才会把 B 面复制到 A 面。如果 A 面无效，只要 B 面有效即可恢复到 A 面。

修改版本号后需要重新构建 app，并使用重新生成的 `*_vXXXXXXXX_upgrade.hex` 升级。

## 使用 blhost 升级固件

### 准备工作

1. 使用调试器或 MCUXpresso IDE 将 `mcxc162_bl` 烧录到 `0x00000000`。
2. 构建 app 工程，确认生成 `mcxc162_example_app_v00010002_upgrade.hex`。
3. 连接 FRDM-MCXC162 的 UART 到 PC。
4. 确认串口参数为 `115200 8N1`。
5. 将下面命令中的 `COMx` 替换为实际串口号。

如果 A 面已有有效应用，bootloader 默认只等待约 300 ms 的连接窗口。执行 `blhost` 命令前建议先复位板卡，并在复位后立即执行连接或下载命令。

### 查询设备

```sh
blhost -p COMx,115200 -- get-property 1
```

该命令用于确认 `blhost` 能和 bootloader 正常通信。

### 擦除升级暂存区

```sh
blhost -p COMx,115200 -- flash-erase-region 0x4000 0x6000
```

命令地址写的是 A 面地址，但 bootloader 会映射到 B 面执行擦除。不要直接使用 `0xA000`，否则会超出 bootloader 对主机暴露的下载地址范围。

### 写入升级文件

```sh
blhost -p COMx,115200 -- write-memory 0x4000 mcxc162_example_app_v00010002_upgrade.hex
```

写入地址同样使用 A 面起始地址 `0x4000`。bootloader 会把写入数据映射到 B 面。

### 复位并升级

```sh
blhost -p COMx,115200 -- reset
```

复位后 bootloader 会重新校验 A/B 两面。如果 B 面 image 完整且版本更高，bootloader 会复制 B 到 A，然后跳转到 A 面应用。

## 示例应用行为

`mcxc162_example_app` 是最小验证应用，用于确认 bootloader 跳转和升级链路。启动后会通过同一 UART 打印一次：

```text
================================
MCXC162 Example App
slot=0x00004000
header=0x00004200
version=0x00010002
size=0x00001C50
crc=0xE18EA444
================================
```

如果直接调试未经过 post-build patch 的 raw AXF，image header 可能为空，应用会打印 `image=unpatched`。

随后红色 LED 以约 500 ms 周期翻转，同时每 1 秒打印一次 app counter：

```text
MCXC162 app counter=0x00000001
MCXC162 app counter=0x00000002
MCXC162 app counter=0x00000003
```

该周期日志用于让客户明确看到当前已经从 bootloader 跳转到 app。

## 常见问题

### 下载 raw HEX 后无法启动

raw HEX 没有 image header 和 CRC。必须使用 post-build 生成的 `*_vXXXXXXXX_upgrade.hex`。

### B 面下载成功但没有覆盖 A 面

常见原因是 B 面版本号不大于 A 面版本号。升级时应递增 `source/app_version.h` 中的 `APP_IMAGE_VERSION`。

### 启动日志显示 B invalid

表示 B 面 image 校验失败。请确认下载文件是 `*_vXXXXXXXX_upgrade.hex`，下载地址是 `0x4000`，下载过程没有中断。

### App 工程直接链接到 0x00000000

这种 image 不符合 bootloader ABI。app 必须链接到 `0x00004000`，并保留 `0x00004200-0x0000423F` 给 image header。

### 应用超过 24 KB

当前 A/B slot 大小均为 24 KB。超过 `0x6000` 的 app image 会被后处理脚本拒绝，需要重新规划 Flash 分区或减少 app 体积。
