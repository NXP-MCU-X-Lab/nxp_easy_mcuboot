# FRDM-MCXC162 A/B 备份升级 Bootloader

## 功能概述

本工程演示 FRDM-MCXC162 UART bootloader：

- 使用 NXP KBOOT/MCUBOOT 串口协议，兼容 `blhost` 和 KinetisFlashTool。
- 升级前先将当前 A 面完整备份到 B 面，备份校验成功后才擦写 A 面。
- A 面升级中断或镜像无效时，从 B 面恢复。
- 新版本连续发生 3 次 WWDT 复位时，从 B 面回退到旧版本。

这是便于客户理解和移植的 Demo，不是完整的生产级 trial/confirm bootloader。

## 工程目录

| 路径 | 说明 |
| --- | --- |
| `mcxc162_bl` | Bootloader 工程，烧录到 `0x00000000`。 |
| `mcxc162_example_app` | 示例 APP，链接到 A 面 `0x00004000`。 |
| `common/bl_trial.h` | Bootloader 与 APP 共用的 WWDT trial 状态定义。 |
| `tools/mcxc162_patch_image.py` | 生成带 header、version、size 和 CRC 的升级 HEX/BIN。 |

## Flash 分区

| 区域 | 地址范围 | 大小 | 用途 |
| --- | --- | --- | --- |
| Bootloader | `0x00000000-0x00003FFF` | 16 KB | 启动、升级和恢复。 |
| App A | `0x00004000-0x00009FFF` | 24 KB | 当前运行镜像。 |
| Backup B | `0x0000A000-0x0000FFFF` | 24 KB | 升级前的 A 面备份。 |

主机只访问 A 面地址 `0x4000-0x9FFF`。B 面由 bootloader 管理，不对下载工具开放。

## Image 格式与版本规则

APP 必须链接到 `0x00004000`。64-byte image header 固定在 `0x00004200`：

| 项目 | 值 |
| --- | --- |
| Magic | `0x4D435831` |
| Header offset | `0x200` |
| Header version | `1` |
| Image version | `APP_IMAGE_VERSION` |
| Image size | 16-byte 对齐 |
| CRC32 | CRC 字段自身按 0 计算 |

Bootloader 会检查 vector table、header、size 和 CRC。版本策略如下：

| A/B 状态 | 启动行为 |
| --- | --- |
| A、B 都无效 | 留在 bootloader。 |
| 只有 A 有效 | 启动 A。 |
| 只有 B 有效 | B→A 恢复后启动。 |
| A 版本高于 B | 启动 A，并进入 WWDT trial。 |
| A 版本低于 B | B→A 恢复。 |
| 同版本且 size/CRC 相同 | 启动 A。 |
| 同版本但内容不同 | B→A 恢复。 |

升级版本必须严格递增。版本号只在以下文件中修改：

```c
mcxc162_example_app/source/app_version.h

#define APP_IMAGE_VERSION (0x00010007u)
```

## 构建

使用 MCUXpresso IDE 25.6 分别构建：

1. `mcxc162_bl`：Debug 或 Release。
2. `mcxc162_example_app`：Debug 或 Release。

APP 工程已配置 post-build，构建后会生成：

```text
mcxc162_example_app.axf
mcxc162_example_app.raw.hex
mcxc162_example_app_vXXXXXXXX_upgrade.hex
mcxc162_example_app_vXXXXXXXX_upgrade.bin
```

HEX 和 BIN 来自同一个 patched image，header、version、size 和 CRC 完全相同。BIN 的第 0 字节对应 A 面地址 `0x4000`，文件长度等于 image header 中的 `image_size`。

- KinetisFlashTool 使用 `*_upgrade.hex`。
- `blhost write-memory` 使用 `*_upgrade.bin`。
- `blhost flash-image` 使用 `*_upgrade.hex`。

`raw.hex` 没有有效 image header 和 CRC，不能用于升级。构建成功不能只看 IDE 的 `0 errors`；还应看到 `patched ...hex and ...bin` 日志，并确认两个版本化文件都已生成。

提交或交付前检查 map：

- Bootloader `_image_end <= 0x4000`。
- APP `_image_size <= 0x6000`。
- 两个工程的 `g_bl_trial_state` 都位于 `0x20001000`。

可单独验证 patch 脚本：

```sh
python tools/mcxc162_patch_image.py --self-test
```

## UART 升级流程

UART 参数为 `115200 8N1`。A 面有效时，bootloader 等待主机连接约 300 ms，因此建议先准备好命令，再复位板卡并立即执行。

### blhost 推荐流程：BIN + write-memory

```sh
blhost -p COMx,115200 -- get-property 1
blhost -p COMx,115200 -- flash-erase-region 0x4000 0x6000
blhost -p COMx,115200 -- write-memory 0x4000 mcxc162_example_app_vXXXXXXXX_upgrade.bin
blhost -p COMx,115200 -- reset
```

`write-memory` 将输入文件作为原始二进制数据发送，不能直接传入 Intel HEX。否则 HEX 文本会被写入 A 面，导致 vector/header 校验失败。

### blhost 备选流程：HEX + flash-image

```sh
blhost -p COMx,115200 -- flash-image mcxc162_example_app_vXXXXXXXX_upgrade.hex erase
blhost -p COMx,115200 -- reset
```

`flash-image` 由 blhost 在主机端解析 Intel HEX，并通过 erase/write 命令完成升级。KinetisFlashTool 同样使用版本化 HEX。

将 `COMx` 和升级文件名替换为实际值。协议约束：

- 必须先 erase，再 write。
- 第一次合法 erase 会先执行 A→B 备份；备份失败时 A 不会被擦除。
- erase 起始地址必须是 `0x4000`，请求长度允许 `1..0x6000`。
- 无论合法请求长度是多少，bootloader 都会完整擦除 24 KB A 槽。
- write 只允许访问 `0x4000-0x9FFF`。
- 写入和恢复过程中 B 面保持不变。

复位后正常升级应看到 A 为新版本、B 为升级前版本：

```text
A valid ver=0x00010007 ...
B valid ver=0x00010006 ...
Keep A
Jump A
```

## WWDT 自动回退 Demo

只有 A、B 都有效且 `A.version > B.version` 时才监控 WWDT 复位。

示例 APP 配置：

- WWDT0 时钟：1 MHz。
- 超时时间：约 5 秒。
- 正常喂狗周期：1 秒。
- 连续健康运行 30 秒：清零已记录的 WWDT 复位次数。

通过 APP UART 发送一次大写 `F`，APP 会停止喂狗：

```text
WWDT fault enabled
```

故障标志保存在 SRAM 中，可跨 WWDT reset 保留，因此不需要重复发送 `F`。三次复位日志依次为：

```text
Trial WWDT reset 1/3
Trial WWDT reset 2/3
Trial rollback 3/3
Restore B to A
Restore ok
```

回退完成后 A 与 B 都是旧版本，bootloader 启动恢复后的 A。

## Trial 状态与限制

16-byte trial 状态固定在 SRAM `0x20001000`。`0x20000000` 会被启动代码和 RAM function 使用，不能保存跨复位状态。

该 Demo 有以下明确限制：

- 不使用 Data Flash，掉电后不保证计数保留。
- 非 WWDT 复位会清零计数和 Demo 故障标志。
- B 无效时无法回退，也不会打印 trial 回退日志。
- 只有严格更高版本的 A 才进入 trial；A/B 同版本不会进入。
- 未实现持久化 confirm、业务健康确认或恶意 APP 防护。

## 常见问题

### 没有打印 `Trial WWDT reset`

先检查启动日志。必须同时满足 A、B 有效且 A 版本高于 B，例如 `A=0x00010007`、`B=0x00010006`。

### `write-memory` 被拒绝

必须在同一次 bootloader 会话中先成功执行 erase。备份失败、地址越界或未执行 erase 都会拒绝写入。

### 下载后显示 `A invalid`

确认 `write-memory` 使用的是 `*_upgrade.bin`，而不是 HEX。使用 HEX 时应改用 `flash-image ... erase` 或 KinetisFlashTool。

### 下载后仍是旧版本

检查实际选择的版本化 HEX/BIN 及启动日志中的 version、size、CRC。不要使用旧产物或 `raw.hex`。

### A 面升级中断

复位后如果 A 无效而 B 有效，bootloader 会重新执行 B→A。恢复再次中断也不会修改 B，下次复位可继续恢复。
