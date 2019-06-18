# NXP EASY MCU bootlaoder 

这是NXP官方 MCUBOOT 的 简化版C实现，砍掉了大部分功能，只保留串口下载功能。可以配合 官方的命令行工具blhost 或者 GUI工具： Kinetis Flash Tool 来实现一个基于MCU串口的bootloader.  非常容易移植到任何MCU上



## 简介

* 基础实现和具体硬件完全解耦。 只有2个.c .h文件:kptl.c  mcuboot.c。 除Systick中断外 不使用任何中断。只使用串口轮训发送和接收数据。
* 所有与硬件相关的必要操作全部由回调函数显示。 kptl 和  mcuboot 本身没有任何硬件依赖。非常容易移植到任何ARM Cortex MCU上
* 目前提供LPC802和 FRDM-K64的移植demo



有关完整的MCUBOOT 协议，软件及其他内容看参看官方[page](https://www.nxp.com/support/developer-resources/software-development-tools/mcuxpresso-software-and-tools/mcuboot-mcu-bootloader-for-nxp-microcontrollers:MCUBOOT):

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

bootloader本身只有4个 文件: `kptl.c ` `kptl.h`  `mcuboot.c`, `mcuboot.h`.

kptl 负责MCUBOOT协议的基本实现，拆包封包等， mcuboot 实现 bootloader的基本命令。 注意mcuboot已经砍掉了大部分 官方完整版的命令实现，只保留用于串口下载的最基本命令如 擦写flash, 获得芯片信息等等。



### 移植步骤

1. 准备好的你的MCU： 确保你已经熟悉了你的MCU。MCU 可以正常跑，串口收发功能已经调通并没有问题， Flash编程，擦写功能已经调通并没有问题。
2. 在工程中加入kptl.c 和 mcuboot.c 并且添加对应的include路径
3. 在工程中定义全局结构： `static mcuboot_t mcuboot;`

4. 在初始化必要的硬件资源后(串口和Flash肯定要初始化吧)。 给mcuboot结构填写必要的参数和回调接口


   具体需要实现的回调操作如下

   | 回调接口     | 说明                                                         |
   | ------------ | ------------------------------------------------------------ |
   | op_send      | 串口发送数据                                                 |
   | op_reset     | 复位MCU                                                      |
   | op_jump      | MCU跳转到指定地址                                            |
   | op_complete  | 在bootloader 接收完完整的image后会回调这个函数，一般用于释放硬件资源，改关闭的外设关闭，中断该关的关 |
   | op_mem_erase | flash擦除                                                    |
   | op_mem_write | flash编程                                                    |
   | op_mem_read  | flash读取                                                    |

   配置：

   | 配置                  | 说明                                                         |
   | --------------------- | ------------------------------------------------------------ |
   | cfg_flash_start       | APP启动地址(示例中一般默认为0x1000(4K) 或者0x8000(32K))      |
   | cfg_flash_size        | FLash 大小                                                   |
   | cfg_flash_sector_size | flash sector 大小                                            |
   | cfg_ram_start         | RAM起始地址                                                  |
   | cfg_ram_size          | RAM大小                                                      |
   | cfg_device_id         | MCU ID 例子中一般填了一个0x12345678而已，有需要的话需要从MCU中读取 |
   | cfg_uuid              | MCU UID， 唯一ID                                             |

   例子：

   ```
   /* config the mcuboot */
   mcuboot.op_send = mcuboot_send;
   mcuboot.op_reset = mcuboot_reset;
   mcuboot.op_jump = mcuboot_jump;
   mcuboot.op_complete = mcuboot_complete;
   
   mcuboot.op_mem_erase = memory_erase;
   mcuboot.op_mem_write = memory_write;
   mcuboot.op_mem_read = memory_read;
   
   mcuboot.cfg_flash_start = APPLICATION_BASE; 
   mcuboot.cfg_flash_size = TARGET_FLASH_SIZE;
   mcuboot.cfg_flash_sector_size = FLASH_GetSectorSize();
   mcuboot.cfg_ram_start = 0x20000000;
   mcuboot.cfg_ram_size = 128*1024;
   mcuboot.cfg_device_id = 0x12345678;
   mcuboot.cfg_uuid = GetUID();
   
   mcuboot_init(&mcuboot);
   ```



5. 在while1 中，轮训接受串口数据，接到数据就调用一次`mcuboot_recv`,  并轮训调用`mcuboot_proc`. 同时，可以通过Systick中断设置一个超时函数，超过多少秒后没有收到串口数据就尝试直接跳到app.(例子一般默认300MS)

       while(1)
       {
           if(UART_GetChar(HW_UART0, &c) == CH_OK)
           {
               mcuboot_recv(&mcuboot, &c, 1);
           }
           
           if(timeout_jump == 1)
           {
               mcuboot_jump(APPLICATION_BASE, 0, 0);
               timeout_jump = 0;
           }
           mcuboot_proc(&mcuboot);
       }



## 其他一些注意的

1. Flash的操作实现很重要。 一般flash都是块设备，有最小的擦除和编程单位(尤其是LPC, 最小擦除单位和最小编程单位都不一样，而且很大，很恶心)。这就需要在实现`memory_write`函数的时候格外注意，该对齐的对齐，该补全的补全，没到最小编程单位的需要提取读出之前的数据并合并，超过最小编程单位的需要多次写入flash
2. 对于MCU复位的实现，直接调用 CMSIS库函数 `NVIC_SystemReset`即可
3. 最后跳转之前一般需要把外设全部反初始化，中断(包括SYsTick)该关的都要关掉
4. 最后的跳转到用户app 。主要需要干三件事： 重新设置PC,SP, 重新设置中断向量表的入口地址：SCB->VTOR。其中SP为Image的0-3字节，PC为image的4-7字节。可直接使用祖传函数：

```
void JumpToImage(uint32_t addr)
{
    uint32_t *vectorTable = (uint32_t*)addr;
    uint32_t sp = vectorTable[0];
    uint32_t pc = vectorTable[1];
    
    typedef void(*app_entry_t)(void);

    uint32_t s_stackPointer = 0;
    uint32_t s_applicationEntry = 0;
    app_entry_t s_application = 0;

    s_stackPointer = sp;
    s_applicationEntry = pc;
    s_application = (app_entry_t)s_applicationEntry;

    // Change MSP and PSP
    __set_MSP(s_stackPointer);
    __set_PSP(s_stackPointer);
    
    SCB->VTOR = addr;
    
    // Jump to application
    s_application();

    // Should never reach here.
    __NOP();
}
```

