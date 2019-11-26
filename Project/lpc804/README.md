# LPC804 Uart secondary bootloader

本工程提供了一个简单的LPC804 Uart SBL的例程，用户将lpc804_bl工程编译下载到目标板之后，便可通过uart的方式更新app固件。



## 工程简介

* lpc804_bl -- 此工程实现了uart secondary bootloader的功能。
* lpc804_example_app -- 一个简单的app例程，实现了uart数据的回环测试。
* lpc804_driver -- 一些必要的驱动文件，如flash, gpio, swm 的驱动。


## Toolchain supported

Keil MDK 5.28




## Hardware requirements

* 一块LPC804开发板
* 一根micro USB线
* PC机
* 一个USB转串口模块


## Board settings

1. 将LPC804 的PIO0_1(TX), PIO0_0(RX), GND 分别连接到USB 转串口模块的RX, TX, GND上。
2. 将USB转串口模块连接到PC机上。



## 运行程序

1. 使用J-LINK或者CMSIS-DAP等调试器将lpc804_bl工程编译出的bin或hex文件烧写到LPC804的0x00000000的地方。
2. 使用KinetisFlashTool.exe工具将lpc804_example_app工程编译出的bin或hex文件以uart的方式发送给LPC804目标板, secondary bootloader程序会将uart接收到数据写入到用户指定的app地址处，本例程中app起始地址为0x00001800。
3. 打开PC机上的串口调试工具tera term, 设置如下：
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
4.  复位板子，串口调试工具将会打印出如下信息：
     example app: main address:0x1971
     CoreClock:15000000Hz
通过串口调试工具向LPC804发送任意字符，可以看到发送的字符被返回至PC机上。




