# LPC802 Bootloader 说明



路径：

1. bootloader 工程:\Project\Internal\lpc802_bl
2. app 工程： \Project\Internal\example_app
3. PC机工具：\pc_tool



使用说明：

1. 修改bootloader 串口： 默认P0_0, P0_1

   ```
       /* UART: P0_0 P0_1 */
       SWM_Config(0, 1, 0);
       SWM_Config(0, 0, 8);
   ```

2. 编译bootloader工程，下载到目标上。

3. 编译app工程，生成hex文件

4. 打开KinetisFlashTool.exe, 选择好串口和波特率(固定115200). 点击connect，(如果不行，复位后300MS内点击connect)

5. 在image处选择编译好的app的 hex 文件(一定是hex).然后选择update即可。

6. 升级完成后会打印app中的串口log:

   ```
   example app: main address:0x1971
   CoreClock:15000000Hz
   
   ```

   

​	