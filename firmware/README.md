# Follow 2026-07-13

这是 2026-07-13 晚调试并烧录验证的算法 6.2 跟随行李箱代码。

主要内容：

- RPLIDAR C1：UART2，TX=GPIO17，RX=GPIO18，460800 baud
- BU UWB：UART1，TX=GPIO47，RX=GPIO48，115200 baud
- 左右 ESC：GPIO4、GPIO5，50 Hz RC PWM
- 左右编码器：GPIO6/7、GPIO15/16
- 左右超声波：GPIO38、GPIO39
- FSR：GPIO8
- 雷达任务 CPU 占用修复
- UWB 跟随、雷达/超声波避障和 BLE 遥控
- 手机 App 支持跟随速度、跟随转向和遥控速度 0~100% 实时调节

固件仅广播 BLE 设备 `SmartSuitcase`，不启动 Wi-Fi 热点或网页服务器。

```powershell
cd C:\path\to\follow-2026-07-13
D:\1Download\Espressif\frameworks\esp-idf-v5.4.3\export.ps1
idf.py build
idf.py -p COM10 flash monitor
```
