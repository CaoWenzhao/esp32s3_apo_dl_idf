## Follow Draft

这份是当前已经烧录到 ESP32-S3 的跟随代码。

工程入口：

```text
examples/follow_only
```

主要改动方向：

- 左右轮反向配置保持当前实测可用状态。
- 使用直接 PWM 脉冲控制，减少低速顿挫。
- 提高前进和转向响应速度。
- 使用 UWB 距离/角度做跟随，IMU 只做车体 yaw 航向辅助修正。

编译示例：

```bat
cd examples\follow_only
idf.py build
idf.py -p COM10 flash
```
