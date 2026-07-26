# SmartSuitcase Watch

独立 Wear OS 手表客户端，直接通过 BLE 连接名为 `SmartSuitcase` 的行李箱控制器。

## 页面

- 总览：连接、运行模式、电量、重量、目标距离、目标方向、急停和解除急停
- 运动：运行状态、速度、角速度、电机脉宽和三项速度设置
- 跟随：跟随/遥控模式切换、UWB 目标、雷达前向净空、左右超声波距离
- 遥控：前、后、左、右和停止
- 传感器：UWB、激光雷达、左右超声波、压力传感器和编码器状态

支持左右滑动、表冠旋转、旋转表圈以及导航键翻页。第一页和最后一页不会循环，继续翻动时会显示对应方向的边缘光效。

## 构建

```powershell
gradle :watch:assembleDebug
```

生成文件：`watch/build/outputs/apk/debug/watch-debug.apk`
