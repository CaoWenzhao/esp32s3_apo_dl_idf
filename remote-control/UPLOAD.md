## Remote Control

这份是之前的遥控工程代码。

工程入口：

```text
examples/remote_box
```

公共底盘和网页控制组件已经一起放在本目录的 `components` 下。

编译示例：

```bat
cd examples\remote_box
idf.py build
idf.py -p COM10 flash
```
