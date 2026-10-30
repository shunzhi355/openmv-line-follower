# OpenMV 黑色中线循线

本实现参照
`C:\Users\user\Desktop\省赛双足\控制\orangepi` 中的五扫描带循线逻辑：

- 多条扫描带跟踪同一条物理中线，而不是把扫描带当作多条赛道。
- 初次看到三条候选时选中间线；只看到一条时假定机器人已经摆在中线上。
- 锁定后优先保持上一帧的线身份，丢失的近端线不会自动跳到边线。
- 用 `0.35 * 横向偏差 + 0.80 * 方向偏差` 生成图像空间转向量。
- 至少三条扫描带有效并连续稳定三帧后才允许行走。
- 丢线时立即把行走标志和转向量清零；持续偏离会锁停，需要重新摆正并复位。

## 文件

把 `main.py` 和 `line_core.py` 上传到 OpenMV 的同一个目录。`main.py`
负责相机、扫描和 UART，`line_core.py` 负责线关联、离群扫描带剔除和转向计算。
本机可以直接运行离线测试：

```powershell
python -m unittest test_line_core.py test_main_mock.py -v
python -m py_compile main.py line_core.py test_line_core.py test_main_mock.py
```

## 接线与输出

默认使用 OpenMV `UART3`、`115200 baud`，常见引脚为 `P4=TXD`、`P5=RXD`。
只循线时通常只需连接 OpenMV `P4/TX` 到主控 `RX`，并连接双方 `GND`。
两板均为 3.3 V 逻辑时可直接连接；使用 5 V 串口时必须确认电平兼容。

每帧发送一行 ASCII：

```text
LF,<sequence>,<active>,<near_milli>,<far_milli>,<yaw_milli>\r\n
```

| 字段 | 含义 |
| --- | --- |
| `sequence` | 0 到 255 循环序号 |
| `active` | `1` 才允许按固定前进速度行走；`0` 表示停车 |
| `near_milli` | 近端线位置，范围约 -1000 到 +1000，正值在图像左侧 |
| `far_milli` | 远端线位置，范围约 -1000 到 +1000，正值在图像左侧 |
| `yaw_milli` | 转向控制量，约 -120 到 +120，对应参考项目的 ±0.12 rad/s |

相机识别不到线时，`active=0` 且 `yaw_milli=0`。主控应把 `active=1`
映射到参考项目的 `vx=0.06`，把 `yaw_milli / 1000.0` 映射到
`yaw_rate`；需要把转向量映射为舵机或差速量时，请在主控侧完成，不要直接
把 `yaw_milli` 当作舵机 PWM。

## 上场前校准

1. 摄像头倒装时将 `CAMERA_ROTATED_180` 改成 `True`，正装保持 `False`。
2. 在 OpenMV IDE 中观察画面。黑线没有被完整选中时，检查现场亮度并调整
   `MAX_BLACK_THRESHOLD`；滤除阴影或浅灰条纹时适当降低它。
3. 如果机器人转向相反，把 `TURN_SIGN` 从 `1` 改成 `-1`。
4. 机器人静止摆在中线上时，若近端位置不是 0，把 `NEAR_REFERENCE`
   设置为静止时的 `near` 值；方向不正时同样校准 `HEADING_REFERENCE`。
5. 赛道以单条黑线为主时保留 `LINE_SELECTION="auto"`。若一定能同时看到
   三条线且只允许中间线起步，改成 `"middle"`；只允许单条线起步时改成
   `"single"`。
6. 不想现在接主控时可设 `UART_ENABLED=False`，OpenMV IDE 中仍能看到扫描带、
   被选中心点和实时状态。

推荐先在 `DRAW_DEBUG=True` 下架空车轮或架起机器人验证左右方向、弯道和丢线
停车，再接通执行机构。固定参数只有在纸面或合成图测试通过，不能代替实机
赛道验收。
