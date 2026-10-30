# OpenMV 黑色中线循线

本实现面向当前项目的 **503 双足竞步赛白底黑色中线**，组合了本地
`控制/orangepi` 的五扫描带关联逻辑和 GitHub 上 OpenMV 官方的多区域加权循线算法：

- 多条扫描带跟踪同一条物理中线，而不是把扫描带当作多条赛道。
- 初次看到三条候选时选中间线；只看到一条时假定机器人已经摆在中线上。
- 锁定后优先保持上一帧的线身份，丢失的近端线不会自动跳到边线。
- 默认按近到远的权重 `(0.40, 0.25, 0.18, 0.11, 0.06)` 计算中线偏差，
  用 `atan` 转成图像偏转角，再结合方向偏差生成转向量。
- 缺失或异常扫描带不参与权重求和，避免把未识别的区域当作图像边缘。
- 至少三条扫描带有效并连续稳定三帧后才允许行走。
- 丢线时立即把行走标志和转向量清零；持续偏离会锁停，需要重新摆正并复位。

## GitHub 来源与融合范围

- 双足项目参考：[Hiwonder/TonyPi 的 VisualPatrol.py](https://github.com/Hiwonder/TonyPi/blob/dc452eaac9991516600860f706f4b889f3305d1d/Functions/VisualPatrol.py)。
  它用多区域中心加权控制双足机器人，但依赖树莓派 OpenCV、NumPy 和欢尔动作组。
  该版本没有发现根目录许可证，故本项目仅记录思路参考，没有复制或调用其源码。
- 实际移植来源：[OpenMV 官方 black_grayscale_line_following.py，v4.5.9](https://github.com/openmv/openmv/blob/f1aa0247d8052d43ba0a9a07a4fbcd867c47d134/scripts/examples/02-Image-Processing/02-Color-Tracking/black_grayscale_line_following.py)。
  MIT 许可，移植其加权中心与 `atan` 偏转角计算；固定版本原文、许可和 SHA256
  保存在 [third_party/openmv](third_party/openmv/README.md)。

与原示例相比，本项目扩展为五个扫描带，只对跟踪确认的中线点加权，
并保留 `near/far` 方向修正、启动确认、失线停车和原串口协议。
运行不需要安装 OpenCV、NumPy 或额外固件包。历史修复的 Blob/Statistics
整数属性兼容，以及 `draw_circle((x, y, r), ...)` 元组接口仍然保留。
文字绘制通过 `draw_string_compat()` 优先调用新固件的
`draw_string((x, y), text, ...)`，旧固件则回退到 `draw_string(x, y, text, ...)`。

尚未找到能确认使用同一张 503 赛道地图、且能直接运行于本机 OpenMV 的完整双足项目。
本次是算法适配，不代表原项目已在本赛道验收。起终点横线使某个扫描带过宽时会剔除该带；
剩余不足三带时停车。不包含岔路规划、圈数统计或自动终点停车。

## 文件

把 `main.py` 和 `line_core.py` 上传到 OpenMV 的同一个目录。`main.py`
负责相机、扫描和 UART，`line_core.py` 负责线关联、离群扫描带剔除和转向计算。
`third_party/` 只用于追溯来源，不会被运行；部署或转发时请同时保留其中的许可证。
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
7. 新算法默认 `STEERING_MODE="weighted_roi"`。用 `ROI_ANGLE_GAIN` 调整横向修正，
   用 `HEADING_GAIN` 调整朝向修正；`ROI_WEIGHTS` 必须与五个扫描带一一对应且全为正。
   增大近端权重会更偏重脚前的线，增大远端权重会更偏重前方的线。
8. 对比旧算法时设 `STEERING_MODE="near_far"`，恢复
   `0.35 * 横向偏差 + 0.80 * 方向偏差`；此模式使用 `LATERAL_GAIN`。

加权模式的计算方式如下（`t` 为从近端 0 到远端 1 的扫描位置）：

```text
每带误差 = 中线位置 - (NEAR_REFERENCE + HEADING_REFERENCE * t)
加权误差 = sum(有效带权重 * 每带误差) / sum(有效带权重)
bearing = atan(去死区后的加权误差 * (图像宽 - 1) / 图像高)
yaw = clip(TURN_SIGN * (0.30 * bearing + 0.80 * 去死区后的方向误差), ±0.12)
```

`bearing` 是图像中的弧度值，`yaw` 是请求主控执行的转向量，二者不是同一个物理量。
参考偏差逐带计算，因此标定后的中线不会因部分扫描带缺失而产生额外的偏置。
模式切换会改变转向响应，需重新检查现场增益。

推荐先在 `DRAW_DEBUG=True` 下架空车轮或架起机器人验证左右方向、弯道和丢线
停车，再接通执行机构。固定参数只有在纸面或合成图测试通过，不能代替实机
赛道验收。

离线测试覆盖：加权转向符号与限幅、缺失带重新归一化、三线中选中线、
参考位置标定、横线遮挡时继续/停车、丢线立即停车及重新稳定三帧、
两种转向模式的 UART 输出，以及旧方法/新整数属性 API。
模拟测试不能验证相机真实曝光、阈值、帧率或双足行走稳定性。
