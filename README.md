# 多功能主从 XY 绘图仪

基于 ESP32-S3、BLDC 无刷电机、MT6701 磁编码器、SimpleFOC 和 ESP-NOW 的主从式 XY 绘图仪工程原型。

项目由主端、从端和通信链路组成。主端负责交互输入与力反馈，从端负责云台执行与后续 UV 绘图，通信链路负责坐标命令、状态回传和故障诊断。

当前工程状态：**主机力反馈与主从 SingleX 单轴同步验证阶段**。完整双轴绘图、UV 曝光绘图、BLE 控制和自动绘图仍属于后续阶段。

以便于git，主从机上传不同的仓库，从机仓库链接：https://github.com/Coyme7/ESP_NOW_Slave
---

## 1. 系统架构

```text
主端 Master（交互 / 力反馈）
    ↓ ESP-NOW 坐标命令
通信链路 Communication
    ↑ telemetry / ack / fault / timing
从端 Slave（执行 / 绘图）
```

### 1.1 主端：控制与力反馈

主端是系统输入侧。用户转动两个 BLDC 旋钮，主端读取角度并生成 X/Y 坐标命令，同时通过电流环产生阻尼、虚拟墙和边界回推手感。

| 模块 | 选型 / 状态 |
|---|---|
| MCU | ESP32-S3 |
| 输入电机 | 双 BLDC 2804，作为旋钮 |
| 编码器 | MT6701 磁编码器，SPI / SSI 读取 |
| 驱动板 | DengFoc 双路电机驱动 V3 |
| 电流采样 | 高侧电流采样 |
| 控制方式 | SimpleFOC `foc_current` 电流力矩控制 |
| 当前重点 | 单轴电流环、中心阻尼、虚拟墙、热路径优化 |

### 1.2 从端：跟随执行

从端是系统输出侧。它接收主端发送的 X/Y 坐标，通过位置环和速度环控制云台电机，使 UV 光点在光敏纸上移动。

| 模块 | 选型 / 状态 |
|---|---|
| MCU | ESP32-S3 |
| 执行电机 | 双轴 BLDC 2208 云台 |
| 编码器 | MT6701 磁编码器 |
| 驱动板 | 2 × DengFoc mini 单路驱动 |
| 控制方式 | SimpleFOC 电压控制位置执行 |
| 绘图输出 | 紫光灯 / UV 笔，当前默认关闭 |
| 当前重点 | X 轴 SingleX 同步、5kHz 热路径、状态回传 |

### 1.3 通信

| 项目 | 当前基准 |
|---|---|
| 主通信 | ESP-NOW |
| 后续扩展 | BLE |
| 坐标命令频率 | 约 200Hz |
| 状态回传 | 低频 telemetry，避免污染控制热路径 |
| 包语义 | `seq`、`ack_seq`、`stale`、`duplicate`、`timeout` 分离 |
| 控制策略 | 采用 latest target 覆盖，不逐包重发旧坐标 |

---

## 2. 功能规划

### 2.1 双端协同绘图

主端左右旋钮分别映射到从端云台的 X/Y 摆角。中间按键用于抬笔 / 落笔控制。主端在接近纸面边界时产生反向力矩，形成物理限位感。

当前状态：

| 功能 | 状态 |
|---|---|
| 主端单旋钮力反馈 | 已进入实测调试 |
| 主从 X 轴 SingleX 同步 | 初步可用，实时性继续收口 |
| 主从双轴 DualXY | 未进入当前阶段 |
| UV 绘图 | 默认关闭，安全联锁未完成 |
| 纸面边界虚拟墙 | 算法方案已形成，仍需实测回归 |

### 2.2 自动绘图与逆向反馈

后续从端可执行预设轨迹，例如圆、五角星或图案路径。自动绘图时，数据流可反向回传：从端实时位置通过 ESP-NOW 回传主端，主端旋钮跟随运动。

当前状态：规划中。该功能依赖 DualXY 闭环、路径规划、UV 安全联锁和主端反向驱动策略。

### 2.3 BLE 控制模式

后续主端可作为手机或电脑的 BLE 控制器使用。左旋钮可模拟切歌段落感，右旋钮可控制音量，中键可控制播放 / 暂停。

当前状态：规划中。BLE 不参与当前 SingleX 验收。

---

## 3. 光路与绘图范围

当前光路基准：

| 项目 | 数值 |
|---|---:|
| 云台 / 紫光笔到纸面距离 | 约 30cm |
| 有效绘图范围 | 约 25cm × 25cm |
| 单轴半幅 | 约 12.5cm / 125mm |
| 单轴半角 | 约 22.6° |
| 单轴总摆角 | 约 45.2° |

这些数值用于坐标映射、纸面边界判断和虚拟墙设计。当前调试仍以单轴 X 跟随为主，完整 XY 几何标定尚未完成。

---

## 4. 算力与控制频率基准

当前工程阶段按以下频率分层：

| 层级 | 当前基准 | 说明 |
|---|---:|---|
| 主端单旋钮电流环 | 8kHz 调试基准 | 用于验证主机 ADC、SSI、current sense 热路径 |
| 主端双旋钮力反馈 | 5kHz 评估基准 | 双电机后重新评估总热路径 |
| 从端 SingleX | 5kHz 压测基准 | `200us / FOC every 1`，用于验证 X 轴极限实时性 |
| 从端双轴绘图 | 2kHz 工程基准 | 双轴阶段优先保证稳定轨迹和低漏拍 |
| 从端外环 / 插值 / UV 安全 | 约 1kHz | 轨迹平滑、限速、落笔安全状态机 |
| ESP-NOW 坐标命令 | 约 200Hz | 不追求 1kHz 高频通信 |
| 状态输出 | 低频 | 串口日志与 telemetry 不进入高频控制路径 |
| 10kHz FOC | 后续目标 | 不作为当前双轴绘图验收频率 |

任务分配：

| 核心 | 主要职责 |
|---|---|
| Core 0 | ESP-NOW 收发、状态显示、后续 BLE |
| Core 1 | 电机控制、FOC、实时控制任务 |

---

## 5. 开发环境

| 项目 | 配置 |
|---|---|
| 开发板 | RYMCU ESP32-S3-DevKitC-1-N8R2 |
| 主控 | ESP32-S3，双核 Xtensa LX7，240MHz |
| Flash | 8MB Quad SPI |
| PSRAM | 2MB Quad SPI |
| IDE | VS Code + PlatformIO |
| 构建系统 | CMake 3.30.2 |
| 构建工具 | Ninja 1.9.0 |
| 编译器 | GNU 8.4.0，`xtensa-esp32s3-elf-gcc` |
| Python | 3.13.5，PIO 独立虚拟环境 |
| ESP-IDF | 4.4.7 |
| Arduino Core | 3.20017.x，作为 ESP-IDF component 使用 |
| SimpleFOC | 2.3.3，放置于 `lib` 文件夹 |
| FreeRTOS Tick | 1000Hz / 1ms，来自 `sdkconfig.defaults` |

工程路径保持纯英文、无特殊字符。

---

## 6. 软件结构

当前保持主端和从端两个 PlatformIO 工程，`common` 目录用于同步协议、状态和基础工具。

```text
ESP_NOW_Master/
└── src/
    ├── common/
    │   ├── protocol/
    │   ├── state/
    │   ├── timing/
    │   └── math/
    └── master/
        ├── config/
        ├── control/
        ├── haptics/
        ├── hardware/
        ├── comm/
        ├── status/
        └── diagnostics/

ESP_NOW_Slave/
└── src/
    ├── common/
    │   ├── protocol/
    │   ├── state/
    │   ├── timing/
    │   └── math/
    └── slave/
        ├── config/
        ├── control/
        ├── motion/
        ├── hardware/
        ├── comm/
        ├── safety/
        ├── status/
        └── diagnostics/
```

结构边界：

- 高频控制路径只处理传感器读取、FOC、电流 / 位置控制。
- ESP-NOW 回调只做轻量接收和缓存，不直接执行控制逻辑。
- 串口日志、状态发布、telemetry 不进入电机控制热路径。
- `common` 中的协议结构体需保持主从一致。
- `pen_req` 表示主端请求，`uv_out` 表示从端实际输出，二者不混用。

---

## 7. 阶段递进

项目按底层闭环到完整绘图逐级推进。

| 阶段 | 内容 | 状态 |
|---:|---|---|
| 0 | 项目架构、供电、IO、安全边界 | 部分明确 |
| 1 | 主机单轴电流环 bring-up | 已进入实测，需回归验证 |
| 2 | 主机 ADC / SSI / 实时性热路径 | 已定位主要瓶颈，继续观察调度长尾 |
| 3 | 主机中心阻尼与虚拟墙 | 方案已形成，手感未定型 |
| 4 | 配置宏、日志、注释、结构清理 | 部分完成 |
| 5 | ESP-NOW 协议语义 | 部分完成 |
| 6 | 主从 SingleX 单轴同步 | 初步可用 |
| 7 | 从机 5kHz 热路径收口 | 当前主线 |
| 8 | fault / safety / UV / pen 隔离 | 部分完成 |
| 9 | Y 轴、DualXY、UV 绘图、BLE、AUTO_DRAW | 未进入 |

---

## 8. 当前工程进展

### 8.1 主机电流环

已完成 / 已明确：

- 主机旋钮采用 `foc_current` 电流力矩控制。
- 0A 烟测、固定 `+Iq / -Iq` 是主机 bring-up 的基础测试。
- `current` 日志字段更适合作为目标电流理解，实际电流状态应结合 `iq / id` 判断。
- 电流采样 offset、gain sign、相序、sensor direction 是每次大改后的回归项。
- Q/D 电流环积分可能导致 `Vq / Vd` windup，调试阶段保留 P-only 更安全。

仍需处理：

- 跳过 `driverAlign()` 后，非零电流方向与相位仍需固定测试确认。
- 虚拟墙强力矩参数不能跳过 0A / 固定电流回归直接启用。

### 8.2 主机实时性

已完成 / 已明确：

- 早期 10kHz 控制存在 WDT 和长尾超时。
- MT6701 SSI 优化后不再是主要瓶颈。
- `adc1_get_raw()` 不适合放在 current sense 高频热路径中。
- SimpleFOC `adcRead(pin)` fast path 显著降低 `cs / foc / ctrl` 平均耗时。
- timing 诊断必须分级使用，Level 2 只适合短时间定位问题。

仍需处理：

- ESP-NOW、Wi-Fi、状态打印、调度抖动仍可能带来少量漏周期。
- 双旋钮 5kHz 需要重新评估双 current sense、双 `loopFOC()` 和双编码器读取开销。

### 8.3 主机力反馈手感

已完成 / 已明确：

- 中心区域不采用常态 disable，而是保留均匀阻尼。
- 虚拟墙用于纸面边界约束，不等同于简单角度限位。
- 89°~90° 附近的麻感更可能来自目标电流纹波、窄墙区入口过硬、角度噪声和机械触感放大。
- 墙区算法使用角度 LPF、Schmitt 迟滞、smootherstep、最小贴墙电流和入墙阻尼。

仍需处理：

- 慢速入墙、墙内停手、快速撞墙三种场景下的实测回归。
- 默认参数、强手感实验参数和安全参数需要分表管理。

### 8.4 主从通信与 SingleX

已完成 / 已明确：

- ESP-NOW 控制包频率以 200Hz 为当前基准。
- 控制包采用 latest target 覆盖，不逐包重发旧坐标。
- `stale / duplicate / timeout` 用于通信质量判断，不直接等同硬件故障。
- 从机 X 轴稳态跟随已经初步可用。

仍需处理：

- `MasterRtCommand / SlaveRtCommand` 语义继续收口。
- `sysData` 不能长期作为实时发包源。
- `seq / ack_seq / rxok / rxbad / rxrej` 统计需要继续保持可解释。

### 8.5 从机 5kHz 热路径

已完成 / 已明确：

- `200us / 5kHz / FOC every 1` 是 SingleX 当前压测基准。
- Level 2 timing 会污染 5kHz 判断，不能只看偶发 max。
- Level 0 / Level 1 更接近运行负载。
- 热路径需要拆分 `sensor / loopFOC / move / state publish / timing`。

仍需处理：

- `step_us / ctrl_miss_delta / x_sensor_us / x_foc_us / x_move_us` 的可信度验证。
- FULL_CONTROL 与性能隔离模式的对比。
- 进入 Y 轴前完成 SingleX 长时间稳定性测试。

### 8.6 fault、UV、pen

已完成 / 已明确：

- 当前不启用 UV 绘图。
- `pen_req` 是主端请求，`uv_out` 是从端实际输出。
- UV 硬件未接入时，`UV_INTERLOCK` 不参与 SingleX 电机调试结论。
- fault 需要区分 local、remote、link 三类来源。

仍需处理：

- `active_faults / latched_faults / protocol_fault_flags` 的最终显示和清除策略。
- UV 输出前的硬件联锁、默认关闭、异常切断和状态回传。

---

## 9. 当前限制

- 不能按当前状态宣称完整 XY 绘图已完成。
- Y 轴闭环未完成真实硬件验证。
- UV / 紫光灯绘图默认关闭。
- 从机 5kHz 仍在热路径验证中。
- 主机虚拟墙手感未最终定型。
- fault 来源仍需继续拆分。
- 配置宏仍需继续清理，尤其是 skip align、sensor direction、timing level 和日志开关。
- 主机单旋钮结果不能直接等价于双旋钮结果。

---

## 10. 调试与验收基准

### 10.1 主机

- 0A 烟测下无自转、啸叫和异常发热。
- 固定 `+Iq / -Iq` 力矩方向稳定、可重复。
- `iq / id / vq / vd` 不出现持续 windup。
- 关闭高等级 timing 后控制周期稳定。
- 中心阻尼连续、均匀。
- 虚拟墙慢速入墙、墙内停手、快速撞墙时无明显细碎嗡振。

### 10.2 通信

- 主端发送计数正常增长。
- 从端 `rxok` 正常增长，`rxbad / rxrej` 可解释。
- `seq / ack_seq` 逻辑清楚。
- `stale / duplicate / timeout` 独立统计。
- timeout 不清除最后有效命令缓存。
- telemetry 能稳定回传 X 轴状态、fault 和 timing。

### 10.3 从机

- X 轴可接收主端命令并稳定跟随。
- 稳态 `x_total_err` 保持在可接受范围。
- 快速段误差可通过限速、限加速度和控制长尾解释。
- Level 0 / Level 1 下 `ctrl_miss_delta` 可接受。
- `x_sensor_us / x_foc_us / x_move_us / step_us` 统计可信。
- UV/pen/fault 历史状态不影响 SingleX 电机调试。

---

## 11. 暂缓内容

以下功能不进入当前版本验收：

- 完整 XY 绘图路径规划。
- UV 笔 / 紫光灯真实曝光输出。
- BLE 手机 / 电脑控制。
- AUTO_DRAW 自动绘图。
- 从端反向驱动主端旋钮。
- Surface Dial、音乐盒、太空人等扩展模式。
- 双旋钮产品化手感。
- 外壳、PCB 产品化和量产结构。

---

## 12. 后续路线

| 阶段 | 目标 |
|---|---|
| v0.3.x | 主机电流环回归、虚拟墙实测、SingleX 稳定性、从机 5kHz 热路径分解 |
| v0.4.x | SingleX 长时间稳定运行、`MasterRtCommand / SlaveRtCommand` 收口、动态误差优化 |
| v0.5.x | YSensorOnly、YMotorOpenLoop、YClosedLoop、DualXYFramework |
| v0.6.x | `pen_req / uv_out` 接入、UV interlock、纸面尺寸映射、抬笔 / 落笔状态机 |
| v0.7.x | DualXY 绘图、自动轨迹、BLE 控制、模式切换和结构优化 |

---

## 13. 安全说明

- 电机、电源和驱动板调试从低电压、低电流开始。
- 电流采样方向、offset、相序未确认前不提高电流上限。
- 普通杜邦线不作为电机功率线或 12V 主电源线。
- UART 只用于调试 / 通信，不作为最终主供电入口。
- GPIO 不直接作为整机总电源开关，只用于 EN、MOSFET、继电器或安全联锁控制信号。
- UV 硬件完成安全联锁前保持默认关闭。
- 高频控制路径内不放串口打印、无线发送、动态内存分配和阻塞调用。
