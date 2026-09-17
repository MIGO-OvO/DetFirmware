# DetFirmware

## safety1 控制保护

当前源码身份增加 `CAP=WATCHDOG1`。首次控制先发 `WATCHDOG:ARM`（会停止全部输出），随后主机每 500 ms 发送 `WATCHDOG:KEEPALIVE`；超过 3000 ms 无心跳停止开环、PID、校准、PID 测试和进样泵，并保持故障锁存。迟到心跳不会恢复执行；需新控制会话重新 ARM 并显式发起任务。`STOPALL` 始终可用。诊断只读命令仍可在未 ARM 时查询。

必须与更新后的 ROS/Windows 上位机成套使用；手动终端调试运动也必须维护心跳，不能照旧只发送一次长运动命令。该软件保护不替代独立硬件断电手段。

FULL 压测分光帧使用 bit4 (`0x10`) 标记测试数据且清除有效 bit0，禁止写入真实实验记录。二进制长度不变。平台和 TMCStepper 已锁定为本仓库验证基线 6.10.0 / 0.7.3。

新增宿主逻辑回归：Linux/WSL `python3 -B tests/test_control_watchdog_native.py`。它验证超时、锁存、时钟回绕，不验证电气/机械停止时间；上板前仍须进行断线及负载台架测试。

PID 测试轮间等待采用非阻塞调度，期间继续处理心跳和 STOPALL；到期/停机撤销尚未启动的下一轮。Linux/WSL `python3 -B tests/test_pid_test_scheduling_native.py` 提取实际固件函数验证该调度路径，不能替代真实设备负载测试。

水质监测无人船检测装置的 ESP32 固件，面向固件开发、串口联调和硬件维护。
负责 X/Y/Z/A 四路步进泵、进样泵 PWM、MT6701 角度反馈、ADS122C04 分光采样及健康遥测。
污染物浓度、历史记录、任务编排和地图展示由 ROS/Web 或 Windows 上位机负责。

本仓库是 `MIGO-OvO` 的独立**私有仓库**，默认分支为 `main`，不是第三方项目的 GitHub fork。
可独立构建，也可放在 `usv_ws/DetFirmware/` 中开发；不需要 ROS、Qt 或 ArduPilot 构建环境。

## 快速开始

### 获取源码

需要已获授权的 GitHub 账号和 Git 凭证。不要将令牌写入克隆 URL 或配置文件。

```bash
git clone https://github.com/MIGO-OvO/DetFirmware.git
cd DetFirmware
```

工作区布局下，也可在 `usv_ws` 根目录运行 `bootstrap_workspace.bat`。
已有目录不会被覆盖；如目录存在但没有独立 `.git`，脚本会停止，需要先备份并处理旧副本。

### 安装开发工具与编译

使用已安装的 PlatformIO Core，或在独立 Python 环境中安装开发工具：

```bash
python -m pip install platformio pytest
pio run -e nodemcu-32s
```

构建入口为 [platformio.ini](platformio.ini)：`espressif32` 平台、`nodemcu-32s` 开发板、
Arduino 框架，声明的外部库为 `teemuatlut/TMCStepper`。
首次构建需要下载工具链和依赖，产物位于 `.pio/build/nodemcu-32s/`，不提交到 Git。

本次迁移环境：PlatformIO Core 6.1.19、Espressif32 6.10.0、
Arduino-ESP32 包 `3.20017.241212+sha.dcc1105b`、TMCStepper 0.7.3。
当前配置已锁定平台及 TMCStepper 版本；框架/工具链解析结果仍应随发布清单记录。
升级工具链后必须重新编译、测试和上板验证。

### 烧录与串口

烧录会重启设备。先停止自动化、断开执行器负载或确保设备处于安全状态，
关闭占用串口的 ROS/上位机，并确认实际串口号。以下 `COM5` 只是示例：

```bash
pio device list
pio run -e nodemcu-32s -t upload --upload-port COM5
pio device monitor --port COM5 --baud 115200
```

Linux 下将端口替换为实际 `/dev/ttyUSB*` 或 `/dev/ttyACM*`。
固件串口为 **115200、8N1**。命令必须以 LF (`\n`) 结束，也支持 CRLF；
只发送 CR 不会触发命令解析。串口同时包含文本和二进制帧，普通终端出现非文本字符是正常现象。

## 源码导航

```text
DetFirmware/
├── README.md                    # 开发、协议及验证入口
├── AGENTS.md                    # 仓库内 Agent 规则
├── platformio.ini               # 开发板、框架和依赖
├── src/
│   ├── main.cpp                 # 任务、控制状态机、串口命令和遥测
│   ├── ads122c04.h              # ADS 配置、I²C 读取、CRC 和电压换算
│   ├── i2c_mux.h                # TCA9548A 通道切换和 MT6701 读取
│   └── protocol_packets.h      # packed 二进制帧与状态位
├── tests/                      # pytest 源码级回归测试
├── test/                       # PlatformIO 测试占位目录，非现有 pytest 套件
├── include/                    # 预留公共头文件目录
└── lib/                        # 预留项目私有库目录
```

`setup()` 初始化外设和看门狗；Core 1 的 `loop()` 生成电机步进脉冲，
Core 0 的 `TaskComms()` 解析命令并发送数据，`TaskSensors()` 刷新角度缓存。
I²C 和电机状态通过互斥锁协调。两核还各有一个低优先级压力测试任务。
避免在步进循环加入日志、阻塞串口或长延时；总线改动需评估分光采样与角度刷新之间的竞争。

## 默认硬件映射

以下是源码默认值，不替代 PCB 原理图和现场接线确认。

| 功能 | GPIO / 地址 / 通道 |
|---|---|
| X 步进 / 方向 | GPIO13 / GPIO12 |
| Y 步进 / 方向 | GPIO14 / GPIO27 |
| Z 步进 / 方向 | GPIO26 / GPIO25 |
| A 步进 / 方向 | GPIO33 / GPIO32 |
| 进样泵 PWM | GPIO2，1 kHz，8 bit |
| I²C SDA / SCL | GPIO21 / GPIO22，100 kHz |
| TCA9548A / MT6701 | `0x70` / `0x06` |
| X / Y / Z / A 角度 TCA 通道 | `0 / 3 / 4 / 7` |
| ADS122C04 | TCA 通道 `2`，地址 `0x40` |

另有 `BOOT=GPIO0`，当前 `setup()` 将其配置为输出低电平；移植开发板前应核对这一行为。
默认 ADS 输入为 AIN0/AVSS、增益 1、PGA bypass、AVDD 参考（换算采用 3.3 V）、
连续转换 90 SPS、目标上传 50 Hz。上电默认不启动 ADS，需先配置再发送 `ADSSTART`。
ADC 转换速率与串口上传速率不是同一概念，实际有效帧率还受调度、总线和数据过滤影响。

## 串口联调

### 不驱动电机的首次检查

逐条发送以下命令（每条以 LF 结束），确认回包与接线一致：

```text
HELLO?
I2CMAP?
ADSSTATUS?
PUMP:STATUS
PIDQUERY
STRESS:STATUS?
```

身份响应格式为 `DET_ID:USV_DETECTOR,FW=<版本>,BAUD=115200`，`DET?` 与 `HELLO?` 等价。
可发送 `GETANGLE` 获取一帧角度数据；失败的角度读取不能当作有效测量。
命令缓冲上限为 160 字符，超限返回 `CMD_ERR:TOO_LONG`。

### 常用命令

| 命令 | 用途与注意事项 |
|---|---|
| `ANGLESTREAM_START` / `ANGLESTREAM_STOP` | 开关角度流；目标间隔 20 ms |
| `I2CMAP:X=0,Y=3,Z=4,A=7,SPEC=2` | 修改通道映射；ADS 实际通道还需核对 `ADSCFG:CH` |
| `ADSCFG:CH=2,ADDR=0x40,AIN=AIN0,REF=AVDD,GAIN=1,DR=90,MODE=CONT,PR=50` | 配置并使能 ADS；配置回包不等于硬件已启动 |
| `ADSSTART` / `ADSSTOP` | 启停 ADS；启动成功回 `ADS_OK:START` |
| `PUMP:OFF` / `PUMP:STATUS` | 停止或查询进样泵 |
| `PUMP:SPD:<0..100>` | 设置进样泵速度；已运行时会改变实际输出 |
| `PUMP:ON` / `PUMP:SET:<0..100>` | 启动泵，或设置速度并启停；会驱动硬件 |
| `CALXYZA` / `CALSTOP` / `CALSTATUS` | 四轴校准、停止、查询；校准会驱动硬件 |
| `PIDSTOP` / `PIDTESTSTOP` | 停止 PID 定位 / PID 测试，不是全系统急停 |

开环、相对角度 PID、PID 参数及测试命令分别见 `main.cpp` 的 `parseCommand()`、
`parsePIDConfig()`、`parsePIDTest()`。首次联调不要批量发送运动命令；
确认泵路、方向、机械限位和独立断电手段后再做低负载测试。
命令解析没有统一的严格参数校验，不要假定任意错误输入都会被安全拒绝。

### 二进制协议

帧布局以 [src/protocol_packets.h](src/protocol_packets.h) 为准，使用 `#pragma pack(push, 1)`。
帧头为 `0x55` 和类型字节，帧尾为 `0x0A`，校验为 XOR；
ESP32 发出的多字节数值为小端、浮点为 32 位。各类型 XOR 范围应按发送函数和接收端核对，
不要假设所有类型都包含相同范围的帧头字节。

| 类型 | 长度（字节） | 内容 |
|---|---:|---|
| `0xAA` | 29 | PID 目标/实际/理论角度、输出、误差，时间戳为微秒 |
| `0xBB` | 18 | PID 测试评分与收敛等指标 |
| `0xCC` | 20 | X/Y/Z/A 四路角度 |
| `0xDD` | 18 | 分光毫秒时间戳、通道、状态、24 位符号扩展原始码、电压 |
| `0xEE` | 37 | v1 健康帧：温度、堆、CPU 频率、任务与栈水位 |

分光状态位：bit0 有效，bit1 I²C 错误，bit2 未配置，bit3 饱和或越界。
健康帧目标周期为 1 秒，另有文本诊断 `ANGLE_AGE_MS`、`ANGLE_AGE_CH_MS` 和 `ADS_HEALTH`。
当前实现对 CRC 错误、重复转换及孤立大跳变可能不发送分光帧，
接收端必须检测数据新鲜度，不能把最后一个有效帧无限延用为实时值。

ADS 当前读取路径启用转换计数器和 CRC16，CRC 错误重读一次，拒绝重复计数器，
对超过 0.020 V 的跳变要求后续样本在 0.010 V 容差内确认。
这些是本次发布源码的行为，不代表已经通过真实光学信号或故障注入验收。

### 压力测试

仅在停机台架环境使用。`STRESS:START:60` 执行 60 秒 CPU 压力测试；
`STRESS:START:60,FULL` 还会产生**虚拟角度、分光及 PID 数据**，不是实际测量。
默认时长 300 秒，允许范围 1–1800 秒；用 `STRESS:STATUS?` 查询、`STRESS:STOP` 停止。
启动时有运动检查，但不要把它当作全程硬件互锁；测试期间不下发运动命令，
也不要把 FULL 测试数据存入真实实验记录。

## 测试与验收

2026-09-17 迁移检查：16 项 pytest 通过，`nodemcu-32s` 编译成功；
RAM 使用 24,668 / 327,680 字节，Flash 使用 330,625 / 1,310,720 字节。未烧录或执行硬件测试。

在本仓库根目录执行：

```bash
python -m pytest tests -q
pio run -e nodemcu-32s
git diff --check
```

现有 16 项 pytest 是**源码结构/字符串回归检查**，覆盖压力测试隔离、I²C 恢复、
看门狗、采样时序、CRC/计数器/瞬态处理及诊断字段；不会运行 ESP32 控制逻辑或证明硬件正确性。
`test/` 只有占位说明，不能用空的 `pio test` 代替上述检查。

发布到设备前还需逐项台架验证：

1. 身份握手、混合文本/二进制解析、校验和及断连重连。
2. 四通道角度、数据年龄、低负载运动、停止和校准。
3. ADS 启停、有效帧率、CRC/重复/瞬态计数与真实电压响应。
4. I²C 断连/恢复、健康帧持续性和看门狗行为。
5. 分别与 ROS 和 Windows 上位机核对协议，记录两端提交号、板卡、接线和复现步骤。

## 跨仓库开发约定

- 总入口：[usv_ws](https://github.com/MIGO-OvO/usv_ws)。
- ROS 对端：[usv_ros](https://github.com/MIGO-OvO/usv_ros) 的 `scripts/pump_control_node.py`。
- Windows 对端：[MotorControlApp_Pyside6](https://github.com/MIGO-OvO/MotorControlApp_Pyside6) 的 `src/`。
- 工作区相对路径分别为 `../src/usv_ros/` 和 `../MotorControlApp_Pyside6/`；独立克隆时需另取对端源码。
- 改动帧长度、字段、校验、命令或状态语义时，必须同时检查两个接收端并补测试。
- 本目录自行 commit/push；根仓库忽略 `DetFirmware/`，不是 Git submodule。
- 不提交 `.pio/`、缓存、固件二进制、串口日志或带凭证的配置。
- 提交前阅读 [AGENTS.md](AGENTS.md)；问题反馈到本私有仓库的 Issues，并附最小复现与脱敏日志。

## 迁移来源与许可

2026-09-17 从 `usv_ws/DetFirmware/` 以当前工作区快照建立独立仓库，
包含当时尚未提交的 ADS 采样完整性源码和测试。
原工作区基准提交为 `0a051be7ef791ee9799baccc498994f41f1a5d1b`；
旧历史保留在总仓库，可在其克隆中执行：

```bash
git log 0a051be7ef791ee9799baccc498994f41f1a5d1b -- DetFirmware/
```

本次不改写或删除总仓库历史；新仓库私有化不会改变旧历史的访问权限。
切换仍跟踪 `DetFirmware/` 的旧工作区分支前，应另行备份独立仓库，避免旧文件与当前目录重叠。
未新增开源许可证；未经维护者明确授权，不将私有源码公开发布。
第三方工具及库继续适用其各自许可证。
