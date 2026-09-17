# DetFirmware Agent Knowledge Base

Generated: 2026-06-15

## OVERVIEW

ESP32 检测装置主控固件：X/Y/Z/A 步进泵、进样泵 PWM、角度采集、ADS122C04 分光采样、健康帧和串口文本命令。

独立私有仓库：`MIGO-OvO/DetFirmware`，默认分支 `main`。先读 `README.md`；在本仓库内提交，不能通过 `usv_ws` 根仓库提交固件源码。独立克隆时，下面的 ROS/Windows 相对路径需按实际工作区调整。

## STRUCTURE

```text
DetFirmware/
├── platformio.ini
└── src/
    ├── main.cpp                # 主控逻辑、串口命令、任务循环
    ├── protocol_packets.h      # 二进制帧格式
    ├── ads122c04.h             # 分光 ADC
    └── i2c_mux.h               # I2C 复用
```

## WHERE TO LOOK

| 任务 | 位置 |
|---|---|
| 串口握手 | `src/main.cpp` 中 `HELLO?` / `DET?` 与 `DET_ID:*` |
| 文本命令 | `src/main.cpp` 串口命令解析 |
| 二进制帧 | `src/protocol_packets.h` |
| 分光帧 | `HEADER2_SPECTRO` / `0xDD` |
| 健康帧 | `HEADER2_HEALTH` / `0xEE`，1 Hz |
| ROS 对端 | `../src/usv_ros/scripts/pump_control_node.py` |
| Windows 对端 | `../MotorControlApp_Pyside6/src/` |

## CONVENTIONS

- 默认串口：115200 8N1，文本命令以 LF 或 CRLF 结束；仅 CR 不触发解析。
- 身份握手期望 `DET_ID:USV_DETECTOR*`。
- 二进制帧头使用 `0x55` + 类型字节；修改布局必须同步 ROS 和上位机解析。
- 分光原始量、电压、valid、基线、健康状态可由固件输出；污染物浓度不在固件计算。

## ANTI-PATTERNS

- 改 `protocol_packets.h` 后不改 ROS `pump_control_node.py`。
- 把污染物浓度、历史记录或热力图写进固件。
- 用阻塞串口/延时破坏角度、健康帧和控制循环节奏。

## COMMANDS

```bash
python -m pytest tests -q
pio run -e nodemcu-32s
git diff --check
rg -n "DET_ID|HELLO\\?|DET\\?|0xDD|0xEE|Serial.write|Serial.printf" src
```

pytest 为源码级回归，不能替代台架测试。未获明确授权不烧录、不发送硬件动作命令；不得提交 `.pio/`、缓存、二进制产物或串口日志。
