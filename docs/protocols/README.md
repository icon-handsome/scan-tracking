# 通信协议与寄存器

| 文件 | 说明 |
|------|------|
| [封头检测工位PLC-IPC Modbus通信协议_v0.1.md](./封头检测工位PLC-IPC%20Modbus通信协议_v0.1.md) | IPC ↔ PLC Modbus TCP |
| [封头检测工位_TCP_IP显控通信协议_v1.0.md](./封头检测工位_TCP_IP显控通信协议_v1.0.md) | IPC ↔ Qt 显控 TCP JSON |
| [plc_modbus_寄存器地址表_v0.3_patched - 副本.xlsx](./plc_modbus_寄存器地址表_v0.3_patched%20-%20副本.xlsx) | **当前编辑版**寄存器对照表（40046 伸缩杆扫描；40051~40057 上下料状态/过程） |
| [plc_modbus_寄存器地址表_v0.3_patched.xlsx](./plc_modbus_寄存器地址表_v0.3_patched.xlsx) | 上一版 patched（40176~40179 下料区封头计数） |
| [plc_modbus_寄存器地址表_v0.3_下料区增量.csv](./plc_modbus_寄存器地址表_v0.3_下料区增量.csv) | 40176~40179 增量行（CSV，可粘贴进 xlsx） |
| [plc_modbus_寄存器地址表_v0.2(1).xlsx](./plc_modbus_寄存器地址表_v0.2(1).xlsx) | 上一版（40176~40179 仍为预留） |

重新生成副本 xlsx 增量行：`python tools/patch_plc_modbus_copy_load_unload.py`

HMI 联调步骤见 [../hmi/](../hmi/)。
