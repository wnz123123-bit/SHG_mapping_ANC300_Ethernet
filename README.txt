SHG Mapping 控制程序（ANC300 以太网 X/Y 扫描）

运行前配置：
1. 从 ANC300 前面板或 DHCP 租约获取 IP 地址；也可按实验室网络规范配置静态地址。
2. 在 config.json 的 anc300.host 填入该 IP，端口使用 TCP 7230。config.json 有意以明文保存 ANC300 密码；请按含敏感配置文件妥善保管，勿上传或共享。
3. 确认 X/Y 模块分别接在配置的 x_axis=1、y_axis=2，并确认控制器工作于 4 K、0–150 V 的硬件配置。未完成此确认时不要置 hardware_profile_confirmed 为 true。

运行步骤：
1. 双击 run_mapping.bat。出厂配置 simulation_mode=false，首次干跑前必须主动勾选“模拟模式”，再确认扫描范围、步进、驻留时间和结果目录。
2. 程序不会自动连接或自动重连。需要真实扫描时，取消“模拟模式”，点击连接；连接过程只读取身份和 X/Y 模块状态，不会自动开启输出。
3. 核对设备身份、X/Y 模块和 4 K / 0–150 V 配置后，按界面提示确认硬件 profile，再显式开启 X/Y 输出。
4. 使用“当前位置设为零点”设定本次会话的估计原点，随后开始扫描。

重要限制：
- ANC300 X/Y 是开环电压定位；位置为估计值。CSV 同时记录每点电压和使用的校准值，便于追溯。
- 支持的安全顺序是：连接 -> 真实硬件确认 profile -> 开启输出 -> 设定会话原点。输出未开启时不能设原点。
- 安全开启和接地都会核对实测输出 geto（绝对容差 0.05 V），并要求 AC-IN/DC-IN 均为 off。任一检查不通过时流程会闭锁失败，不会声称输出已安全开启或接地。
- 不提供 Z 轴控制，也不保留旧 MT 平台的 X/Y/Z 映射、脉冲校准或手动移动控制。
- MT 控制器只用于 HWP（半波片）旋转；其 USB/UART/NET 传输配置独立保存在 rotator 节。

数据保存：
- 默认目录为 results；每次扫描生成一个排他创建、防覆盖的 mapping_时间.csv 文件。
- Mapping 和角度扫描 CSV 的 measurement_source 列会明确标记 pmt_hardware、pmt_simulation 或 reader_simulation。

读数程序：
- 真实模式必须启用 PMT 后端，且本地 vendor_dlls\C8855-01api.dll 必须存在；不允许用 reader.py 常量充当真实测量。
- 只有模拟模式下才可以在 PMT 未启用时使用 reader.py 的显式模拟回退，并在 CSV 中标记为 reader_simulation。
