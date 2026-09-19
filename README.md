# NiNi SlimeNRF tracker 固件

基于 SlimeNRF 生态的 nRF52833 全身追踪器固件。

## 项目来源

本固件沿袭并致谢以下项目：

* [SlimeVR/SlimeVR-Tracker-nRF](https://github.com/SlimeVR/SlimeVR-Tracker-nRF) —— 官方上游，SlimeVR 生态的奠基工作
* [LyallUlric/Stacked-SmolSlime](https://github.com/LyallUlric/Stacked-SmolSlime) —— 叠层 Promicro 路线
* [jitingcn/SlimeVR-Tracker-nRF](https://github.com/jitingcn/SlimeVR-Tracker-nRF) —— 本仓的直接基底（dev @ ad138bf，含 VQF 调参、TDMA、ESB OTA、在线磁校准、校准/静息/按键/电源事件上报等大量改进）

本仓在其上做 5883 板移植与传感器驱动补充，上游演进会持续跟进合并。

## 与 jiting 版的主要差异

* **ICM-40608 驱动**：16 字节 FIFO、±16g/±2000dps 量程、陀螺/加计双 AAF 抗混叠 + UI 低通调优
* **QMC5883P 磁力计驱动**（QST 车规线，上游无）：0x2C 地址、8G 档 3750 LSB/G、原生 Single 模式、Suspend 中转、饱和拒收
* **MMC5983MA 调优**：SET/RESET 时序 500µs、100Hz 档低噪声带宽、软件饱和检测
* **ICM-42686/42688 滤波配置**：OFF 窗口写入 UI 低通 + 陀螺 AAF，补清 INT\_ASYNC\_RESET
* **IMUCLK（pwmclock）门控**：32.768kHz 时钟输出默认关闭，探测到 ICM-4268x/45686 自动开启（驱动自带探活回退），亦可用 `pwmclock on|off|auto` 手动控制

## SDK 与编译环境

`west.yml` 选定 [jitingcn/sdk-nrf](https://github.com/jitingcn/sdk-nrf) `v3.4-branch`（pin ab62f8df，基于官方 NCS v3.4.0）。

构建需要 **Zephyr SDK 1.0.1 GNU**（`zephyr/gnu`，GCC 14.3.0）与 **Python 3.12**；固件用 Picolibc，CI 在 Ubuntu 24.04 上跑。固件依赖该 SDK fork 的 ESB 扩展与 USB 修复，官方 NCS 不能直接替换。

```bash
west init -l app
west update
export ZEPHYR\_SDK\_INSTALL\_DIR=/opt/zephyr-sdk-1.0.1
west build -b nini\_slimevr\_5883\_uf2 -d build --sysbuild --pristine -s app -- \\
  -DBOARD\_ROOT=$PWD/app
# 产物：build/app/zephyr/zephyr.uf2 / .hex / .elf
```

## LED 状态（当前为上游默认行为，状态机定制开发中）

|场景|表现|
|-|-|
|正常工作|蓝心跳（亮 300ms / 10s 周期）|
|对频中（开机长按 1-5s 松开）|蓝短闪（100ms 亮/900ms 灭）|
|对频成功|绿 4 连闪|
|充电中|琥珀呼吸（5s 周期，红 60%+绿 40%）|
|充满|绿 20% 亮度常亮|
|低电（<10%）|琥珀暗闪（500/500）|
|ESB OTA 中|琥珀快闪（100/100）|
|传感器/连接/系统错误|红每 5s 闪 2/3/4 次（多错轮播）|
|按住按键|蓝常亮|
|关机|蓝渐灭（约 1s）后全灭|
|WOM 睡眠|全灭|

Bootloader 侧：DFU 未挂载 U 盘 = 蓝快呼吸（300ms）；挂载后 = 蓝慢呼吸（3s）；写入中 = 蓝急闪（100ms）；红灯 2s 周期呼吸信标。

## 许可

沿袭上游 Apache-2.0 / MIT 双许可，见 `LICENSE-APACHE` / `LICENSE-MIT`。

