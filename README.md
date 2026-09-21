# NiNi SlimeNRF Tracker 固件

基于 SlimeNRF 生态的 nRF52833 全身追踪器固件。

## 项目来源

本固件沿袭并致谢以下项目：

* [SlimeVR/SlimeVR-Tracker-nRF](https://github.com/SlimeVR/SlimeVR-Tracker-nRF) —— 官方上游，SlimeVR 生态的奠基工作
* [LyallUlric/Stacked-SmolSlime](https://github.com/LyallUlric/Stacked-SmolSlime) —— 叠层 Promicro 路线
* [jitingcn/SlimeVR-Tracker-nRF](https://github.com/jitingcn/SlimeVR-Tracker-nRF) —— 本仓的直接基底（dev @ ad138bf，含 VQF 调参、TDMA、ESB OTA、在线磁校准、校准/静息/按键/电源事件上报等大量改进）

本仓在其上做移植与传感器驱动补充，上游演进会持续跟进合并。

## 主要差异

* **ICM-40608 驱动**：16 字节 FIFO、±16g/±2000dps 量程、陀螺/加计双 AAF 抗混叠 + UI 低通调优
* **QMC5883P 磁力计驱动**（QST 车规线，上游无）：0x2C 地址、8G 档 3750 LSB/G、原生 Single 模式、Suspend 中转、饱和拒收
* **MMC5983MA 调优**：SET/RESET 时序 500µs、100Hz 档低噪声带宽、软件饱和检测
* **ICM-42686/42688 滤波配置**：OFF 窗口写入 UI 低通 + 陀螺 AAF，补清 INT\_ASYNC\_RESET
* **IMUCLK（pwmclock）门控**：32.768kHz 时钟输出默认关闭，探测到 ICM-4268x/45686 自动开启（驱动自带探活回退），亦可用 `pwmclock on|off|auto` 手动控制

## SDK 与编译环境

`west.yml` 选定 [jitingcn/sdk-nrf](https://github.com/jitingcn/sdk-nrf) `v3.4-branch`（跟随分支；基于官方 NCS v3.4.0）。

构建需要 **Zephyr SDK 1.0.1 GNU**（`zephyr/gnu`，GCC 14.3.0）与 **Python 3.12**；固件用 Picolibc，CI 在 Ubuntu 24.04 上跑。固件依赖该 SDK fork 的 ESB 扩展与 USB 修复，官方 NCS 不能直接替换。

```bash
west init -l app
west update
export ZEPHYR\_SDK\_INSTALL\_DIR=/opt/zephyr-sdk-1.0.1
west build -b nini\_slimevr\_5883\_uf2 -d build --sysbuild --pristine -s app -- \\
  -DBOARD\_ROOT=$PWD/app
# 产物：build/app/zephyr/zephyr.uf2 / .hex / .elf
```

## LED 状态机（三通道并行渲染）

固件带**两套状态分配表**，console 切换（重启保持）：

```
ledmode            # 查看当前模式
ledmode debug      # 切到调试表（闪烁族：节拍分明，一眼锁定）
ledmode daily      # 切回日常表（呼吸族：安静和谐，默认）
ledbright          # 查看全局亮度
ledbright 30       # 全局亮度 30%（5-100，一改全改，重启保持）
```

### 日常表（呼吸族·默认）

| 状态 | 灯 | 表现 |
|---|---|---|
| 工作正常 | 绿 | 慢呼吸 10s（2s 升 2s 降 6s 灭），峰 25% |
| 链路已连 | 蓝 | 心跳呼吸 10s 与绿错峰 2.5s（两灯交错起伏） |
| 未配对/搜台 | 蓝 | 双短呼吸 4s |
| 对频成功 | 绿 | 单次渐亮渐灭 1.2s |
| 充电中 | 琥珀 | 低亮度常亮（15%±5% 微呼吸）直到充满 |
| 充满（插线） | 绿 | 5% 浅常亮；无 USB 通讯 3 分钟后全灭 |
| 低电 <10% | 琥珀 | 虚弱浅呼吸（1.5s 起伏 4.5s 灭） |
| OTA 中 | 琥珀 | 流动快呼吸 1.2s |
| 磁校准中 | 红→绿 | 进度写在颜色里（红满→绿满连续插值+微呼吸） |
| 错误 | 红（独占） | 每 5s 一次深呼吸 |
| 按住按键 2s | 绿 | 快闪预告关机（3s 执行） |
| 关机 | 全彩 | 渐灭 ~1.2s |
| WOM 睡眠 | — | 全灭 |

### 调试表（闪烁族）

| 状态 | 灯 | 表现 |
|---|---|---|
| 工作正常 | 绿 | 300ms blip/10s |
| 链路已连 | 蓝 | 300ms blip/10s（与绿错相） |
| 未配对 | 蓝 | 快闪 100/900 |
| 对频成功 | 绿 | 4 连闪 |
| 充电中 | 琥珀 | 30% 常亮 |
| 充满 | 绿 | 20% 常亮 → 无通讯 3 分钟后全灭 |
| 低电 | 琥珀 | 双闪（150/150/150/600） |
| OTA | 琥珀 | 快闪 100/100 |
| 磁校准 | 红→绿 | 颜色即进度，500ms 硬闪 |
| 错误 | 三色 | 红→绿→蓝轮播各 500ms |
| 长按 2s | 绿 | 快闪 100/100 |

Receiver 侧差异：绿=在岗呼吸（10s）；蓝=通讯域——已连心跳呼吸（日常表峰值随已连台数 1-10 台从 25%→55% 渐满，调试表连跳次数=台数）；配对模式蓝双短呼吸；新 tracker 入网绿渐亮确认。

Bootloader：蓝=三态（3s 柔呼吸等待 U 盘 / 60% 常亮就绪 / 5Hz 快闪写入中）；红/绿不参与。

## 许可

沿袭上游 Apache-2.0 / MIT 双许可，见 `LICENSE-APACHE` / `LICENSE-MIT`。

