# lab-Micro_power_consumption_backscatter

### 基于 KL03 微功耗backscatter节点设计
#### 使用 NXP 1.x.x SDK 开发
- 该版本SDK 提供了几个 DEMO 供开发参考，兼容支持 IAR 和 MDK 开发环境
- 默认 device 为 MKL0332xxx4 24 pin

##### 该版本问题
- 常用的 MDK 工程为 UV4, 且其默认 device 选型的 vendor 仍为 Freescale, 若使用 NXP 的 DFP 强制更换 device, 编译不通过
- 最新版本SDK 为 v2.x.x

> update @ 02/15
---
