# Tesla 主页车辆信息与氛围灯测试

当前工作区分支：`dev-sp-egpu`。修改设备主页原 comma prime「立即升级」卡片，网页本地控制台也显示同一份车辆摘要。

## 读数

- 电量：VEH `0x292 BMS_socStatus / BMS_socUI`，显示 `%`，超过 100 的编码无效。
- 续航：接入 VEH `0x33A UI_range / UI_ratedRange`。本地 TCAN Model Y schema 未标单位，所以显示原值并标注「单位未标注」，没有把英里或公里猜测伪装成 km。需要与同一时刻原车屏幕读数核对后补换算。
- 胎压：左前、右前、左后、右后，单位 bar，优先原车显示压力，缺失时回退直接传感器压力；保留告警。没有将 last-known 压力当实时值。
- 里程：`DI_odometer`，km；累计充放电：`BMS_kwhCounter`，kWh。
- 观测电耗：连续观测区间 `(放电增量 - 充电增量) / 里程增量 × 100`，kWh/100 km。至少 1 km 后显示；请求中断超过 10 秒、计数回退或数据缺失时重置。该值不是原车行程表平均电耗，也不能把累计 kWh 直接当成百公里电耗。
- 数据解码沿用网页独立 CAN 观察器，不改变 CarInterface 的有效性或控制输入。旧数据按原观察器的五秒时效退为缺失。

SOC/range 位定义来自本地保存的 `tools/tesla_party_can_ble/ios/TeslaPartyBLE/tesla_modely_tcan_schema.json`（其元数据来源 https://tcan.latency.is/?plat=ModelY）。运行时只依赖已加入 opendbc 的两条 DBC 定义，不依赖该未跟踪目录。

## 三秒红色测试

- 支持 onroad、P 挡静止时测试。每次点击只选择一侧，以 10 Hz 连续请求 3 秒，最多 30 帧。控制循环延迟不会通过突发补发追赶。
- 报文：`0x679 UI_ambientLightingCtrls`，VEH / bus 1，8 字节；RGB 固定 `255 / 0 / 0`，启用状态 ON、即时切换、关闭音频可视化。
- 左侧目标：现有 HW4 DBC 的 DOORFL / DOORRL / IPFL；右侧目标：DOORFR / DOORRR / IPFR。按钮不会同时选择两侧。目标位的实车效果仍待验证。
- 使用新鲜原车模板，保持原有亮度及无关位。需要原车亮度在 1–100 之间。每次发送前重新检查新鲜挡位、零车速与氛围灯模板；异常提前停止。
- 请求通过 Params 交给现有 card 发送线程，避免创建第二个 `sendcan` publisher。请求超时、过期、重复与并发有界处理。
- Panda 新增固定红色报文检查、新鲜 P 挡/零车速检查、速率及三秒/30 帧上限。辅助观察钩子只缓存氛围灯/挡位，不把可选灯光报文加入驾驶必需 RX 检查，避免缺少灯光报文导致驾驶控制掉线。
- 显示已提交帧数与 Panda 回显帧数；拒绝或无回显都不声称实车变色。原车报文仍可能覆盖红色。
- `noOutput` 保持禁止发送；设备离线主页按钮不可用。实车需要运行 card 和 Tesla safety，建议从网页操作。

## 部署与实车核验

需要同步主项目与 `opendbc_repo` 修改、重新构建 Params 与 Panda 固件，并重启相关进程。只更新网页/Python 不足以放行 `0x679`；旧固件会拒绝。此任务没有刷写或向实车发送报文。

实车先在原屏设置非红色且开启氛围灯；保持 P 挡静止、onroad，点击左侧红色 3 秒，观察右侧是否保持原色。用原车界面恢复初始颜色后，再测试右侧。这样可区分单侧控制、两侧一起变化、原车覆盖以及完全无响应。

## 验证与预览

Python 解码、摘要计算、三秒发送状态机、card 接入、HTTP 路由、输入限制、Panda safety、中文字符覆盖均有测试。主页截图使用模拟数据，不能作为 CAN 实车证据：`artifacts/tesla_home_vehicle/home-preview.png`。

扩充现有中文字体时保留原有字符和图标，仅补充缺失字形；字形来源为本地另一工作树保存的 Noto Sans CJK SC Regular 2.004 原版字体（SIL OFL 1.1），无需在设备上加载完整版字体。

广泛 safety 测试有 7,541 项通过、3,104 项按套件规则跳过；3 个 MISRA mutation 子测试未通过，其脚本先运行 uv 环境同步，在当前沙箱下被缓存目录权限阻止。另用现有 cppcheck 对当前代码及 HEAD 基线分别运行检查，两者均有同样 6 条既有告警（9.3 / 10.4 / 15.5），此次修改没有增加告警；因此不能声称分支完整 MISRA 检查通过。

最终定向回归：528 passed、135 skipped、225 subtests passed；本地 HTTP 接口与权限回归：21 passed。补充的三秒发送上限及字体覆盖回归：15 passed。Ruff、JavaScript 语法及 git diff 空白检查通过。

## 视觉调整

主页和网页车辆卡片改为深色仪表布局：电量大字与进度条、俯视车辆图与四轮胎压、图标化里程/电耗、左右氛围灯按钮。累计数据和测试说明默认折叠，点击主页右上角信息图标或网页「详细信息」查看。测试反馈精简，完整结果留在详情。CAN 解码和三秒发送逻辑未变。

已检查桌面及 390px 手机视口，使用模拟接口验证按钮与详情展开；字体覆盖 5 项、HTTP/权限回归 21 项通过。新预览：`artifacts/tesla_home_vehicle/home-preview-beautified.png`（模拟数据）。
