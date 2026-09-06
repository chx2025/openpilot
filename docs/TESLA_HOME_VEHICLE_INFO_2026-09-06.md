# Tesla 主页车辆信息与氛围灯测试

当前工作区分支：`dev-sp-egpu`。修改设备主页原 comma prime「立即升级」卡片，网页本地控制台也显示同一份车辆摘要。

## 读数

- 电量：VEH `0x292 BMS_socStatus / BMS_socUI`，显示 `%`，超过 100 的编码无效。
- 续航：接入 VEH `0x33A UI_range / UI_ratedRange`。本地 TCAN Model Y schema 未标单位，所以显示原值并标注「单位未标注」，没有把英里或公里猜测伪装成 km。需要与同一时刻原车屏幕读数核对后补换算。
- 胎压：左前、右前、左后、右后，单位 bar，优先原车显示压力，缺失时回退直接传感器压力；保留告警。没有将 last-known 压力当实时值。
- 里程：`DI_odometer`，km；累计充放电：`BMS_kwhCounter`，kWh。
- 电耗：未发现可信的原厂实时或行程平均电耗信号。`UI_whpm` 是续航标定耗电率，累计 kWh 也不能直接当作百公里电耗，因此主页取消电耗显示。
- 主页新增 TPMS 传感器电压、四轮刹车温度、高压电池电压/电流/功率/接触器与状态，并直接显示总里程及累计充放电。
- 数据解码沿用网页独立 CAN 观察器，不改变 CarInterface 的有效性或控制输入。旧数据按原观察器的五秒时效退为缺失。

SOC/range 位定义来自本地保存的 `tools/tesla_party_can_ble/ios/TeslaPartyBLE/tesla_modely_tcan_schema.json`（其元数据来源 https://tcan.latency.is/?plat=ModelY）。运行时只依赖已加入 opendbc 的两条 DBC 定义，不依赖该未跟踪目录。

## 三秒红色测试

- 支持 offroad 和 onroad 测试，不依赖挡位或车速。每次点击只选择一侧，以 10 Hz 连续请求 3 秒，最多 30 帧。控制循环延迟不会通过突发补发追赶。
- 报文：`0x679 UI_ambientLightingCtrls`，VEH / bus 1，8 字节；RGB 固定 `255 / 0 / 0`，启用状态 ON、即时切换、关闭音频可视化。
- 左侧目标：现有 HW4 DBC 的 DOORFL / DOORRL / IPFL；右侧目标：DOORFR / DOORRR / IPFR。按钮不会同时选择两侧。目标位的实车效果仍待验证。
- 使用新鲜原车模板，测试亮度固定为 100，并保持无关位。挡位和车速不作为测试门槛。
- 请求通过 Params 交给现有 card 发送线程，避免创建第二个 `sendcan` publisher。请求超时、过期、重复与并发有界处理。
- Panda 检查固定红色、新鲜原车 0x679 模板、单侧目标、速率及三秒/30 帧上限。offroad 临时使用 `noOutput(param=1)`，该子模式的 TX 白名单只有 `0x679 / bus 1 / DLC 7`；结束后恢复普通 `noOutput(param=0)`。
- 显示已提交帧数与 Panda 回显帧数；拒绝或无回显都不声称实车变色。原车报文仍可能覆盖红色。
- 普通 `noOutput(param=0)` 仍禁止所有发送；网页测试期间临时使用只放行固定红色 0x679 的 `noOutput(param=1)`，结束即恢复。

## 盲区氛围灯提醒

- 使用 CarState 已有的 `leftBlindspot` / `rightBlindspot`，来源为原车 `DAS_status` 的 `DAS_blindSpotRearLeft` / `DAS_blindSpotRearRight`。
- 只把原厂枚举 1、2（WARNING_LEVEL_1 / WARNING_LEVEL_2）视为占用；0 是无告警，3 是 SNA，不触发灯光。
- 左侧占用只选择左前门、左后门和左仪表台；右侧对应右边；两侧同时占用时同时选择两侧。
- 每 100 ms 在固定红色亮度 100 与 0 之间切换，形成约 5 Hz 的完整闪烁周期。盲区清除后立即停止发送，由约 2 Hz 的原车 0x679 恢复原色。
- 一次连续占用最多闪烁 15 秒、150 帧；必须先清除盲区状态才能重新开始，避免故障信号导致无限发送。
- Panda 只接受新鲜原车模板、0x679 / bus 1 / DLC 7、红色、亮度 0/100 和左/右/双侧目标，其他颜色、目标、长度、总线及超限频率全部拒绝。

## 部署与实车核验

需要同步主项目与 `opendbc_repo` 修改、重新构建 Params 与 Panda 固件，并重启相关进程。只更新网页/Python 不足以放行 `0x679`；旧固件会拒绝。此任务没有刷写或向实车发送报文。

实车先在原屏设置非红色且开启氛围灯，点击左侧红色 3 秒，观察右侧是否保持原色。用原车界面恢复初始颜色后，再测试右侧。这样可区分单侧控制、两侧一起变化、原车覆盖以及完全无响应。

## 验证与预览

Python 解码、摘要计算、三秒发送状态机、card 接入、HTTP 路由、输入限制、Panda safety、中文字符覆盖均有测试。主页截图使用模拟数据，不能作为 CAN 实车证据：`artifacts/tesla_home_vehicle/home-preview.png`。

扩充现有中文字体时保留原有字符和图标，仅补充缺失字形；字形来源为本地另一工作树保存的 Noto Sans CJK SC Regular 2.004 原版字体（SIL OFL 1.1），无需在设备上加载完整版字体。

广泛 safety 测试有 7,541 项通过、3,104 项按套件规则跳过；3 个 MISRA mutation 子测试未通过，其脚本先运行 uv 环境同步，在当前沙箱下被缓存目录权限阻止。另用现有 cppcheck 对当前代码及 HEAD 基线分别运行检查，两者均有同样 6 条既有告警（9.3 / 10.4 / 15.5），此次修改没有增加告警；因此不能声称分支完整 MISRA 检查通过。

最终定向回归：528 passed、135 skipped、225 subtests passed；本地 HTTP 接口与权限回归：21 passed。补充的三秒发送上限及字体覆盖回归：15 passed。Ruff、JavaScript 语法及 git diff 空白检查通过。

## 视觉调整

主页车辆卡片采用深色仪表布局：电量大字与进度条、俯视车辆图、四轮胎压及传感器电压、总里程、累计充放电、高压电池信息和四轮刹车温度。主页不再显示氛围灯按钮；网页保留测试入口。详细状态可点击主页右上角信息图标查看。

已检查桌面及 390px 手机视口，使用模拟接口验证按钮与详情展开；字体覆盖 5 项、HTTP/权限回归 21 项通过。新预览：`artifacts/tesla_home_vehicle/home-preview-beautified.png`（模拟数据）。
