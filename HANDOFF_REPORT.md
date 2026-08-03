# 交接报告

## 项目整体描述

本项目是 RPent 的独立传统瓶体抓取子项目，用于 AIR G1 双臂机器人瓶装物侧抓。它嵌套部署到父项目目录但不作
submodule；父项目以独立提交接入其规划服务。核心链路为：双目矫正 → CREStereo 深度 → YOLO-World/SAM2 →
瓶心估计 → 受限侧抓姿态候选 → 关节空间预抓取 → 水平直线插入 → TRAC-IK 连续逆解 → 碰撞检查 → 执行和
抓取验证。

## 核心功能与接口

- 高层兼容接口 `search_object`/`approach_object`/`pick_object`/`verify_grasp`，核心替代目标是 RPent 的 `pick_object`。本项目提供无运动持久规划服务，父项目负责后端选择、控制器所有权、计划复验和执行；`pick_object` 已锁定原仅关键字参数及主要结果字段。
- 细粒度接口：`compute_depth_crestereo`、`segment_object`、`mask_depth_to_pointcloud`、`pointcloud_to_body`、规划和执行。
- `offline` 完整模拟闭环；`shadow` 用真实感知/IK但绝不发运动命令；`live` 必须同时通过运动授权、三项标定和碰撞检查门禁。抓取顺序固定为张手、预抓取、抓取位、闭合/接触、抬升、后撤，构造器与接口都不自动回零。

## 主要模块与目录

- `src/rpent_traditional_grasp/`：`api.py` 四接口编排与安全门禁；`config.py`/`gripper.py` 配置与 Dex1-1 规格校验；`stereo.py` 标定/矫正/CREStereo 适配与执行提供者诊断；`perception.py` YOLO-World 检测与 SAM2 框提示分割；`geometry.py` 商标带鲁棒深度与瓶心估计；`planning.py` ±30° 组合侧抓与插值；`ik.py` 持久 TRAC-IK 与 FK 残差；`collision.py` 复用父项目检查器的自碰撞预筛（配 `scripts/collision_worker.py`）；`diagnostics.py` 无运动可达性诊断；`execution.py` 碰撞/执行器协议与接触证据；`thor.py` 相机适配、注入式 CapX 与感知设备选择；`xyz.py` 图片到 TCP 验收；`visualization.py` 双后端 TCP 对比。`native/` 官方 TRAC-IK 与 G1 文本链求解器；`robot/` 已核对 URDF 与运动链；`scripts/` 运动链导出、原生构建、Thor 与本机影子入口；`tests/` 覆盖几何、配置、安全、可达性、接口闭环与真实原生 IK。

## 技术栈与外部依赖

- Python 3.10–3.12、NumPy、OpenCV、Ultralytics、SAM2、ONNX Runtime；C++17、CMake/Ninja、Eigen3、Orocos KDL、NLopt。`traclabs/trac_ik@90162ac2...`（BSD-3-Clause，源码在 `native/vendor/`）；reBot 仅参考流程概念，未复制其源码和 GraspNet，详见 `UPSTREAM.md`。
- 模型权重与第三方推理代码均为外部资源，不进入本仓库；本机放在 `/Users/firmiana/project/rpent-models/`（权重分三个子目录，`vendor/` 存两个公开仓库）。

## 运行入口、配置与数据流

- 配置入口：Thor 用 `thor.example.json`，本机用 `macos.example.json`，两份共用 `config/` 下同一批标定文件。感知设备由 `thor.py` 按 cuda→mps→cpu 自动选，`RPENT_TRADITIONAL_GRASP_DEVICE` 可覆盖。原生入口 `native/build/g1_trac_ik`（每臂一个求解进程）；影子入口 `scripts/run_thor_shadow.py`，默认只运行 `search`。
- 日志默认 INFO；关键配置、感知、执行提供者、IK、可达性、门禁和执行均保留上下文及异常链；服务在文件描述符层隔离 JSON 回复与第三方输出，父项目另有限量抗噪读取。在线抓取把原始/校正双目、SAM2 框选图、掩码和叠加图保存到父项目单次运行目录，并用帧 SHA-256、输入框、候选分数及掩码框串联 INFO 日志。
- 三维点先位于左相机坐标系，再通过配置外参变换到 `torso_link` 机身坐标系。

## 常用命令

```bash
PYTHONPATH=src python -m pytest -q
./scripts/build_native_thor.sh
PYTHONPATH=src python scripts/diagnose_ik_reachability.py --help
# 影子/XYZ 入口在 thor 与 macos 上同名成对，参数一致：
#   run_{thor,macos}_image_shadow.sh --left-image L.jpg --right-image R.jpg --operation pick
#   run_{thor,macos}_image_xyz.sh    --left-image L.jpg --right-image R.jpg --output-json /tmp/x.json
#   run_thor_image_gripper_xyz.sh    --left-image L.jpg --right-image R.jpg

# 复刻 11:50 现场场景的仿真（成功；换 0.5093 0.0856 0.0517 即复刻 11:03 的失败）。解释器为
# .venvs/reBot-DevArm-Grasp-macos-brew/bin/python；要开可视化窗口改用同目录 mjpython 并加 --viewer
cd ../../reBot-DevArm-Grasp && python scripts/sim_traditional_grasp.py \
  --config ../.config/reBot-DevArm-Grasp/g1d_mujoco.macos.yaml \
  --arm left --bottle-torso-xyz-m 0.4778 0.0666 0.0564 \
  --rest-bottle-on-table --output-dir outputs/sim_1150_success
```

## 当前状态与已验证事实

- **`search_object` 服务分派已入库（2026-08-03，修掉一处跨仓同步风险）**。父项目**已提交**的
  `traditional_grasp_backend.search_object` 向规划服务发 `operation="search_object"`，而子项目**已提交**的
  `service.py` 只认 `ping`/`close`/`plan_pick`，遇到该操作直接抛 `不支持的规划服务操作`；链路能在 thor 上跑
  **仅仅因为那 51 行一直躺在服务器工作区没提交**，任何 `git checkout .` 或重新 clone 都会让 LLM 拿不到 bbox
  （11:50 促成抓取成功的 bbox 正来自这条链路）。已审阅提交并补 4 个测试锁住契约（分派与元数据、空
  `object_prompt` 拒绝、SAM2 `last_artifacts` 写入与**逐帧清零**、分割器无该属性时仍正常返回）；同时补上诊断
  图路径回传，父项目 `_normalize_search_result` 早已在读这四个键、此前恒为 `None`。**thor 已同步到同一提交**
  （树哈希一致、工作区干净），原 51 行经逐字节核对后 stash 留底，并在生产解释器上复验了上述四项。
- 历史结论（详见 Git 历史）：macOS/Thor 原生 TRAC-IK 可用；G1 URDF 与 Thor `xr_teleoperate` 逐字节一致；CapX `ArmController` 构造会自动回零、只接受上层注入控制器；Dex1-1 名义 TCP `0.150215608966 m`（两指内表面间的面积加权质心）；肩部 `[0.004,±0.1002,0.2478]`，杆长上限 `0.560610 m`。腰部三关节在 `torso_link` **之下**，规划器不使用，遥操也未用。
- 缓存图本机基准（**当前配置为旧标定**）：现场图 `raw_*_20260801_164911` 瓶心 `[0.5443,0.0075,0.0249]`、深度 0.6073、瓶径 0.0624（与 thor 实测一致）；新标定下为 `[0.5937,0.0841,-0.0492]`、0.6872、0.0590。桌面图 `left/right.jpg` 新标定瓶径 0.0644、旧标定 0.0691。
- 棋盘 PnP（25 mm 方格，重投影 0.3 px）反解：设计名义外参下桌面法向与竖直差 `5.63°`（瓶心变化 `68 mm`），是外参旋转误差上界。新标定 fx 比旧的高 **20.5%**——同一台相机 fx 不应变，说明**至少一次标定病态**（标定板在画面里只占 0.6–0.9%）。极线错位只检验旋转、**不检验尺度**；棋盘也不能验尺度，它就是标定所用的板，属循环论证。
- **已切回旧标定**（2026-08-02，用户决定）：0801 现场与本机复现两次独立测量都报瓶径 0.062 m；旧标定把瓶心拉到左肩距 0.5918 m，距实测无解边界仅差 19 mm（新标定差 88 mm）。标定与外参**成对**切回 legacy（外参必须同切，20260801 外参由新标定的矫正旋转 R1 复合而来）。**保留疑点**：桌面缓存图那组旧标定瓶径偏 +11.5%，未解释。
- **可达性预检已补进生产路径**（2026-08-01）：原先两处用**机身原点**距离对 `max_reach_m=0.78` 判断，而手臂挂在肩上，现按肩距判定。硬拒绝**仅当两项标定验收布尔量同时为真时生效**，否则只记 WARNING 照常求解——遥操已抓到被判超界的瓶子。
- 侧抓可解半径远严于杆长上界。种子姿态**只影响 TRAC-IK 收敛、不改变解是否存在**：11:50 帧 60 个随机种子中 8 个成功（零位在内），11:03 帧 60 个全败；肩俯仰 0→0.80 rad 每档 24 候选在 11:03 帧亦全无解，故**零位本身已是好种子**。曾误报"±8 mm 窄成功窗口"与"太近抓不住"，**已作废**——那是仿真前移瓶子时桌子没跟着移、瓶心越过桌沿自由落体所致，仿真侧已修。
- **真实可抓边界（现场场景，左臂，瓶子稳放桌面）**：左肩距 0.5730 m 无逆解；0.5392 / 0.5226 / 0.4902 / 0.4591 m **全部成功**（双指接触、抬升 91–93 mm）。**近端无下界**，边界在 0.5392～0.5730 m；`side_grasp_planning_radius_m=0.54` 依据即"已验证的最远可抓距离"。现场帧亦印证：0.5121 m 可规划、0.5422 m 全无解。
  **抓取失败与侧抓姿态候选无关**（已验证）：各成功档选中的候选各不相同，与成败无相关，唯一失败是远端真无逆解；姿态惩罚 `0.02×偏角²`（30° 仅 0.0055）比关节行程平方和小两个数量级，排序几乎只看关节移动量。
- **"TRAC-IK chain 与 Pinocchio FK 差 11.3 cm"的外部结论已证伪**（2026-08-02）：本链 FK 在 q=0 得 `[0.353953,0.148633,0.051225]`，与对方算的 TRAC-IK 值**逐位一致**；差额完全分解为两项**定义差**——① TCP 不同（`L_ee` 是 wrist+0.05 m 遥操内部帧，本项目用 wrist+0.150216 m 的抓取中心，差 100.216 mm）；② **根坐标系不同**（对方 `pelvis`、本项目 `torso_link`，差额正是 URDF `waist_roll_joint` 原点 `[-0.0039635,0,0.044]`，加此偏移**精确到 1e-6 m** 复现对方数值）。故"改写 `g1_trac_ik.cpp` 从 URDF 建链"不改变任何数值。顺带复核外参：平移与 URDF `d435_joint` 只差 ~12 mm（误用 pelvis 系会差 44 mm），**确系 `torso_link` 系**。
- 经典算法替换已否决：SGBM 24 ms、GrabCut 292 ms（比 SAM2 慢 1.8 倍）、YOLO-World 无经典替代；SGBM 在现场图直接失败（MAD `76.6 mm` 超 25 mm 门限）。
- **CREStereo 跑 CPU 已修复**（2026-08-01，环境侧）：根因是 `yolo_world` conda 环境装了纯 CPU 版 onnxruntime，而同机唯一 GPU 轮子按 NumPy 1.x 编译、装进 NumPy 2.4.4 的 `yolo_world` 会 `ImportError`（已还原，备份在 `/home/aiot/backup_onnxruntime_cpu/`）。改用 `--system-site-packages` 叠加环境 `/home/aiot/wuxi/venvs/rpent-grasp-gpu` 只覆盖 numpy 1.26.4 与 GPU onnxruntime，`3544→49.2 ms`。`ExternalCREStereoBackend` 另加执行提供者自判：已在跑加速器就原样保留（保住 fp16 与引擎缓存），只落 CPU 且有 CUDA 才强切，否则 WARNING 回退。
- 本机模型部署已完成，影子链路不依赖服务器；三个权重（CREStereo ONNX 25 MB / YOLO-World `.pt` 140 MB / SAM2 176 MB）全来自公开发布源且体积与 Thor 一致，推理代码取 `ibaiGorordo/ONNX-CREStereo-Depth-Estimation` 与 `facebookresearch/sam2`。本机 `.pt` 走 `set_classes` 需 CLIP，Thor 走 `.engine` 不需要。环境 `.venvs/rpent-traditional-grasp-macos`。本机与 Thor 一致性：桌面缓存图瓶心欧氏差 `0.54 mm`、0802 现场图 `< 0.3 mm`（残差来自 Thor fp16 TensorRT 与本机 fp32 CPU）；本机全链路 8.2 s，**可离线复算任一现场帧的完整规划**——但按"注意事项"的工作方式，这只是服务器不在线时的临时手段，结论须回 thor 复验。

## 未确认、阻塞问题与下一步

- **2026-08-02 16:42：夹爪首次真正夹住瓶子。** 上层审计写 `failure_unverified`，但那是被下游到位判据
  卡住，**抓取本身成功**：执行顺序 张手 → `grasp`(index 21) → `close_gripper(require_contact=True)` →
  抬升 → 后撤；**本次没有 `close_gripper failed`**（15:35 空抓那次有），而闭合无接触必抛异常，故
  **夹爪确实合上并检测到持续接触**；失败在 **index=24 = `lift:3/10`**，已在 grasp 之后。经验偏置已生效
  （选右臂，目标由 `[0.4750,0.0705,0.0431]` 移到 `[0.4414,0.0969,0.0898]`）。**卡住原因是 CapX 到位判据的
  死区**：正常到位要 `误差<=0.03 且 速度<=0.05`，受阻到位要 `速度<=0.10 且 误差>=0.05 且 力矩偏差>=1.5Nm`，
  实测 `误差 0.0342 / 速度 0.0123` **落在 0.03 与 0.05 之间**，两条都不满足只能干等 5 s 超时。速度说明手臂
  已静止只是稳态位置有偏差——夹住瓶子后负载增加而 `sol_tauff` 的重力前馈按空臂算，PD 必然留下
  `稳态误差=负载力矩/kp`。**属父项目/CapX 侧，不是规划或几何问题。**
- **"缓慢下放后突然举起双手"已定位到 CapX 保活线程**（2026-08-02 现场观察）。`_keepalive_loop` 以 30 Hz
  **重发 `_hold_q`——最后一次指令目标，不是实测位置**，恢复时不插值，而 `_home_q` 为零位（"抬起前伸"）。
  故 **DDS 指令流一断双臂在重力下下垂，保活一恢复就把下垂前的目标当阶跃发出、瞬间弹回零位**（`_hold_q`
  是 14 维全臂向量，解释了为何是"双手"）。**与种子漂移同根因**——漂移正是在这段下垂窗口测到的（14:18 的
  0.724→0.672 rad 是被拉回的过程，16:42 复发 `max_drift=0.744`）；`_get_collision_checker()` 在 arm_service
  同进程构建 1257 个碰撞对耗时 4 s、持 GIL 饿死保活线程。**修法（属父项目/CapX）**：① 保活恢复先重置
  `_hold_q` 为实测位置再平滑过渡（**父项目已实施，未真机验证**）；② 碰撞检查器移出该进程或启动时构建
  （**未修**）；③ 放宽后段到位容差或让 `sol_tauff` 计入负载（**未修**）。
- **2026-08-02 15:28/15:35 首次真机执行：空抓并推开瓶子**（已被 16:42 取代）。安全链正确中止；自碰撞
  预筛丢弃 25/24 个候选后 `rank=1` 一次通过、`max_seed_drift_rad` 仅 0.0004/0.001 rad。**"阶跃参考"假说
  已排除**（规划侧 33 点连续、下发侧 `move_arm:797` 做 30 Hz smoothstep 并逐点等收敛）；遗留观察是
  `duration=0.12` 只有 3 个插值子步、逐点加减速导致顿挫、容差 0.03 rad 约合 15 mm 落点散布——不足以解释
  87 mm 偏差，故判定几何偏置为主嫌，16:42 已证实。
- **经验抓取偏置已接入（2026-08-02，用户决定先试）**：`left/right_empirical_grasp_offset_m`，原始值左臂
  `[+0.0146,+0.0874,+0.0013]`、右臂 `[-0.0047,+0.0388,+0.0301]` m，在与上游 `build_horizontal_contact_pose`
  **完全相同的接触系**（x=肩→瓶水平方向，y=其左，z=上）中施加；该标定接触系原点在瓶心后 0.1034 m，本项目
  等效点在瓶心后 0.100216 m，修正量即二者之差。**thor 实测**：15:35 那帧抓取点 body 位移
  `[+19.8,+86.4,+1.3]` mm，左肩距 0.4798→0.5004 m，仍在 0.540 内。
  - **这不是 TCP 标定，不得据此翻转 `gripper_tcp_calibration_validated`。** `calibrate_tcp_from_vla.py:132-142`
    的"物体中心真值"来自**同一条被质疑的双目链路**，把 TCP 误差与感知误差**绑死、无法分离**；左右臂
    侧向差 87 vs 39 mm，镜像对称的同款硬件不可能差 48 mm，**这个不对称本身就说明它测的不是几何**
    （残差均值 19–20 mm、样本仅 5–8 个）。**风险**：当前摆位可能能抓，换位置会失效且掩盖真正的标定
    问题。故做成独立可关闭项、全零即关闭、不并入 `tip_offset_m`，施加时落 WARNING。
  - **已扣除两条链路的瓶心定义差**（2026-08-02，用户决定）。legacy 真值是 `median(整个掩码点云)`，
    掩码只看得见前半边故中位数偏前；本项目取商标带鲁棒前表面深度再**推半个瓶径**到几何中心。11:03 与
    11:50 两帧实测差 **24.6 mm** 且分解一致：沿接近 **+17.9 mm**、侧向 +2.5 mm、竖直 **-16.6 mm**。
    配置值已改为左臂 `[-0.0033,+0.0849,+0.0179]`、右臂 `[-0.0226,+0.0363,+0.0467]` m。
  - **关键分野**：轴向与竖直分量是 legacy 感知定义的产物、不可直接迁移；但**侧向的 87 mm 几乎不受
    影响**（只差 2.5 mm），两链在侧向上一致——故侧向要么是真实夹爪几何偏置，要么是**两链共有**的侧向
    偏差（如共用的相机-机身外参偏航），正对应现场目视的"夹爪停在瓶子右侧"。**前提**：24.6 mm 只在同一
    瓶、相近视角的两帧上测过，遥操标定用的是 `VLA_record4` 的其他场景，**摆位样本仍只有一种**。
  - **仍应做的**：用尺直接量夹爪两指内表面中点相对腕部的实际位置（轴向+侧向），这是唯一**不经过感知
    链**、能把 TCP 误差单独摘出来的办法；若尺子证明 TCP 本来就对，则那 87 mm 属感知误差，该去修外参。
- **2026-08-02 14:18(本项目) / 14:21(legacy) 同场景 A/B**：本项目规划成功但三个候选全被父项目
  `validate_traditional_grasp_path` 拒绝——rank1 自碰撞（thor 上用父项目检查器复验：39 点里 17 点撞
  `torso_link_0 <-> left_shoulder_yaw_link_0`，集中在桥接段与后撤段，**不在抓取位姿本身**），rank2/rank3
  是 `plan seed is stale`。**legacy 成功那次该校验调用 0 次**，也没有计划签名、TTL 和种子漂移检查——不是
  抓取质量之争，是**我们多了一整套安全门**。两个缺陷此后都已处理。
- **pick 后端归属判别（命名极易混淆，务必先看这条）**：父项目 `wuxi_adapter.py:421` `_pick_object_legacy`
  的错误串字面写着 `traditional grasp detector did not return a grasp message`，那是 **Contact-GraspNet
  时代的旧叫法**，比本项目还早。判别只能靠**唯一串**计数（`did not return a grasp message` /
  `contact_graspnet` / `required_base_advance_m` / 收到 `plan_pick`）：**11:03 = 0/0/2/1 与 14:18 → 本项目**；
  11:47 = 2/0/0/0、11:50 = 2/2/0/0、14:21 → legacy。**父项目从不记录当前 pick 后端模式**，这正是归属只能反推
  的根本原因，**建议父项目在 `wuxi_adapter._pick_backend_mode()` 落一条 INFO 并加进 `observe_scene`**。机制上
  无歧义：默认 `traditional-live`，进 legacy **必须显式 export**且无自动回退路径；`AIR_ROBOT_PICK_BACKEND`
  未见于任何脚本或 `.bashrc`，最可能是手工 export 后跨运行残留。
- **11:03 / 11:47 / 11:50 三次已结案**（原始数据在 gitignore 的 `logs/`）。11:47、11:50 的 pick 由旧
  Contact-GraspNet 承担，本项目只做 `search_object`；11:03 是唯一由本项目 pick 的，失败原因是目标恰在
  可解边界之外。① **离线复算与 MuJoCo 仿真在三个位置的成败与现场逐一吻合**：11:50 位（左肩距 0.5121）
  与 11:47 位（0.5099/0.5114）仿真均抓取成功，11:03 位（0.5422）离线与仿真均失败——**只差 30 mm 跨过
  边界，独立物理引擎给出同一分界**；同次 TRAC-IK 与 MuJoCo FK 交叉审计 641 采样最大误差 **4.0e-07 m**。
  ② **11:47 崩溃源于坏 bbox 触发硬失败**：LLM 目测的 `[268,204,312,294]` 落在空台面上，几何门禁按设计拒绝
  但本项目**抛异常而非回退**；11:50 的 LLM 调了不带 bbox 的 `search_object`，**促成对方抓取成功的 bbox 正是
  本项目给的**。③ **"把仿真的肩后收搬到实机"已实测否决**（见上文种子结论）；生产链路没有准备姿态，
  `prepare_arm` 只初始化控制器而构造即回零。④ **姿态网格明显不对称**，只有正俯角有解，一半候选是无效
  开销；11:50 的 3 次 0.1 m 前进原语**实际没移动底盘**。⑤ 仿真 viewer 开头"先动一下再停"是脚本固定准备段，非异常。
- **早期现场运行留档**：11:03 瓶心 `[0.5093,0.0856,0.0517]`、深度 0.5658 m、瓶径 0.0587 m，左肩距 0.5422 m 超实测规划半径 2.2 mm，24 个候选全 `no IK solution`；0801 旧标定 `depth=0.607m diameter=0.062m`，底盘全程未动，30 s 内深度由 0.607 变 0.527 m 是画面里的人把桌子推近约 80 mm。**瓶径真值仍存疑、暂缓**：现场量得瓶底 55.0 mm 且称同一瓶（与此前 62 mm 冲突），而本项目估的是中部商标带（掩码中部 38 px、瓶底 33 px，下收 13%），同一深度图量瓶底得 57.9 mm；**要定案需现场补量瓶身最宽处直径与瓶高**，在此之前不要用瓶径反推深度尺度。
- **手眼标定待办。** 遥操在底盘、躯干都不动、只用手臂的条件下抓到了本链路判为超界的瓶子，是唯一不依赖标定的硬证据。做法：手臂摆若干已知姿态各拍一组图，用正运动学 TCP 作真值求解——夹爪就是精度最高的靶标，**多姿态才能把尺度、旋转、平移分开**。`service.py` 已补 `current_q_rad` 数值日志。相机-机身外参目前是设计名义值复合，旋转偏差上界 `5.63°`（68 mm），平移无观测约束，验收前 `camera_to_body_validated` 保持 false。零位手臂当靶的初步测量（单张、目测 ±10–20 px，不足定论）：两套标定预测**都比实际夹爪高 30–50 px**，共模误差指向 URDF `d435_joint` 设计俯仰。
- **父项目 `traditional_grasp_backend.py` 的 `AIR_ROBOT_TRADITIONAL_GRASP_PYTHON` 默认值仍是 `yolo_world`**，2026-08-02 现场日志已实测证实：生产链路仍跑 **CPU 版 CREStereo**，单帧 4.3–4.9 s。子项目的自判逻辑正常工作（正确落 WARNING），缺的是父项目改默认解释器，需父项目单独提交。
- 仿真侧曾把瓶底埋进桌面、又曾让瓶子走出桌沿，均已修（`--rest-bottle-on-table` 同时调高度与水平位置，出生点随桌走，穿模硬拦截）；Oracle 真值输入下瓶心 z 仍偏高 23–30 mm，**仿真 yaml 仍内联新标定、与子项目已不一致**，仿真仓库另有 12 个提交未推送。**底盘不移动是硬约束**（用户 2026-08-02 明确）：遥操就是在底盘与躯干都不动、只用手臂的条件下抓到的，故 `required_base_advance_m` 只能作为"差多远"的诊断量，不能作为方案。尚未做在线相机采集回归；商标区域在反光、透明瓶、遮挡、低纹理下的深度成功率未实测。环境障碍物碰撞因缺场景模型未实现，实机前需清场和急停；Dex1-1 驱动量到毫米开口、接触阈值、TCP 六自由度外参也未做真机标定。`visualization.py` 与三个对比脚本为另一会话的未提交工作，未经本轮验证。
- 下一步（按价值）：**① 父项目修剩余两处 CapX 问题**（保活阶跃已修待真机验证；碰撞检查器仍需移出 arm_service 进程或启动时构建；抓取后段容差仍需计入负载）→ **② 用尺实测夹爪几何**（唯一能分离 TCP 误差与感知误差的手段，决定偏置是留是撤）→ **③ 若嫌动作顿挫，把 `AIR_ROBOT_TRADITIONAL_POINT_DURATION_S` 调大或减少路点**（当前每点只有 3 个插值子步且都要等收敛）→ ③ 显式 bbox 被拒时回退自带检测器 → ④ 加抓取深度参数让 TCP 可落在瓶心之前 → ⑤ 姿态网格向有解一侧加密 → ⑥ 现场补量瓶身最宽处直径与瓶高 → ⑦ 多姿态手眼标定 → ⑧ 父项目改 `AIR_ROBOT_TRADITIONAL_GRASP_PYTHON` 上 GPU → ⑨ 清场急停后限速小步真机验证。

## 注意事项

- **`ssh air-thor` 是主环境，事实认定以服务器上的运行结果为准**（用户 2026-08-03 明确）。真机、GPU 与完整部署只在 thor，本机没有硬件后端且推理精度不同（Thor fp16 TensorRT vs 本机 fp32 CPU），故本机**只承担服务器不在线时的部分验证**，其结论是临时参考、服务器恢复后须复验，落文时要显式标注。注意 thor 的 `yolo_world` 与 `rpent-grasp-gpu` 环境**都不装 pytest**，服务器侧验证要用生产解释器直接跑校验脚本。
- 父、子项目必须分别提交和同步；子项目不作 submodule，其路径由父项目本地排除规则隔离，不要在父项目提交中纳入 `traditional_grasp/` 内容。同步方向为本机提交 → `git push origin` → thor `git fetch && git merge --ff-only`；**thor 工作区若有未提交改动，merge 会被拒，必须先 `git stash` 留底**。
- 不得把示例配置直接切为 `live`，不得绕过碰撞检查和标定验证布尔门禁；TRAC-IK 只负责运动学求解，不提供碰撞安全保证。`macos.example.json` 与 `rpent-models/` 的绝对路径是本机部署事实，换机器须改；两个 `run_macos_*.sh` 支持用环境变量覆盖解释器与 SAM2 仓库。
