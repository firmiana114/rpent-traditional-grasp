# 交接报告

## 项目整体描述

本项目是 RPent 的独立传统瓶体抓取子项目，用于 AIR G1 双臂机器人瓶装物侧抓。
它嵌套部署到父项目目录但不作 submodule；父项目以独立提交接入其规划服务。
核心链路为：双目矫正 → CREStereo 深度 → YOLO-World/SAM2 → 瓶心估计 →
受限侧抓姿态候选 → 关节空间预抓取 → 水平直线插入 → TRAC-IK 连续逆解 →
碰撞检查 → 执行和抓取验证。

## 核心功能与接口

- 高层兼容接口 `search_object`/`approach_object`/`pick_object`/`verify_grasp`，核心替代目标是 RPent 的 `pick_object`。本项目提供无运动持久规划服务，父项目负责后端选择、控制器所有权、计划复验和执行；`pick_object` 已锁定原仅关键字参数及主要结果字段。
- 细粒度接口：`compute_depth_crestereo`、`segment_object`、`mask_depth_to_pointcloud`、`pointcloud_to_body`、规划和执行。
- `offline` 完整模拟闭环；`shadow` 用真实感知/IK但绝不发运动命令；`live` 必须同时通过运动授权、三项标定和碰撞检查门禁。抓取顺序固定为张手、预抓取、抓取位、闭合/接触、抬升、后撤，构造器与接口都不自动回零。

## 主要模块与目录

- `src/rpent_traditional_grasp/`：`api.py` 四接口编排与安全门禁；`config.py`/`gripper.py` 配置与 Dex1-1 规格校验；`stereo.py` 标定/矫正/CREStereo 适配与执行提供者诊断；`perception.py` YOLO-World 检测与 SAM2 框提示分割；`geometry.py` 商标带鲁棒深度与瓶心估计；`planning.py` ±30° 组合侧抓与插值；`ik.py` 持久 TRAC-IK 与 FK 残差；`diagnostics.py` 无运动可达性诊断；`execution.py` 碰撞/执行器协议与接触证据；`thor.py` 相机适配、注入式 CapX 与感知设备选择；`xyz.py` 图片到 TCP 验收；`visualization.py` 双后端 TCP 对比。
- `native/` 官方 TRAC-IK 与 G1 文本链求解器；`robot/` 已核对 URDF 与运动链；`scripts/` 运动链导出、原生构建、Thor 与本机影子入口；`tests/` 覆盖几何、配置、安全、可达性、接口闭环与真实原生 IK。

## 技术栈与外部依赖

- Python 3.10–3.12、NumPy、OpenCV、Ultralytics、SAM2、ONNX Runtime；C++17、CMake/Ninja、Eigen3、Orocos KDL、NLopt。
- `traclabs/trac_ik@90162ac2...`（BSD-3-Clause，源码在 `native/vendor/`）；reBot 仅参考流程概念，未复制其源码和 GraspNet，详见 `UPSTREAM.md`。
- 模型权重与第三方推理代码均为外部资源，不进入本仓库；本机放在 `/Users/firmiana/project/rpent-models/`（权重分三个子目录，`vendor/` 存两个公开仓库）。

## 运行入口、配置与数据流

- 配置入口：Thor 用 `thor.example.json`，本机用 `macos.example.json`，两份共用 `config/` 下同一批标定文件。感知设备由 `thor.py` 按 cuda→mps→cpu 自动选，`RPENT_TRADITIONAL_GRASP_DEVICE` 可覆盖。
- 原生入口 `native/build/g1_trac_ik`（每臂一个求解进程）；影子入口 `scripts/run_thor_shadow.py`，默认只运行 `search`。
- 日志默认 INFO；关键配置、感知、执行提供者、IK、可达性、门禁和执行均保留上下文及异常链；服务在文件描述符层隔离 JSON 回复与第三方输出，父项目另有限量抗噪读取。在线抓取把原始/校正双目、SAM2 框选图、掩码和叠加图保存到父项目单次运行目录，并用帧 SHA-256、输入框、候选分数及掩码框串联 INFO 日志。
- 三维点先位于左相机坐标系，再通过配置外参变换到 `torso_link` 机身坐标系。

## 常用命令

```bash
PYTHONPATH=src python -m pytest -q
./scripts/build_native_thor.sh
./scripts/run_thor_image_shadow.sh --left-image L.jpg --right-image R.jpg --operation pick
./scripts/run_thor_image_xyz.sh --left-image L.jpg --right-image R.jpg --output-json /tmp/x.json
./scripts/run_thor_image_gripper_xyz.sh --left-image L.jpg --right-image R.jpg
./scripts/run_macos_image_xyz.sh --left-image L.jpg --right-image R.jpg --output-json /tmp/x.json
./scripts/run_macos_image_shadow.sh --left-image L.jpg --right-image R.jpg --operation pick
PYTHONPATH=src python scripts/diagnose_ik_reachability.py --help

# 复刻 11:50 现场场景的仿真（成功；换 0.5093 0.0856 0.0517 即复刻 11:03 的失败）
# 解释器 /Users/firmiana/project/.venvs/reBot-DevArm-Grasp-macos-brew/bin/python
# 要开可视化窗口把 python 换成同目录的 mjpython 并加 --viewer
cd ../../reBot-DevArm-Grasp && python scripts/sim_traditional_grasp.py \
  --config ../.config/reBot-DevArm-Grasp/g1d_mujoco.macos.yaml \
  --arm left --bottle-torso-xyz-m 0.4778 0.0666 0.0564 \
  --rest-bottle-on-table --output-dir outputs/sim_1150_success
```

## 当前状态与已验证事实

- 历史结论（详见 Git 历史）：macOS/Thor 原生 TRAC-IK 可用；G1 URDF 与 Thor `xr_teleoperate` 逐字节一致；CapX `ArmController` 构造会自动回零、只接受上层注入控制器；Dex1-1 名义 TCP `0.150215608966 m`（两指内表面间的面积加权质心）；肩部 `[0.004,±0.1002,0.2478]`，杆长上限 `0.560610 m`。腰部三关节在 `torso_link` **之下**，规划器不使用，遥操也未用。
- 缓存图本机基准（**当前配置为旧标定**）：现场图 `raw_*_20260801_164911` 瓶心 `[0.5443,0.0075,0.0249]`、深度 0.6073、瓶径 0.0624（与 thor 实测一致）；新标定下为 `[0.5937,0.0841,-0.0492]`、0.6872、0.0590。桌面图 `left/right.jpg` 新标定瓶径 0.0644、旧标定 0.0691。
- 棋盘 PnP（25 mm 方格，重投影 0.3 px）反解：设计名义外参下桌面法向与竖直差 `5.63°`（瓶心变化 `68 mm`），是外参旋转误差上界。判定过程留档：新标定 fx 比旧的高 **20.5%**——同一台相机 fx 不应变，说明**至少一次标定病态**（标定板在画面里只占 0.6–0.9%）。极线错位 4.4px→1.6px 只检验旋转、**不检验尺度**；棋盘也不能验尺度，它就是标定所用的板，属循环论证。
- **已切回旧标定**（2026-08-02，用户决定）。依据：0801 现场运行与本机复现**两次独立测量**都报瓶径 0.062 m，与真值分毫不差；旧标定把瓶心拉到左肩距 0.5918 m，距实测无解边界 0.573 m 仅差 19 mm（新标定差 88 mm）。两份配置的标定与外参**成对**切回 legacy（外参必须同切，20260801 外参由新标定的矫正旋转 R1 复合而来）。**保留疑点**：桌面缓存图那组旧标定瓶径 69.1 mm 偏 +11.5%，未解释。
- **可达性预检已补进生产路径**（2026-08-01）：原先两处用**机身原点**距离对 `max_reach_m=0.78` 判断，而手臂挂在肩上，现按肩距判定。硬拒绝**仅当两项标定验收布尔量同时为真时生效**，否则只记 WARNING 照常求解——遥操已抓到被判超界的瓶子。
- 侧抓可解半径远严于杆长上界。种子姿态**只影响 TRAC-IK 收敛、不改变解是否存在**：11:50 帧 60 个随机种子中 8 个成功（零位在内），11:03 帧 60 个全败（详见下方多起点实测）。曾误报"±8 mm 窄成功窗口"与"太近抓不住"，**已作废**——那是仿真前移瓶子时桌子没跟着移、瓶心越过桌沿自由落体所致，仿真侧已修。
- **真实可抓边界（现场场景，左臂，瓶子稳放桌面）**：左肩距 0.5730 m 无逆解；0.5392 / 0.5226 / 0.4902 / 0.4591 m **全部成功**（双指接触、抬升 91–93 mm）。**近端无下界**，边界在 0.5392～0.5730 m；`side_grasp_planning_radius_m=0.54` 依据即"已验证的最远可抓距离"。现场帧亦印证：0.5121 m 可规划、0.5422 m 全无解。
- **抓取失败与侧抓姿态候选无关**（已验证）：各成功档选中的候选各不相同，与成败无相关；唯一失败是远端真无逆解。姿态惩罚 `0.02×偏角²`（30° 仅 0.0055）比关节行程平方和小两个数量级，排序几乎只看关节移动量。
- **"TRAC-IK chain 与 Pinocchio FK 差 11.3 cm"的外部结论已证伪**（2026-08-02）。本链 FK 在 q=0 得
  `[0.353953,0.148633,0.051225]`，与对方算的 TRAC-IK 值**逐位一致**；差额完全分解为两项**定义差**：
  ① TCP 不同（`G1_29_ArmIK` 的 `L_ee` 是 wrist+0.05 m 遥操内部帧，本项目用 wrist+0.150216 m 的 Dex1-1
  抓取中心，差 100.216 mm）；② **根坐标系不同**（对方 `pelvis`、本项目 `torso_link`，差额正是 URDF
  `waist_roll_joint` 原点 `[-0.0039635,0,0.044]`；本链 FK 加此偏移**精确到 1e-6 m** 复现对方数值）。
  故"改写 `g1_trac_ik.cpp` 从 URDF 建链"不改变任何数值。顺带复核外参：`thor_camera_to_body_legacy.json`
  平移与 URDF `d435_joint` 只差 ~12 mm（误用 pelvis 系会差 44 mm），**确系 `torso_link` 系**。
- 经典算法替换已否决：SGBM 24 ms、GrabCut 292 ms（比 SAM2 慢 1.8 倍）、YOLO-World 无经典替代；SGBM 在现场图直接失败（MAD `76.6 mm` 超 25 mm 门限）。
- **CREStereo 跑 CPU 已修复**（2026-08-01，环境侧）。根因：`yolo_world` conda 环境装的是纯 CPU 版
  onnxruntime；同机唯一 GPU 轮子按 NumPy 1.x 编译，装进 NumPy 2.4.4 的 `yolo_world` 会 `ImportError`
  （已还原，备份在 `/home/aiot/backup_onnxruntime_cpu/`）。用 `--system-site-packages` 叠加环境
  `/home/aiot/wuxi/venvs/rpent-grasp-gpu` 只覆盖 numpy 1.26.4 与 GPU onnxruntime，`3544→49.2 ms`。
  `ExternalCREStereoBackend` 另加执行提供者自判：已在跑加速器就原样保留（保住 fp16 与引擎缓存），
  只落 CPU 且有 CUDA 才强切，否则 WARNING 回退。
- 本机模型部署已完成，影子链路不依赖服务器；三个权重（CREStereo ONNX 25 MB / YOLO-World `.pt` 140 MB / SAM2 176 MB）全来自公开发布源且体积与 Thor 一致，推理代码取 `ibaiGorordo/ONNX-CREStereo-Depth-Estimation` 与 `facebookresearch/sam2`。本机 `.pt` 走 `set_classes` 需 CLIP，Thor 走 `.engine` 不需要。环境 `.venvs/rpent-traditional-grasp-macos`。
- 本机与 Thor 一致性：桌面缓存图瓶心欧氏差 `0.54 mm`、0802 现场图 `< 0.3 mm`；残差来自 Thor fp16 TensorRT 与本机 fp32 CPU。本机全链路 8.2 s。**本机可离线复算任一现场帧的完整规划**，是当前最快的对照手段。

## 未确认、阻塞问题与下一步

- **2026-08-02 14:18(本项目,失败) / 14:21(legacy,成功) 是第一次同场景跨方法 A/B，两者相隔 3 分钟、
  任务同为"不移动自身"、底盘均未动。失败模式已从"够不着"变成"过不了权威校验"。**
  - 本项目这次**规划是成功的**：`depth=0.573m diameter=0.062m`，可达性预检 `within_radius=True`、
    `required_base_advance=0`，产出 3 个 ranked 候选（`pitch_+30deg` / `pitch_+20deg_yaw_-10deg` /
    `pitch_+20deg`，按 score 0.461/0.572/0.642 升序）。死在父项目
    `validate_traditional_grasp_path`：`status=collision_failed`，
    `error=no ranked side-grasp candidate passed authoritative validation`。
  - **legacy 成功那次 `validate_traditional_grasp_path` 调用 0 次**——它**根本不过这道校验**，也没有
    计划签名、TTL、种子漂移检查。所以这不是抓取质量之争，是**我们多了一整套安全门而对方没有**。
    对方自身拒了 4 个点云候选后执行第 5 个（`candidate_id=4`）。
  - **在 thor 上用父项目自己的检查器逐点复验了这三条轨迹**（只读）：`pitch_+30deg` **39 点里 17 点
    碰撞**、`pitch_+20deg` 34 点里 13 点碰撞，都是 `torso_link_0 <-> left_shoulder_yaw_link_0`；
    **`pitch_+20deg_yaw_-10deg` 34 点零碰撞**。碰撞点集中在**开头的关节空间桥接段与结尾的后撤段**
    （肩 roll −0.19..−0.06），**不在抓取位姿本身**。检查器在零位报 0 碰撞，故**不是坏掉的门**。
  - **两个独立缺陷由此坐实**：① **本项目排序对自碰撞完全无感**（`collision_check_detail=
    "collision checker not configured"`，我方没有碰撞检查器，只按关节行程排序），把真会撞的候选排在
    第一；② **零碰撞的候选 2 也没通过，且原因无从查证**——父项目的
    `traditional side-grasp candidate validation: ... rank=... success=... error=...` 这行 INFO
    **不落在运行目录任何文件里**。已排除关节限位（三条轨迹 0 越界）与碰撞，剩余嫌疑是计划 TTL(15 s)、
    种子漂移(容差 0.05 rad，16:43 曾实测漂到 0.102)、计划摘要校验，需父项目补日志才能定位。

- **pick 后端归属判别（命名极易混淆，务必先看这条）**：父项目 `wuxi_adapter.py:421` `_pick_object_legacy`
  的错误串字面写着 `traditional grasp detector did not return a grasp message`，那是 **Contact-GraspNet
  时代的旧叫法**，比本项目还早；总结 JSON 里 LLM 又自己写 `"backend": "air_robot traditional method
  stack"`。判别只能靠**唯一串**计数（依次为 `did not return a grasp message` / `contact_graspnet` /
  `required_base_advance_m` / 收到 `plan_pick`）：**11:03 = 0/0/2/1 与 14:18 → 本项目**；11:47 = 2/0/0/0、
  11:50 = 2/2/0/0、14:21 → legacy。**父项目从不记录当前 pick 后端模式**（`run.log`、`observe_scene` 里都
  没有），这正是归属只能反推的根本原因，也导致过测试人员与日志证据的分歧。**建议父项目在
  `wuxi_adapter._pick_backend_mode()` 落一条 INFO 并把 mode 加进 `observe_scene` 输出。** 机制上无歧义：
  默认 `traditional-live`，进 legacy **必须显式 export**；分发是 `if mode != "legacy"` 二选一，
  `traditional_grasp_backend.py` **不含任何回退 legacy 的路径**；`AIR_ROBOT_PICK_BACKEND` 未见于任何脚本
  或 `.bashrc`，故最可能是手工 export 后跨运行残留。
- **11:03 / 11:47 / 11:50 三次已结案**（`logs/` 已 gitignore，原始数据都在）。11:47、11:50 的 pick 都由旧
  Contact-GraspNet 承担，本项目只做 `search_object`；11:03 是唯一由本项目 pick 的，失败原因是**目标刚好
  在可解边界之外**。已验证的要点：
  - **离线复算 + MuJoCo 仿真在三个位置的成败与现场逐一吻合**（命令见"常用命令"）：11:50 位
    `[0.4778,0.0666,0.0564]`（左肩距 0.5121）与 11:47 位（0.5099/0.5114）**仿真均抓取成功**——双指接触、
    抬升 91.4–91.5 mm，产物 `outputs/sim_1150_success/`、`sim_1147/`；11:03 位 `[0.5093,0.0856,0.0517]`
    （0.5422）**离线与仿真均失败**（`no continuous IK path`）。**只差 30 mm 跨过边界，独立物理引擎给出同一
    分界。** 同次运行的本项目 TRAC-IK 与 MuJoCo FK 交叉审计 641 采样**最大位置误差 4.0e-07 m**——第三次
    独立证伪上面的 FK 不一致结论。仿真已知偏差：Oracle 把瓶心 z 估高 **27.5 mm**（瓶模型含瓶颈）。
  - **11:47 崩溃 100% 源于坏 bbox 触发硬失败，与可达性无关**：那轮 LLM 只喂自己目测的框，
    `[268,204,312,294]` 落在**空台面**上（已叠图确认），几何门禁按设计拒绝（`瓶径越界 0.1608m`、
    `商标带深度离散过大 mad=0.0284m`），但本项目**抛异常**而非回退。而 11:50 的 LLM 调了不带 bbox 的
    `search_object`，本项目 YOLO-World 正确定位（0.85–0.88），**促成对方抓取成功的 bbox 正是本项目给的**。
  - **"把仿真的肩后收搬到实机"已实测否决**：种子就是规划时刻的真实关节角。11:03 帧把肩俯仰从 0 扫到
    0.80 rad **每一档 24 个候选全无解**；关节限位内随机多起点 **60 个种子 0 次成功**（11:50 帧 60 个中
    8 次成功且零位在内）。故目标不在灵巧工作空间内，任何准备姿态都救不了，**且零位本身已是好种子**。
    生产链路**根本没有准备姿态**——`client.py:700` 的 `prepare_arm` 只初始化 CapX 控制器，构造即自动回零。
  - **仿真 viewer 里"先动一下再停"不是异常**：开头是仿真脚本 `prepare_fixed_ready_pose`（与瓶子无关的固定
    准备段，`shoulder_pitch +0.35 rad` 1.0 s 后再解到固定准备 TCP 1.5 s），随后**静止约 10 s 是规划阻塞**
    （可行候选每个约 0.63 s 做 29 点连续逆解，被拒的约 0.02 s），最后才是抓取轨迹。
  - 其他：11:50 的 3 次 0.1 m 前进原语**实际没移动底盘**（前后瓶心只变 1.5–6 mm）；**姿态网格明显不对称**，
    只有正俯角有解，负俯角与偏航全灭，一半候选是无效开销。
- **2026-08-02 11:03 现场运行**（`logs/20260802-11:03:28...`）：瓶心 `[0.5093,0.0856,0.0517]`、深度 `0.5658 m`、瓶径 `0.0587 m`、MAD 4.1 mm，本机复现与 Thor 差 **< 0.3 mm**；左肩距 `0.5422 m` 在杆长上界内但超实测规划半径 2.2 mm，种子 14 维全 `|q| < 0.015 rad`，**24 个姿态候选全部 `no IK solution`**。**瓶径真值存疑，暂缓**：现场量得**瓶底** 55.0 mm 且称昨今同一瓶（与此前"62 mm"冲突），而本项目估计的是**中部商标带**（掩码中部 38 px、瓶底 33 px，下收 13%），同一深度图量瓶底得 57.9 mm；**要定案需现场补量瓶身最宽处直径与瓶高**，在此之前不要用瓶径反推深度尺度。
- **0801 现场运行**（`logs/20260801-16:48:44_air_robot_task_s0`）：**旧标定**，`depth=0.607m diameter=0.062m`；底盘全程未动（位移 8.5e-06 m）。两次采集相隔 30 s 深度由 0.607 变 0.527 m 而机器人没动，是画面里的人把桌子推近了约 80 mm。
- **手眼标定待办。** 遥操在底盘、躯干都不动、只用手臂的条件下抓到了本链路判为超界的瓶子，是唯一不依赖标定的硬证据。做法：手臂摆若干已知姿态各拍一组图，用正运动学 TCP 作真值求解——夹爪就是精度最高的靶标，**多姿态才能把尺度、旋转、平移分开**。`service.py` 已补 `current_q_rad` 数值日志。相机-机身外参目前是设计名义值复合，旋转偏差上界 `5.63°`（68 mm），平移无观测约束，验收前 `camera_to_body_validated` 保持 false。零位手臂当靶的初步测量（单张、目测 ±10–20 px，不足定论）：两套标定预测**都比实际夹爪高 30–50 px**，共模误差指向 URDF `d435_joint` 设计俯仰。
- **父项目 `traditional_grasp_backend.py` 的 `AIR_ROBOT_TRADITIONAL_GRASP_PYTHON` 默认值仍是 `yolo_world`**，2026-08-02 现场日志已实测证实：生产链路仍跑 **CPU 版 CREStereo**，单帧 4.3–4.9 s。子项目的自判逻辑正常工作（正确落 WARNING），缺的是父项目改默认解释器，需父项目单独提交。
- 仿真侧曾把瓶底埋进桌面、又曾让瓶子走出桌沿，均已修（`--rest-bottle-on-table` 同时调高度与水平位置，出生点随桌走，穿模硬拦截）。Oracle 真值输入下瓶心 z 仍偏高 23–30 mm。**仿真 yaml 仍内联新标定，与子项目已不一致**；仿真仓库另有 12 个提交未推送。
- **底盘不移动是硬约束**（用户 2026-08-02 明确）：遥操就是在底盘与躯干都不动、只用手臂的条件下抓到的，故 `required_base_advance_m` 只能作为"差多远"的诊断量，不能作为方案。尚未做在线相机采集回归；商标区域在反光、透明瓶、遮挡、低纹理下的深度成功率未实测。环境障碍物碰撞因缺场景模型未实现，实机前需清场和急停；Dex1-1 驱动量到毫米开口、接触阈值、TCP 六自由度外参也未做真机标定。`visualization.py` 与三个对比脚本为另一会话的未提交工作，未经本轮验证。
- 下一步（按价值，已按 14:18/14:21 的 A/B 重排）：**① 本项目排序纳入自碰撞**（自带检查器或复用父项目的，至少让零碰撞候选排前）→ **② 父项目补 `candidate validation` 落盘日志**，否则非碰撞拒绝无从定位 → ③ 显式 bbox 被拒时回退自带检测器 → ④ 加抓取深度参数让 TCP 可落在瓶心之前 → ⑤ 姿态网格向有解一侧加密 → ⑥ 现场补量瓶身最宽处直径与瓶高 → ⑦ 多姿态手眼标定 → ⑧ 父项目改 `AIR_ROBOT_TRADITIONAL_GRASP_PYTHON` 上 GPU → ⑨ 清场急停后限速小步真机验证。

## 注意事项

- 父、子项目必须分别提交和同步；子项目不作 submodule，其路径由父项目本地排除规则隔离，不要在父项目提交中纳入 `traditional_grasp/` 内容。
- 不得把示例配置直接切为 `live`，不得绕过碰撞检查和标定验证布尔门禁；TRAC-IK 只负责运动学求解，不提供碰撞安全保证。
- `macos.example.json` 与 `rpent-models/` 的绝对路径是本机部署事实，换机器须改；两个 `run_macos_*.sh` 支持用环境变量覆盖解释器与 SAM2 仓库。
