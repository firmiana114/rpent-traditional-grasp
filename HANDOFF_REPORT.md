# 交接报告

## 项目整体描述

本项目是 RPent 的独立传统瓶体抓取子项目，用于 AIR G1 双臂机器人瓶装物侧抓。
它嵌套部署到父项目目录但不作 submodule；父项目以独立提交接入其规划服务。

核心链路为：双目矫正 → CREStereo 深度 → YOLO-World/SAM2 → 瓶心估计 →
受限侧抓姿态候选 → 关节空间预抓取 → 水平直线插入 → TRAC-IK 连续逆解 →
碰撞检查 → 执行和抓取验证。

## 核心功能与接口

- 高层兼容接口 `search_object`/`approach_object`/`pick_object`/`verify_grasp`，核心
  替代目标是 RPent 的 `pick_object`。本项目提供无运动持久规划服务，父项目负责后端
  选择、控制器所有权、计划复验和执行；`pick_object` 已锁定原仅关键字参数及主要结果
  字段。
- 细粒度接口：`compute_depth_crestereo`、`segment_object`、`mask_depth_to_pointcloud`、`pointcloud_to_body`、规划和执行。
- `offline` 完整模拟闭环；`shadow` 用真实感知/IK但绝不发运动命令；`live` 必须同时通过运动授权、三项标定和碰撞检查门禁。抓取顺序固定为张手、预抓取、抓取位、闭合/接触、抬升、后撤，构造器与接口都不自动回零。

## 主要模块与目录

- `src/rpent_traditional_grasp/`：`api.py` 四接口编排与安全门禁；`config.py`/
  `gripper.py` 配置与 Dex1-1 规格校验；`stereo.py` 标定/矫正/CREStereo 适配与执行
  提供者诊断；`perception.py` YOLO-World 检测与 SAM2 框提示分割；`geometry.py`
  商标带鲁棒深度与瓶心估计；`planning.py` ±30° 组合侧抓与插值；`ik.py` 持久
  TRAC-IK 与 FK 残差；`diagnostics.py` 无运动可达性诊断；`execution.py` 碰撞/
  执行器协议与接触证据；`thor.py` 相机适配、注入式 CapX 与感知设备选择；
  `xyz.py` 图片到 TCP 验收；`visualization.py` 双后端 TCP 对比。
- `native/` 官方 TRAC-IK 与 G1 文本链求解器；`robot/` 已核对 URDF 与运动链；`scripts/` 运动链导出、原生构建、Thor 与本机影子入口；`tests/` 覆盖几何、配置、安全、可达性、接口闭环与真实原生 IK。

## 技术栈与外部依赖

- Python 3.10–3.12、NumPy、OpenCV、Ultralytics、SAM2、ONNX Runtime；C++17、CMake/Ninja、Eigen3、Orocos KDL、NLopt。
- `traclabs/trac_ik@90162ac2...`（BSD-3-Clause，源码在 `native/vendor/`）；reBot 仅参考流程概念，未复制其源码和 GraspNet，详见 `UPSTREAM.md`。
- 模型权重与第三方推理代码均为外部资源，不进入本仓库；本机放在 `/Users/firmiana/project/rpent-models/`（权重分三个子目录，`vendor/` 存两个公开仓库）。

## 运行入口、配置与数据流

- 配置入口：Thor 用 `thor.example.json`，本机用 `macos.example.json`，两份共用
  `config/` 下同一批标定文件。感知设备由 `thor.py` 按 cuda→mps→cpu 自动选择，
  可用 `RPENT_TRADITIONAL_GRASP_DEVICE` 强制覆盖。
- 原生入口 `native/build/g1_trac_ik`（每臂一个求解进程）；影子入口 `scripts/run_thor_shadow.py`，默认只运行 `search`。
- 日志默认 INFO；关键配置、感知、执行提供者、IK、可达性、门禁和执行均保留上下文及异常链；服务在文件描述符层隔离 JSON 回复与第三方输出，父项目另有限量抗噪读取。在线抓取把原始/校正双目、SAM2 框选图、掩码和叠加图保存到父项目单次运行目录，并用帧 SHA-256、输入框、候选分数及掩码框串联 INFO 日志。
- 三维点先位于左相机坐标系，再通过配置外参变换到 `torso_link` 机身坐标系。

## 常用命令

```bash
PYTHONPATH=src python -m pytest -q
./scripts/build_native_thor.sh
./scripts/run_thor_image_shadow.sh --left-image L.jpg --right-image R.jpg --operation pick
./scripts/run_thor_image_xyz.sh --left-image L.jpg --right-image R.jpg \
  --output-json /tmp/image_xyz.json
./scripts/run_thor_image_gripper_xyz.sh --left-image L.jpg --right-image R.jpg
./scripts/run_macos_image_xyz.sh --left-image L.jpg --right-image R.jpg \
  --output-json /tmp/image_xyz.json
./scripts/run_macos_image_shadow.sh --left-image L.jpg --right-image R.jpg --operation pick
PYTHONPATH=src python scripts/diagnose_ik_reachability.py --help
```

## 当前状态与已验证事实

- 历史结论（详见 Git 历史）：macOS/Thor 原生 TRAC-IK 可用；G1 URDF 与 Thor `xr_teleoperate`
  逐字节一致；CapX `ArmController` 构造会自动回零、只接受上层注入控制器；Dex1-1 名义 TCP
  `0.150215608966 m`（两指内表面间的面积加权质心）；肩部 `[0.004,±0.1002,0.2478]`，杆长上限
  `0.560610 m`（URDF 逐项复核一致）。腰部三关节在 `torso_link` **之下**（俯仰 ±29.8°），规划器
  不使用；遥操也未用。CaP-X 旧后端对照已放弃。
- 缓存图本机基准（后续仿真统一用现场图那组）。**当前配置为旧标定**：现场图
  `raw_*_20260801_164911` 瓶心 `[0.5443,0.0075,0.0249]`、深度 0.6073、瓶径 0.0624（与 thor
  现场实测 0.607/0.062 一致）；新标定下为 `[0.5937,0.0841,-0.0492]`、0.6872、0.0590。桌面图
  `left/right.jpg` 新标定瓶心 `[0.6037,-0.0320,-0.0565]`、瓶径 0.0644，旧标定瓶径 0.0691。
  两图为同一瓶水，真值 62 mm。
- 棋盘 PnP（25 mm 方格，重投影 0.3 px）反解：设计名义外参下桌面法向与竖直差 `5.63°`（瓶心位置变化 `68 mm`），是外参旋转误差上界。
- **已切回旧标定**（2026-08-02，用户决定）。依据：现场运行与本机复现**两次独立测量**都报瓶径
  0.062 m，与真值 62 mm 分毫不差；旧标定把瓶心拉到左肩距 0.5918 m，距实测无解边界 0.573 m 仅
  差 19 mm（新标定差 88 mm），与"遥操能抓到"更相容。两份配置的标定与外参**成对**切回 legacy
  （外参必须同切，20260801 外参由新标定的矫正旋转 R1 复合而来）。**保留疑点**：桌面缓存图那组
  旧标定瓶径 69.1 mm 偏 +11.5%，未解释。
- 判定过程留档：新标定 fx 比旧的高 **20.5%**——同一台相机 fx 不应变，说明**至少一次标定病态**，
  最可能因标定板在画面里太小（实拍只占 0.6–0.9%）。极线错位 4.4px→1.6px 只检验旋转、**不检验
  尺度**，与本结论不矛盾。棋盘不可用于验尺度：它就是标定所用的板，属循环论证。
- **可达性预检已补进生产路径**（2026-08-01）。原先两处都用**机身原点**距离对 `max_reach_m=0.78`
  判断，而手臂挂在肩上。现按肩距判定。**但上界虽精确，作用的却是"感知出来的"目标**——遥操已抓到
  被判超界的瓶子，故硬拒绝改为**仅当两项标定验收布尔量同时为真时生效**，否则只记 WARNING 照常求解。
- 侧抓可解半径远严于杆长上界，且**强烈依赖种子姿态与俯角**：零关节种子跑桌面缓存图
  方向时右臂 0.5197 m 无解、0.5118 m 首次可解；仿真准备姿态跑现场场景时左臂 0.5475 m
  仍无解、0.5392 m 才可解。所以它只能建议、不能当门禁。
- **曾误报"±8 mm 窄成功窗口"与"太近抓不住"，已作废**：仿真里前移瓶子时桌子没跟着移，瓶心越过桌沿即自由落体，手臂没碰到却记成失败；仿真侧已修（桌面与出生点都随瓶子走）。
- **真实可抓边界（现场场景，左臂，瓶子稳放桌面）**：左肩距 0.5730 m 无逆解；0.5392 / 0.5226 /
  0.4902 / 0.4591 m **全部成功**（双指接触、抬升 91–93 mm）。**近端无下界**，边界落在
  0.5392～0.5730 m。`side_grasp_planning_radius_m=0.54` 数值保留但依据已换成"已验证的最远
  可抓距离"。
- **抓取失败与侧抓姿态候选无关**（已验证）：各成功档选中的候选各不相同（`pitch_+30deg`、
  `pitch_+20deg_yaw_-10deg`、`pitch_+10deg_yaw_-20deg`），与成败无相关；唯一失败是远端真无
  逆解。姿态惩罚 `0.02×偏角²`（30° 仅 0.0055）比关节行程平方和小两个数量级，排序几乎只看
  关节移动量。
- 经典算法替换已否决：SGBM 24 ms、GrabCut 292 ms（比 SAM2 慢 1.8 倍）、YOLO-World 无经典替代；SGBM 在现场图直接失败（MAD `76.6 mm` 超 25 mm 门限）。
- **CREStereo 跑 CPU 已修复**（2026-08-01）。根因是 `yolo_world` conda 环境装了纯 CPU 版
  onnxruntime，`object_grab.py` 请求的 TensorRT/CUDA 被静默回退；同机唯一 GPU 版轮子按
  NumPy 1.x 编译，装进 NumPy 2.4.4 的 `yolo_world` 会 `ImportError`（已还原，备份在
  `/home/aiot/backup_onnxruntime_cpu/`）。最终用 `--system-site-packages` 叠加环境
  `/home/aiot/wuxi/venvs/rpent-grasp-gpu`，只覆盖 numpy 1.26.4 与 GPU 版 onnxruntime，稳态
  `3544 ms → 49.2 ms`（72 倍）。`ExternalCREStereoBackend` 另加执行提供者自判：已在跑加速器
  就原样保留（保住厂商类配的 fp16 与引擎缓存），只落 CPU 时若 onnxruntime 有 CUDA 就
  `set_providers` 强制切过去（不强推 TensorRT，会触发首次引擎编译），确实没有才 WARNING 回退。
- 本机模型部署已完成，影子链路不依赖服务器。三个权重全来自公开发布源且体积与 Thor 一致：
  CREStereo ONNX 25 MB（PINTO `284_CREStereo`）、YOLO-World `.pt` 140 MB（ultralytics/assets
  v8.3.0）、SAM2 176 MB（`dl.fbaipublicfiles.com`）；推理代码取
  `ibaiGorordo/ONNX-CREStereo-Depth-Estimation` 与 `facebookresearch/sam2`。本机 `.pt` 走
  `set_classes` 需 CLIP，Thor 走 `.engine` 不需要；ultralytics `weights_dir` 已改指
  `rpent-models/ultralytics_weights`。本机环境 `.venvs/rpent-traditional-grasp-macos`。
- 本机与 Thor 结果一致性（同一对桌面缓存图）：瓶心欧氏差 `0.54 mm`、有效深度像素同为 1161、瓶径差 0.045 mm；YOLO `.pt` 框与 `.engine` 框差 1 px。残差来自 Thor fp16 TensorRT 与本机 fp32 CPU。本机全链路 8.2 s。

## 未确认、阻塞问题与下一步

- **现场那次运行的记录**（`logs/20260801-16:48:44_air_robot_task_s0`）：跑的是**旧标定**，
  `depth=0.607m diameter=0.062m`。`get_robot_state` 底盘位移 8.5e-06 m、`approach_object` 被
  `base motion is disabled` 挡下，**底盘全程未动**。两次采集相隔 30 s 深度由 0.607 变 0.527 m
  而机器人没动，是画面里的人把桌子推近了约 80 mm。
- **零位手臂当标定靶的初步测量**（单张照片、目测 ±10–20 px，不足以定论）：零位 TCP 投影，旧标定
  得间距 171.4 px、新标定 217.0 px，目测实际约 198 px 夹在两者之间，**横向不能判定**；但两套预测
  **都比实际夹爪高 30–50 px**，属共模误差，指向共用的 URDF `d435_joint` 设计俯仰（5–8°，与棋盘
  PnP 的 5.63° 同量级）。
- **首要待办：手眼标定。** 遥操在底盘、躯干都不动、只用手臂的条件下抓到了本链路判为超界的
  瓶子，是唯一不依赖标定的硬证据。正式做法：把手臂摆到若干已知姿态各拍一组图，用正运动学
  TCP 作真值求解——机器人自己的夹爪就是精度最高的靶标，**多姿态才能把尺度、旋转、平移分开**，
  单张照片做不到。`service.py` 已补 `current_q_rad` 数值日志（此前只记形状，而逆解失败时不
  产生任何轨迹，导致 16:49 那次完全没留下手臂姿态）。
- 相机-机身外参为设计名义值复合，旋转偏差上界 `5.63°`（68 mm），平移无观测约束；验收前 `camera_to_body_validated` 保持 false。
- **本轮代码提交尚未同步到 air-thor**：改完 GPU 环境后服务器即以
  `kex_exchange_identification: Connection closed` 拒连（TCP 通、sshd 立即断），随后
  确认服务器可能已关机。Thor 端已生效的只是运行环境。恢复后用 `git bundle` 同步，
  注意 Thor 上另有 `api.py`/`perception.py`/`service.py` 三个未提交改动。
- 父项目 `traditional_grasp_backend.py` 的 `AIR_ROBOT_TRADITIONAL_GRASP_PYTHON` 默认值
  仍是 `yolo_world`，即**生产规划服务目前仍跑 CPU 版 CREStereo**；父项目改动需在
  air-thor 恢复后单独提交。
- 仿真侧曾把瓶底埋进桌面、又曾让瓶子走出桌沿，均已修（`--rest-bottle-on-table` 同时调高度与水平位置，出生点随桌走，穿模硬拦截）。Oracle 真值输入下瓶心 z 仍偏高 23–30 mm（仿真瓶模型含瓶颈）。
- **底盘不移动是硬约束**（用户 2026-08-02 明确）：遥操就是在底盘与躯干都不动、只用手臂的
  条件下抓到的。故 `required_base_advance_m` 只能作为"差多远"的诊断量，不能作为方案；
  真正要修的是感知报远。可抓边界 0.539–0.573 m 对现场瓶心 0.6605 m（新标定）/ 0.5918 m
  （旧标定）意味着感知至少偏远 **22%**（新）或 **10%**（旧）。
- 尚未做在线相机采集回归；商标区域在反光、透明瓶、遮挡、低纹理下的深度成功率未实测。环境障碍物碰撞因缺场景模型未实现，实机前需清场和急停；Dex1-1 驱动量到毫米开口、接触阈值、TCP 六自由度外参也未做真机标定。
- `visualization.py` 与三个对比脚本为另一会话的未提交工作，未经本轮验证。
- 下一步：卷尺实验分离焦距/外参误差 → 回退或重做双目标定 → 重测窄窗口 → 同步 Thor 与父项目 → 清场急停后限速小步真机验证。

## 注意事项

- 父、子项目必须分别提交和同步；子项目不作 submodule，其路径由父项目本地排除规则隔离，不要在父项目提交中纳入 `traditional_grasp/` 内容。
- 不得把示例配置直接切为 `live`，不得绕过碰撞检查和标定验证布尔门禁；TRAC-IK 只负责运动学求解，不提供碰撞安全保证。
- `macos.example.json` 与 `rpent-models/` 的绝对路径是本机部署事实，换机器须改；两个 `run_macos_*.sh` 支持用环境变量覆盖解释器与 SAM2 仓库。
