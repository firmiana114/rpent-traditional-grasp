# 交接报告

## 项目整体描述

本项目是 RPent 的独立传统瓶体抓取子项目，用于 AIR G1 双臂机器人瓶装物侧抓。
它嵌套部署到父项目目录但不作 submodule；父项目以独立提交接入其规划服务。

核心链路为：双目矫正 → CREStereo 深度 → YOLO-World/SAM2 → 瓶心估计 →
受限侧抓姿态候选 → 关节空间预抓取 → 水平直线插入 → TRAC-IK 连续逆解 →
碰撞检查 → 执行和抓取验证。

## 核心功能与接口

- 高层兼容接口：`search_object`、`approach_object`、`pick_object`、
  `verify_grasp`。
- 核心替代目标是 RPent 的 `pick_object` 逻辑；本项目提供无运动持久规划服务，
  父项目负责后端选择、控制器所有权、计划复验和执行。
- `pick_object` 已锁定原仅关键字参数及主要结果字段；阶段测试改走内部预览
  方法，不向未来 RPent 调用暴露测试开关。
- 细粒度接口：`compute_depth_crestereo`、`segment_object`、
  `mask_depth_to_pointcloud`、`pointcloud_to_body`、规划和执行。
- `offline` 用于完整模拟闭环；`shadow` 使用真实感知/IK但绝不发运动命令；
  `live` 必须同时通过运动授权、三项标定和碰撞检查门禁。
- 抓取执行顺序固定为张手、到达预抓取、到达抓取位、闭合/接触、抬升、后撤。
  构造器和接口均不自动回零。

## 主要模块与目录

- `src/rpent_traditional_grasp/api.py`：四接口编排、状态和安全门禁。
- `config.py`/`gripper.py`：配置、安全门与共享 Dex1-1 规格校验。
- `stereo.py`：双目标定、矫正、外部 CREStereo 适配和深度计算。
- `perception.py`：YOLO-World 检测与 SAM2 框提示分割适配。
- `geometry.py`：商标带、鲁棒深度、瓶径和三维瓶心估计。
- `planning.py`：±30° 内俯仰/偏转组合侧抓、关节桥接和笛卡尔插值。
- `ik.py`：持久 TRAC-IK 子进程、连续求解、FK 残差和关节变化诊断。
- `diagnostics.py` 及对应脚本：无运动精确位姿、仅位置、链长和连续路径诊断。
- `execution.py`：碰撞检查协议、执行器协议、模拟执行器与接触证据。
- `thor.py`：Thor 相机、现有模型资源和注入式 CapX/碰撞检查适配。
- `native/`：官方 TRAC-IK 核心、ROS2 最小兼容层和 G1 文本链求解器。
- `robot/`：经核对的 G1 URDF 与左右 7 轴运动链。
- `scripts/`：运动链导出、Thor/macOS 原生构建、影子运行入口。
- `xyz.py` 及对应脚本：图片到物体/夹爪 TCP XYZ 验收；不启动 IK 或控制器。
- `tests/traditional_grasp/`：几何、配置、安全、接口闭环和真实原生 IK 测试。

## 技术栈与外部依赖

- Python 3.10–3.12、NumPy、OpenCV、Ultralytics、SAM2、ONNX Runtime。
- C++17、CMake/Ninja、Eigen3、Orocos KDL、NLopt。
- `traclabs/trac_ik@90162ac2...`，BSD-3-Clause，源码在 `native/vendor/`。
- reBot 仅参考流程概念，未复制其源码和 GraspNet；详见 `UPSTREAM.md`。
- Thor 的模型权重均为外部资源，不进入本仓库。

## 运行入口、配置与数据流

- 配置入口：`thor.example.json`；双目标定和相机外参分别位于 `config/`。
- 原生入口：`native/build/g1_trac_ik`，每条手臂保持一个求解进程。
- Thor 入口：`scripts/run_thor_shadow.py`，默认只运行 `search` 影子流程。
- 日志默认 INFO；关键配置、感知、IK、门禁和执行均保留上下文及异常链；服务在文件描述符层隔离 JSON 回复与第三方输出，父项目另有限量抗噪读取。
- 在线抓取把原始/校正双目、SAM2 框选图、掩码和叠加图保存到父项目单次运行目录，并用帧 SHA-256、输入框、候选分数及掩码框串联 INFO 日志。
- 三维点先位于左相机坐标系，再通过配置外参变换到 `torso_link` 机身坐标系。

## 常用命令

```bash
docker build -t rpent-traditional-grasp:test .
docker run --rm rpent-traditional-grasp:test
./scripts/build_native_thor.sh
PYTHONPATH=src python -m pytest -q
./scripts/run_thor_image_shadow.sh \
  --left-image /path/to/left.jpg \
  --right-image /path/to/right.jpg \
  --operation pick
./scripts/run_thor_image_xyz.sh \
  --left-image /path/to/left.jpg \
  --right-image /path/to/right.jpg \
  --output-json /tmp/image_xyz.json
./scripts/run_thor_image_gripper_xyz.sh \
  --left-image /path/to/left.jpg \
  --right-image /path/to/right.jpg
PYTHONPATH=src python scripts/diagnose_ik_reachability.py --help
```

## 当前状态与已验证事实

- macOS arm64 原生 TRAC-IK 已构建；Thor 启用原生求解器后 50 项 pytest 全通过。
- GitHub 私有远端为 `firmiana114/rpent-traditional-grasp`。
- 已移除本项目自设的 `0.18 rad` 关节跳变硬门限，与原始 `pick_object`
  对齐；最大关节变化仍写入 INFO 日志及返回结果，仅作为诊断指标。
- G1 URDF SHA-256 为
  `8bbf006633fc50b616f665c7a970780cc296577a0adfd7d28b049e751c238735`，
  与 Thor 上 `xr_teleoperate` 模型逐字节一致。
- Thor 已核实 YOLO-World、SAM2、CREStereo、URDF 外部资源存在；`abot-claw` 环境可导入相关推理依赖；协议隔离回归确认 Python 与原生 fd 1 输出均不会进入 JSON 通道。
- Thor 是 Ubuntu 24.04/aarch64，账号无免密 sudo；无 sudo 依赖构建脚本已提供。
- 当前 CapX `ArmController` 构造器会自动回零，因此本项目只接受上层持有者注入
  的控制器，禁止自行实例化。
- Thor 已使用用户指定的历史左右图完成真实 shadow 感知：CREStereo 有效深度
  比例 1.000，YOLO 检出瓶子（0.887），SAM2 掩码 3356 像素（0.974），估计
  深度 0.567 m、瓶径 0.061 m、深度 MAD 0.0072 m。
- 第一阶段独立图片入口已在 Thor 临时副本用同一历史样本复现；输出相机瓶心
  `[-0.0544, -0.0992, 0.5973]` m、机身瓶心
  `[0.5242, 0.0841, 0.0618]` m。该结果是软件回归基线，不是物理真值。
- 已由厂家 Dex1-1 URDF/网格推导 `0.150215608966 m` 腕部到 TCP 名义偏移，
  传统运动链与仿真共用 `config/g1d_dex1_1_nominal.json`；该值未做当前真机
  标定，`gripper_tcp_calibration_validated` 必须保持 false。
- 第二阶段经内部 `preview_pick_object_xyz` 进入，与 `pick_object` 共用感知
  和目标计算逻辑，并在 IK 和运动前返回。
- 第二阶段独立入口已在 Thor 临时副本用历史图片完成真实推理回归：自动选择
  左臂，输出 TCP `[0.5242, 0.0841, 0.0618]` m，未启动 IK 或控制器。
- 验证图片是 `../logs/20260730-15:39:59_air_robot_task_s0/sensor/` 中同时间戳
  `20260730_154013_543214444` 的 `left_*.jpg` 与 `right_*.jpg`。
- 图像时刻早于控制器启动约 63 秒，没有同步真实关节状态；诊断使用相邻任务
  自动回零后的真实关节角做代理，零位与代理状态的单点结果一致。
- 最新现场目标 TCP `[0.4818,0.0739,0.0484]` m 在初始手腕姿态无精确解；
  `pitch +20° / yaw -10°` 受限侧抓可生成完整路径。
- 更新后的运动链以 `torso_link` 为根，串联杆长理论上限均为 `0.560610 m`；
  旧历史样本的可达性诊断需用新运动链重新执行，原超限结论不再沿用。
- 最新现场目标的组合侧抓共 38 个规划点，经父项目整机模型 87 个采样点验证
  无自碰撞；纯 `pitch +20°/+30°` 路径因上臂靠近躯干被正确拒绝。
- CapX 的 FK 以 `pelvis` 为根，本项目以 `torso_link` 为根；应用 URDF 固定
  平移 `[-0.0039635,0,0.044]` m 后左右 FK 完全一致，TCP/运动链未发现错误。

## 未确认、阻塞问题与下一步

- 按当前验证约束仅使用历史左右图；在线相机采集暂不继续。
- `config/` 中的双目标定和相机到机身外参来自旧代码，状态为
  `legacy_unvalidated`；必须重标定并量化误差。
- 第一阶段尚缺人工测量的多位置机身坐标真值；在至少覆盖近/中/远和左右视野
  的样本通过误差阈值前，不得把可重复 XYZ 等同于物理坐标准确。
- 瓶体商标区域在反光、透明瓶、遮挡和低纹理场景的深度成功率尚未实测。
- 子项目返回按关节变化与姿态偏移排序的候选；父项目 arm service 对每个候选做
  G1 全身自碰撞采样，环境障碍物碰撞仍因缺少场景模型未实现。
- 父项目已在独立提交接入 `WuxiAdapter.pick_object` 可选后端；默认仍是旧后端，
  尚未做 RPent 进程与真机联调。细粒度接口的文件型参数兼容需求也未确认。
- Dex1-1 型号和厂家模型已高置信确认；驱动量到毫米开口、接触阈值、TCP 精确
  六自由度外参和真实瓶径允许范围仍未做当前真机标定。
- 最新近距离样本已通过 IK 与自碰撞无运动验证；尚需清场后低速单次实机试抓。
- 下一步依次为：同步采集图片与关节状态、可达样本 shadow、标定验收、
  环境清场与急停确认、上层持有者注入控制器、限速小步真机验证。

## 注意事项

- 父、子项目必须分别提交和同步；子项目路径仍由父项目本地排除规则隔离。
- 父项目和子项目使用独立 Git 提交；同步前后都要核对各自 HEAD 和工作区状态。
- 子项目不作 submodule，不要在父项目提交中纳入 `traditional_grasp/` 内容。
- 不得把示例配置直接切为 `live`，不得绕过碰撞检查和标定验证布尔门禁。
- TRAC-IK 只负责运动学求解，不提供碰撞安全保证。
