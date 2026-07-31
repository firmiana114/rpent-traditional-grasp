# 交接报告

## 项目整体描述

本项目是 RPent 的独立传统瓶体抓取子项目，目标是在 AIR G1 双臂机器人上完成
不依赖 GraspNet 的瓶装物侧抓。它是独立 Git 仓库，可嵌套部署到上层 RPent 的
`traditional_grasp/`，但不作为 submodule，也不修改上层仓库的受控文件、分支、
索引或远端。

核心链路为：双目取帧和矫正 → CREStereo 深度 → YOLO-World 目标框 →
SAM2 掩码 → 商标中央带鲁棒深度 → 圆柱瓶身中心估计 → 固定侧抓笛卡尔路径 →
官方 TRAC-IK 连续 7 轴逆解 → 外部碰撞检查 → 执行和抓取验证。

## 核心功能与接口

- 高层兼容接口：`search_object`、`approach_object`、`pick_object`、
  `verify_grasp`。
- 核心替代目标是 RPent 的 `pick_object` 逻辑；当前只在独立仓库按该方法的
  感知、规划、执行职责分阶段验收，尚未接入或修改父项目 RPent。
- `pick_object` 已锁定原仅关键字参数及主要结果字段；阶段测试改走内部预览
  方法，不向未来 RPent 调用暴露测试开关。
- 细粒度接口：`compute_depth_crestereo`、`segment_object`、
  `mask_depth_to_pointcloud`、`pointcloud_to_body`、
  `plan_contact_grasp`、`execute_grasp`。
- `offline` 用于完整模拟闭环；`shadow` 使用真实感知/IK但绝不发运动命令；
  `live` 必须同时通过运动授权、三项标定和碰撞检查门禁。
- 抓取执行顺序固定为张手、到达预抓取、到达抓取位、闭合/接触、抬升、后撤。
  构造器和接口均不自动回零。

## 主要模块与目录

- `src/rpent_traditional_grasp/api.py`：四接口编排、状态和安全门禁。
- `config.py`：JSON 配置、路径解析、`live` 安全配置校验。
- `stereo.py`：双目标定、矫正、外部 CREStereo 适配和深度计算。
- `perception.py`：YOLO-World 检测与 SAM2 框提示分割适配。
- `geometry.py`：商标带、鲁棒深度、瓶径和三维瓶心估计。
- `planning.py`：固定侧抓姿态和笛卡尔插值。
- `ik.py`：持久 TRAC-IK 子进程、上一解播种、FK 残差和关节变化诊断。
- `execution.py`：碰撞检查协议、执行器协议、模拟执行器与接触证据。
- `thor.py`：Thor 相机、现有模型资源和注入式 CapX/碰撞检查适配。
- `native/`：官方 TRAC-IK 核心、ROS2 最小兼容层和 G1 文本链求解器。
- `robot/`：经核对的 G1 URDF 与左右 7 轴运动链。
- `scripts/`：运动链导出、Thor 无 sudo 原生构建、影子运行入口。
- `xyz.py` 与 `scripts/run_image_to_xyz.py`：左右图片到相机/机身 XYZ 的
  物体/最终夹爪 TCP 结构化验收及可选真值误差门禁；不启动 IK 或控制器。
- `tests/traditional_grasp/`：几何、配置、安全、接口闭环和真实原生 IK 测试。

## 技术栈与外部依赖

- Python 3.10–3.12、NumPy、OpenCV、Ultralytics、SAM2、ONNX Runtime。
- C++17、CMake/Ninja、Eigen3、Orocos KDL、NLopt。
- `traclabs/trac_ik@90162ac2...`，BSD-3-Clause，源码保留在
  `native/vendor/trac_ik/`。
- reBot 仅参考流程概念，未复制其源码和 GraspNet；详见 `UPSTREAM.md`。
- Thor 的模型权重均为外部资源，不进入本仓库。

## 运行入口、配置与数据流

- 配置入口：`thor.example.json`；双目标定和相机外参分别位于 `config/`。
- 原生入口：`native/build/g1_trac_ik`，每条手臂保持一个求解进程。
- Thor 入口：`scripts/run_thor_shadow.py`，默认只运行 `search` 影子流程。
- 日志入口：`logging.py`，默认 INFO；关键配置、感知、几何、IK、自适应细分、
  碰撞门禁和执行阶段均记录诊断上下文，异常保留原始异常链。
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
```

## 当前状态与已验证事实

- 本机 Linux/aarch64 容器已通过 C++ 无警告构建、左右臂 CTest 和 32 项
  pytest；Thor 正式目录使用现有原生二进制同样为 32 项全部通过。
- Thor 已将本项目克隆为独立嵌套仓库；无 sudo 本地依赖构建和左右臂原生自检
  通过，上层 RPent 的分支、HEAD、索引树和工作区状态在部署前后完全一致。
- GitHub 私有远端为 `firmiana114/rpent-traditional-grasp`。
- 已移除本项目自设的 `0.18 rad` 关节跳变硬门限，与原始 `pick_object`
  对齐；最大关节变化仍写入 INFO 日志及返回结果，仅作为诊断指标。
- G1 URDF SHA-256 为
  `8bbf006633fc50b616f665c7a970780cc296577a0adfd7d28b049e751c238735`，
  与 Thor 上 `xr_teleoperate` 模型逐字节一致。
- Thor 已核实 YOLO-World、SAM2、CREStereo、URDF 外部资源存在；
  `abot-claw` 环境可导入相关推理依赖。
- Thor 是 Ubuntu 24.04/aarch64，账号无免密 sudo；无 sudo 依赖构建脚本已提供。
- 当前 CapX `ArmController` 构造器会自动回零，因此本项目只接受上层持有者注入
  的控制器，禁止自行实例化。
- Thor 已使用用户指定的历史左右图完成真实 shadow 感知：CREStereo 有效深度
  比例 1.000，YOLO 检出瓶子（0.887），SAM2 掩码 3356 像素（0.974），估计
  深度 0.567 m、瓶径 0.061 m、深度 MAD 0.0072 m。
- 第一阶段独立图片入口已在 Thor 临时副本用同一历史样本复现；输出相机瓶心
  `[-0.0544, -0.0992, 0.5973]` m、机身瓶心
  `[0.5242, 0.0841, 0.0618]` m。该结果是软件回归基线，不是物理真值。
- 第二阶段在运动学模型中将夹爪 TCP 定义为两指抓取中心；最终 TCP XYZ 与
  瓶体抓取中心相同，工具链已有 0.05 m 腕部偏移，不重复补偿。姿态继承初始
  末端旋转并保持不变；TCP 真机标定门禁当前为 false。
- 第二阶段经内部 `preview_pick_object_xyz` 进入，与 `pick_object` 共用感知
  和目标计算逻辑，并在 IK 和运动前返回。
- 第二阶段独立入口已在 Thor 临时副本用历史图片完成真实推理回归：自动选择
  左臂，输出 TCP `[0.5242, 0.0841, 0.0618]` m，未启动 IK 或控制器。
- 验证图片是 `../logs/20260730-15:39:59_air_robot_task_s0/sensor/` 中同时间戳
  `20260730_154013_543214444` 的 `left_*.jpg` 与 `right_*.jpg`。
- 该样本瓶心为机身坐标 `[0.524, 0.084, 0.062]` m；移除 `0.18 rad` 门限后
  已复验，左右臂分别在预抓取第 13/19、16/29 段因 TRAC-IK 无解而规划失败。

## 未确认、阻塞问题与下一步

- 按当前验证约束仅使用历史左右图；在线相机采集暂不继续。
- `config/` 中的双目标定和相机到机身外参来自旧代码，状态为
  `legacy_unvalidated`；必须重标定并量化误差。
- 第一阶段尚缺人工测量的多位置机身坐标真值；在至少覆盖近/中/远和左右视野
  的样本通过误差阈值前，不得把可重复 XYZ 等同于物理坐标准确。
- 瓶体商标区域在反光、透明瓶、遮挡和低纹理场景的深度成功率尚未实测。
- 机器人自碰撞、环境碰撞和路径扫掠检查器尚未实现；`live` 因此保持阻断。
- 上层 RPent 到高层四接口的最终接线未完成；上层当前有人并行修改，禁止直接
  改其受控文件。细粒度接口的文件型参数兼容需求也未确认。
- 手爪闭合量、接触阈值、TCP 精确外参和真实瓶径允许范围尚未做真机标定。
- 必须确认目标超出当前手臂工作区时，是由 `approach_object` 接入底盘靠近，
  还是要求选取/摆放到无需底盘的近距离样本；本轮没有底盘执行授权。
- 下一步依次为：确认底盘靠近策略、标定验收、碰撞检查接入、上层持有者完成
  控制器注入、限速小步真机验证。

## 注意事项

- 上层 RPent 只允许在本机 `.git/info/exclude` 加 `/traditional_grasp/`；
  同步前后必须核对其分支、HEAD 和 `git status --short` 完全一致。
- Thor 父项目当前另有 `arm_service.py`、`vision_service.py` 并行未提交修改，
  时间早于本轮服务器复验；本项目未改动、暂存或覆盖它们。
- 不得在上层 RPent 执行本项目提交、拉取、切分支或 submodule 操作。
- 不得把示例配置直接切为 `live`，不得绕过碰撞检查和标定验证布尔门禁。
- TRAC-IK 只负责运动学求解，不提供碰撞安全保证。
