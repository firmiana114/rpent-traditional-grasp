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
- `visualization.py` 与可视化/对比脚本：投影两套 `pick_object` 的机身 TCP，
  标注矫正后的左右目图片并输出差值；固定图片复验全程不发送运动。
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

- 历史结论（详见 Git 历史）：macOS/Thor 原生 TRAC-IK 可用；G1 URDF 与
  Thor `xr_teleoperate` 逐字节一致；CapX `ArmController` 构造会自动回零，
  只接受上层注入控制器；CapX FK 以 `pelvis` 为根，加固定平移后与本项目
  `torso_link` 链完全一致；Dex1-1 名义 TCP `0.150215608966 m`；运动链
  串联杆长上限 `0.560610 m`；Thor 历史图 shadow 感知与两阶段图片入口均
  已回归；最新现场目标经 38 点组合侧抓与父项目 87 点自碰撞采样验证。
- 2026-08-01 换新双目标定 `config/thor_stereo_20260801.json`（Thor 9x6 内角点
  25 mm 棋盘）：旧标定焦距偏低约 20%、相对旋转差 1.6°，缓存图 SIFT 极线错位
  由 4.4px 降至 1.6px。新外参 `config/thor_camera_to_body_20260801.json` 由
  URDF `d435_joint` 设计俯仰与新标定矫正旋转 R1（8.45°）复合、平移取模组中心
  偏半基线；旧 legacy 外参是 `object_grab.py` 手写值且混淆矫正系，已替换保留。
  `thor.example.json` 指向两新文件，门禁保持 false，53 项 pytest 通过。
- 仿真仓库已同步新标定与新外参（repo 与 macos 两份 yaml）；仿真 16 项
  测试通过，`x=0.48` 右臂全物理抓取冒烟通过（感知误差 7 mm）。
- CaP-X 旧后端对照并已放弃（分支 `test/capx-body` 仅作记录）：其 IK 末端
  为腕前 `0.05 m` 加经验修正（残差 19–43 mm）、臂展预检 `0.70 m` 无出处、
  标定低估距离，三者在现场近距互相抵消才“能抓”。搬入本项目后仿真实证：
  原生距离 0.554 m 全候选无解，近距 0.38 m 因手指超伸 10 cm 撞倒瓶子。
- 新标定已在 Thor 真实模型链路（CREStereo/YOLO-World/SAM2）用桌面缓存图
  跑通 `search`：瓶心 body `[0.6036,-0.0320,-0.0564] m`、深度 `0.681 m`、
  瓶径 `0.0644 m`、深度 MAD `0.0099 m`、置信 0.865。瓶径与实物瓶吻合；
  与本机 SGBM 近似深度 `0.6845 m` 相差 3.6 mm，互为交叉验证。该样本距
  双肩约 `0.68 m`，超上限 12 cm，任何外参微调都不可达，须底盘先逼近。
  SGBM 在棋盘/地毯等周期纹理会锁错周期，不可用于标定验收。
- 棋盘 PnP（25 mm 方格，重投影误差 0.3 px）反解：设计名义外参下桌面法向
  与竖直差 `5.63°`；使桌面水平需 pitch `-5.35°`、roll `+1.75°`，对应瓶心
  位置变化 `68 mm`。该角度是外参旋转误差的上界（含桌面自身倾斜未分离），
  是当前链路最大误差源。
- 生产 `pick_object` 路径没有可达性预检：够不着时报 `no continuous IK
  path`，不提示需要底盘逼近；几何可达性检查目前只存在于 `diagnostics.py`。
- **CREStereo 一直跑在 CPU 上**：影子入口使用的 `yolo_world` conda 环境装的是
  纯 CPU 版 onnxruntime（仅 Azure/CPU provider），`object_grab.py` 请求的
  TensorRT/CUDA 被静默回退。同机 `abot-claw` 环境有 TensorRT/CUDA provider，
  实测同一模型同一对图：CPU 稳态 `3579 ms`，GPU 稳态 `49 ms`，相差 73 倍。
  全链路稳态由 `3747 ms` 降到 `217 ms`，无任何精度代价。
- 经典算法替换同机实测（Thor，稳态中位）：SGBM 深度 `24 ms`、GrabCut 分割
  `292 ms`、YOLO-World（TensorRT）`8 ms`、SAM2 `160 ms`。GrabCut 比 SAM2 慢
  1.8 倍；YOLO-World 无经典替代（GrabCut 需要外部输入框，不能检测）。
  精度：场景一掩码 IoU 0.929，瓶心偏移 GrabCut 2.4 mm、SGBM 6.2 mm、两者
  9.1 mm；SGBM 掩码内深度覆盖 0.907 且 MAD 14.3 mm（CREStereo 1.000/9.9 mm）。
  场景二（现场历史图）SGBM 直接失败：商标带深度 MAD `76.6 mm` 超 25 mm 门限
  被拒，CREStereo 同图 MAD 仅 5.5 mm。SGBM 不是精度略差而是不可靠。
- 外部模型体积：CREStereo ONNX 25 MB、YOLO-World `.pt` 140 MB、
  SAM2 checkpoint 176 MB，合计约 341 MB，具备本机部署条件；`.engine`
  为 TensorRT 产物仅限 NVIDIA，本机须走 `.pt`。

## 未确认、阻塞问题与下一步

- 相机-机身外参为设计名义值复合，已测得约 `5.63°` 旋转偏差上界（68 mm
  位置影响），是当前最大误差源；平移方向尚无任何观测约束。必须做手眼
  标定或多位置人工真值验收，通过前 `camera_to_body_validated` 保持 false。
- 新双目标定已用缓存图在 Thor 真实模型链路回归；尚未做在线相机采集回归。
- air-thor 经常离线，两端同步性差；计划把 CREStereo/YOLO-World/SAM2 及其
  依赖部署到本机，使影子链路不依赖服务器。本机须走 `.pt`（`.engine` 仅限
  NVIDIA），且需装 onnxruntime/torch/ultralytics。
- 待办：给影子入口换用带 GPU provider 的 onnxruntime（或改指 `abot-claw`
  环境），把 CREStereo 从 3579 ms 降到 49 ms；这是当前性价比最高的一项。
- 瓶体商标区域在反光、透明瓶、遮挡和低纹理场景的深度成功率尚未实测。
- 环境障碍物碰撞仍因缺少场景模型未实现；实机前需人工清场和急停。
- Dex1-1 驱动量到毫米开口、接触阈值、TCP 六自由度外参未做真机标定。
- `visualization.py` 与三个对比脚本为另一会话的未提交工作，未经本轮验证。
- 下一步依次为：Thor 端标定生效回归、手眼标定验收、同步采集图片与关节
  状态、清场急停后限速小步真机验证。

## 注意事项

- 父、子项目必须分别提交和同步；子项目路径仍由父项目本地排除规则隔离。
- 父项目和子项目使用独立 Git 提交；同步前后都要核对各自 HEAD 和工作区状态。
- 子项目不作 submodule，不要在父项目提交中纳入 `traditional_grasp/` 内容。
- 不得把示例配置直接切为 `live`，不得绕过碰撞检查和标定验证布尔门禁。
- TRAC-IK 只负责运动学求解，不提供碰撞安全保证。
