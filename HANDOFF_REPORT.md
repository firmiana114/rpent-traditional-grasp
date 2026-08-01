# 交接报告

## 项目整体描述

本项目是 RPent 的独立传统瓶体抓取子项目，用于 AIR G1 双臂机器人瓶装物侧抓。
它嵌套部署到父项目目录但不作 submodule；父项目以独立提交接入其规划服务。

核心链路为：双目矫正 → CREStereo 深度 → YOLO-World/SAM2 → 瓶心估计 →
受限侧抓姿态候选 → 关节空间预抓取 → 水平直线插入 → TRAC-IK 连续逆解 →
碰撞检查 → 执行和抓取验证。

## 核心功能与接口

- 高层兼容接口 `search_object`/`approach_object`/`pick_object`/`verify_grasp`，
  核心替代目标是 RPent 的 `pick_object`。本项目提供无运动持久规划服务，父项目
  负责后端选择、控制器所有权、计划复验和执行；`pick_object` 已锁定原仅关键字
  参数及主要结果字段，阶段测试改走内部预览方法。
- 细粒度接口：`compute_depth_crestereo`、`segment_object`、
  `mask_depth_to_pointcloud`、`pointcloud_to_body`、规划和执行。
- `offline` 完整模拟闭环；`shadow` 用真实感知/IK但绝不发运动命令；`live` 必须
  同时通过运动授权、三项标定和碰撞检查门禁。
- 抓取顺序固定为张手、预抓取、抓取位、闭合/接触、抬升、后撤；不自动回零。

## 主要模块与目录

- `src/rpent_traditional_grasp/`：`api.py` 四接口编排与安全门禁；`config.py`/
  `gripper.py` 配置与 Dex1-1 规格校验；`stereo.py` 标定/矫正/CREStereo 适配与执行
  提供者诊断；`perception.py` YOLO-World 检测与 SAM2 框提示分割；`geometry.py`
  商标带鲁棒深度与瓶心估计；`planning.py` ±30° 组合侧抓与插值；`ik.py` 持久
  TRAC-IK 与 FK 残差；`diagnostics.py` 无运动可达性诊断；`execution.py` 碰撞/
  执行器协议与接触证据；`thor.py` 相机适配、注入式 CapX 与感知设备选择；
  `xyz.py` 图片到 TCP 验收；`visualization.py` 双后端 TCP 对比。
- `native/` 官方 TRAC-IK 与 G1 文本链求解器；`robot/` 已核对 URDF 与运动链；
  `scripts/` 运动链导出、原生构建、Thor 与本机影子入口；`tests/` 覆盖几何、配置、
  安全、接口闭环与真实原生 IK。

## 技术栈与外部依赖

- Python 3.10–3.12、NumPy、OpenCV、Ultralytics、SAM2、ONNX Runtime。
- C++17、CMake/Ninja、Eigen3、Orocos KDL、NLopt。
- `traclabs/trac_ik@90162ac2...`，BSD-3-Clause，源码在 `native/vendor/`。
- reBot 仅参考流程概念，未复制其源码和 GraspNet；详见 `UPSTREAM.md`。
- 模型权重与第三方推理代码均为外部资源，不进入本仓库；本机放在
  `/Users/firmiana/project/rpent-models/`（权重分三个子目录，`vendor/` 存两个
  公开仓库）。

## 运行入口、配置与数据流

- 配置入口：Thor 用 `thor.example.json`，本机用 `macos.example.json`，两份共用
  `config/` 下同一批标定文件。感知设备由 `thor.py` 按 cuda→mps→cpu 自动选择，
  可用 `RPENT_TRADITIONAL_GRASP_DEVICE` 强制覆盖。
- 原生入口：`native/build/g1_trac_ik`，每条手臂保持一个求解进程。
- 影子入口：`scripts/run_thor_shadow.py`，默认只运行 `search` 影子流程。
- 日志默认 INFO；关键配置、感知、IK、门禁和执行均保留上下文及异常链；服务在文件描述符层隔离 JSON 回复与第三方输出，父项目另有限量抗噪读取。
- 在线抓取把原始/校正双目、SAM2 框选图、掩码和叠加图保存到父项目单次运行目录，并用帧 SHA-256、输入框、候选分数及掩码框串联 INFO 日志。
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

- 历史结论（详见 Git 历史）：macOS/Thor 原生 TRAC-IK 可用；G1 URDF 与 Thor
  `xr_teleoperate` 逐字节一致；CapX `ArmController` 构造会自动回零、只接受上层
  注入控制器，其 FK 加固定平移后与本项目 `torso_link` 链完全一致；Dex1-1 名义
  TCP `0.150215608966 m`；运动链串联杆长上限 `0.560610 m`。2026-08-01 换用新双目
  标定与新相机外参（`config/thor_stereo_20260801.json`、
  `config/thor_camera_to_body_20260801.json`）：旧标定焦距偏低约 20%，极线错位由
  4.4px 降至 1.6px；仿真两份 yaml 已同步，`x=0.48` 右臂全物理抓取冒烟通过。
  CaP-X 旧后端对照已放弃（分支 `test/capx-body` 仅作记录），其虚拟 TCP、无出处的
  `0.70 m` 臂展预检和低估距离的标定三者互相抵消才“能抓”。
- 桌面缓存图在 Thor 真实模型链路的基准结果：瓶心 body
  `[0.6037,-0.0320,-0.0565] m`、深度 `0.6810 m`、瓶径 `0.0644 m`、MAD `0.0099 m`、
  置信 0.865、有效深度像素 1161。该样本距双肩约 0.68 m，超上限 12 cm，任何外参
  微调都不可达，须底盘先逼近。SGBM 在周期纹理会锁错周期，不可用于标定验收。
- 棋盘 PnP（25 mm 方格，重投影 0.3 px）反解：设计名义外参下桌面法向与竖直差
  `5.63°`（对应瓶心位置变化 `68 mm`），是外参旋转误差上界与当前最大误差源。
- 生产 `pick_object` 没有可达性预检：够不着时只报 `no continuous IK path`，不提示
  需要底盘逼近；几何可达性检查目前只在 `diagnostics.py`。
- 经典算法替换已否决：SGBM 24 ms、GrabCut 292 ms（比 SAM2 的 160 ms 慢 1.8 倍）、
  YOLO-World 无经典替代；SGBM 在现场图直接失败（MAD `76.6 mm` 超 25 mm 门限）。
- **CREStereo 跑 CPU 已修复**（2026-08-01）。根因是影子入口的 `yolo_world` conda
  环境装了纯 CPU 版 onnxruntime（只有 Azure/CPU provider），`object_grab.py`
  请求的 TensorRT/CUDA 被静默回退。同机唯一 GPU 版 onnxruntime 是本地源码编译的
  `onnxruntime_gpu-1.24.0`（`/home/aiot/mingjuwang/Models/onnxruntime/build/` 下，
  有 cp310/cp312/cp313 三份），按 NumPy 1.x 编译，装进 NumPy 2.4.4 的 `yolo_world`
  会 `ImportError: import numpy failed`（已试、已还原，备份在
  `/home/aiot/backup_onnxruntime_cpu/`）。最终方案是 `--system-site-packages` 叠加
  虚拟环境 `/home/aiot/wuxi/venvs/rpent-grasp-gpu`，继承 `yolo_world` 的
  torch 2.13+cu130/cv2/ultralytics/tensorrt/clip，只覆盖 numpy 1.26.4 与 GPU 版
  onnxruntime，`yolo_world` 本体零改动。实测 CREStereo 稳态 `3544 ms → 49.2 ms`
  （72 倍），视差均值差 0.013 px，端到端瓶心差 0.1 mm；两个 Thor 图片入口的
  `THOR_YOLO_PYTHON` 默认值已改指该虚拟环境。
- 本机模型部署已完成，影子链路不再依赖服务器。三个权重全部来自公开发布源且体积
  与 Thor 一致：CREStereo ONNX 25 MB（PINTO model zoo `284_CREStereo` 的
  `resources_iter5.tar.gz`）、YOLO-World `.pt` 140 MB（ultralytics/assets v8.3.0）、
  SAM2 176 MB（`dl.fbaipublicfiles.com`）。第三方推理代码同样取公开仓库：
  `ibaiGorordo/ONNX-CREStereo-Depth-Estimation`（`CREStereo` 类与 Thor 的
  `object_grab.CREStereo` 同源，仅 provider 列表不同）与 `facebookresearch/sam2`。
  本机 `.pt` 走 `set_classes` 需要 CLIP 文本编码器（`ultralytics/CLIP`），Thor 走
  `.engine` 不需要，是两端唯一依赖差异；ultralytics 的 `weights_dir` 已改指
  `rpent-models/ultralytics_weights`，以免 CLIP 权重落进仓库。本机环境
  `/Users/firmiana/project/.venvs/rpent-traditional-grasp-macos`。
- 本机与 Thor 结果一致性（同一对桌面缓存图）：固定同一目标框时瓶心欧氏差
  `0.54 mm`、有效深度像素同为 1161、瓶径差 0.045 mm；全自动检测时 YOLO `.pt` 框
  `(305,176,351,283)` 对 Thor `.engine` 框 `(304,175,351,282)` 差 1 px，瓶心欧氏差
  仍为 `0.54 mm`。残差来自 Thor fp16 TensorRT 与本机 fp32 CPU；权重同源由行为
  一致性证实。本机全链路 8.2 s（macOS 无 GPU provider，CREStereo 落 CPU 2.0 s，
  是可用性方案不是性能方案）。61 项 pytest 通过。

## 未确认、阻塞问题与下一步

- 相机-机身外参为设计名义值复合，旋转偏差上界约 `5.63°`（68 mm），平移方向无任何
  观测约束。必须做手眼标定或多位置人工真值验收，通过前
  `camera_to_body_validated` 保持 false。
- **本轮代码提交尚未同步到 air-thor**：改完 GPU 环境后服务器即以
  `kex_exchange_identification: Connection closed` 拒连（TCP 通、sshd 立即断），
  10 分钟重试未恢复；Thor 端已生效的只是运行环境。恢复后用 `git bundle` 同步，
  注意 Thor 上另有 `api.py`/`perception.py`/`service.py` 三个未提交改动
  （`search_object` 转发与 SAM2 诊断图路径透出）。
- 父项目 `traditional_grasp_backend.py` 的 `AIR_ROBOT_TRADITIONAL_GRASP_PYTHON`
  默认值仍是 `yolo_world`，即**生产规划服务目前仍跑 CPU 版 CREStereo**；本轮只改了
  子项目两个图片入口，父项目改动需在 air-thor 恢复后单独提交。
- 新双目标定尚未做在线相机采集回归；瓶体商标区域在反光、透明瓶、遮挡和低纹理
  场景的深度成功率也尚未实测。
- 环境障碍物碰撞因缺场景模型未实现，实机前需清场和急停；Dex1-1 驱动量到毫米开口、接触阈值、TCP 六自由度外参也未做真机标定。
- `visualization.py` 与三个对比脚本为另一会话的未提交工作，未经本轮验证。
- 下一步：同步 Thor 与父项目 → 手眼标定验收 → 补 `pick_object` 可达性预检 →
  清场急停后限速小步真机验证。

## 注意事项

- 父、子项目必须分别提交和同步；子项目不作 submodule，其路径由父项目本地排除
  规则隔离，不要在父项目提交中纳入 `traditional_grasp/` 内容。
- 不得把示例配置直接切为 `live`，不得绕过碰撞检查和标定验证布尔门禁。
- TRAC-IK 只负责运动学求解，不提供碰撞安全保证。
- `macos.example.json` 与 `rpent-models/` 的绝对路径是本机部署事实，换机器须改；
  两个 `run_macos_*.sh` 支持用环境变量覆盖解释器与 SAM2 仓库。
