# 上游来源与改造边界

本目录实现 RPent 的传统几何抓取，不包含 GraspNet。

- `reBot-DevArm-Grasp`：
  `EclipseaHime017/reBot-DevArm-Grasp@5b8b2bd4055dfa962bb6ab13a25f6f3a4127653d`。
  仅参考了其“检测/分割、掩码中央截面、深度反投影”的流程概念；
  本目录为瓶体商标带、几何圆柱中心、固定侧抓与连续七轴逆解重新实现，
  未复制 GraspNet 目录及模型。该仓库 README 声称 MIT，但核实时仓库中没有
  LICENSE 文件，因此不能把其源码作为可再分发依赖；部署前仍需项目方确认许可。
- `traclabs/trac_ik`：
  `90162ac2ecc6ea8f88c6e99df6ee01efd217a3fb`（rolling）。
  `native/vendor/trac_ik` 保留其 BSD-3-Clause 许可证与文件头。为脱离 ROS2，
  本项目只增加了最小 `rclcpp` 时间/日志兼容层，排除 ROS 参数和 URDF
  构造器，并修正了上游 Node 构造器中 `_chain` 的明显变量引用错误。
- `xr_teleoperate`：
  `OpenTeleVision/TeleVision` 派生项目
  `EclipseaHime017/xr_teleoperate@64ed45b4177e6297936940866df623b72621643a`。
  `robot/g1_body29_hand14.urdf` 的 SHA-256 为
  `8bbf006633fc50b616f665c7a970780cc296577a0adfd7d28b049e751c238735`，
  与此前在 Thor 核实的模型逐字节一致。

Thor 上的 YOLO-World、SAM2、CREStereo 权重不进入仓库，通过 JSON 配置或
`RPENT_*` 环境变量注入。2026-07-31 已在 Thor 核实
`thor.example.json` 中的 YOLO-World TensorRT/PT 权重、SAM2 仓库/配置/权重、
CREStereo 代码/ONNX 权重和机器人 URDF 路径均存在。配置中的双目标定与
相机到机身外参来自 Thor 旧版 `object_grab.py`，仅作为
`legacy_unvalidated` 影子测试输入，尚未完成面向当前相机的标定验收，不能授权
真机运动。
