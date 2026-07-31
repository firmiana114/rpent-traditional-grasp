# RPent 独立传统瓶体抓取

本目录是可独立提交、构建和部署的 Git 仓库。部署时可以放在上层 RPent
工作树的 `traditional_grasp/`，但不作为 submodule，不修改上层仓库的分支、
索引、远端或任何受版本控制文件。上层仓库只需在其本机
`.git/info/exclude` 中加入 `/traditional_grasp/`，这样协作者原有的
`git status`、提交和拉取流程不会出现这个嵌套仓库。

技术链路：

`YOLO-World 检测 -> SAM2 框提示分割 -> 矫正双目/CREStereo 深度 ->
商标带鲁棒深度与瓶身几何中心 -> 固定侧抓路径 -> 连续 7 轴 TRAC-IK ->
碰撞检查 -> 执行与接触/抬升验证`

对外保留四个接口名：

- `search_object`
- `approach_object`
- `pick_object`
- `verify_grasp`

其中 `pick_object(object_prompt, arm_side, bbox, bbox_format)` 是本项目需要
独立验证并最终替换的核心逻辑。RPent 当前链路是
`AirRobotClient.pick_object -> WuxiAdapter.pick_object -> 传统检测 ->
机械臂执行`；本阶段不修改或接入 RPent，只在本仓库内按相同职责验证
`TraditionalGraspAPI.pick_object`。待各阶段可靠性达标后，再替换 RPent
适配层的后端实现。该方法保持原有的仅关键字参数，以及 `success`、`status`、
`requested_arm_side`、`selected_arm_side`、`verification`、`execution`
等返回语义；阶段测试能力不会增加到该外部签名中。

同时保留 `compute_depth_crestereo`、`segment_object`、
`mask_depth_to_pointcloud`、`pointcloud_to_body`、`plan_contact_grasp` 和
`execute_grasp` 细粒度方法，便于接回 Thor 现有适配层。

## 本机构建

在本独立仓库根目录执行：

```bash
python3 scripts/export_g1_chains.py \
  robot/g1_body29_hand14.urdf \
  robot/chains

docker build \
  -t rpent-traditional-grasp:test \
  .

docker run --rm rpent-traditional-grasp:test
```

镜像构建会在 Linux/aarch64 下安装 KDL 与 NLopt、编译官方 TRAC-IK 核心、
对左右臂各做一次真实 IK/FK 自检；容器启动后运行 Python 几何、连续逆解和
四接口离线闭环测试。

## Thor 原生构建

Thor 是 Ubuntu 24.04/aarch64，现有账号没有免密 sudo。脚本使用
`apt-get download` 将开发包下载到临时目录，再解包到忽略版本控制的
`native/.deps/`；不会安装系统包或修改 dpkg 数据库：

```bash
./scripts/build_native_thor.sh
```

随后在 Thor 已有的 `abot-claw` Python 环境中运行：

```bash
PYTHONPATH=src python -m pytest -q
./scripts/run_thor_image_shadow.sh \
  --config thor.example.json \
  --left-image /path/to/left.jpg \
  --right-image /path/to/right.jpg \
  --operation pick
```

图片模式不会初始化 ZMQ 或采集在线相机。`shadow` 会使用指定图片、
CREStereo、YOLO-World、SAM2 与 TRAC-IK，但始终使用模拟执行器，不会实例化
CapX 控制器，也不会发送手臂指令。只有显式增加 `--online-camera` 才会启用
在线相机，并且该参数不能与左右图片同时使用。

Thor 的 `yolo_world` 环境已有 TensorRT、CLIP 和 CUDA，但缺少 SAM2 的三个
纯 Python 运行包；上述包装脚本只在临时目录中链接 `abot-claw` 已有的
`hydra`、`iopath` 和 `portalocker`，退出时删除临时目录，不安装或修改任何
共享 Conda 环境。代码同时强制关闭 Ultralytics 自动安装。

## 第一阶段：左右图片输出 XYZ

独立验收入口只读取指定左右目图片，不连接在线相机，不构造 CapX 控制器，也
不启动 TRAC-IK 进程，不发送任何机器人运动。它输出瓶体几何中心在左相机
光学坐标系和机器人机身坐标系中的 XYZ，以及深度离散度、有效像素数和标定
状态：

```bash
./scripts/run_thor_image_xyz.sh \
  --config thor.example.json \
  --left-image /path/to/left.jpg \
  --right-image /path/to/right.jpg \
  --target bottle \
  --output-json /tmp/image_xyz.json
```

机身坐标约定为 x 向前、y 向左、z 向上；相机坐标约定为 x 向右、y 向下、
z 向前。`object_center_*` 是用估计瓶径从可见前表面向瓶内补偿半径后的瓶体
几何中心，`front_surface_camera_xyz_m` 则是不做半径补偿的前表面点。

如果已有人工测量的机身坐标，可直接进行欧氏距离误差验收：

```bash
./scripts/run_thor_image_xyz.sh \
  --left-image /path/to/left.jpg \
  --right-image /path/to/right.jpg \
  --expected-body-xyz-m 0.52 0.08 0.06 \
  --tolerance-m 0.03
```

返回码为 0 表示成功输出 XYZ，且在提供真值时误差不超过阈值；找不到目标或
真值超差返回 1，配置、依赖、文件读写或推理异常返回 2。没有提供真值时，
`acceptance.evaluated=false`，这只证明链路可运行，不能证明物理精度正确。
当前示例标定仍为 `legacy_unvalidated`，因此输出中的
`calibration.metric_xyz_approved` 保持 `false`。

已知目标框时可增加 `--bbox X1 Y1 X2 Y2 --bbox-format pixel`，绕过
YOLO-World 检测，单独验证分割、双目深度和坐标变换。`--output-json` 使用
临时文件加原子替换写入纯 JSON，避免第三方推理库的控制台诊断污染结果文件。

## 第二阶段：左右图片输出最终夹爪 XYZ

第二阶段保持夹爪初始姿态完全不变，只输出最终夹爪 TCP（工具中心点）需要
到达的机身坐标。入口调用内部阶段方法 `preview_pick_object_xyz`，它与
`pick_object` 共用感知和目标计算逻辑，但不会改变 RPent 所见的外部方法
签名，也不读取关节、不运行 IK、不发送运动：

```bash
./scripts/run_thor_image_gripper_xyz.sh \
  --config thor.example.json \
  --left-image /path/to/left.jpg \
  --right-image /path/to/right.jpg \
  --arm auto \
  --output-json /tmp/gripper_xyz.json
```

`gripper_target.final_tcp_body_xyz_m` 是最终输出。`auto` 根据瓶体机身 y 坐标
选择同侧手臂：y 大于等于 0 选左臂，y 小于 0 选右臂；也可以显式指定
`--arm left` 或 `--arm right`。输出会记录
`orientation_policy=preserve_initial` 和 `orientation_commanded=false`。

本项目运动学模型的 `left_tcp_link/right_tcp_link` 定义在两指抓取中心，导出
的运动链已经包含腕部到 TCP 的固定 `0.05 m` 偏移。因此最终 TCP XYZ 等于
估计的瓶体抓取中心 XYZ；不能再次减去 `0.05 m`，否则会重复应用工具偏移。
后续 IK 路径也从机器人当前末端位姿读取初始旋转，并在预抓取、抓取、抬升和
后撤全程保持该旋转不变。

`calibration.metric_gripper_xyz_approved` 只有在双目标定、相机到机身外参和
夹爪 TCP 外参三项均已验证时才为 `true`。示例中的
`gripper_tcp_calibration_validated=false` 表示当前 0.05 m 仍是模型值，尚未
通过真机测量确认；这不阻止离线输出，但禁止把结果当作已批准的真机坐标。

若有人工测量的最终夹爪 TCP 真值，可增加
`--expected-gripper-xyz-m X Y Z --tolerance-m 0.03` 进行误差验收。这里只
输出目标，不代表目标已通过 IK 或碰撞检查。

## 仓库隔离验收

同步前后分别在上层 RPent 运行以下只读命令，分支、HEAD 与状态输出应一致：

```bash
git branch --show-current
git rev-parse HEAD
git status --short
```

子项目必须满足：

```bash
git -C traditional_grasp rev-parse --show-toplevel
git -C traditional_grasp rev-parse HEAD
```

第一条输出必须指向 `traditional_grasp` 自身，不能指向上层 RPent。

## Thor 上线边界

1. 以 `thor.example.json` 为模板配置资源。不要直接把示例切为 `live`。
2. 使用同步标定板数据复核双目重投影误差，重新核实相机到 `torso_link` 外参。
3. 由上层 RPent 已经持有控制权的进程注入 CapX 控制器，禁止本项目自行构造
   `ArmController`，因为当前构造器会自动回零。
4. 实现并注入机器人自碰撞、环境碰撞和路径扫掠检查器。
5. 先以 `shadow` 跑录制帧和在线帧，只记录检测、几何和 IK，不发送运动。
6. 低速、软限位、急停就绪后才将两个标定验证位和 `allow_motion` 同时置真。

构造器和抓取 API 都不会自动回零。TRAC-IK 不做碰撞检查，因此真机配置默认
要求外部碰撞检查器，缺失时失败关闭。
