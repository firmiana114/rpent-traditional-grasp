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
