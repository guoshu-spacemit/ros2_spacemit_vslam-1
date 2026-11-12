# spacemit-vslam

在 SpacemiT K3 上运行的双目定位与三维建图应用。桌面窗口可显示相机画面、位姿轨迹、双目深度和 TSDF/ESDF 地图。

以下命令在 K3 板端执行。需要 Bianbu 4.0.1、ROS 2 Humble 和可提供嵌入式曝光时间戳与 IMU 数据的双目 UVC 相机。运行账号需要相机、MPP、DMA-BUF 和 GPU 的访问权限，桌面显示还需要已登录的 X11 或 Xwayland 会话。

## 1. 下载 Robot SDK

```bash
sudo apt update
sudo apt install repo wget

mkdir -p ~/spacemit_robot
cd ~/spacemit_robot
repo init -u https://github.com/spacemit-robotics/manifest.git -b main -m default.xml
repo sync
```

项目路径是 `middleware/ros2/slam/spacemit-vslam`。

## 2. 编译

```bash
cd ~/spacemit_robot
source build/envsetup.sh
cd middleware/ros2/slam/spacemit-vslam
mm --with-deps
```

构建入口编译 Robot SDK 的 MPP、LAS2 外部输入后端及 IMU 依赖，以及本项目的 Camera 模块和 ROS 2 包。VSLAM 和 Mapping 随项目提供 K3 Linux RISC-V 64 位预编译库、头文件及 CMake 配置，构建时校验后安装。系统依赖见 [package.xml](package.xml)，首次构建按 SDK 提示安装缺少的依赖。产物安装到 SDK 根目录的 `output/staging`。

板端运行需要 OpenCV 4.10（`libopencv_*.so.410`）、匹配的 MPP 库和插件，以及 OpenCL 和 EGL/GLES/DRM 环境。默认相机配置启用解码输入池，需要包含 `SYS_ConfigStreamDmaBufPool` 的 MPP。

## 3. 下载深度模型

使用运行应用的账号，将 LAS2 模型下载到 `~/.cache/models/vision`。

```bash
mkdir -p ~/.cache/models/vision
cd ~/.cache/models/vision
wget https://archive.spacemit.com/spacemit-ai/model_zoo/vision/stereo_depth/LAS2_M_288x384.fp16.onnx
```

## 4. 启动桌面应用

在板端桌面终端执行。

```bash
cd ~/spacemit_robot
source build/envsetup.sh
sros2_setup
ros2 run bringup run_vslam_desktop.sh
```

桌面启动默认选用已安装的 [UVC 相机配置](src/camera_io/config/usb_stereo_4000x1200.json)和 [Kalibr 标定](src/camera_io/config/0.yaml)。该配置使用 `/dev/video9`，接收 4000×1200 的双目打包 MJPEG，每路输出 640×400。桌面脚本以 30 FPS 采集，可用 `camera_fps:=` 覆盖。

换相机时，复制示例 JSON 并修改 `camera.device`、采集尺寸和双目裁剪。在 JSON 的 `calibration.file` 中填写实机 Kalibr 文件，供相机节点生成 CameraInfo。相对路径以 JSON 所在目录解析。启动时将同一份双目标定传给 VIO 和 LAS2。如果 MPP 缺少解码输入池接口，将配置中的 `buffers.decoder_input_pool.enabled` 设为 `false`。

```bash
ros2 run bringup run_vslam_desktop.sh \
    camera_config_file:=/path/to/camera.json \
    calibration_file:=/path/to/calibration.yaml
```

标定文件可同时包含双目和 IMU 参数。IMU 标定单独保存时，再传入 `imu_calibration_file`。启动后，在有纹理的场景中缓慢移动相机以完成惯性初始化。按 `Ctrl+C` 退出。

## 录制和回放

先退出正在使用相机的程序。录制目录须为新目录。以下录制命令使用示例相机配置，使用其他相机时传入与桌面启动相同的配置和标定。

```bash
ros2 launch bringup vio_record_launch.py \
    session_directory:="$HOME/recording" \
    model_path:="$HOME/.cache/models/vision/LAS2_M_288x384.fp16.onnx"
```

用 `Ctrl+C` 结束录制，再指定录制目录和新的输出目录进行回放。

```bash
ros2 run bringup run_vio_replay.sh "$HOME/recording" "$HOME/replay"
```

回放使用会话中的标定和录制数据。旧会话若启用了静止初始化或设置了最小观测深度，可复制其参数文件，将这两项恢复为 `false` 和 `0.0`，再通过 `vio_parameters_file` 传入副本。桌面启动的完整参数可用 `ros2 launch bringup vio_las2_esdf_launch.py --show-args` 查看。

后台启动脚本会打印日志目录。用同一脚本的 `--status` 查看启动状态或异常退出原因，用 `--stop` 关闭该会话。进程开始运行后，相机和模型仍需初始化。

## 模块与 ROS 2 接口

| ROS 2 包 | 职责和主要接口 |
| --- | --- |
| `camera_io` | 用 MPP 采集及解码，用 `spacemit_camera` 处理元数据和标定。发布 `/camera/left/frame_desc`、`/camera/right/frame_desc` 和 `/camera/imu/data_raw`，并提供录制和回放节点 |
| `vio` | 使用 `spacemit_vslam` 处理双目图像与 IMU，在 `/vio/pose/image` 发布图像时刻位姿（`nav_msgs/msg/Odometry`） |
| `las2` | 使用 Robot SDK 的 `stereo_matching::external_input` 推理双目深度，在 `/las2/depth_desc` 发布深度描述符（`common/msg/DmaDepthFrameDesc`） |
| `esdf_drm` | 使用 `spacemit_mapping` 按图像位姿融合深度并更新 TSDF 和 ESDF，向桌面发布 `/esdf/render/frame_desc` |
| `common` | 定义深度和特征消息，提供生命周期工具 |
| `bringup` | 组织桌面、录制、回放等启动流程 |

图像和深度像素保留在 DMA-BUF 中，ROS 2 消息携带缓冲区描述符和时间戳。相机和渲染的图像描述符类型均为 `mpp_msgs/msg/MppVideoFrameDesc`。

## C++ 接入

| 功能 | 公开头文件 | CMake 目标 |
| --- | --- | --- |
| 定位 | [`spacemit_vslam.h`](modules/vslam/include/spacemit_vslam.h) | `spacemit_vslam::spacemit_vslam` |
| 建图和渲染 | [`spacemit_mapping.h`](modules/mapping/include/spacemit_mapping.h) | `spacemit_mapping::spacemit_mapping` |

定位库支持双目 VO/VIO 和在线全局 SLAM。定位地图保存和加载重定位暂不可用。TSDF/ESDF 建图由独立的建图库完成。

项目许可证见 [LICENSE](LICENSE)。预编译库的授权及第三方声明汇总在 [NOTICE](NOTICE)，安装时与 `LICENSE` 一起放在 `share/licenses/spacemit-vslam/`。
