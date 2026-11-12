# spacemit-vslam

在 SpacemiT K3 上运行的双目定位与三维建图应用。桌面窗口可显示相机画面、位姿轨迹、双目深度和 TSDF/ESDF 地图。

## 1. 下载 Robot SDK

以下命令在 K3 板端执行。准备 Bianbu 4.0.1、ROS 2 Humble 和带嵌入式曝光时间戳与 IMU 的双目相机。桌面显示需要已登录的 X11 或 Xwayland 会话。

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

首次构建时，按 SDK 提示安装缺少的依赖。产物安装到 SDK 根目录的 `output/staging`。定位库 `libspacemit_vslam.so` 使用板端 OpenCV 4.10（`libopencv_*.so.410`）。

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

默认使用已安装的[相机配置示例](src/camera_io/config/usb_stereo_4000x1200.json)和[标定示例](src/camera_io/config/0.yaml)。其他相机请使用实机 Kalibr 标定，并在相机 JSON 中填写设备路径和相应的图像配置。

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

## C++ 接入

| 功能 | 公开头文件 | CMake 目标 |
| --- | --- | --- |
| 定位 | [`spacemit_vslam.h`](modules/vslam/include/spacemit_vslam.h) | `spacemit_vslam::spacemit_vslam` |
| 建图和渲染 | [`spacemit_mapping.h`](modules/mapping/include/spacemit_mapping.h) | `spacemit_mapping::spacemit_mapping` |

定位库支持双目 VO/VIO 和在线全局 SLAM。定位地图保存和加载重定位暂不可用。TSDF/ESDF 建图由独立的建图库完成。

项目许可证见 [LICENSE](LICENSE)。预编译库的授权及第三方声明汇总在 [NOTICE](NOTICE)，安装时与 `LICENSE` 一起放在 `share/licenses/spacemit-vslam/`。
