cmake_minimum_required(VERSION 3.16)
project(bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(
  FILES scripts/calibration_bundle.py scripts/run_detached_launch.py
    scripts/sdk_paths.sh scripts/terminal_ui.py
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(
  PROGRAMS
    scripts/replay_session_config.py
    scripts/replay_command.py
    scripts/check_vio_runtime.py
    scripts/replay_imu_receipt.py
    scripts/run_vio_session.sh
    scripts/vio_session_config.py
    scripts/run_reconstruction_video_transmission.sh
    scripts/reconstruction_video_transmission_config.py
    scripts/run_checked_launch.py
    scripts/run_vio_replay.sh
    scripts/replay_keyboard.py
    scripts/check_vio_replay.py
    scripts/run_vio_record.sh
    scripts/check_vio_recording.py
    scripts/run_stereo_las2_rtsp.sh
    scripts/run_headless_esdf_rtsp.sh
    scripts/run_live_vio_foxglove.sh
    scripts/run_vio_las2_esdf.sh
    scripts/run_vslam_desktop.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
