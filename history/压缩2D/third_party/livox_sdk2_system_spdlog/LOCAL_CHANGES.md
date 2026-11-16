# Local Livox SDK2 with system spdlog

Source: read-only copy of /home/fishbone/GO2/Livox-SDK2 (SDK 0.0.2), excluding build, samples, Git metadata and bundled spdlog. Original license files are retained.

The previous shared SDK embedded spdlog 1.3.1 headers while ROS loaded system spdlog 1.9.2. The captured shutdown crash is in libspdlog.so.1 from the SDK's global shared_ptr<spdlog::logger> destructor. This matches the upstream report https://github.com/Livox-SDK/Livox-SDK2/issues/44 .

Changes: link both SDK targets against system spdlog::spdlog; build/install only the shared target, with distinct SONAME liblivox_lidar_sdk_hap.so. The SDK logger is private, is never registered globally, and is flushed/reset instead of calling spdlog::drop_all() on other applications' loggers. See also https://github.com/Livox-SDK/Livox-SDK2/issues/122 .

Run `bash scripts/build_livox_sdk.sh` from HAP_ws. scripts/build.sh calls it before colcon. The ROS 2 driver CMake selects this local library when installed and records its path in RUNPATH. No /usr/local or original GO2 files are modified.

Regression: reports/check_livox_shutdown_20261102.py starts only the real laser driver for two cycles and asserts normal SIGINT shutdown. It publishes no robot or navigation commands.
