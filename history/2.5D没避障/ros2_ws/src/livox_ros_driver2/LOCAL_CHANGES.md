# HAP navigation clock correction

This workspace copy retains the MIT-licensed Livox ROS Driver 2 source from
`/home/fishbone/GO2/livox_ws/src/livox_ros_driver2` (version 1.2.7).
The original workspace is unchanged. Source `scripts/env.sh` to select this
workspace's overlay after building it.

For `time_type=0`, upstream `GetEthPacketTimestamp` substituted host callback
receipt time for the device timestamp. Under scheduling jitter, this made
regular IMU samples appear separated by more than FAST-LIO's 50 ms guard and
also distorted point timing during turns. HAP's protocol specifies that type
0 contains nanoseconds since device power-on:
<https://github.com/Livox-SDK/Livox-SDK2/wiki/Livox-SDK-Communication-Protocol-HAP#22-%E6%97%B6%E9%97%B4%E6%88%B3>.

`DeviceClock` estimates a per-device ROS epoch from the smallest arrival
offset in the first second, then freezes that offset. LiDAR and IMU share it.
Device sampling intervals are preserved despite delivery jitter; duplicates,
gaps and time regressions are never clamped or filled. PTP/GPS timestamps
retain the original direct path. FAST-LIO's existing input and estimator
guards are unchanged. The first second is a clock calibration interval and
is covered by the existing stationary estimator initialization.

Build:

```bash
source scripts/env.sh
cd ros2_ws
colcon build --packages-select livox_ros_driver2 --symlink-install \
  --allow-overriding livox_ros_driver2 --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

Standalone clock regression (no robot or ROS participant):

```bash
g++ -std=c++14 -O2 ros2_ws/src/livox_ros_driver2/src/comm/test_device_clock.cpp \
  -o /tmp/test_livox_device_clock
/tmp/test_livox_device_clock
```


## 2026-11-02 shutdown correction

Use the workspace SDK built by `bash scripts/build_livox_sdk.sh` before rebuilding the driver. It shares ROS's system spdlog ABI and owns a private logger. The driver uses its distinct liblivox_lidar_sdk_hap.so SONAME, with a workspace RUNPATH. Global SDK/GO2 sources are unchanged. SDK observers/point conversion threads stop before SDK uninitialization; the exit flag is atomic and blocked publisher semaphores are woken before joining.

Two real-driver cycles now exit 0 in 0.365 s each; see reports/navigation_lyingdown_driver_shutdown_20261102.json. The prior SIGSEGV backtrace is reports/navigation_lyingdown_driver_gdb_20261102.log.
