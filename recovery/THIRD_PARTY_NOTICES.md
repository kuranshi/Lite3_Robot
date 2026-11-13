# Third-party notices

The native `0x0901` state decoder in `ros2_ws/src/lite3_recovery_telemetry/lite3_recovery_telemetry/protocol.py`
was extracted from the MIT-licensed native bridge associated with the monocular-depth-navigation project.
The receive-only adapter follows that bridge's IMU and diagnostic layout, with navigation,
gateway, range publishing and cross-host clock-writing functionality removed.

The original project's notice is retained in [licenses/LICENSE-MIT](licenses/LICENSE-MIT):
Copyright (c) 2025 Yassine Gharbi. Modifications are identified in the derived source files.

New recovery tooling follows the repository's [Apache-2.0 license](../LICENSE).
The extracted MIT portions retain their original terms. No manufacturer binary, SDK library,
private configuration or robot recording is distributed.
