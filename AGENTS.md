## Build & Install

```bash
mkdir -p build && cmake -S . -B build && cmake --build build
cp build/driver_joycon.so ~/.local/share/Steam/steamapps/common/SteamVR/drivers/joycon/bin/linux64/
```

Binary is `driver_joycon.so` (no `lib` prefix — CMake sets `PREFIX ""`). Install target: `~/.local/share/Steam/steamapps/common/SteamVR/drivers/joycon/bin/linux64/`. CMake also copies to `joycon/bin/linux64/` post-build.

Deps: `cmake g++ libhidapi-dev pkg-config`. JoyShockLibrary is bundled in `include/external/JSL/source/` as a CMake subdirectory.

No tests, no lint, no typecheck.

## Architecture

- **Entrypoint**: `HmdDriverFactory` in `src/driver_factory.cc` — returns a global `Provider` singleton
- **Provider** (`src/provider.cc`): creates two `JoyconDriver` instances (left/right) in `Init()`, calls their `RunFrame()` each frame
- **JSLGlue** (`src/jsl_glue.cc`): singleton that manages JSL device discovery and routes the `poll` callback → `processInput()` per deviceId
- **JoyconDriver** (`src/joycon_driver.cc`): implements `ITrackedDeviceServerDriver`

## Two Threads

- **JSL callback thread** (`poll` → `processInput`): reads IMU from JSL, maps buttons, handles recenter timer, writes `m_rawQuat`
- **SteamVR thread** (`RunFrame`): computes `qRotation = qOffset * m_rawQuat` via `quatMul`, runs arm-swing position model, submits pose
- No locks — `m_rawQuat` is a single quaternion write per frame, and `RunFrame` is the sole writer of `m_pose.qRotation`

## IMU Sign Convention (Important)

Both controllers use the same sign fix on raw JSL quaternion: `{-qx, qy, -qz}` (negate x and z). This is applied in `processInput` when storing `m_rawQuat`.

## Recenter

Hold `+` (right) or `-` (left) for 0.5s. Sets `qOffset = yawQ * conj(m_rawQuat)` in `processInput` (JSL thread), zeros `m_frozenRel`. The HMD yaw is extracted from `GetRawTrackedDevicePoses`.

## Arm-Swing Position

`position = hmdPos + rotate(armLocal, qRotation)` where `armLocal = {±0.3, -0.3, -0.5}` (+ right, - left). Computed each frame in `RunFrame`.

## Input Map

Presents as `vive_controller` (`Prop_ControllerType_String`, `Prop_RenderModelName_String = vr_controller_vive_1_5`). Components created in `Activate` match the Vive convention. Button mapping per controller role in `processInput`.

## Haptics

`ProcessEvent` handles `VREvent_Input_HapticVibration` → `JslSetRumble`. `RunFrame` checks `m_rumbleEndTime` to stop vibration.

## Git

- Upstream (read-only): `origin` → `SieR-VR/openvr-joycon-driver`
- Fork (push): `fork` → `oliik2013/openvr-joycon-driver`
