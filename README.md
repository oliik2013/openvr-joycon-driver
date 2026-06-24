# OpenVR JoyCon Driver

OpenVR driver for Nintendo Joy-Con controllers, using [JoyShockLibrary](https://github.com/JibbSmart/JoyShockLibrary).

## Building

```bash
mkdir build && cmake -S . -B build && cmake --build build
```

Requires: `cmake`, `g++`, `hidapi` (for JoyShockLibrary).

### Dependencies (Ubuntu/Debian)

```bash
sudo apt install cmake g++ libhidapi-dev pkg-config
```

## Installing

The build copies the driver to `joycon/bin/linux64/`. To install for SteamVR:

```bash
DRIVER_DIR="$HOME/.local/share/Steam/steamapps/common/SteamVR/drivers/joycon"
mkdir -p "$DRIVER_DIR/bin/linux64"
cp JoyCon/driver.vrdrivermanifest "$DRIVER_DIR/"
cp joycon/bin/linux64/* "$DRIVER_DIR/bin/linux64/"
```

Restart SteamVR after installing.

## Usage

1. Pair your Joy-Cons to your PC via Bluetooth
2. Start SteamVR
3. The driver activates automatically (configured with `"alwaysActivate": true`)
4. Left and right Joy-Cons appear as tracked controllers

This driver also works with combined Joy-Cons (left + right paired together, e.g. in a grip/mount). Simply connect both via Bluetooth, then press **L + R** (or **ZL + ZR**) on both controllers at the same time to combine them. Each JoyCon will be picked up as a separate tracked controller.

The driver presents as a **Vive controller** (`vive_controller`) so most SteamVR games work out of the box without manual binding setup. Generic gamepad-style buttons (A/B/X/Y, D-pad, L/R) are not mapped, but all essential VR inputs work.

## Controller Mapping

**Right JoyCon** maps to the right virtual controller:

| JoyCon Button | VR Input | Typical Use |
|---------------|----------|-------------|
| ZR | `/input/trigger/click` | Trigger (fire, grab, interact) |
| SL | `/input/grip/click` | Grip (grab) |
| SR | `/input/grip/touch` | Grip touch detect |
| + (Start) | `/input/system/click` | System / Steam menu |
| Home (circle) | `/input/application_menu/click` | Game menu |
| Stick move | `/input/trackpad/{x,y}` | Trackpad (movement/teleport) |
| Stick click | `/input/trackpad/click` | Trackpad press (sprint/confirm) |

**Left JoyCon** maps to the left virtual controller:

| JoyCon Button | VR Input | Typical Use |
|---------------|----------|-------------|
| ZL | `/input/trigger/click` | Trigger (fire, grab, interact) |
| SL | `/input/grip/click` | Grip (grab) |
| SR | `/input/grip/touch` | Grip touch detect |
| - (Minus) | `/input/system/click` | System / Steam menu |
| Capture (square) | `/input/application_menu/click` | Game menu |
| Stick move | `/input/trackpad/{x,y}` | Trackpad (movement/teleport) |
| Stick click | `/input/trackpad/click` | Trackpad press (sprint/confirm) |

## Recenter

Hold **+** on the right JoyCon or **-** on the left JoyCon for 0.5 seconds to recenter. This aligns the controller's forward direction with where you're currently looking (HMD forward) and freezes the arm-swing position to prevent jumping.

**Note:** A short press of **+**/**-** opens the SteamVR system menu (prevents conflict between recenter and menu).

## Features

- **IMU-based 6-DOF tracking**: Uses JoyShockLibrary's sensor fusion for orientation
- **Arm-swing position model**: Controller position swings with your arm rotation around the HMD (GearVR-style 3-DOF positional illusion)
- **Haptic feedback**: Vibration/rumble works in compatible games via `/output/haptic`
- **Auto-calibration**: Gyro is continuously calibrated to reduce drift
- **Vive controller compatibility**: Appears as `vive_controller` so existing SteamVR bindings work without manual setup

## Uninstalling

```bash
rm -rf "$HOME/.local/share/Steam/steamapps/common/SteamVR/drivers/joycon"
```
