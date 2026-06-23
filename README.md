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

## Controller Mapping

**Right JoyCon** maps to the right virtual controller:

| JoyCon Button | VR Input | Typical Use |
|---------------|----------|-------------|
| A | `/input/a/click` | A button |
| B | `/input/b/click` | B button |
| X | `/input/x/click` | X button |
| Y | `/input/y/click` | Y button |
| R | `/input/shoulder_right/click` | Right bumper |
| ZR | `/input/trigger_right/click` | Right trigger |
| SL | `/input/grip/click` | Grip (click) |
| SR | `/input/grip/touch` | Grip (touch) |
| + | `/input/start/click` | Start / Menu |
| Home (circle) | `/input/guide/click` | System / Guide |
| Stick click | `/input/joystick_right/click` | Right stick press |
| Stick move | `/input/joystick_right/{x,y}` | Right stick X/Y |

**Left JoyCon** maps to the left virtual controller:

| JoyCon Button | VR Input | Typical Use |
|---------------|----------|-------------|
| D-pad Up | `/input/dpad/up` | D-pad up |
| D-pad Down | `/input/dpad/down` | D-pad down |
| D-pad Left | `/input/dpad/left` | D-pad left |
| D-pad Right | `/input/dpad/right` | D-pad right |
| L | `/input/shoulder_left/click` | Left bumper |
| ZL | `/input/trigger_left/click` | Left trigger |
| SL | `/input/grip/click` | Grip (click) |
| SR | `/input/grip/touch` | Grip (touch) |
| - | `/input/back/click` | Back |
| Capture (square) | `/input/capture/click` | Capture / screenshot |
| Stick click | `/input/joystick_left/click` | Left stick press |
| Stick move | `/input/joystick_left/{x,y}` | Left stick X/Y |

## Uninstalling

```bash
rm -rf "$HOME/.local/share/Steam/steamapps/common/SteamVR/drivers/joycon"
```
