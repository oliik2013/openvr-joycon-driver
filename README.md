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

## Uninstalling

```bash
rm -rf "$HOME/.local/share/Steam/steamapps/common/SteamVR/drivers/joycon"
```
