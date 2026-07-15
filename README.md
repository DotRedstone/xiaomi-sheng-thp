# xiaomi-sheng-thp

`xiaomi-sheng-thp` provides userspace multitouch and Xiaomi Focus Pen input
for the Xiaomi Pad 6S Pro (`sheng`) with an NT36532E touch controller. It
reads raw THP frames from the kernel and creates standard Linux input devices
through uinput.

Finger input, pen coordinates, tilt, hover, and Bluetooth HID pressure are
supported. Please note that pen scanning is only available when the panel is
running at 60 Hz or 120 Hz.

## Requirements

- A Linux kernel for Xiaomi Sheng exposing:

  ```text
  /proc/nvt_thp_raw
  /proc/nvt_thp_stream
  /proc/nvt_thp_status
  /proc/nvt_thp_stylus
  ```

- `/dev/uinput`
- BlueZ for Xiaomi Focus Pen pressure input
- A C++20 compiler and GNU Make when building from source

[xiaomi-pen-status](https://github.com/ianchb/xiaomi-pen-status) is recommended for pen detection, Bluetooth connection
assistance, and refresh-rate notifications. It is optional and is not required
for the touch service to start.

The service runs as root because it controls the THP stream, reads the pen HID
transport, and creates uinput devices.

## Build

```sh
make -j"$(nproc)"
```

The executable is written to `build/xiaomi-sheng-thp`.

## Install

```sh
sudo make install
sudo systemctl daemon-reload
sudo systemctl enable --now xiaomi-sheng-thp.service
```

## Debian Package

```sh
./build-deb.sh
sudo apt install ./xiaomi-sheng-thp_*_arm64.deb
```

## License

Apache License 2.0. See `LICENSE`.
