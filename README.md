# xiaomi-sheng-thp

`xiaomi-sheng-thp` provides userspace multitouch and Xiaomi Focus Pen input
for the Xiaomi Pad 6S Pro (`sheng`) with an NT36532E touch controller. It
reads raw THP frames from the kernel and creates standard Linux input devices
through uinput.

Finger input, multi-touch tracking, palm rejection, pen coordinates, tilt,
hover, Bluetooth HID pressure, and both barrel buttons are supported. While the
Focus Pen air pointer is moving, its two barrel buttons also act as left and
right mouse clicks; approaching the screen returns them to normal stylus
buttons. Both contexts can be remapped at runtime through the control socket.
Please note that pen scanning is only available when the panel is running at
60 Hz or 120 Hz.

## Requirements

- A Linux kernel for Xiaomi Sheng exposing:

  ```text
  /proc/nvt_thp_raw
  /proc/nvt_thp_stream
  /proc/nvt_thp_status
  /proc/nvt_thp_stylus
  ```

- `/dev/uinput`
- BlueZ for Xiaomi Focus Pen input
- `libssc` for the Focus Pen Pro posture sensor path
- A C++20 compiler, GNU Make, pkg-config, GLib development files, and libssc
  development files when building from source

[xiaomi-pen-status](https://github.com/DotRedstone/xiaomi-pen-status) is
recommended for pen detection, Bluetooth connection assistance, refresh-rate
notifications, and button mapping. It is optional and is not required for the
touch service to start.

The service runs as root because it controls the THP stream, reads the pen HID
transport, and creates uinput devices.

## Button Mapping

The service listens on `/run/xiaomi-sheng-thp/button-mapping.sock`. Members of
the `input` group can send one datagram containing:

```text
map PEN_PRIMARY PEN_SECONDARY AIR_PRIMARY AIR_SECONDARY
```

Each action is one of `native`, `left`, `right`, `middle`, `back`, `forward`,
`undo`, `redo`, `screenshot`, `overview`, or `disabled`. Defaults are
`map native native left right`. The status application stores the selection per
user and reapplies it when the graphical session starts.

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
