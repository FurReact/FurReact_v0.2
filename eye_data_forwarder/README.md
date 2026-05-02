# eye_data_forwarder

Reads the gaze pose from a running SteamVR instance and writes it as ASCII
lines to a USB Serial/JTAG target on the same machine (ESP32-S3, VID:PID
`303a:1001`).

## Protocol

One line per gaze sample, ~1 kHz:

```
LLLL,LLLL,RRRR,RRRR\n
```

Each field is a zero-padded 4-digit integer in `[0, 1024]`, with `512` at
dead-center. They're the normalized X/Y components of the left and right
gaze vectors, mapped from `[-1, 1] → [0, 1024]` and clamped.

## Prerequisites

- A **Steam Frame** (aarch64 Linux, SteamOS-like) with SteamVR installed
  and **running**.
- An **ESP32-S3** device (`303a:1001`) physically connected to the Frame's
  USB-C port, with firmware already flashed and listening on the CDC Data
  interface's bulk OUT endpoint.
- **SSH** access to the Frame. Examples below use `steamos@<frame>`; sub in
  your host.
- A **dev machine** (macOS or Linux) for editing + `rsync`.

## 1. Put the Frame's USB-C port in host mode

By default the Frame's USB-C is configured as a USB *device* (so it can
enumerate as a peer when plugged into a PC). To make it enumerate the ESP
instead, flip the role:

```sh
ssh steamos@<frame> 'sudo sh -c "echo host > /sys/kernel/debug/usb/a600000.usb/mode"'
```

Verify:

```sh
ssh steamos@<frame> 'cat /sys/kernel/debug/usb/a600000.usb/mode'    # -> host
ssh steamos@<frame> 'lsusb | grep 303a:1001'                        # ESP appears
```

Notes:

- **Not persistent.** The role resets to `device` at every Frame reboot.
  Re-run the command (and replug the ESP) after each reboot.
- If the ESP was already plugged in when you flipped the role, **unplug
  and replug it**. Devices attached in the wrong role land in a stuck
  state and won't enumerate on their own.
- To go back to device mode (so the Frame can be dev-connected to a PC
  again): `echo device > /sys/kernel/debug/usb/a600000.usb/mode`.

## 2. Install Rust on the Frame

The Frame's rootfs is read-only, so `pacman -S rust` fails. Install
rustup into the user's home directory:

```sh
ssh steamos@<frame> \
  'curl --proto "=https" --tlsv1.2 -sSf https://sh.rustup.rs \
     | sh -s -- -y --profile minimal --default-toolchain stable --no-modify-path'
```

This puts `cargo` at `~/.cargo/bin/cargo` and the toolchain at
`~/.rustup/`. Roughly 1 GB on disk, one-time.

Sanity check:

```sh
ssh steamos@<frame> '~/.cargo/bin/rustc --version'
```

## 3. Grant USB access to the steamos user

Raw USB access goes through `/dev/bus/usb/…`, which is `root:input 660`.
The `steamos` user needs to be in the `input` group. On a stock Steam
Frame it already is — confirm:

```sh
ssh steamos@<frame> 'id | tr "," "\n" | grep input'
```

If absent:

```sh
ssh steamos@<frame> 'sudo usermod -aG input steamos'
```

then log out and back in (or prefix the run command with `sg input -c
"..."` to pick up the supplementary group within an existing session).

## 4. Sync the source

From your dev machine, inside this directory:

```sh
rsync -az --delete --exclude=target ./ \
  steamos@<frame>:/home/steamos/work/eye_data_forwarder/
```

## 5. Build on the Frame

```sh
ssh steamos@<frame> \
  'cd /home/steamos/work/eye_data_forwarder && ~/.cargo/bin/cargo build --release'
```

~30 seconds on first build (fetches `libloading` + `rusb` + deps), a
couple of seconds on subsequent ones.

libusb is required at build and run time. SteamOS ships it (`pacman -Qo
/usr/lib/libusb-1.0.so`) — no extra install needed.

## 6. Run

```sh
ssh steamos@<frame> \
  'sg input -c "/home/steamos/work/eye_data_forwarder/target/release/eye_data_forwarder"'
```

The happy path is **silent** — the binary just pumps ~1 line/ms to the
ESP. Ctrl-C (or close the SSH session) to stop it.

To leave it running in the background:

```sh
ssh steamos@<frame> \
  'sg input -c "nohup /home/steamos/work/eye_data_forwarder/target/release/eye_data_forwarder >/tmp/edf.log 2>&1 &"'
```

## Troubleshooting

- **`openvr init failed: … is SteamVR running?`** — `vrserver` isn't up.
  Check with `ssh steamos@<frame> 'pgrep -af vrserver'`.

- **`serial open failed: no ESP JTAG/serial 303a:1001 found …`** —
  either the port isn't in host mode (step 1) or the cable isn't seated.
  `lsusb` is ground truth.

- **`serial open failed: claim …: LIBUSB_ERROR_ACCESS`** — the `input`
  group membership isn't active for this session. Use `sg input -c
  "..."` or log in again.

- **`serial write: bulk_transfer: LIBUSB_ERROR_TIMEOUT`** on every
  iteration — the ESP firmware's TX is stalled (host isn't draining IN
  fast enough). This binary drains on every iteration, so if you see
  this, the ESP firmware probably crashed or is in a weird state;
  replug the Xiao to reset it.

- **Binary runs, no errors, but the ESP doesn't act on the data.** The
  ESP is enumerating but possibly stuck in its own fault state. Check
  by watching its output on the Frame: briefly stop the forwarder and
  read the ESP's IN endpoint with a short libusb probe, or replug.

## Maintenance

A few pinned values in `src/main.rs` are tied to the current SteamVR
runtime's ABI. All three have comments alongside them giving the exact
commands to re-derive them from the shipping binaries (vrclient.so is
shipped unstripped, which is what makes this tractable):

- `IVR_CLIENT_INTERNAL_VERSION` — interface version string passed to
  `VR_GetGenericInterface`. Recoverable with `strings vrclient.so | grep
  '^IVRClient'`.

- `VTABLE_IDX_GET_LATEST_EYE_POSE` — which vtable slot of the returned
  interface is `GetLatestEyePose`. Recoverable by a three-step walk:
  `nm` out the non-virtual thunk and the composite class's typeinfo,
  dump `.data.rel.ro`, and compute `slot = (thunk_loc −
  (nearest_typeinfo_loc + 8)) / 8`. Full recipe in the comment above the
  constant.

- `VREyeTrackingStatePose` field layout — can be black-box probed (pass
  a zeroed buffer, hex-dump it after the call) or, more precisely,
  recovered by disassembling `CHmdSystemLatest::GetLatestEyePose` and
  reading the STR/STP offsets directly.

If a SteamVR update breaks gaze reading, start with the vtable slot —
it's the most likely thing to shift when virtual methods are added or
reordered.
