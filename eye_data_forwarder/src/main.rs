//! Forwards SteamVR gaze data to a USB Serial/JTAG target.
//!
//! Talks to SteamVR by dlopen'ing libopenvr_api.so and calling the standard
//! VR_InitInternal2 / VR_GetGenericInterface / VR_ShutdownInternal exports
//! (see `nm -D /usr/lib/libopenvr_api.so | grep ' VR_'`). Writes one formatted
//! ASCII line per gaze sample to an ESP32-S3 (VID:PID 303a:1001) over libusb.
//!
//! ─── Maintenance notes ────────────────────────────────────────────────────
//! A handful of constants below are pinned to the current SteamVR runtime
//! ABI and may need updating after an update. Each one is labelled with the
//! exact command to re-verify it against the shipping binaries on the
//! headset — no other tooling or reference material is needed.

use std::ffi::{c_char, c_int, c_void};
use std::ptr;
use std::thread;
use std::time::{Duration, Instant};

use libloading::{Library, Symbol};
use rusb::UsbContext;

// ═════ OpenVR types ════════════════════════════════════════════════════════
#[repr(C)]
#[derive(Copy, Clone, Default, Debug)]
struct HmdVec3 {
    v: [f32; 3],
}

#[repr(i32)]
#[allow(dead_code)]
enum VRApplicationType {
    Other = 0,
    Scene = 1,
    Overlay = 2,
    Background = 4,
    Utility = 5,
}

// Layout of the struct GetLatestEyePose writes through. Re-derivable two
// ways, in increasing order of rigor:
//
//   (a) Black-box: pass a pre-zeroed scratch buffer to GetLatestEyePose
//       and hex-dump the first ~96 bytes. Expected shape = one u64 tick
//       counter near the top, then 2×3×f32 gaze vectors whose components
//       land in roughly [-1, 1] when looking at the headset centre, then
//       2×3×f32 variances (small positive floats), then 3×f32 for the
//       gaze point, then three bool bytes. Walk offsets until they align.
//
//   (b) Static: vrclient.so is shipped unstripped, and the impl of
//       GetLatestEyePose is named, so its disassembly reveals every
//       field write.
//
//         objdump -d --disassemble='CHmdSystemLatest::GetLatestEyePose*' \
//                 /opt/steamvr/bin/linuxarm64/vrclient.so
//
//       The aarch64 STR/STP/STUR instructions targeting the out-pointer
//       (passed in x0 or x1 depending on sret) directly give field
//       offsets and sizes. Cross-reference the sequence of offsets with
//       the expected layout and fix up any field widths/alignments that
//       have changed.
#[repr(C)]
#[derive(Copy, Clone)]
struct VREyeTrackingStatePose {
    pose_time_in_ticks: u64,
    gaze_vec:           [HmdVec3; 2],
    gaze_variance:      [HmdVec3; 2],
    gaze_point:         HmdVec3,
    good_gaze_point:    bool,
    blinking:           bool,
    eyes_in_headset:    bool,
    _tail_pad:           [u8; 5], // natural 8-byte alignment tail
}

impl Default for VREyeTrackingStatePose {
    fn default() -> Self { unsafe { std::mem::zeroed() } }
}

// Vtable slot for GetLatestEyePose on the sub-vtable that exposes the
// IVRClientInternal-shaped interface. vrclient.so is shipped unstripped,
// so this is fully re-derivable — the concrete impls are named in the
// regular symbol table, even though they're absent from the dynamic one.
//
// Recipe (SteamOS aarch64 build):
//
//   1. Find the non-virtual thunk for the method. In Itanium multi-
//      inheritance, secondary-base sub-vtables hold thunks that adjust
//      `this` before jumping to the real impl, so the thunk is the
//      pointer actually sitting in the sub-vtable:
//
//        nm /opt/steamvr/bin/linuxarm64/vrclient.so | c++filt \
//          | grep 'non-virtual thunk to.*GetLatestEyePose'
//        # e.g. 1acaa8 t non-virtual thunk to CHmdSystemLatest::GetLatestEyePose(...)
//
//      The impl class name (CHmdSystemLatest here) comes out of the same
//      grep.
//
//   2. Find the composite class's typeinfo address — every sub-vtable in
//      the composite vtable carries this in its preamble:
//
//        nm /opt/steamvr/bin/linuxarm64/vrclient.so \
//          | grep ' _ZTI16CHmdSystemLatest$'
//        # e.g. 5ffab0 d _ZTI16CHmdSystemLatest
//
//   3. Dump .data.rel.ro and locate both addresses in it:
//
//        objdump -s -j .data.rel.ro /opt/steamvr/bin/linuxarm64/vrclient.so
//
//      Record the in-file addresses of (a) the word containing the thunk
//      pointer (LE-encoded 0x1acaa8) and (b) each word containing the
//      typeinfo pointer (LE 0x5ffab0).
//
//   4. slot = (thunk_loc − (nearest_typeinfo_loc_preceding_thunk + 8)) / 8.
//      The +8 skips the typeinfo word in the sub-vtable preamble.
//
// On the current build: thunk at file-addr 0x5f6df0, nearest preceding
// typeinfo pointer at 0x5f6ca0, so first fn slot is 0x5f6ca8 and
// slot = (0x5f6df0 − 0x5f6ca8) / 8 = 41.
const VTABLE_IDX_GET_LATEST_EYE_POSE: usize = 41;

// Interface version string for VR_GetGenericInterface. Verify with
//     strings /opt/steamvr/bin/linuxarm64/vrclient.so | grep '^IVRClient'
// (the "XXX" is literal — the runtime uses a sentinel rather than a
// numeric version for this interface).
const IVR_CLIENT_INTERNAL_VERSION: &[u8] = b"IVRClientInternal_XXX\0";

// ═════ OpenVR dlopen shell ═════════════════════════════════════════════════
struct OpenVR {
    _lib:      Library,
    shutdown:  unsafe extern "C" fn(),
    client:    *mut c_void,
}

impl OpenVR {
    fn init() -> Result<Self, String> {
        unsafe {
            let lib = Library::new("libopenvr_api.so")
                .or_else(|_| Library::new("libopenvr_api.so.0"))
                .or_else(|_| Library::new("/usr/lib/libopenvr_api.so"))
                .map_err(|e| format!("dlopen libopenvr_api.so: {e}"))?;

            let init_internal: Symbol<
                unsafe extern "C" fn(*mut c_int, VRApplicationType, *const c_char) -> u32,
            > = lib.get(b"VR_InitInternal2")
                .map_err(|e| format!("VR_InitInternal2: {e}"))?;
            let get_generic: Symbol<
                unsafe extern "C" fn(*const c_char, *mut c_int) -> *mut c_void,
            > = lib.get(b"VR_GetGenericInterface")
                .map_err(|e| format!("VR_GetGenericInterface: {e}"))?;
            let shutdown: Symbol<unsafe extern "C" fn()> = lib
                .get(b"VR_ShutdownInternal")
                .map_err(|e| format!("VR_ShutdownInternal: {e}"))?;

            // Boot VR as a Background app so we don't take over compositing.
            let mut err: c_int = 0;
            init_internal(&mut err, VRApplicationType::Background, ptr::null());
            if err != 0 {
                return Err(format!("VR_InitInternal2 failed: EVRInitError={err}"));
            }

            // Standard VR_GetGenericInterface lookup for IVRClientInternal.
            err = 0;
            let client = get_generic(
                IVR_CLIENT_INTERNAL_VERSION.as_ptr() as *const c_char,
                &mut err,
            );
            if client.is_null() || err != 0 {
                return Err(format!(
                    "VR_GetGenericInterface(IVRClientInternal_XXX) \
                     returned null (EVRInitError={err}) — is SteamVR running?"
                ));
            }

            let shutdown_fn: unsafe extern "C" fn() = *shutdown;
            Ok(Self { _lib: lib, shutdown: shutdown_fn, client })
        }
    }

    /// Direct-dispatch into IVRClientInternal's vtable slot for GetLatestEyePose.
    fn latest_eye_pose(&self) -> VREyeTrackingStatePose {
        unsafe {
            // Object starts with a vtable pointer (Itanium C++ ABI, single
            // inheritance). The vtable is an array of fn pointers.
            let vtbl = *(self.client as *const *const usize);
            let slot_addr = *vtbl.add(VTABLE_IDX_GET_LATEST_EYE_POSE);
            let f: unsafe extern "C" fn(*mut c_void, *mut VREyeTrackingStatePose) =
                std::mem::transmute(slot_addr);
            let mut pose = VREyeTrackingStatePose::default();
            f(self.client, &mut pose);
            pose
        }
    }
}

impl Drop for OpenVR {
    fn drop(&mut self) {
        unsafe { (self.shutdown)(); }
    }
}

// ═════ libusb writer ═══════════════════════════════════════════════════════
const ESP_VID: u16 = 0x303a;
const ESP_PID: u16 = 0x1001;

struct Serial {
    handle:      rusb::DeviceHandle<rusb::Context>,
    iface_data:  u8,
    iface_comms: Option<u8>,
    ep_out:      u8,
    ep_in:       u8,
}

impl Serial {
    fn open() -> Result<Self, String> {
        let ctx = rusb::Context::new().map_err(|e| format!("libusb init: {e}"))?;
        let devices = ctx.devices().map_err(|e| format!("list devices: {e}"))?;
        let dev = devices.iter().find(|d| {
            d.device_descriptor()
                .map(|dd| dd.vendor_id() == ESP_VID && dd.product_id() == ESP_PID)
                .unwrap_or(false)
        }).ok_or_else(|| {
            format!("no ESP JTAG/serial {ESP_VID:04x}:{ESP_PID:04x} found \
                     — is the port in host mode and the Xiao plugged in?")
        })?;

        let cfg_desc = dev.active_config_descriptor()
            .or_else(|_| dev.config_descriptor(0))
            .map_err(|e| format!("config descriptor: {e}"))?;

        let (mut iface_data, mut iface_comms, mut ep_out, mut ep_in) =
            (None::<u8>, None::<u8>, 0u8, 0u8);
        for iface in cfg_desc.interfaces() {
            for id in iface.descriptors() {
                match id.class_code() {
                    0x02 /* CDC Communications */ => {
                        iface_comms.get_or_insert(id.interface_number());
                    }
                    0x0A /* CDC Data */ => {
                        for ep in id.endpoint_descriptors() {
                            if ep.transfer_type() != rusb::TransferType::Bulk { continue; }
                            match ep.direction() {
                                rusb::Direction::Out if ep_out == 0 => {
                                    ep_out = ep.address();
                                    iface_data.get_or_insert(id.interface_number());
                                }
                                rusb::Direction::In  if ep_in  == 0 => {
                                    ep_in = ep.address();
                                }
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
        let iface_data = iface_data.ok_or("no CDC Data interface with bulk OUT")?;

        let handle = dev.open().map_err(|e| format!("open: {e}"))?;
        let _ = handle.set_auto_detach_kernel_driver(true);
        let _ = handle.set_active_configuration(1);

        if let Some(c) = iface_comms {
            handle.claim_interface(c).map_err(|e| format!("claim comms({c}): {e}"))?;
        }
        handle.claim_interface(iface_data)
            .map_err(|e| format!("claim data({iface_data}): {e}"))?;

        // CDC-ACM SET_CONTROL_LINE_STATE: DTR|RTS. Without this the ESP32-S3's
        // USB Serial/JTAG peripheral NAKs bulk OUT.
        if let Some(c) = iface_comms {
            let _ = handle.write_control(
                0x21, 0x22, 0x0003, c as u16,
                &[], Duration::from_millis(500),
            );
        }

        let mut s = Self { handle, iface_data, iface_comms, ep_out, ep_in };
        s.drain();
        Ok(s)
    }

    fn write(&mut self, data: &[u8]) -> Result<(), String> {
        let n = self.handle.write_bulk(self.ep_out, data, Duration::from_millis(100))
            .map_err(|e| format!("bulk OUT: {e}"))?;
        if n != data.len() { return Err(format!("short write {}/{}", n, data.len())); }
        Ok(())
    }

    /// Pull any queued echoes from the ESP so its TX FIFO doesn't stall the
    /// firmware's RX task via async-write backpressure.
    fn drain(&mut self) {
        let mut buf = [0u8; 256];
        for _ in 0..16 {
            match self.handle.read_bulk(self.ep_in, &mut buf, Duration::from_millis(1)) {
                Ok(n) if n > 0 => continue,
                _ => break,
            }
        }
    }
}

impl Drop for Serial {
    fn drop(&mut self) {
        let _ = self.handle.release_interface(self.iface_data);
        if let Some(c) = self.iface_comms { let _ = self.handle.release_interface(c); }
    }
}

// ═════ Mapping + main loop ════════════════════════════════════════════════
const PROTOCOL_CENTER: i32 = 512;

fn axis_to_protocol(v: f32) -> i32 {
    // Gaze component [-1,1] → 0..1024 (center 512). ESP firmware clamps
    // further to its physical range.
    let s = (1.0_f32 + v) * 512.0_f32;
    s.clamp(0.0, 1024.0) as i32
}

const RETRY_BACKOFF:    Duration = Duration::from_secs(2);
const HEARTBEAT_PERIOD: Duration = Duration::from_secs(1);
const USB_MODE_PATH:    &str = "/sys/kernel/debug/usb/a600000.usb/mode";

/// Best-effort check that the Frame's USB-C port is in host mode. Prints a
/// pointed warning if the role file says otherwise. Silently no-ops if the
/// file doesn't exist (not a Frame) or we can't read it (debugfs perms).
fn warn_if_not_host_mode() {
    match std::fs::read_to_string(USB_MODE_PATH) {
        Ok(s) => {
            let mode = s.trim();
            if mode != "host" {
                eprintln!(
                    "[fwd] WARNING: USB-C role is {:?}, expected \"host\". The ESP \
                     will not enumerate until you run:\n  \
                     sudo sh -c 'echo host > {}'\n  \
                     (and then physically replug the ESP, per the dwc3 stuck-state \
                     thing).",
                    mode, USB_MODE_PATH
                );
            }
        }
        Err(_) => {
            // Not a Frame, or steamos can't read debugfs — nothing useful to say.
        }
    }
}

fn run_session() -> Result<(), String> {
    let vr = OpenVR::init()?;
    let mut ser = Serial::open().map_err(|e| {
        // If we can't find the ESP, the most common cause on the Frame is
        // the port still being in `device` role. Surface that hint inline.
        warn_if_not_host_mode();
        e
    })?;
    eprintln!("[fwd] session up: openvr + esp serial ready");

    let mut last_heartbeat = Instant::now();
    let mut last_state = "";

    loop {
        let pose = vr.latest_eye_pose();

        // Only send centered when the headset isn't on a head. Blinking is
        // a normal, frequent event — chasing the gaze through a blink looks
        // more natural than snapping the ears to neutral every time.
        let (state, lx, ly, rx, ry) = if !pose.eyes_in_headset {
            ("no-eyes", PROTOCOL_CENTER, PROTOCOL_CENTER, PROTOCOL_CENTER, PROTOCOL_CENTER)
        } else {
            (
                "follow",
                axis_to_protocol(pose.gaze_vec[0].v[0]),
                axis_to_protocol(pose.gaze_vec[0].v[1]),
                axis_to_protocol(pose.gaze_vec[1].v[0]),
                axis_to_protocol(pose.gaze_vec[1].v[1]),
            )
        };

        let msg = format!("{:04},{:04},{:04},{:04}\n", lx, ly, rx, ry);
        ser.write(msg.as_bytes())
            .map_err(|e| format!("serial write: {e}"))?;
        ser.drain();

        // Rate-limited heartbeat — also fires immediately on state changes
        // so you see follow ↔ no-eyes transitions without waiting for the
        // next tick.
        let now = Instant::now();
        let state_changed = state != last_state;
        if state_changed || now.duration_since(last_heartbeat) >= HEARTBEAT_PERIOD {
            eprintln!("[fwd] state={} tx={}", state, msg.trim_end());
            last_heartbeat = now;
            last_state = state;
        }

        thread::sleep(Duration::from_millis(1));
    }
}

fn main() {
    // Outer retry loop: any failure (SteamVR not yet up, ESP unplugged,
    // bulk write error mid-session, etc.) ends the inner session, sleeps
    // briefly, and re-tries. systemd will Restart=always us if the
    // process itself dies (e.g. segfault from a SteamVR crash).
    //
    // Log discipline: print on the first occurrence of an error, then
    // suppress identical repeats unless ≥10 s have passed. Keeps the
    // journal readable while the headset is e.g. asleep for hours.
    const ERROR_REPEAT_PERIOD: Duration = Duration::from_secs(10);
    let mut last_err = String::new();
    let mut last_err_logged: Option<Instant> = None;
    loop {
        match run_session() {
            Ok(()) => {} // unreachable; run_session loops forever on success
            Err(e) => {
                let now = Instant::now();
                let should_log = e != last_err
                    || last_err_logged
                        .map(|t| now.duration_since(t) >= ERROR_REPEAT_PERIOD)
                        .unwrap_or(true);
                if should_log {
                    eprintln!("[fwd] session ended: {e}");
                    last_err = e;
                    last_err_logged = Some(now);
                }
            }
        }
        thread::sleep(RETRY_BACKOFF);
    }
}
