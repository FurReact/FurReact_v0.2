#!/usr/bin/env python3
"""
Host-side smoke test for solcatears-fw.

Opens the Xiao ESP32-S3 over native USB CDC, sends a few calibration lines,
and reads back the firmware's echo ("OK ..."), bad-line ("BAD"), and
state-transition ("MODE follow|center|sweep|limp") frames.

The 500 ms failsafe now drops to Center (servos held at neutral) rather
than going fully limp; firmware only goes limp after LIMP_AFTER_MS
(30 s default). So the failsafe test below expects "MODE center".

stdlib only — no pyserial.
"""

import os, sys, termios, fcntl, select, time, re, argparse

DEFAULT_PORT = "/dev/cu.usbmodem101"


def open_raw(path: str, flush: bool = True) -> int:
    # O_NONBLOCK so open() doesn't hang on DCD; we'll clear it after.
    fd = os.open(path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
    # Now put it back to blocking; we'll use select() for timeouts.
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)

    # Raw-ish termios: disable canonical mode, echo, signal gen, CR/NL xforms.
    # USB Serial/JTAG ignores baud, but macOS tty layer still cooks unless told not to.
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs
    iflag &= ~(
        termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
        | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON
    )
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB)
    cflag |= termios.CS8
    cc = list(cc)
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])
    # Optional flush — for the smoke test we want a clean slate, but for
    # listen-only we'd be tossing the BOOT/MODE-sweep banner that's
    # already buffered by the macOS CDC driver.
    if flush:
        termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_lines(fd: int, timeout_s: float, hard_cap_s: float = 3.0, max_lines: int = 200):
    """Yield lines; stop after timeout_s of silence, or hard_cap_s wall time, or max_lines."""
    buf = b""
    start = time.monotonic()
    last_data = start
    emitted = 0
    while True:
        now = time.monotonic()
        if now - start > hard_cap_s:
            break
        if now - last_data > timeout_s:
            break
        r, _, _ = select.select([fd], [], [], min(timeout_s, hard_cap_s - (now - start)))
        if not r:
            break
        chunk = os.read(fd, 4096)
        if not chunk:
            break
        last_data = time.monotonic()
        buf += chunk
        while b"\n" in buf:
            line, _sep, buf = buf.partition(b"\n")
            yield line.rstrip(b"\r").decode("utf-8", errors="replace")
            emitted += 1
            if emitted >= max_lines:
                return
    if buf:
        yield buf.decode("utf-8", errors="replace") + "  [NO-LF]"


def send_line(fd: int, line: str):
    data = (line + "\n").encode("ascii")
    os.write(fd, data)


def listen_only(port: str):
    """Read forever without ever writing. Useful for watching the
    BOOT banner and `MODE sweep` self-test transition without poking
    the firmware out of its standalone state.

    Auto-reopens the device when it disappears (e.g. you power-cycle
    the ESP), so the natural workflow is just: start this, then plug
    in / cycle the ESP whenever and watch fresh boots. Ctrl-C to stop.
    """
    print(f"[host] listen-only on {port} — Ctrl-C to stop. Reopens on disconnect.")
    t0 = time.monotonic()
    try:
        while True:
            # Wait for the device node to exist (handles power-cycle / unplug).
            while not os.path.exists(port):
                time.sleep(0.1)
            try:
                # IMPORTANT: don't flush — the BOOT banner is usually
                # already sitting in the macOS CDC buffer by the time
                # we get here, and tcflush would discard it.
                fd = open_raw(port, flush=False)
            except OSError as e:
                print(f"  (+{time.monotonic()-t0:7.3f}s) [host] open failed: {e}; retrying")
                time.sleep(0.3)
                continue
            print(f"  (+{time.monotonic()-t0:7.3f}s) [host] opened {port}")
            buf = b""
            try:
                while True:
                    r, _, _ = select.select([fd], [], [], 1.0)
                    if not r:
                        # heartbeat tick — also lets us notice unplug via the
                        # next read attempt failing
                        if not os.path.exists(port):
                            break
                        continue
                    try:
                        chunk = os.read(fd, 4096)
                    except OSError:
                        chunk = b""
                    if not chunk:
                        break  # device gone
                    buf += chunk
                    while b"\n" in buf:
                        line, _sep, buf = buf.partition(b"\n")
                        text = line.rstrip(b"\r").decode("utf-8", errors="replace")
                        print(f"  (+{time.monotonic()-t0:7.3f}s) < {text!r}", flush=True)
            finally:
                if buf:
                    text = buf.decode("utf-8", errors="replace")
                    print(f"  (+{time.monotonic()-t0:7.3f}s) < {text!r}  [NO-LF]")
                os.close(fd)
            print(f"  (+{time.monotonic()-t0:7.3f}s) [host] disconnected, waiting for reconnect…")
    except KeyboardInterrupt:
        print("\n[host] bye")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--wait-boot", type=float, default=2.0,
                    help="Seconds to wait at startup for device boot banner")
    ap.add_argument("--listen-only", action="store_true",
                    help="Open the device read-only and print everything it "
                         "sends until Ctrl-C. Does not send any input — useful "
                         "for watching the firmware's standalone Sweep self-test.")
    args = ap.parse_args()

    if args.listen_only:
        # listen_only handles its own (re)open lifecycle.
        listen_only(args.port)
        return

    print(f"[host] opening {args.port}")
    fd = open_raw(args.port)
    try:
        # Drain any boot banner already buffered, then wait briefly for a fresh one.
        print("[host] listening for boot banner...")
        for line in read_lines(fd, timeout_s=args.wait_boot):
            print(f"    < {line!r}")

        # Test cases: (input, expected_ok)
        cases = [
            ("0512,0512,0512,0512", True),   # centered
            ("0256,0768,0768,0256", True),   # extreme corners
            ("0384,0640,0640,0384", True),   # halfway
            ("oops,no,good,data",   False),  # should be BAD
            ("0100,0100,0100",      False),  # only 3 fields → BAD
        ]
        for i, (line, want_ok) in enumerate(cases):
            print(f"\n[host] case {i}: sending {line!r} (expect {'OK' if want_ok else 'BAD'})")
            send_line(fd, line)
            got = list(read_lines(fd, timeout_s=0.4))
            for l in got:
                print(f"    < {l!r}")
            first = got[0] if got else ""
            ok = first.startswith("OK") if want_ok else first.startswith("BAD")
            print(f"    => {'PASS' if ok else 'FAIL'}")
            # Short gap so each response completes before next send.
            time.sleep(0.05)

        # Failsafe test: nudge the FW back into Follow with a valid line,
        # then stop sending for > 500 ms, expect a fresh "MODE center"
        # transition. (If we skip the refresh, the BAD-case fallout above
        # may have already transitioned us to Center and there's nothing
        # left to re-emit during the silent window.)
        print("\n[host] failsafe test: refresh to Follow, then silent 0.9s, expect 'MODE center'")
        send_line(fd, "0512,0512,0512,0512")
        for line in read_lines(fd, timeout_s=0.2):
            print(f"    < {line!r}")
        t0 = time.monotonic()
        saw_center = False
        for line in read_lines(fd, timeout_s=0.9):
            print(f"    < {line!r}  (+{time.monotonic()-t0:.3f}s)")
            if line.startswith("MODE center"):
                saw_center = True
        print(f"    => {'PASS' if saw_center else 'FAIL'}")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
