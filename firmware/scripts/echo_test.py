#!/usr/bin/env python3
"""
Host-side smoke test for solcatears-fw.

Opens the Xiao ESP32-S3 over native USB CDC, sends a few calibration lines,
and reads back the firmware's echo ("OK ..."), bad-line ("BAD"), and
failsafe ("LIMP") frames.

stdlib only — no pyserial.
"""

import os, sys, termios, fcntl, select, time, re, argparse

DEFAULT_PORT = "/dev/cu.usbmodem101"


def open_raw(path: str) -> int:
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
    # Flush anything stale.
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--wait-boot", type=float, default=2.0,
                    help="Seconds to wait at startup for device boot banner")
    args = ap.parse_args()

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

        # Failsafe test: stop sending for > 500 ms, expect "LIMP".
        print("\n[host] failsafe test: silent for 0.9s, expecting 'LIMP' frame")
        t0 = time.monotonic()
        saw_limp = False
        for line in read_lines(fd, timeout_s=0.9):
            print(f"    < {line!r}  (+{time.monotonic()-t0:.3f}s)")
            if line.startswith("LIMP"):
                saw_limp = True
        print(f"    => {'PASS' if saw_limp else 'FAIL'}")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
