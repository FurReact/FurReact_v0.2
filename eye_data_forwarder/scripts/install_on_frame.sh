#!/usr/bin/env bash
# Install the forwarder + systemd units on a Steam Frame.
#
# Usage:
#   FRAME=steamos@frame.local ./install_on_frame.sh
#   FRAME=steamos@<ip>        ./install_on_frame.sh
#
# Assumes:
#   - rust is already installed in the steamos user's home (~/.cargo/bin)
#     (one-time, see eye_data_forwarder/README.md step 2)
#   - the steamos user can sudo without password for systemctl/cp into
#     /etc/systemd/system (default on stock SteamOS)
set -euo pipefail

: "${FRAME:?set FRAME=user@host (e.g. steamos@frame.local)}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_DIR=/home/steamos/work/eye_data_forwarder

# echo "→ rsyncing source to $FRAME:$REMOTE_DIR"
# ssh "$FRAME" "mkdir -p $REMOTE_DIR"
# rsync -az --delete --exclude=target --exclude=.git \
#   "$HERE/" "$FRAME:$REMOTE_DIR/"

# echo "→ building --release on the Frame"
# ssh "$FRAME" "cd $REMOTE_DIR && ~/.cargo/bin/cargo build --release"

# echo "→ makerootfsrw"
# ssh "$FRAME" "makerootfsrw"

echo "→ installing systemd units"
ssh "$FRAME" "sudo install -m 0644 \
    $REMOTE_DIR/systemd/solcatears-usb-host.service \
    $REMOTE_DIR/systemd/solcatears-forwarder.service \
    /etc/systemd/system/"
ssh "$FRAME" "sudo systemctl daemon-reload"

echo "→ enabling + (re)starting services"
ssh "$FRAME" "sudo systemctl enable --now solcatears-usb-host.service"
ssh "$FRAME" "sudo systemctl enable solcatears-forwarder.service"
ssh "$FRAME" "sudo systemctl restart solcatears-forwarder.service"

echo
echo "✓ done. status:"
ssh "$FRAME" "systemctl --no-pager status solcatears-usb-host.service solcatears-forwarder.service" || true
echo
echo "  follow logs with:  ssh $FRAME 'journalctl -fu solcatears-forwarder'"
echo "  if the ESP wasn't enumerating, replug it (Linux dwc3 limitation when"
echo "  a device was attached during the wrong USB role)."
