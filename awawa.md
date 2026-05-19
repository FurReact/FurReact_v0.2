# install-on-frame
cd /home/steamos/
mkdir workspace
cd workspace

# Install rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
. ~/.bashrc


# install esp toolchain
cargo install espup --locked
espup install
sh ~/export-esp.sh
cargo install espflash

git clone git@github.com:FurReact/FurReact_v0.2.git
