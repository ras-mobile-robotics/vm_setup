#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Define symbols and colors
INFO="[ \033[00;34m..\033[0m ]"  # Blue ..
OK="[ \033[00;32mOK\033[0m ]"    # Green OK
ERR="[ \033[00;31m!!\033[0m ]"   # Red !!

# Export DEBIAN_FRONTEND to prevent interactive prompts
export DEBIAN_FRONTEND=noninteractive

echo -e "\n$INFO Starting Full System & ROS 2 Jazzy Installation..."

# 1. Locale & Language Packs
echo -e "$INFO Setting up Locales and Language Packs..."
sudo apt-get update -qq
sudo apt-get install -y -qq locales language-pack-en language-pack-en-base language-pack-gnome-en language-pack-gnome-en-base > /dev/null
sudo locale-gen en_US en_US.UTF-8 > /dev/null
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo -e "$OK Language and Locales configured"

# 2. Add Repositories (ROS 2, VS Code, Tailscale)
echo -e "$INFO Configuring External Repositories..."

# ROS 2
sudo apt-get install -y -qq software-properties-common curl git wget unzip > /dev/null
sudo add-apt-repository -y universe > /dev/null
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -sL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb > /dev/null

# VS Code Key & Source
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
sudo install -D -o root -g root -m 644 microsoft.gpg /usr/share/keyrings/microsoft.gpg
rm -f microsoft.gpg
sudo bash -c "cat <<EOF > /etc/apt/sources.list.d/vscode.sources
Types: deb
URIs: https://packages.microsoft.com/repos/code
Suites: stable
Components: main
Architectures: amd64,arm64,armhf
Signed-By: /usr/share/keyrings/microsoft.gpg
EOF"

# Tailscale
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/noble.noarmor.gpg | sudo tee /usr/share/keyrings/tailscale-archive-keyring.gpg >/dev/null
curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/noble.tailscale-keyring.list | sudo tee /etc/apt/sources.list.d/tailscale.list >/dev/null

sudo apt-get update -qq
echo -e "$OK All repositories added"

# 3. Main Application Installation
echo -e "$INFO Installing System Tools, Desktop Apps, and CLI Utilities..."
sudo apt-get install -y -qq \
  apt-transport-https bsdutils cmake code curl dash diffutils \
  efibootmgr fd-find findutils gparted grep gzip hostname init login \
  mesa-utils ncurses-base ncurses-bin openssh-server pkg-config \
  python3-colcon-clean python3-netifaces python3-pip ripgrep \
  software-properties-common tilix tmux vim zsh \
  open-vm-tools open-vm-tools-desktop spice-vdagent spice-webdavd \
  python3-colcon-clean ntpdate \
  tailscale tailscale-archive-keyring > /dev/null
echo -e "$OK Utilities and Desktop tools installed"

# 4. Ubuntu Desktop & Drivers
echo -e "$INFO Installing Ubuntu Desktop and Drivers..."
sudo apt-get install -y -qq \
  ubuntu-desktop-minimal ubuntu-minimal ubuntu-standard \
  ubuntu-restricted-addons ubuntu-wallpapers \
  linux-generic-hwe-24.04 shim-signed > /dev/null
echo -e "$OK System core updated"

# 5. ROS 2 & Robotics Stack
echo -e "$INFO Installing ROS 2 Jazzy and TurtleBot 4 packages..."
sudo apt-get install -y -qq \
  ros-dev-tools ros-jazzy-desktop gz-harmonic \
  ros-jazzy-irobot-create-nodes ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-turtlebot4-description ros-jazzy-turtlebot4-desktop \
  ros-jazzy-turtlebot4-msgs ros-jazzy-turtlebot4-navigation \
  ros-jazzy-turtlebot4-node ros-jazzy-turtlebot4-simulator > /dev/null
echo -e "$OK ROS 2 and Robotics stack installed"

# 6. FastDDS Discovery Server Service
echo -e "$INFO Configuring FastDDS Discovery Server..."
SERVICE_FILE="/etc/systemd/system/fastdds-discovery.service"
USER_NAME="eva"

sudo bash -c "cat <<EOF > $SERVICE_FILE
[Unit]
Description=FastDDS Discovery Server for ROS 2 Simulation
After=network.target

[Service]
User=$USER_NAME
Type=simple
ExecStart=/bin/bash -c \"source /opt/ros/jazzy/setup.bash && /opt/ros/jazzy/bin/fast-discovery-server -i 0 -p 11811\"
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF"

sudo systemctl daemon-reload
sudo systemctl enable fastdds-discovery.service
sudo systemctl start fastdds-discovery.service
echo -e "$OK FastDDS Discovery Server service started"

# 7. CLI Power Tools (Autojump & FZF)
echo -e "$INFO Installing Autojump and FZF..."
cd /tmp
git clone https://github.com/wting/autojump.git > /dev/null
cd autojump
./install.py > /dev/null
cd ..

git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf > /dev/null
~/.fzf/install --all > /dev/null
echo -e "$OK CLI tools configured"

# 8. Nerd Fonts (JetBrains Mono)
echo -e "$INFO Installing JetBrains Mono Nerd Font..."
mkdir -p ~/.local/share/fonts
wget -q -P ~/.local/share/fonts https://github.com/ryanoasis/nerd-fonts/releases/latest/download/JetBrainsMono.zip
cd ~/.local/share/fonts
unzip -o JetBrainsMono.zip > /dev/null
rm JetBrainsMono.zip
fc-cache -fv > /dev/null
echo -e "$OK Fonts installed"

# 9. Rosdep
echo -e "$INFO Initializing Rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init > /dev/null 2>&1 || true
fi
rosdep update > /dev/null
echo -e "$OK Rosdep initialized"

# 10. Oh My Bash
echo -e "$INFO Installing Oh My Bash..."
export CHSH=no
export RUNZSH=no
bash -c "$(wget https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh -O -)" --unattended > /dev/null 2>&1 || true
echo -e "$OK Oh My Bash installed"

# 11. Input Methods
echo -e "$INFO Installing Input Methods..."
sudo apt-get install -y -qq \
  ibus-table-cangjie-big ibus-table-cangjie3 ibus-table-cangjie5 \
  libchewing3 libchewing3-data libm17n-0 libmarisa0 libopencc-data \
  libopencc1.1 libotf1 libpinyin-data libpinyin15 m17n-db wbritish > /dev/null
echo -e "$OK Input methods installed"

echo -e "\n\033[00;32m=======================================\033[0m"
echo -e "\033[00;32m   FULL INSTALLATION SUCCESSFUL       \033[0m"
echo -e "\033[00;32m=======================================\033[0m"
echo -e "NOTE: To use autojump, ensure you add '[ -f /home/$USER_NAME/.autojump/etc/profile.d/autojump.sh ] && . /home/$USER_NAME/.autojump/etc/profile.d/autojump.sh' to your .bashrc"

read -p "Would you like to reboot now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo reboot
fi