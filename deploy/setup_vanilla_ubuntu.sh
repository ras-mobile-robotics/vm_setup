#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Define symbols and colors
INFO="[ \033[00;34m..\033[0m ]"  # Blue ..
OK="[ \033[00;32mOK\033[0m ]"    # Green OK
ERR="[ \033[00;31m!!\033[0m ]"   # Red !!

echo -e "\n$INFO Starting Full System & ROS 2 Jazzy Installation..."

# 1. Locale & Language Packs
echo -e "$INFO Setting up Locales and Language Packs..."
sudo apt update -qq
sudo apt install -y locales language-pack-en language-pack-en-base language-pack-gnome-en language-pack-gnome-en-base > /dev/null
sudo locale-gen en_US en_US.UTF-8 > /dev/null
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo -e "$OK Language and Locales configured"

# 2. Add Repositories (ROS 2, VS Code, Tailscale)
echo -e "$INFO Configuring External Repositories..."

# ROS 2
sudo apt install -y software-properties-common curl git > /dev/null
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

sudo apt update -qq
echo -e "$OK All repositories added"

# 3. Main Application Installation
echo -e "$INFO Installing System Tools, Desktop Apps, and CLI Utilities..."
sudo apt install -y \
  apt-transport-https autojump bsdutils cmake code curl dash diffutils \
  efibootmgr fd-find findutils gparted grep gzip hostname init login \
  mesa-utils ncurses-base ncurses-bin openssh-server pkg-config \
  python3-colcon-clean python3-netifaces python3-pip ripgrep \
  software-properties-common tilix tmux vim wget zsh \
  open-vm-tools open-vm-tools-desktop spice-vdagent spice-webdavd \
  tailscale tailscale-archive-keyring bc > /dev/null
echo -e "$OK Utilities and Desktop tools installed"

# 4. Ubuntu Desktop & Drivers
echo -e "$INFO Installing Ubuntu Desktop and Drivers..."
sudo apt install -y \
  ubuntu-desktop-minimal ubuntu-minimal ubuntu-standard \
  ubuntu-restricted-addons ubuntu-wallpapers \
  linux-generic-hwe-24.04 shim-signed grub-efi-amd64 grub-efi-amd64-signed > /dev/null
echo -e "$OK System core updated"

# 5. ROS 2 & Robotics Stack
echo -e "$INFO Installing ROS 2 Jazzy and TurtleBot 4 packages..."
sudo apt install -y \
  ros-dev-tools ros-jazzy-desktop gz-harmonic \
  ros-jazzy-irobot-create-nodes ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-turtlebot4-description ros-jazzy-turtlebot4-desktop \
  ros-jazzy-turtlebot4-msgs ros-jazzy-turtlebot4-navigation \
  ros-jazzy-turtlebot4-node ros-jazzy-turtlebot4-simulator > /dev/null
echo -e "$OK ROS 2 and Robotics stack installed"



# 6. Rosdep
echo -e "$INFO Initializing Rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init > /dev/null 2>&1
fi
rosdep update > /dev/null
echo -e "$OK Rosdep initialized"

# 7. Git Repo Setup
# echo -e "$INFO Cloning vm_setup repository..."
# if [ ! -d "$HOME/vm_setup" ]; then
#     git clone https://github.com/ras-mobile-robotics/vm_setup "$HOME/vm_setup" > /dev/null
#     echo -e "$OK Repository cloned to ~/vm_setup"
# else
#     echo -e "$INFO ~/vm_setup already exists, skipping clone."
# fi

# 8. Oh My Bash
echo -e "$INFO Installing Oh My Bash..."
# We use OSH_NO_WELCOME_MESSAGE and keep stdin non-interactive to prevent the script from hanging
export CHSH=no
export RUNZSH=no
bash -c "$(wget https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh -O -)" --unattended > /dev/null 2>&1 || true
echo -e "$OK Oh My Bash installed"

# 9. Input Methods (IBus / Chewing)
echo -e "$INFO Installing Input Methods and IBus tables..."
sudo apt install -y \
  ibus-table-cangjie-big ibus-table-cangjie3 ibus-table-cangjie5 \
  libchewing3 libchewing3-data libm17n-0 libmarisa0 libopencc-data \
  libopencc1.1 libotf1 libpinyin-data libpinyin15 m17n-db wbritish > /dev/null
echo -e "$OK Input methods installed"

echo -e "\n\033[00;32m=======================================\033[0m"
echo -e "\033[00;32m   FULL INSTALLATION SUCCESSFUL       \033[0m"
echo -e "\033[00;32m=======================================\033[0m"
echo -e "NOTE: Please REBOOT your VM to apply all changes."

read -p "Would you like to reboot now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo reboot
fi