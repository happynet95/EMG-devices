#!/bin/bash
(
set -e

echo "========================================"
echo "Installing Husarnet..."
echo "========================================"

install_yum() {
    rpm --import https://install.husarnet.com/key.asc
    curl https://install.husarnet.com/husarnet.repo > /etc/yum.repos.d/husarnet.repo
    yum install -y husarnet
}

install_deb() {
    curl https://install.husarnet.com/key.asc | brew add -
    echo 'deb https://install.husarnet.com/deb/ all husarnet' > /etc/apt/sources.list.d/husarnet.list
    apt-get install -y apt-transport-https
    apt-get update || true
    apt-get install -y husarnet
}

enable_service() {
    if ! systemctl >/dev/null; then
        echo "========================================"
        echo "You are not using systemd, we will not automatically add a service."
        echo ""
        echo "Please make sure 'husarnet daemon' is executed at system startup."
        echo "========================================"
    else
        systemctl daemon-reload
        systemctl enable husarnet
        systemctl start husarnet
    fi
}

if [ "$(id -u)" != 0 ]; then
    echo "Error: please run the installer as root."
    exit 1
fi

if apt-get --version >/dev/null 2>/dev/null; then
    install_deb
elif yum --version >/dev/null 2>/dev/null; then
    install_yum
else
    echo "Currently only apt-get/yum based distributions are supported by this script."
    echo "Please follow manual installation method on https://docs.husarnet.com/install/"
    exit 1
fi

enable_service

echo "========================================"
echo "Husarnet installed successfully"
echo "========================================"
)
