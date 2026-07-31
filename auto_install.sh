#!/bin/bash

set -e

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="${script_dir}"
workspace_dir="$(cd "${package_dir}/../.." && pwd)"

pushd "${package_dir}" > /dev/null
echo "Working Path: $(pwd)"

ROS_VERSION=""
ROS_HUMBLE=""

# Set working ROS version
if [ "${1:-}" = "${VERSION_ROS2}" ]; then
    ROS_VERSION=${VERSION_ROS2}
elif [ "${1:-}" = "${VERSION_HUMBLE}" ]; then
    ROS_VERSION=${VERSION_ROS2}
    ROS_HUMBLE=${VERSION_HUMBLE}
elif [ "${1:-}" = "${VERSION_ROS1}" ]; then
    ROS_VERSION=${VERSION_ROS1}
elif [ -n "${ROS_VERSION:-}" ] && [ "${ROS_VERSION}" = "2" ]; then
    ROS_VERSION=${VERSION_ROS2}
elif [ -n "${ROS_VERSION:-}" ] && [ "${ROS_VERSION}" = "1" ]; then
    ROS_VERSION=${VERSION_ROS1}
else
    echo "Usage: bash auto_install.sh ROS1|ROS2|humble"
    exit 1
fi

echo "ROS version is: ${ROS_VERSION}"

# clear build artifacts from the workspace root.
rm -rf "${workspace_dir}/build/"
rm -rf "${workspace_dir}/devel/"
rm -rf "${workspace_dir}/install/"

# clear src/CMakeLists.txt if it exists.
if [ -f "${workspace_dir}/src/CMakeLists.txt" ]; then
    rm -f "${workspace_dir}/src/CMakeLists.txt"
fi

# substitute the package manifest according to ROS version.
if [ -f "${package_dir}/package.xml" ]; then
    rm -f "${package_dir}/package.xml"
fi

if [ "${ROS_VERSION}" = "${VERSION_ROS1}" ]; then
    cp -f "${package_dir}/package_ROS1.xml" "${package_dir}/package.xml"
else
    cp -f "${package_dir}/package_ROS2.xml" "${package_dir}/package.xml"
fi

# set ROS environment if available
if [ "${ROS_VERSION}" = "${VERSION_ROS1}" ]; then
    if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    elif [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
        ROS_DISTRO="noetic"
    fi
else
    if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        source "/opt/ros/${ROS_DISTRO}/setup.bash"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        ROS_DISTRO="humble"
    fi
fi

if [ -n "${ROS_DISTRO:-}" ]; then
    echo "ROS distro is: ${ROS_DISTRO}"
fi

# 执行脚本的实际任务
echo "编译软件包......"
cd "${workspace_dir}"
if [ "${ROS_VERSION}" = "${VERSION_ROS1}" ]; then
    catkin_make --pkg handsfree_rtk
else
    colcon build --packages-select handsfree_rtk --cmake-args -DROS_EDITION=${VERSION_ROS2} -DHUMBLE_ROS=${ROS_HUMBLE}
fi

echo "设置udev USB规则......"
cd "${package_dir}"
sudo cp usb_rules/handsfree_rtk.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb

# 修改 python 文件权限
chmod 777 scripts/*
chmod 777 demo/python/*

echo "软件包安装程序运行完毕"

popd > /dev/null
