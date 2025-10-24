#!/bin/bash

# Desired WiFi network
TARGET_SSID="dog_go_zoom"

# Get current connected WiFi SSID
CURRENT_SSID=$(iwgetid -r)

# Check if connected to the correct WiFi
if [[ "$CURRENT_SSID" != "$TARGET_SSID" ]]; then
    echo "Error: Not connected to WiFi network '$TARGET_SSID'."
    return
fi

echo "Connected to '$TARGET_SSID'."

# Source ROS environment (change 'melodic' if using another distro)
if [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
else
    echo "Error: ROS distro not found at /opt/ros/melodic/setup.bash"
    return
fi

# Source your workspace
WORKSPACE=~/medos-demo-ws
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
    source "$WORKSPACE/devel/setup.bash"
else
    echo "Error: Workspace setup file not found at $WORKSPACE/devel/setup.bash"
    return
fi

# Set ROS_MASTER_URI (change if needed)
export ROS_MASTER_URI="http://192.168.123.161:11311"

# --- FIXED: Get ROS_IP from correct WiFi interface ---
WIFI_INTERFACE=$(iw dev | awk '$1=="Interface"{print $2}')

if [ -z "$WIFI_INTERFACE" ]; then
    echo "Error: No WiFi interface found."
    return
fi

ROS_IP=$(ip -4 addr show "$WIFI_INTERFACE" | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n 1)

if [ -z "$ROS_IP" ]; then
    echo "Error: No IP found for interface '$WIFI_INTERFACE'."
    return
fi

export ROS_IP="$ROS_IP"

# Confirm values
echo "Set ROS parameters: "
printenv | grep ROS
