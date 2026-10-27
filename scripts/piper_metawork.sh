#!/usr/bin/bash
PKG_NAME=PiPER-control-LIU
[ "$CAN_PORT" = "" ] && export CAN_PORT=can_piper_r4e31
[ "$ROBOT_TYPE" = "" ] && export ROBOT_TYPE="$PKG_NAME"
[ "$ROBOT_UUID" = "" ] && export ROBOT_UUID=MA100101000019005100xxx
[ "$CA_CERTS_PATH" = "" ] && \
    export CA_CERTS_PATH=~/.local/share/mkcert/rootCA.pem
[ "$MQTT_URI" = "" ] && export MQTT_URI=wss://192.168.207.145:8333
ros2 launch mqtt_teleop teleop_agx_arm.launch.py \
     can_port:="$CAN_PORT" \
     arm_type:=piper \
     broker_uri:="$MQTT_URI"
