#!/usr/bin/bash
function can_up () {
    local RES
    local x
    RES=`ip -br a show "$1"` && \
	echo "$RES" && \
	x=($RES) && \
	[[ ${x[1]} != 'UP' ]] && \
	sudo ip link set "$1" type can bitrate 1000000 && \
	sudo ip link set "$1" up &&\
	ip -br a show "$1"
}
if [ "$0" == "${BASH_SOURCE}" ]; then
    for ff in /sys/class/net/can*;  do
	can_up `basename "$ff"`
    done
fi
