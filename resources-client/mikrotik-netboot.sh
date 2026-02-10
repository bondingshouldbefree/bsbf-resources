#!/bin/sh

IFNAME="$1"
FILENAME="$2"

ip addr replace 192.168.1.10/24 dev $IFNAME
ip link set dev $IFNAME up

dnsmasq \
--no-daemon \
--interface=$IFNAME \
--port=0 \
--dhcp-authoritative \
--dhcp-range=192.168.1.100,192.168.1.200,12h \
--bootp-dynamic \
--dhcp-boot=$FILENAME \
--enable-tftp \
--tftp-root=$(pwd)
