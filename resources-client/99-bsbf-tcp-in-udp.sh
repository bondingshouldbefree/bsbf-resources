#!/bin/sh

DEVICE="$1"
ACTION="$2"

[ "$ACTION" = "up" ] || exit

[ "$DEVICE" = qmimux* ] && { bsbf-tcp-in-udp l3 "$DEVICE"; exit; }
bsbf-tcp-in-udp l2 "$DEVICE"
