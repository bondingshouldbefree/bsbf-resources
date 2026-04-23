#!/bin/sh
# SPDX-License-Identifier: AGPL-3.0-or-later
# Copyright (C) 2025-2026 Chester A. Unal <chester.a.unal@arinc9.com>

DEVICE="$1"
ACTION="$2"

[ "$ACTION" = "up" ] || exit

[ "$DEVICE" = qmimux* ] && { bsbf-tcp-in-udp l3 "$DEVICE"; exit; }
bsbf-tcp-in-udp l2 "$DEVICE"
