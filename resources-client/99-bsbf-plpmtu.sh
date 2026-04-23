#!/bin/sh
# SPDX-License-Identifier: AGPL-3.0-or-later
# Copyright (C) 2026 Chester A. Unal <chester.a.unal@arinc9.com>

DEVICE="$1"
ACTION="$2"

[ "$ACTION" = "up" ] || exit

case "$DEVICE" in
	eth*|en*)
		bsbf-plpmtu "$DEVICE" &
		;;
esac
