#!/bin/sh
# SPDX-License-Identifier: AGPL-3.0-or-later
# Copyright (C) 2025-2026 Chester A. Unal <chester.a.unal@arinc9.com>

usage() {
	echo "Usage: $0 --target <TARGET> --subtarget <SUBTARGET> --profile <PROFILE> --server-ipv4 <ADDR> --server-port <PORT> --uuid <UUID> [--local]"
	exit 1
}

# Parse arguments.
while [ $# -gt 0 ]; do
	case "$1" in
	--target)
		[ -z "$2" ] && usage
		target="$2"
		shift 2
		;;
	--subtarget)
		[ -z "$2" ] && usage
		subtarget="$2"
		shift 2
		;;
	--profile)
		[ -z "$2" ] && usage
		profile="$2"
		shift 2
		;;
	--server-ipv4)
		[ -z "$2" ] && usage
		server_ipv4="$2"
		shift 2
		;;
	--server-port)
		[ -z "$2" ] && usage
		server_port="$2"
		shift 2
		;;
	--uuid)
		[ -z "$2" ] && usage
		uuid="$2"
		shift 2
		;;
	--local)
		local=1
		shift
		;;
	*)
		usage
		;;
	esac
done

# Show usage if target, subtarget, profile, server IPv4 address, server port,
# and UUID were not provided.
{ [ -z "$target" ] || [ -z "$subtarget" ] || [ -z "$profile" ] || [ -z "$server_ipv4" ] || [ -z "$server_port" ] || [ -z "$uuid" ]; } && usage

if [ -n "$local" ]; then
	# Download imagebuilder if its directory doesn't exist.
	if [ ! -d "openwrt-imagebuilder-$target-$subtarget.Linux-x86_64" ]; then
		curl -O "https://downloads.openwrt.org/snapshots/targets/$target/$subtarget/openwrt-imagebuilder-$target-$subtarget.Linux-x86_64.tar.zst"
		tar xf "openwrt-imagebuilder-$target-$subtarget.Linux-x86_64.tar.zst"
	fi
	cd openwrt-imagebuilder-$target-$subtarget.Linux-x86_64

	# Configure bsbf-bonding.
	mkdir -p files/etc/bsbf
	cat <<-EOF > files/etc/bsbf/bsbf-bonding.conf
	server_ipv4=$server_ipv4
	server_port=$server_port
	uuid=$uuid
	EOF

	# Generate an image.
	make image \
	PROFILE="$profile" \
	PACKAGES="kmod-ifb kmod-nft-tproxy bsbf-bonding" \
	FILES="files"
	cd ..

	exit
fi

# Request an image from OpenWrt's Attendedsysupgrade.
ASU_BUILD_URL="https://sysupgrade.openwrt.org/api/v1/build"
ASU_STORE_URL="https://sysupgrade.openwrt.org/store"

resp=$(echo "# Configure bsbf-bonding.
mkdir -p /etc/bsbf
cat <<EOF > /etc/bsbf/bsbf-bonding.conf
server_ipv4=$server_ipv4
server_port=$server_port
uuid=$uuid
EOF" |
jq -Rs \
  --arg profile "$profile" \
  --arg target "$target/$subtarget" '
{
  defaults: .,
  packages: [
    "kmod-ifb",
    "kmod-nft-tproxy",
    "bsbf-bonding"
  ],
  profile: $profile,
  target: $target,
  version: "SNAPSHOT"
}' |
curl -fsS -X POST "$ASU_BUILD_URL" \
  -H "Content-Type: application/json" \
  -d @-) || {
	echo "ASU build request failed." >&2
	exit 1
}

request_hash=$(printf '%s\n' "$resp" | jq -er .request_hash) || {
	echo "No request_hash in ASU response." >&2
	printf '%s\n' "$resp" >&2
	exit 1
}

while true; do
	body=$(curl -fsS "$ASU_BUILD_URL/$request_hash") || {
		echo "ASU poll failed." >&2
		continue
	}
	urls=$(printf '%s\n' "$body" | jq -r --arg base "$ASU_STORE_URL" '
		.bin_dir as $d | (.images // [])[] | "\($base)/\($d)/\(.name)"')
	if [ -n "$urls" ]; then
		printf '%s\n' "$urls"
		exit
	fi
	status=$(printf '%s\n' "$body" | jq -r '.detail // "unknown"')
	echo "Image build status: $status" >&2
	sleep 5
done
