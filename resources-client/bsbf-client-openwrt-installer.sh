#!/bin/sh
# SPDX-License-Identifier: AGPL-3.0-or-later
# Copyright (C) 2026 Chester A. Unal <chester.a.unal@arinc9.com>

usage() {
	echo "Usage: $0 --server-ipv4 <ADDR> --server-port <PORT> --uuid <UUID>"
	exit 1
}

# Parse arguments.
while [ $# -gt 0 ]; do
	case "$1" in
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
	*)
		usage
		;;
	esac
done

# Show usage if server IPv4 address, server port, and UUID were not provided.
{ [ -z "$server_ipv4" ] || [ -z "$server_port" ] || [ -z "$uuid" ]; } && usage

echo "Configuring bsbf-bonding."
mkdir -p /etc/bsbf
cat <<EOF > /etc/bsbf/bsbf-bonding.conf
server_ipv4=$server_ipv4
server_port=$server_port
uuid=$uuid
EOF

echo "Installing kmod-ifb, kmod-nft-tproxy, and bsbf-bonding."
apk add kmod-ifb kmod-nft-tproxy
# Install bsbf-bonding last as it depends on packages above.
apk add bsbf-bonding

[ $? -ne 0 ] && echo "Installation failed. Try building an image from https://fs.bondingshouldbefree.org/ instead." && exit 1
