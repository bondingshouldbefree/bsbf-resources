#!/bin/sh
# SPDX-License-Identifier: AGPL-3.0-or-later
# Copyright (C) 2025-2026 Chester A. Unal <chester.a.unal@arinc9.com>

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

BSBF_RESOURCES="https://raw.githubusercontent.com/bondingshouldbefree/bsbf-resources/refs/heads/main"

# Install bash, curl, fping, git, jq, lighttpd, mptcpize, and usb-modeswitch.
apt update
apt install -y bash curl fping git jq lighttpd mptcpize usb-modeswitch

# Install bsbf-bonding.
curl -s $BSBF_RESOURCES/resources-client/bsbf-bonding -o /usr/local/sbin/bsbf-bonding
chmod +x /usr/local/sbin/bsbf-bonding

# Install bsbf-client-web.
mkdir -p /srv/bsbf-client-web/cgi-bin
curl -s $BSBF_RESOURCES/resources-client/bsbf-client-web/index.html -o /srv/bsbf-client-web/index.html
curl -s $BSBF_RESOURCES/resources-client/bsbf-client-web/index.js -o /srv/bsbf-client-web/index.js
curl -s $BSBF_RESOURCES/resources-client/bsbf-client-web/styles.css -o /srv/bsbf-client-web/styles.css
curl -s $BSBF_RESOURCES/resources-client/bsbf-client-web/cgi-bin/bsbf-client-web -o /srv/bsbf-client-web/cgi-bin/bsbf-client-web
chmod +x /srv/bsbf-client-web/cgi-bin/bsbf-client-web
curl -s $BSBF_RESOURCES/resources-client/lighttpd-bsbf-client-web.conf -o /etc/lighttpd/bsbf-client-web.conf
sed -e 's#/run/lighttpd.pid#/run/lighttpd-bsbf-client-web.pid#g' \
    -e 's#/etc/lighttpd/lighttpd.conf#/etc/lighttpd/bsbf-client-web.conf#g' \
    -e '/^RestrictAddressFamilies=/ s/$/ AF_NETLINK/' \
    /usr/lib/systemd/system/lighttpd.service > /usr/lib/systemd/system/lighttpd-bsbf-client-web.service
echo ReadWritePaths=/usr/local/etc/bsbf >> /usr/lib/systemd/system/lighttpd-bsbf-client-web.service

# Install bsbf-mptcp.
mkdir -p /usr/local/sbin
curl -s $BSBF_RESOURCES/resources-client/bsbf-mptcp -o /usr/local/sbin/bsbf-mptcp
chmod +x /usr/local/sbin/bsbf-mptcp
curl -s $BSBF_RESOURCES/resources-client/bsbf-mptcp-helper -o /usr/local/sbin/bsbf-mptcp-helper
chmod +x /usr/local/sbin/bsbf-mptcp-helper
curl -s $BSBF_RESOURCES/resources-client/bsbf-mptcp.service -o /usr/lib/systemd/system/bsbf-mptcp.service

# Install bsbf-rate-limiting.
curl -s $BSBF_RESOURCES/resources-client/bsbf-rate-limiting -o /usr/local/sbin/bsbf-rate-limiting
chmod +x /usr/local/sbin/bsbf-rate-limiting

# Install xray and its configuration.
curl -fsSL https://github.com/XTLS/Xray-install/raw/main/install-release.sh | bash -s -- install
systemctl disable --quiet xray
systemctl stop xray
curl -s $BSBF_RESOURCES/resources-client/xray.json -o /usr/local/etc/xray/bsbf-bonding.json
curl -s $BSBF_RESOURCES/resources-client/99-bsbf-bonding.conf -o /etc/systemd/system/xray@.service.d/99-bsbf-bonding.conf
mkdir -p /etc/nftables
curl -s $BSBF_RESOURCES/resources-client/bsbf_bonding.nft -o /etc/nftables/bsbf_bonding.nft

# Reload all unit files in case they have been modified.
systemctl daemon-reload

# Configure bsbf-bonding.
mkdir -p /usr/local/etc/bsbf
cat <<EOF > /usr/local/etc/bsbf/bsbf-bonding.conf
server_ipv4=$server_ipv4
server_port=$server_port
uuid=$uuid
EOF

# Enable bonding.
bsbf-bonding --enable
