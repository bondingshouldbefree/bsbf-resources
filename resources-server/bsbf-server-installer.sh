#!/bin/sh
# SPDX-License-Identifier: AGPL-3.0-or-later
# Copyright (C) 2025-2026 Chester A. Unal <chester.a.unal@arinc9.com>

usage() {
	echo "Usage: $0 [--client-limit <COUNT>] | --uninstall"
	exit 1
}

# Parse arguments.
while [ $# -gt 0 ]; do
	case "$1" in
	--client-limit)
		[ -z "$2" ] && usage
		CLIENT_LIMIT="$2"
		shift 2
		;;
	--uninstall)
		# Disable and stop systemd services.
		systemctl disable bsbf-add-client-api bsbf-mptcp-configuration
		systemctl stop bsbf-add-client-api bsbf-mptcp-configuration

		# Disable and stop xray and delete configuration for existing
		# clients.
		for file in /usr/local/etc/xray-bsbf-bonding/[0-9]*-*.json; do
			[ -e "$file" ] || continue
			systemctl disable xray-bsbf-bonding@"$(basename "$file" .json)"
			systemctl stop xray-bsbf-bonding@"$(basename "$file" .json)"
			rm "$file"
		done

		# Remove files.
		rm -rf \
			/usr/local/sbin/bsbf-add-client \
			/usr/local/sbin/bsbf-add-client-api \
			/usr/lib/systemd/system/bsbf-add-client-api.service \
			/usr/local/sbin/bsbf-list-client \
			/usr/lib/systemd/system/bsbf-mptcp-configuration.service \
			/etc/network/if-up.d/99-bsbf-rate-limiting \
			/usr/local/sbin/bsbf-rate-limiting \
			/usr/local/sbin/bsbf-remove-client \
			/usr/local/etc/xray-bsbf-bonding/ \
			/etc/systemd/system/xray-bsbf-bonding@.service \
			/etc/systemd/system/xray-bsbf-bonding@.service.d/
		exit
		;;
	*)
		usage
		;;
	esac
done

if [ -n "$CLIENT_LIMIT" ]; then
	case "$CLIENT_LIMIT" in
	*[!0-9]*)
		echo "Error: --client-limit must be a number" >&2
		exit 1
		;;
	esac

	if [ "$CLIENT_LIMIT" -gt 16384 ]; then
		echo "Error: --client-limit must not exceed 16384" >&2
		exit 1
	fi
fi

BSBF_RESOURCES="https://raw.githubusercontent.com/bondingshouldbefree/bsbf-resources/refs/heads/main"

echo "Installing bash and nodejs."
apt-get update >/dev/null
apt-get install -y bash nodejs >/dev/null

echo "Installing bsbf-add-client."
curl -s $BSBF_RESOURCES/resources-server/bsbf-add-client -o /usr/local/sbin/bsbf-add-client
chmod +x /usr/local/sbin/bsbf-add-client
[ -n "$CLIENT_LIMIT" ] && sed -i "s/^CLIENT_LIMIT=.*/CLIENT_LIMIT=$CLIENT_LIMIT/" /usr/local/sbin/bsbf-add-client

echo "Installing bsbf-add-client-api."
curl -s $BSBF_RESOURCES/resources-server/bsbf-add-client-api -o /usr/local/sbin/bsbf-add-client-api
chmod +x /usr/local/sbin/bsbf-add-client-api
curl -s $BSBF_RESOURCES/resources-server/bsbf-add-client-api.service -o /usr/lib/systemd/system/bsbf-add-client-api.service

echo "Installing bsbf-list-client."
curl -s $BSBF_RESOURCES/resources-server/bsbf-list-client -o /usr/local/sbin/bsbf-list-client
chmod +x /usr/local/sbin/bsbf-list-client

echo "Installing bsbf-mptcp-configuration systemd service."
curl -s $BSBF_RESOURCES/resources-server/bsbf-mptcp-configuration.service -o /usr/lib/systemd/system/bsbf-mptcp-configuration.service

echo "Installing bsbf-rate-limiting."
mkdir -p /usr/local/sbin
curl -s $BSBF_RESOURCES/resources-server/99-bsbf-rate-limiting -o /etc/network/if-up.d/99-bsbf-rate-limiting
chmod +x /etc/network/if-up.d/99-bsbf-rate-limiting
curl -s $BSBF_RESOURCES/resources-server/bsbf-rate-limiting -o /usr/local/sbin/bsbf-rate-limiting
chmod +x /usr/local/sbin/bsbf-rate-limiting

echo "Installing bsbf-remove-client."
curl -s $BSBF_RESOURCES/resources-server/bsbf-remove-client -o /usr/local/sbin/bsbf-remove-client
chmod +x /usr/local/sbin/bsbf-remove-client

echo "Installing xray and its configuration."
curl -fsSL https://github.com/XTLS/Xray-install/raw/main/install-release.sh | bash -s -- install >/dev/null 2>&1
systemctl disable --quiet xray
systemctl stop xray
mkdir -p /usr/local/etc/xray-bsbf-bonding
curl -s $BSBF_RESOURCES/resources-server/core-config.json -o /usr/local/etc/xray-bsbf-bonding/core-config.json
cp /etc/systemd/system/xray@.service /etc/systemd/system/xray-bsbf-bonding@.service
mkdir -p /etc/systemd/system/xray-bsbf-bonding@.service.d
curl -s $BSBF_RESOURCES/resources-server/99-bsbf-bonding.conf -o /etc/systemd/system/xray-bsbf-bonding@.service.d/99-bsbf-bonding.conf

echo "Reloading all unit files in case they have been modified."
systemctl daemon-reload

echo "Enabling and (re)starting systemd services."
systemctl enable --quiet bsbf-mptcp-configuration
systemctl restart bsbf-mptcp-configuration

echo "Starting xray for existing clients."
for file in /usr/local/etc/xray-bsbf-bonding/[0-9]*-*.json; do
	[ -e "$file" ] || continue
	systemctl start xray-bsbf-bonding@"$(basename "$file" .json)"
done

# Use the interface from the preferred default route. Apply what the if-up.d
# scripts would run for the interface. If there's no interface, the if-up.d
# scripts will be applied when an interface comes up.
IFACE=$(ip route show default | awk 'NR==1 {for (i=1;i<=NF;i++) if ($i=="dev") print $(i+1)}')
[ -n "$IFACE" ] && bsbf-rate-limiting "$IFACE" >/dev/null 2>&1
