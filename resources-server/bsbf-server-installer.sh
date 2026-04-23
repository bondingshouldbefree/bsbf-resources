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
		for file in /usr/local/etc/xray/[0-9]*-*.json; do
			[ -e "$file" ] || continue
			systemctl disable xray@"$(basename "$file" .json)"
			systemctl stop xray@"$(basename "$file" .json)"
			rm "$file"
		done

		# Remove files.
		rm -f \
			/usr/local/sbin/bsbf-add-client \
			/usr/local/sbin/bsbf-add-client-api \
			/usr/lib/systemd/system/bsbf-add-client-api.service \
			/usr/local/sbin/bsbf-list-client \
			/usr/lib/systemd/system/bsbf-mptcp-configuration.service \
			/etc/network/if-up.d/99-bsbf-plpmtu \
			/usr/local/sbin/bsbf-plpmtu \
			/etc/network/if-up.d/99-bsbf-rate-limiting \
			/usr/local/sbin/bsbf-rate-limiting \
			/usr/local/sbin/bsbf-remove-client \
			/etc/network/if-up.d/99-bsbf-tcp-in-udp \
			/usr/local/sbin/bsbf-tcp-in-udp \
			/usr/local/etc/xray/core-config.json \
			/etc/systemd/system/xray@.service.d/99-bsbf-bonding.conf
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

	if [ "$CLIENT_LIMIT" -eq 0 ] || [ $(( CLIENT_LIMIT & (CLIENT_LIMIT - 1) )) -ne 0 ]; then
		echo "Error: --client-limit must be a power of 2: 1, 2, 4, ..., 16384" >&2
		exit 1
	fi
fi

BSBF_RESOURCES="https://raw.githubusercontent.com/bondingshouldbefree/bsbf-resources/refs/heads/main"

# Install bash, ethtool, fping, git, make, nodejs, python3, clang, libelf-dev,
# libc6-dev-i386, and libbpf-dev.
apt update
apt install -y bash ethtool fping git make nodejs python3 clang libelf-dev libc6-dev-i386 libbpf-dev

# Install bsbf-add-client.
curl -s $BSBF_RESOURCES/resources-server/bsbf-add-client -o /usr/local/sbin/bsbf-add-client
chmod +x /usr/local/sbin/bsbf-add-client
[ -n "$CLIENT_LIMIT" ] && sed -i "s/^CLIENT_LIMIT=.*/CLIENT_LIMIT=$CLIENT_LIMIT/" /usr/local/sbin/bsbf-add-client

# Install bsbf-add-client-api.
curl -s $BSBF_RESOURCES/resources-server/bsbf-add-client-api -o /usr/local/sbin/bsbf-add-client-api
chmod +x /usr/local/sbin/bsbf-add-client-api
curl -s $BSBF_RESOURCES/resources-server/bsbf-add-client-api.service -o /usr/lib/systemd/system/bsbf-add-client-api.service

# Install bsbf-list-client.
curl -s $BSBF_RESOURCES/resources-server/bsbf-list-client -o /usr/local/sbin/bsbf-list-client
chmod +x /usr/local/sbin/bsbf-list-client

# Install bsbf-mptcp-configuration systemd service.
curl -s $BSBF_RESOURCES/resources-server/bsbf-mptcp-configuration.service -o /usr/lib/systemd/system/bsbf-mptcp-configuration.service

# Install bsbf-plpmtu.
curl -s $BSBF_RESOURCES/resources-server/99-bsbf-plpmtu -o /etc/network/if-up.d/99-bsbf-plpmtu
chmod +x /etc/network/if-up.d/99-bsbf-plpmtu
curl -s $BSBF_RESOURCES/resources-shared/bsbf-plpmtu -o /usr/local/sbin/bsbf-plpmtu
chmod +x /usr/local/sbin/bsbf-plpmtu

# Install bsbf-rate-limiting.
mkdir -p /usr/local/sbin
curl -s $BSBF_RESOURCES/resources-server/99-bsbf-rate-limiting -o /etc/network/if-up.d/99-bsbf-rate-limiting
chmod +x /etc/network/if-up.d/99-bsbf-rate-limiting
curl -s $BSBF_RESOURCES/resources-server/bsbf-rate-limiting -o /usr/local/sbin/bsbf-rate-limiting
chmod +x /usr/local/sbin/bsbf-rate-limiting

# Install bsbf-remove-client.
curl -s $BSBF_RESOURCES/resources-server/bsbf-remove-client -o /usr/local/sbin/bsbf-remove-client
chmod +x /usr/local/sbin/bsbf-remove-client

# Install bsbf-tcp-in-udp.
curl -s $BSBF_RESOURCES/resources-server/99-bsbf-tcp-in-udp -o /etc/network/if-up.d/99-bsbf-tcp-in-udp
chmod +x /etc/network/if-up.d/99-bsbf-tcp-in-udp
curl -s $BSBF_RESOURCES/resources-server/bsbf-tcp-in-udp -o /usr/local/sbin/bsbf-tcp-in-udp
chmod +x /usr/local/sbin/bsbf-tcp-in-udp
[ -n "$CLIENT_LIMIT" ] && sed -i "s/^CLIENT_LIMIT=.*/CLIENT_LIMIT=$CLIENT_LIMIT/" /usr/local/sbin/bsbf-tcp-in-udp

# Install plp-mtu-discovery.
git clone https://github.com/bondingshouldbefree/plp-mtu-discovery
cd plp-mtu-discovery && make install
install -D systemd/plpmtu-udp-server.service /usr/lib/systemd/system/plpmtu-udp-server.service
cd .. && rm -rf plp-mtu-discovery

# Install tcp-in-udp.
git clone https://github.com/multipath-tcp/tcp-in-udp.git
cd tcp-in-udp && make install && cd .. && rm -rf tcp-in-udp

# Install xray and its configuration.
curl -fsSL https://github.com/XTLS/Xray-install/raw/main/install-release.sh | bash -s -- install
systemctl disable --quiet xray
systemctl stop xray
curl -s $BSBF_RESOURCES/resources-server/core-config.json -o /usr/local/etc/xray/core-config.json
curl -s $BSBF_RESOURCES/resources-server/99-bsbf-bonding.conf -o /etc/systemd/system/xray@.service.d/99-bsbf-bonding.conf

# Reload all unit files in case they have been modified.
systemctl daemon-reload

# Enable and (re)start systemd services.
systemctl enable bsbf-mptcp-configuration plpmtu-udp-server
systemctl restart bsbf-mptcp-configuration plpmtu-udp-server

# Start xray for existing clients.
for file in /usr/local/etc/xray/[0-9]*-*.json; do
	[ -e "$file" ] || continue
	systemctl start xray@"$(basename "$file" .json)"
done

# Use the interface from the preferred default route. Apply what the if-up.d
# scripts would run for the interface. If there's no interface, the if-up.d
# scripts will be applied when an interface comes up.
IFACE=$(ip route show default | awk 'NR==1 {for (i=1;i<=NF;i++) if ($i=="dev") print $(i+1)}')
if [ -n "$IFACE" ]; then
	nohup bsbf-plpmtu "$IFACE" >/dev/null 2>&1 &
	bsbf-rate-limiting "$IFACE" >/dev/null 2>&1
	bsbf-tcp-in-udp l2 "$IFACE"
fi
