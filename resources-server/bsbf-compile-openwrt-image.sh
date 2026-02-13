#!/bin/sh
# This script compiles an OpenWrt image with the BSBF bonding solution client.
# Author: Chester A. Unal <chester.a.unal@arinc9.com>

usage() {
	echo "Usage: $0 --target <TARGET> --subtarget <SUBTARGET> --profile <PROFILE> --server-ipv4 <ADDR> --server-port <PORT> --uuid <UUID> [IBCG_ARGS]"
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
	*)
		IBCG_ARGS="${IBCG_ARGS:+$IBCG_ARGS }$1"
		shift
		;;
	esac
done

# Show usage if target, subtarget, profile, server IPv4 address, server port,
# and UUID were not provided.
{ [ -z "$target" ] || [ -z "$subtarget" ] || [ -z "$profile" ] || [ -z "$server_ipv4" ] || [ -z "$server_port" ] || [ -z "$uuid" ]; } && usage

# Update feeds and install all packages.
./scripts/feeds update && ./scripts/feeds install -a

# Clone bsbf-client-openwrt-config-generator.
git clone https://github.com/bondingshouldbefree/bsbf-client-openwrt-config-generator
cd bsbf-client-openwrt-config-generator

# Generate BSBF bonding configuration and install it.
PACKAGES=$(./bsbf-client-openwrt-config-generator.sh --server-ipv4 "$server_ipv4" --server-port "$server_port" --uuid "$uuid" $IBCG_ARGS)
install -D 99-bsbf-bonding ../files/etc/uci-defaults/99-bsbf-bonding

# Remove bsbf-client-openwrt-config-generator.
cd ..
rm -rf bsbf-client-openwrt-config-generator

# Configure the build system.
# Select device.
echo "CONFIG_TARGET_"$target"=y
CONFIG_TARGET_"$target"_"$subtarget"=y
CONFIG_TARGET_"$target"_"$subtarget"_DEVICE_"$profile"=y" > .config

# Enable BSBF packages.
for pkg in $PACKAGES; do
	# If package starts with a dash, remove it and disable the package.
	if [ "${pkg#-}" != "$pkg" ]; then
		pkg="${pkg#-}"
		echo "CONFIG_PACKAGE_${pkg}=n" >> .config
		continue
	fi
	echo "CONFIG_PACKAGE_${pkg}=y" >> .config
done
make defconfig

# Compile an image.
make -j$(nproc)

# Clean up.
rm -rf files
