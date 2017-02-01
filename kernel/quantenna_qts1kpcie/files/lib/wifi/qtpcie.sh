#!/bin/sh
#
# Script for Quantenna 5gz pcie radios
#
append DRIVERS "qtpcie"
dev=
found=

find_qtpcie_phy() {
	return 1 #there is no phy for quantenna devices
}

check_qtpcie_device() {
	local ifname=$1
	if [ "${ifname#wifi}" = "wifi" ]; then
		config_get control_interface "$ifname" control_interface
		[ "$control_interface" = "$dev" ] && found=1
	fi
	return 0
}

detect_qtpcie() {
	local devidx=0
	config_load wireless
	while :; do 
		config_get type "wifi$devidx" type
		[ -n "$type" ] || break
		devidx=$(($devidx + 1))
	done

	for _dev in /sys/class/net/*; do
		[ -d "$_dev" ] || continue
		_dev="${_dev##*/}"
		expr match "$_dev" "host" > /dev/null || continue
		[ $(echo "$_dev"|tr -dc '0-9') -ge $devidx ] || continue
		dev="$_dev"

		found=0
		config_foreach check_qtpcie_device wifi-device
		[ "$found" -gt 0 ] && {
			devidx=$(($devidx + 1))
			continue
		}

		cat <<EOF
config wifi-device  wifi$devidx
        option type              qtpcie
        option channel           auto
        option macaddr           $(cat /sys/class/net/${dev}/address)
        option hwmode            11ac
        option control_interface $dev
        option control_cdir		1.1.1.1/29
        # REMOVE THIS LINE TO ENABLE WIFI:
        option disabled          1

config wifi-iface
        option device     wifi$devidx
        option network    lan
        option confidx    $devidx
        option mode       ap
        option ssid       OpenWrt
        option encryption none

EOF
		devidx=$(($devidx + 1))
	done
}
