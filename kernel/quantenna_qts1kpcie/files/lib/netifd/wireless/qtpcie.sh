#!/bin/sh
#
# Script for Quantenna 5gz pcie radios
#
. /lib/netifd/netifd-wireless.sh

init_wireless_driver "$@"

QT_LOG_FILE=/tmp/qtpcie-wireless.log
QT_SESSION_FILE=/tmp/qtpcie-session
QT_SESSION_TIMEOUT=250
ready_to_set=0
initialized=0
wait_for_ready() {
	local times=0
	local mode

	while [ $times -le 20 ]; do
		echo "qcsapi_sockrpc --host $control_device_ip get_mode wifi0" >> $QT_LOG_FILE
		local mode=`qcsapi_sockrpc --host $control_device_ip get_mode wifi0`
		if [ "$mode" = "Access point" -o "$mode" = "Station" ]; then
			ready_to_set=1
			break;
		fi
		sleep 1s
		times=$(($times + 1))
		echo "Check times = $times" >> $QT_LOG_FILE
	done
}

is_alive() {
	[ "$ready_to_set" = "1" ] && return

	if [ -f $QT_SESSION_FILE ]; then
		local last_time=`cat $QT_SESSION_FILE`
		local curr_time=`cat /proc/uptime | awk -F . '{print $1}'`

		time_delta=$(( $curr_time - $last_time ))
		[ $time_delta -le $QT_SESSION_TIMEOUT ] && {
			ready_to_set=1
			echo "Session Has yet to expire: [$last_time]-[$curr_time]" >> $QT_LOG_FILE
		}
	fi

	if [ "$ready_to_set" = "0" ]; then
		wait_for_ready
		[ "$ready_to_set" = "1" ] && {
			cat /proc/uptime | awk -F . '{print $1}' > $QT_SESSION_FILE
			echo "Session ID: "`cat $QT_SESSION_FILE` >> $QT_LOG_FILE
		}
	fi
}

qt_call() {
	is_alive
	if [ "$ready_to_set" = "0" ]; then
		wireless_setup_failed "cannot communicate with qt chip, exit qt_call"
		return
	fi

	idx=$1
	req=$2
	shift
	shift
	local val_num= val1= val2= val3= val4= val=
	val_num=$#
	[ "$#" != "0" ] && val1=$1 && shift
	[ "$#" != "0" ] && val2=$1 && shift
	[ "$#" != "0" ] && val3=$1 && shift
	[ "$#" != "0" ] && val4=$1 && shift
	val=$@
	local do_wait="&"

	[ "$ready_to_set" = "0" ] && return
	case "$req" in
		set_passphrase|set_PSK)
			qt_wifi_intf="wifi${idx} 0"
			;;
		rename_SSID)
			cur_ssid=`qcsapi_sockrpc --host $control_device_ip get_SSID_list wifi${idx}`
			if [ -z "$cur_ssid" ]; then
				req="create_SSID"
			else
				[ "$cur_ssid" = "$val1" ] && return
				val2="$val1"
				val1="$cur_ssid"
				val_num=2
			fi
			qt_wifi_intf="wifi${idx}"
			;;
		disable_wps|wifi_create_bss|save_wps_ap_pin|apply_security_config)
			do_wait=
			qt_wifi_intf="wifi${idx}"
			;;
		rfenable)
			[ "$val1" -eq "1" ] && {
				wireless_process_kill_all
                                killall qevt_client
				/usr/sbin/qevt_client -h $control_device_ip &
				wireless_add_process "$!" "/usr/sbin/qevt_client" 1
			}
			[ "$val1" -eq "0" ] && {
				wireless_process_kill_all
				killall qevt_client
			}
			do_wait=
			qt_wifi_intf=""
			;;
		set_wifi_macaddr)
			do_wait=
			qt_wifi_intf=""
			;;
		*)
			qt_wifi_intf="wifi${idx}"
			;;
	esac


	echo "qcsapi_sockrpc --host $control_device_ip $req $qt_wifi_intf $val1 $val2 $val3 $val4 $val $do_wait" >> $QT_LOG_FILE
	case "$val_num" in
		0)
			qcsapi_sockrpc --host $control_device_ip $req $qt_wifi_intf $do_wait >> $QT_LOG_FILE 2>&1
			;;
		1)
			qcsapi_sockrpc --host $control_device_ip $req $qt_wifi_intf "$val1" $do_wait >> $QT_LOG_FILE 2>&1
			;;
		2)
			qcsapi_sockrpc --host $control_device_ip $req $qt_wifi_intf "$val1" "$val2" $do_wait >> $QT_LOG_FILE 2>&1
			;;
		3)
			qcsapi_sockrpc --host $control_device_ip $req $qt_wifi_intf "$val1" "$val2" "$val3" $do_wait >> $QT_LOG_FILE 2>&1
			;;
		4)
			qcsapi_sockrpc --host $control_device_ip $req $qt_wifi_intf "$val1" "$val2" "$val3" "$val4" $do_wait >> $QT_LOG_FILE 2>&1
			;;
		*)
			qcsapi_sockrpc --host $control_device_ip $req $qt_wifi_intf "$val1" "$val2" "$val3" "$val4" "$val" $do_wait >> $QT_LOG_FILE 2>&1
			;;
	esac
}

drv_qtpcie_init_device_config() {
	config_add_string country

	config_add_string 'macaddr:macaddr'
	config_add_string hwmode
	config_add_string htmode
	config_add_int bf
	config_add_string control_interface
	config_add_string control_host_ip 
	config_add_string control_device_ip
	config_add_int txpower
}

drv_qtpcie_init_iface_config() {
	config_add_string 'bssid:macaddr' 'ssid:string'
	config_add_string 'password:wpakey'

	config_add_int confidx
	config_add_array wdspeerlist
	config_add_string macfilter
	config_add_array 'maclist:list(macaddr)'
	config_add_boolean isolate
	config_add_boolean hidden
}


qtpcie_interface_cleanup() {
	json_select config
	json_get_var confidx confidx
	json_select ..
	local op_mode=`qcsapi_sockrpc --host $control_device_ip get_mode wifi${confidx}`
	[ "$op_mode" = "Access point" ] && [ "$confidx" -ne 0 ] && qt_call $confidx wifi_remove_bss
}

drv_qtpcie_cleanup() {
	json_select config
	json_get_var control_device_ip control_device_ip
	json_select ..
	if [ "$initialized" = "1" ]; then
		for_each_interface "sta ap adhoc" qtpcie_interface_cleanup
		qt_call 0 rfenable 0
	fi
}

drv_qtpcie_setup() {
	json_select config
	json_get_vars \
			control_interface macaddr \
			country txpower hwmode \
			htmode bf disabled \
			channel \
			control_cidr
	json_select ..
	
	local result = $(sh ipcalc.sh $control_cidr)
	echo "$result" | source /dev/stdin
	local baseaddr="$(echo $ip | cut -d. -f1-3)"
	local lsv="$(echo $ip | cut -d. -f4)"
	control_device_ip = $baseaddr.($lsv+1)
	uci set network.quantenna_$control_interface=interface
	uci set network.quantenna_$control_interface.proto=static
	uci set network.quantenna_$control_interface.ifname=$control_interface
	uci set network.quantenna_$control_interface.ipaddr=$START
	uci set network.quantenna_$control_interface.netmask=$NETMASK
	uci set network.quantenna_$control_interface.broadcast=$BROADCAST
	uci commit

	ifconfig $control_interface down
	ifconfig $control_interface 1.1.1.1 netmask 255.255.255.248 broadcast 1.1.1.7
	ifconfig $control_interface up	

	sleep 1s

	qcsapi_sockrpc --host 1.1.1.2 set_ip br0 netmask $NETMASK >> $QT_LOG_FILE
	qcsapi_sockrpc --host 1.1.1.2 set_ip br0 broadcast $BROADCAST
	qcsapi_sockrpc --host 1.1.1.2 set_ip br0 ipaddr $control_device_ip >> $QT_LOG_FILE
	
	ifconfig $control_interface down
	ifconfig $control_interface $START netmask $NETMASK broadcast $BROADCAST
	ifconfig $control_interface up
	
	sleep 1s

	is_alive
	if [ "$ready_to_set" = "0" ]; then
		wireless_setup_failed "cannot communicate with wifi card."
		return
	fi

	wait_for_ready
	for_each_interface "sta ap adhoc" qtpcie_interface_cleanup
	for_each_interface "sta ap adhoc" setup_vif
	qt_call 0 apply_security_config
	wireless_set_data control_interface=$control_interface
	wireless_set_up
	initialized=1
}

setup_vif() {
	json_select config
	json_get_vars ssid confidx \
			wdspeerlist encryption \
			password hidden maclist \
			macfilter isolate mode
	json_select ..
	[ "$confidx" = "0" ] && {
		qtpcie_setup_hardware
	}

	[ "$mode" = "sta" ] || qt_call $confidx wifi_create_bss

	[ -n "$wdspeerlist" ] && {
			for pmac in $wdspeerlist; do
				qt_call $confidx wds_add_peer $pmac
			done
	}

	case "$encryption" in
			none)
				authproto="Basic"
				[ "$mode" = "sta" ] && authtype="NONE"
				;;
			wep)
				authproto="Basic"
				[ "$mode" = "sta" ] && authtype="NONE"
				;;
			*wpa_mixed*)
				authproto="WPAand11i"
				authtype="EAPAuthentication"
				crypto="TKIPandAESEncryption"
				;;
			*wpa2*)
				authproto="11i"
				authtype="EAPAuthentication"
				crypto="AESEncryption"
				;;
			*wpa*)
				authproto="WPA"
				authtype="EAPAuthentication"
				crypto="TKIPEncryption"
				;;
			*mixed*)
				authproto="WPAand11i"
				authtype="PSKAuthentication"
				crypto="TKIPandAESEncryption"
				;;
			*psk2*)
				authproto="11i"
				authtype="PSKAuthentication"
				crypto="AESEncryption"
				;;
			*psk*)
				authproto="WPA"
				authtype="PSKAuthentication"
				crypto="TKIPEncryption"
				;;
		esac

		case "$encryption" in
			*tkip+aes|*tkip+ccmp|*aes+tkip|*ccmp+tkip) crypto="TKIPandAESEncryption";;
			*aes|*ccmp) crypto="AESEncryption";;
			*tkip) crypto="TKIPEncryption";;
		esac

		# enforce CCMP for 11ng and 11na
		case "$hwmode:$crypto" in
			*ng:TKIP|*na:TKIP*|*ac:TKIP*) crypto="TKIPandAESEncryption";;
		esac

		case "$mode" in
			ap)
				[ -n "$authproto" ] && qt_call $confidx set_beacon "$authproto"
				[ -n "$authtype" ] && qt_call $confidx set_WPA_authentication_mode  "$authtype"
				[ -n "$crypto" ] && {
					qt_call $confidx set_WPA_encryption_modes "$crypto"
					[ -n "$password" -a "$authtype" != "EAPAuthentication" ] && {
						if [ ${#password} -eq 64 ]; then
							qt_call $confidx set_PSK "$password"
						else
							qt_call $confidx set_passphrase "$password"
						fi
					}
				}
				[ -n "$ssid" ] && {
					qt_call $confidx set_ssid "$ssid"
				}
				;;
			sta*)
				[ -n "$ssid" ] && {
					qt_call $confidx rename_SSID "$ssid"
				}
				[ -n "$authproto" -a "$authproto" != "Basic" ] && qt_call $confidx SSID_set_proto "$ssid" "$authproto"
				[ -n "$authtype" ] && qt_call $confidx SSID_set_authentication_mode "$ssid"  "$authtype"
				[ -n "$crypto" ] && {
					qt_call $confidx SSID_set_encryption_modes "$ssid" "$crypto"
					qt_call $confidx SSID_set_passphrase "$ssid" 0 "$password"
				}
				;;
		esac
		[ "$mode" = "sta" ] || qt_call $confidx set_option SSID_broadcast "$((hidden^1))"
		[ "$mode" = "sta" ] || {
			case "$macfilter" in
				allow)
					qt_call $confidx set_macaddr_filter 2
					;;
				deny)
					qt_call $confidx set_macaddr_filter 1
					;;
				*)
					# default deny policy if mac list exists
					if [ -n "$maclist" ]; then
						qt_call $confidx set_macaddr_filter 1
					else
						qt_call $confidx set_macaddr_filter 0
					fi
					;;
			esac

			# clear maclist
			qt_call $confidx clear_mac_filters
			[ -n "$maclist" ] && {
				case "$macfilter" in
					allow)
						for mac in $maclist; do
							qt_call $confidx authorize_macaddr "$mac"
						done
						;;
					*)
						for mac in $maclist; do
							qt_call $confidx deny_macaddr "$mac"
						done
						;;
				esac
			}

		}
		[ "$isolate" = "1" ] && {
			qt_call $confidx set_intra_bss_isolate 1
			qt_call $confidx set_bss_isolate 1
		}
		qt_call $confidx set_pmf 0
}

qtpcie_setup_hardware() {
	local ctry=$(echo $country | tr 'A-Z' 'a-z')
	qt_call $confidx update_persistent_param region "$ctry"
	[ "$bf" = "" ] && bf=1
	qt_call $confidx update_persistent_param bf "$bf"
	qt_call $confidx set_wifi_macaddr "$macaddr"
	local original_mode= `qcsapi_sockrpc --host $control_device_ip get_mode wifi$confidx`
	[ "$mode" = "ap" -a "$original_mode" = "Access point" ] || [ "$mode" = "sta" -a "$original_mode" = "Station" ] || qt_call $confidx reload_in_mode "$mode"
	qt_call $confidx rfenable "$((disabled^1))"

	[ "$hwmode" = "" ] && hwmode="auto"
	[ "$htmode" = "" ] && htmode="auto"

	local bw=
	local vht=
	case "$hwmode,$htmode" in
		*na,HT20)
			bw=20
			vht=0
			;;
		*na,HT40*)
			bw=40
			vht=0
			;;
		*ac,HT20)
			bw=20
			vht=1
			;;
		*ac,HT40*)
			bw=40
			vht=1
			;;
		*ac,HT80)
			bw=80
			vht=1
			;;
		*ac,*)
			bw=80
			vht=1
			;;
		*)
			bw=80
			vht=1
			;;
	esac

	qt_call $confidx update_persistent_param vht $vht
	qt_call $confidx set_bw $bw
	qt_call $confidx update_persistent_param bw $bw
	qt_call $confidx set_bb_param 1

	[ "$mode" = "sta" ] && return

	[ "$channel" = "auto" ] && channel=0
	qt_call $confidx set_channel $channel
	qt_call $confidx update_persistent_param channel $channel
	qt_call $confidx enable_scs 0
	qt_call $confidx update_persistent_param scs 0
	local reg_region=`qcsapi_sockrpc --host $control_device_ip get_regulatory_region wifi$confidx`
	local reg_chnls=
	if [ "$reg_region" != "none" ]; then
		reg_chnls=`qcsapi_sockrpc --host $control_device_ip get_list_regulatory_channels $reg_region`
		reg_chnls=$(echo $reg_chnls | sed  -e 's/,/ /g')
	else
		return
	fi
	for chnl in $reg_chnls
	do
		local max_power=`qcsapi_sockrpc --host $control_device_ip get_configured_tx_power wifi$confidx $chnl $reg_region`
		local txpwr=$(($txpower<$max_power?$txpower:$max_power))
		echo "max: $max_power requested: $txpower result: $txpwr" >> $QT_LOG_FILE
		qt_call $confidx set_tx_power $chnl $txpwr
	done
}

drv_qtpcie_teardown() {
	json_select config
	json_get_var control_device_ip control_device_ip
	json_select ..
	for_each_interface "sta ap adhoc" qtpcie_interface_cleanup
	qt_call 0 rfenable 0
	qcsapi_sockrpc --host $control_device_ip set_ip br0 netmask 255.255.255.248
	qcsapi_sockrpc --host $control_device_ip set_ip br0 broadcast 1.1.1.7
	qcsapi_sockrpc --host $control_device_ip set_ip br0 ipaddr 1.1.1.2
	initialized=0
}

add_driver qtpcie
