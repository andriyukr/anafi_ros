#!/bin/bash
# Skycontroller interface configuration script: configure one Skycontroller interface

# All Skycontrollers have a default IP address set to 192.168.53.1. Hence by default, multiple Skycontrollers connected to a host via multiple USB ports are not addressable. This script will create a new IP address (on an independent subnet) for each USB interface. The new IP address (and it's subnet) is intended to be dedicated to this interface and all traffic on this address will be routed to the associated Skycontroller.
    
# USAGE:
#     setup:
#         $ ./config_skycontroller_ip.sh setup usb0 192.168.60.1 200
#         $ ./config_skycontroller_ip.sh setup usb1 192.168.61.1 201
#         ...
#     clean:
#         $ ./config_skycontroller_ip.sh clean usb0 192.168.60.1 200
#         $ ./config_skycontroller_ip.sh clean usb1 192.168.61.1 201
#         ...

set -e

action=$1
interface=$2
address=$3
route_tableid=$4
# shellcheck disable=SC2046,SC2116,SC2086
address_hex=0x$(printf '%02X' $(echo ${address//./ }))

# Outbound traffic iptables rules: packets addressed to the virtual SKYCONTROLLER's IP address are marked in order to be routed on the correct interface and then get NATed to match the actual SKYCONTROLLER's IP address on this interface
OUT_RULES=$(cat <<EOF 
mangle:OUTPUT      --destination      $address     -j MARK --set-mark $address_hex
nat:OUTPUT         -m mark --mark $address_hex     -j DNAT --to-destination 192.168.53.1 
nat:POSTROUTING    -m mark --mark $address_hex     -j SNAT --to-source 192.168.53.254
EOF
)

# Inbound traffic rules: equivalent to the outbound traffic rules except we are only NATing the traffic from the SKYCONTROLLER interface to the virtual SKYCONTROLLER IP address (there is no routing involved)
IN_RULES=$(cat <<EOF
mangle:PREROUTING  --in-interface     $interface   -j MARK --set-mark $address_hex
nat:PREROUTING     -m mark --mark   $address_hex   -j DNAT --to-destination $address
nat:INPUT          -m mark --mark   $address_hex   -j SNAT --to-source $address
EOF
)
RULES=$(cat <<EOF
$OUT_RULES
$IN_RULES
EOF
)

case $action in
setup)
echo "Setup $interface $address ($address_hex) $route_tableid"
  # Add the SKYCONTROLLER "virtual" IP
  sudo ip addr del "$address"/24 dev "$interface" > /dev/null 2>&1|| true
  sudo ip addr add "$address"/24 dev "$interface"

  # Don't rely on the IP the SKYCONTROLLER dhcp gave us
  sudo ip addr del 192.168.53.254/24 dev "$interface" > /dev/null 2>&1|| true
  sudo ip addr add 192.168.53.254/24 dev "$interface"
  sudo ip link set dev "$interface" down
  sudo sysctl -w "net.ipv4.conf.$interface.rp_filter=0"
  # FIXME: We shouldn't have to deactivate rp_filter for all interfaces
  sudo sysctl -w "net.ipv4.conf.all.rp_filter=0"
  while read -r rule; do
    table="${rule%:*}"
    rule="${rule#*:}"
    # shellcheck disable=SC2086
    if ! sudo iptables -t "$table" -C $rule > /dev/null 2>&1; then
      # Add the rule to the table
      sudo iptables --wait -t "$table" -A $rule
    fi
  done <<< "$RULES"
  sudo ip link set dev "$interface" up
  sudo ip route del default via 192.168.53.254 dev "$interface" table "$route_tableid" > /dev/null 2>&1|| true
  sudo ip route add default via 192.168.53.254 dev "$interface" table "$route_tableid"
  sudo ip rule del fwmark "$address_hex" table "$route_tableid" > /dev/null 2>&1|| true
  sudo ip rule add fwmark "$address_hex" table "$route_tableid"
  ;;
clean)
  echo "Cleanup $interface $address ($address_hex) $route_tableid"
  sudo ip link set dev "$interface" up
  sudo ip addr del "$address"/24 dev "$interface" || true
  sudo ip addr del 192.168.53.254/24 dev "$interface" || true
  while read -r rule; do
    table="${rule%:*}"
    rule="${rule#*:}"
    # shellcheck disable=SC2086
    while sudo iptables -t "$table" -D $rule > /dev/null 2>&1; do :;done
  done <<< "$RULES"
  # sudo sysctl -w "net.ipv4.conf.$interface.rp_filter=1"
  sudo ip route del default via 192.168.53.254 dev "$interface" table "$route_tableid" > /dev/null 2>&1 || true
  sudo ip rule del fwmark "$address_hex" table "$route_tableid" > /dev/null 2>&1 || true
  ;;
esac
