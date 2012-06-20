#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import os
import socket

if os.name == "posix":
    import fcntl
    import struct

    # http://code.activestate.com/recipes/439094/
    def get_interface_address(ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15])
        )[20:24])

    def get_interface_mac(ifname):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        info = fcntl.ioctl(s.fileno(), 0x8927,  struct.pack('256s', ifname[:15]))
        return ''.join(['%02x:' % ord(char) for char in info[18:24]])[:-1]

def get_ip_address(interface_list = None):
    ip = socket.gethostbyname(socket.gethostname())
    #if not ip.startswith("127."): return ip
    if interface_list == None:
        interface_list = ["eth0","eth1","eth2","wlan0","wlan1","wifi0","ath0","ath1","ppp0"]

    for ifname in interface_list:
        try:
            ip2 = get_interface_address(ifname)
            return ip2
        except IOError: pass
        except NameError: break

    return ip

