This directory contains the ICE wrapper of the Loquendo ASR system
for Linux.

NOTE: The code does not necessarily compile on all platforms,
but should always compile on Linux, provided that all dependencies
are installed and functioning properly.

Loquendo requires the MAC address to be spoofed as follows:
00:0c:29:72:cf:2a

To compile it do like this:
make ICE_HOME=/usr LASR_INSTALL_PATH=/opt/Loquendo/LASR


