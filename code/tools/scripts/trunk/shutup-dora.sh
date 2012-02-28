#!/bin/bash
# author: Nikolaus Demmel
# date: 17.02.2012
# connects to dora via a serial device (/dev/robotbase) and disconnects straight away.
# this is useful when the base has an open connection and beeps annoyingly.
`dirname $0`/justConnectStatic -rp /dev/robotbase 
