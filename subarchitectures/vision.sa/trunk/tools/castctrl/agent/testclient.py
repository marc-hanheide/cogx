#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import sys, time, traceback, Ice
import modice
import icemodule.castcontrol.CastAgent as CastAgent

status = 0
ic = None
try:
    ic = Ice.initialize(sys.argv)
    base = ic.stringToProxy("CastAgent:tcp -p 10000:udp -p 10000")
    agent = CastAgent.AgentPrx.checkedCast(base)
    if not agent:
        raise RuntimeError("Invalid proxy")

    print agent.getProcessList()
    print agent.startProcess("server-java")
    print agent.startProcess("server-cpp")
    time.sleep(2)
    for msg in agent.readMessages("server-java", 0.0): print msg
    for msg in agent.readMessages("server-cpp", 0.0): print msg
    for msg in agent.readMessages("LOGGER", 0.0): print msg
    print agent.stopProcess("server-java")
    print agent.stopProcess("server-cpp")

except:
    traceback.print_exc()
    status = 1

if ic:
    # Clean up
    try:
        ic.destroy()
    except:
        traceback.print_exc()
        status = 1

sys.exit(status)

