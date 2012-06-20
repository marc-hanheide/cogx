#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import unittest, logging
import sys, os, time, random
this_dir = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(this_dir, "../..")))

from core import castagentsrv, procman, options, messages
LOGGER = messages.CStdoutLogger()
procman.LOGGER = LOGGER
castagentsrv.LOGGER = LOGGER

# For the ice client
import Ice
import core.modice
import icemodule.castcontrol.CastAgent as CastAgent

class TestCastAgentSlave(unittest.TestCase):
    def setUp(self):
        LOGGER.log("-----------Setup--------------")
        self.manager = procman.CProcessManager("localhost")
        self.manager.addProcess(procman.CProcess("Test 1", "python _wait.py A"))
        self.manager.addProcess(procman.CProcess("Test 2", "python _wait.py B"))
        self.manager.addProcess(procman.CProcess("Test 3", "python _wait.py C"))
        self.options = options.CCastOptions()
        self.address = "tcp -p 7832"
        self.agent = None

    def tearDown(self):
        self.manager.stopAll()
        try:
            # Every test should shutdown its own servers
            if self.agent != None:
                LOGGER.warn("Agent still exists at tearDown. A test didn't call _shutdown().")
                self._shutdown(self.agent)
        except:
            pass

    def _shutdown(self, agent):
        count = 10
        agent.shutdown()
        while agent.isAlive():
            LOGGER.log("... waiting for shutdown")
            time.sleep(1.0)
            count -= 1
            if count <= 0: break
        if agent.isAlive():
            LOGGER.warn("Server didn't shut down")
        else:
            LOGGER.log("Server stopped.")

    def testStartStop(self):
        """
        Test if the agent can be started and stopped multiple times.
        This is important for CastControl, but not for the console agent.
        """
        for i in xrange(3):
            LOGGER.log("Run %d" % i)
            self.agent = castagentsrv.CCastSlave(self.manager, self.options, self.address)
            self.agent.start() # TODO: when testing, start() should rethrow errors.
            time.sleep(1.0)
            LOGGER.log("Shutdown %d" % i)
            self._shutdown(self.agent)
            self.agent = None

    def testProcessMonitor(self):
        """
        Test if the agent can run, stop and monitor processes.
        """
        # Server (slave)
        LOGGER.log("Run Monitor")
        self.agent = castagentsrv.CCastSlave(self.manager, self.options, self.address)
        self.agent.start()
        time.sleep(0.2)

        # Client (master)
        ic = Ice.initialize(sys.argv)
        base = ic.stringToProxy("CastAgent:%s" % self.address)
        remote = CastAgent.AgentPrx.checkedCast(base)
        if not remote: raise RuntimeError("Invalid proxy")
        print remote.getProcessList()
        remote.startProcess("Test 1")
        remote.startProcess("Test 3")
        print remote.getProcessList()
        time.sleep(3.0)
        remote.stopProcess("Test 1")
        time.sleep(3.0)
        print remote.getProcessList()

        # Server (slave)
        LOGGER.log("Shutdown Monitor")
        self._shutdown(self.agent)
        self.agent = None


if __name__ == "__main__":
    unittest.main()
