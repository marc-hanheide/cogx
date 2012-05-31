#!/usr/bin/python
# vim: set fileencoding=utf-8 sw=4 sts=4 ts=8 et :vim
#
# Run all the processes and the client from the command line.
# The options from castagent are used with some additional options
# that are not supported by castagent.
#
import castagent as cca
from core import procman

def addOptions(parser):
    # * CAST file
    # * Logging levels
    # * Logger output (filename)
    return

def okToStart: return True

class CRunOrder:
    def __init__(name, waitFor=None, sleepAfter=0, fnPreStart=okToStart):
        self.name = name
        self.waitFor = waitFor
        self.sleepAfter = sleepAfter
        self.fnPreStart = fnPreStart
        self.startedAt = None
        self.process = None

def prepareServerLog4j():
    return True

def prepareClientLog4j():
    return True

# Remove nonexisting processes from the group
def filterGroup(group, processManager):
    pm = processManager
    for task in group:
        task.process = pm.getProcess(task.name)
    group = [ task for task in group if task.process != None ]
    return group

def tryStartGroup(group, processManager):
    if len(group) < 1:
        return True
    pm = processManager
    processNames = {}
    for task in group:
        processNames[task.name] = task


def main():
    groupA = []
    groupA.append(CRunOrder(procman.LOG4J_PROCESS, sleepAfter=2, fnPreStart=prepareServerLog4j))
    groupA.append(CRunOrder("Display"))
    groupA.append(CRunOrder("Abducer"))
    groupA.append(CRunOrder("Gazebo"))
    groupA.append(CRunOrder("Mary.tts"))
    groupA.append(CRunOrder("Golem"))
    groupA.append(CRunOrder("Player", waitFor=[("Gazebo", 10)] ))
    groupA.append(CRunOrder("Peekabot", waitFor=[("Player", 5)] ))
    groupB = []
    groupB.append(CRunOrder("cast-java"))
    groupB.append(CRunOrder("cast-cpp"))
    groupB.append(CRunOrder("cast-python"))
    groupC = []
    groupC.append(CRunOrder("cast-client", fnPreStart=prepareClientLog4j))

    parser = cca.createOptionParser()
    addOptions(parser)
    opts, args = cca.parseOptions(parser)
    agent = CConsoleAgent(opts)

    groups = [ filterGroup(g, agent.manager) for g in [groupA, groupB, groupC] ]


if __name__ == "__main__": main()

