#!/usr/bin/python
# vim: set fileencoding=utf-8 sw=4 sts=4 ts=8 et :vim
#
# Run all the processes and the client from the command line.
# The options from castagent are used with some additional options
# that are not supported by castagent.
#
import castagent as cca
from core import procman
import time

def addOptions(parser):
    # * CAST file
    # * Logging levels
    # * Logger output (filename)
    return

def okToStart(): return True

# waitFor: list of (taskname, seconds)
class CRunOrder:
    def __init__(self, name, waitFor=None, sleepAfter=0, fnPreStart=okToStart):
        self.name = name
        self._waitForNames = [] if waitFor == None else waitFor
        self.waitFor = None
        self.sleepAfter = sleepAfter
        self.fnPreStart = fnPreStart
        self.startedAt = None
        self.process = None

    # Resolve the list of tasks that this task has to wait for; convert names
    # to tasks. Self should belong to the group.
    def resolveWaitFor(self, group):
        tasks = {}
        for task in group:
            tasks[task.name] = task
        #print self.name, self._waitForNames
        self.waitFor = [ (tasks[pn], s) for (pn, s) in self._waitForNames if pn in tasks ]

    def __repr__(self):
        if self.waitFor == None: ref = ":*Unresolved*"
        else:
            if len(self.waitFor) == 0: ref = ""
            else:
                ref = ":" + (",".join( [ "%s+%ds" % (t[0].name, t[1]) for t in self.waitFor ] ))
        s = "CRunOrder(%s%s)" % (self.name, ref)
        return s

    # Return if the task was started and its running time is at least runTimeS seconds.
    def wasStartedBefore(self, runTimeS):
        if self.startedAt == None: return False
        return self.startedAt + runTimeS <= time.time()

    def getTimeToWait(self, runTimeS):
        if self.startedAt == None:
            return runTimeS
        ttw = (self.startedAt + runTimeS) - time.time()
        if ttw < 0: ttw = 0
        return ttw

    def startProcess(self):
        self.process.start()
        time.sleep(0.2)
        self.startedAt = time.time()

def prepareServerLog4j():
    return True

def prepareClientLog4j():
    return True

# Remove nonexisting processes from the group.
# Convert (name, time) to (task, time) in waitFor.
def prepareGroup(group, processManager):
    pm = processManager
    for task in group:
        task.process = pm.getProcess(task.name)
    group = [ task for task in group if task.process != None ]
    for task in group:
        task.resolveWaitFor(group)
    return group

# Try to start the processes in the order defined by waitFor.
def tryStartGroup(group, processManager):
    if len(group) < 1:
        return True
    pm = processManager
    processNames = {}
    for task in group:
        processNames[task.name] = task

    waiting = [t for t in group]
    while len(waiting):
        alltasks = waiting
        waiting = []
        for task in alltasks:
            hasToWait = False
            for ttwf in task.waitFor: # ttwf = (CRunOrder, seconds)
                if not ttwf[0].wasStartedBefore(ttwf[1]):
                    #print task.name, "will wait for", ttwf[0].name, ttwf[1]
                    hasToWait = True
                    waiting.append(task)
                    break
            if hasToWait: continue
            print time.time(), "Starting", task.name

            task.fnPreStart()
            task.startProcess()
            if task.sleepAfter > 0:
                time.sleep(task.sleepAfter)

        if len(waiting):
            print "Waiting:", waiting
            minTtw = 5
            for task in waiting:
                for ttwf in task.waitFor: # ttwf = (CRunOrder, seconds)
                    wf = ttwf[0].getTimeToWait(ttwf[1])
                    if wf > 0: wf += 0.1
                    if wf < minTtw: minTtw = wf
            if minTtw > 0:
                print "Sleeping", minTtw, "s"
                time.sleep(minTtw)

def getRunningTasks(group):
    return [ t for t in group if t.process.isRunning() ]


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
    agent = cca.CConsoleAgent(opts)

    try:
        groups = [ prepareGroup(g, agent.manager) for g in [groupA, groupB, groupC] ]
        tryStartGroup(groups[0], agent.manager)
        print "Running:", getRunningTasks(groups[0])
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print "\nKeyboard Interrupt\n"
    #except Exception as e:
    #    print e
    agent.stopServing()


if __name__ == "__main__": main()

