#!/usr/bin/env python

import sys

from omniORB import CORBA, PortableServer
# Import the stubs for the Naming service
import CosNaming
# Import the stubs and skeletons for the Planner module
import Planner, Planner__POA

import server_implementation_cosy as server_implementation

orb = None

class PlannerServer_i(Planner__POA.PlannerServer,server_implementation.Planner):
    """ inherit from both the CORBA interface and the actual implementation that we're using"""

    def kill_me(self):
        global orb
        print "Planning server will be shut down now.  Thanks for planning with us!"
        orb.shutdown(0)

if __name__ == "__main__":
    test_me = "test" in sys.argv
    #check command line args
    if len(sys.argv) != 4 and not test_me:
        print "incorrect arguments"
        print "call with: server.py -ORBInitRef NameService=corbaname::<naminghost>:<port> <server name>"
        sys.exit()
    # Create an instance of PlannerServer_i
    ei = PlannerServer_i()
    if test_me:
        ei.self_test()
        sys.exit()
    #This call must go before the CORBA init call, as that removes some of the parameters
    serverName = sys.argv[3]
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)
    # Find the root POA
    poa = orb.resolve_initial_references("RootPOA")
    # Create an object reference, and implicitly activate the object
    eo = ei._this()
    # Obtain a reference to the root naming context
    obj         = orb.resolve_initial_references("NameService")
    rootContext = obj._narrow(CosNaming.NamingContext)
    if rootContext is None:
        print "Failed to narrow the root naming context"
        sys.exit(1)
    # Bind the PlannerServer object to the test context
    name = [CosNaming.NameComponent(serverName, "Object")]
    try:
        rootContext.bind(name, eo)
#        testContext.bind(name, eo)
        print "New %s object bound" % serverName 
    except CosNaming.NamingContext.AlreadyBound:
        rootContext.rebind(name, eo)
        print "%s binding already existed -- rebound" % serverName
    # Note that is should be sufficient to just call rebind() without
    # calling bind() first. Some Naming service implementations
    # incorrectly raise NotFound if rebind() is called for an unknown
    # name, so we use the two-stage approach above
    # Activate the POA
    poaManager = poa._get_the_POAManager()
    poaManager.activate()
    # Everything is running now, but if this thread drops out of the end
    # of the file, the process will exit. orb.run() just blocks until the
    # ORB is shut down
    orb.run()
