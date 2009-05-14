#!/usr/bin/env python

import sys


if __name__ == "__main__":
    # Import the CORBA module
    from omniORB import CORBA

    # Import the stubs for CosNaming
    import CosNaming

    import Planner, Planner__POA

    test = ("test" in sys.argv)
    if test:
        sys.argv.remove("test")
        
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

    # Obtain a reference to the root naming context
    obj         = orb.resolve_initial_references("NameService")
    rootContext = obj._narrow(CosNaming.NamingContext)

    if rootContext is None:
        print "Failed to narrow the root naming context"
        sys.exit(1)

    # Resolve the name "test.my_context/ExamplePlannerServer.Object"
    name = [CosNaming.NameComponent("test", "my_context"),
            CosNaming.NameComponent("ExamplePlannerServer", "Object")]

    try:
        obj = rootContext.resolve(name)

    except CosNaming.NamingContext.NotFound, ex:
        print "Name not found"
        sys.exit(1)

    # Narrow the object to an Planner::PlannerServer
    eo = obj._narrow(Planner.PlannerServer)

    if eo is None:
        print "Object reference is not an Planner::PlannerServer"
        sys.exit(1)

    if test:
        eo.self_test()
        sys.exit()

