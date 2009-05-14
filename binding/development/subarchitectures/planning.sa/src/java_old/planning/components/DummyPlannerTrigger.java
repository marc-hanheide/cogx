/**
 * 
 */
package planning.components;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.Properties;

import planning.autogen.Planner.ObjectDeclaration;
import planning.autogen.PlanningData.PlanningState;
import planning.autogen.PlanningData.PlanningStateRequest;
import planning.ontology.PlanningOntology;
import planning.ontology.PlanningOntologyFactory;
import planning.util.PlanningUtils;
import planning.util.TemporaryPlanningState;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * @author nah
 */
public class DummyPlannerTrigger extends PrivilegedManagedProcess {

    private String m_domainFile;

    /**
     * @param _id
     */
    public DummyPlannerTrigger(String _id) {
        super(_id);
        setOntology(PlanningOntologyFactory.getOntology());
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
     */
    @Override
    public void start() {
        super.start();

        // listen for an overwrite
        try {
            addChangeFilter(
                PlanningOntology.PLANNING_STATE_REQUEST_TYPE,
                WorkingMemoryOperation.OVERWRITE, false,
                new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        try {
                            // get the data
                            PlanningStateRequest psr =
                                    (PlanningStateRequest) getWorkingMemoryEntry(
                                        _wmc.m_address).getData();
                            // and delete request
                            deleteFromWorkingMemory(
                                _wmc.m_address.m_id,
                                _wmc.m_address.m_subarchitecture);

                            // now do something
                            trigger(psr.m_state);
                        }
                        catch (SubarchitectureProcessException e) {
                            e.printStackTrace();
                            System.exit(1);
                        }
                    }
                });
        }
        catch (SubarchitectureProcessException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
            System.exit(1);
        }

    }

    int m_dummyCount = 0;
    
    private TemporaryPlanningState extendState(PlanningState _state,
                                               String _initType,
                                               String _rel,
                                               String _type,
                                               int _count,
                                               boolean _extend) {
        TemporaryPlanningState tps = new TemporaryPlanningState();
        String prefix = "dummy_" + _type + "_";
       
        String dummyVar;
        // for each object declaration
        for (ObjectDeclaration decl : _state.m_objects) {

            // if the decl type matches the extension type
            if (decl.type.equals(_initType)) {
                for (int i = 0; i < _count; i++) {
                    dummyVar = prefix + (m_dummyCount++);
                    tps.m_objectList.add(new ObjectDeclaration(
                        dummyVar, _type));
                    tps.m_factList.add(_rel + " " + dummyVar
                        + " " + decl.name);
                }
            }
        }

        if (_extend) {
            tps.toPlanningState(_state);
        }
        return tps;
    }

    /**
     * @param _state
     */
    protected void trigger(PlanningState _state) {

        // first off, extend with some extra waypoints, just to make
        // things
        // work without language for the time being.
//        println(PlanningUtils.toString(_state));

        TemporaryPlanningState es1 =
                extendState(_state, "waypoint", "left_of", "waypoint",
                    2, false);
        TemporaryPlanningState es2 =
                extendState(_state, "waypoint", "right_of", "waypoint",
                    2, false);

        es1.toPlanningState(_state);
        es2.toPlanningState(_state);

        println(PlanningUtils.toString(_state));

        //
//        String goalString =
//                "(forall (?o0 - movable) (imply\n"
//                    + "  (and (initially (colour ?o0 red)))\n"
//                    + "  (exists (?p0 - waypoint)\n"
//                    + "(exists (?o1 - movable ?p1 - waypoint)\n"
//                    + "  (and (initially (colour ?o1 blue)) (pos ?o0 ?p0) (initially (pos ?o1 ?p1))\n"
//                    + "(right_of ?p0 ?p1)\n" + ")))))";
//
//        PlanningProblem pp =
//                new PlanningProblem(goalString, _state.m_objects,
//                    _state.m_facts, m_domainFile);
//
//        try {
//            addToWorkingMemory(newDataID(),
//                PlanningOntology.PLANNING_PROBLEM_TYPE, pp);
//        }
//        catch (SubarchitectureProcessException e) {
//            e.printStackTrace();
//            System.exit(1);
//        }
    }

    @Override
    public void configure(Properties _config) {
        super.configure(_config);

        // must set the path to the example domain file
        String domain = _config.getProperty("--domain");
        // if we haven't been giving anything, take a guess
        if (domain == null) {
            domain =
                    "./tools/planning-components/config/domains/kitty_domain.mapl";
        }

        try {
            // do some sanity checking
            File domainFile = new File(domain);
            if (!domainFile.exists()) {
                throw new FileNotFoundException(
                    "Domain file for kitty domain does not exist: "
                        + domain);
            }
            m_domainFile =
                    domainFile.getAbsoluteFile().getCanonicalPath();
            log("set domain file to: " + m_domainFile);
        }
        catch (IOException e) {
            e.printStackTrace();
            System.exit(1);
        }

    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _taskID) {

    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _taskID) {

    }

    @Override
    protected void runComponent() {

        // wait for a while
        sleepProcess(17000);
        log(" ");
        log("triggering generating state");
        try {
            PlanningStateRequest psr =
                    new PlanningStateRequest(new PlanningState(
                        new ObjectDeclaration[0], new String[0]), new WorkingMemoryAddress("",""));

            addToWorkingMemory(newDataID(),
                PlanningOntology.PLANNING_STATE_REQUEST_TYPE, psr);

        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }
}
