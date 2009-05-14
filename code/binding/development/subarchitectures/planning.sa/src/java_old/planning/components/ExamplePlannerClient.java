/**
 * 
 */
package planning.components;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Properties;

import planning.autogen.Planner.ObjectDeclaration;
import planning.autogen.PlanningData.PlanningProblem;
import planning.ontology.PlanningOntology;
import planning.ontology.PlanningOntologyFactory;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;

/**
 * Demo component that calls the planner to create a plan. This is all
 * done via working memory, but it doesn't have to be. If you look at
 * the actual PlannerComponent you could extract the relevant code
 * yourself.
 * 
 * @author nah
 */
public class ExamplePlannerClient extends ManagedProcess {

    private String m_domainFile;

    /**
     * @param _id
     */
    public ExamplePlannerClient(String _id) {
        super(_id);
        setOntology(PlanningOntologyFactory.getOntology());
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _taskID) {}

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _taskID) {}

    /*
     * (non-Javadoc)
     * 
     * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
     */
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

    /**
     * This will return the goal string to be planned for. This string
     * is in MAPL format.
     * 
     * @return
     */
    private String getGoalString() {
        // Logical form for 'put the red movables to the right of the
        // blue movable':

        // Don't know if all the line breaks are important...

        // return "(forall (?o0 - movable) (imply\n"
        // + " (and (initially (colour ?o0 red)))\n"
        // + " (exists (?p0 - waypoint)\n"
        // + "(exists (?o1 - movable ?p1 - waypoint)\n"
        // + " (and (initially (colour ?o1 blue)) (pos ?o0 ?p0)
        // (initially (pos ?o1 ?p1))\n"
        // + "(right_of ?p0 ?p1)\n" + ")))))";

//        return "(forall " +
//        		"(?o0 - movable) " +
//        		"(imply " +
//        		"       (and (initially (colour ?o0 red)))\n" 
//                   + "  (exists " +
//            		    "    (?p0 - waypoint)\n"
//            + "              (exists " +
//            		"                (?o1 - movable ?p1 - waypoint)\n"
//                            + "      (and " +
//                            		    "(initially (colour ?o1 blue)) " +
//                            		    "(pos ?o0 ?p0) " +
//                            		    "(initially (pos ?o1 ?p1))\n"
//                              +     "(right_of ?p0 ?p1)\n" + "))" +
//                       ")" +
//            	  ")" +
//            	")";

//  this works to      
        return "(and (colour movable1 red)(pos movable1 wp2))";

    }

    /**
     * Get a description of all the objects that are involved in the
     * planning problem.
     * 
     * @return
     */
    private ArrayList<ObjectDeclaration> getObjectList() {
        ArrayList<ObjectDeclaration> objectList =
                new ArrayList<ObjectDeclaration>();
        // these appear to be automatically included
        // objectList.add(new ObjectDeclaration("chips", "robot"));
        // objectList.add(new ObjectDeclaration("chips", "self_agent"));
        // objectList.add(new ObjectDeclaration("michael", "human"));

        objectList.add(new ObjectDeclaration("movable1", "movable"));
        objectList.add(new ObjectDeclaration("movable2", "movable"));
        objectList.add(new ObjectDeclaration("movable3", "movable"));
        objectList.add(new ObjectDeclaration("wp1", "waypoint"));
        objectList.add(new ObjectDeclaration("wp2", "waypoint"));
        objectList.add(new ObjectDeclaration("wp3", "waypoint"));
        objectList.add(new ObjectDeclaration("wp4", "waypoint"));
        objectList.add(new ObjectDeclaration("wp5", "waypoint"));
        objectList.add(new ObjectDeclaration("wp6", "waypoint"));
        return objectList;
    }

    /**
     * Get a list of all the facts that are in the current state of the
     * planning problem. The representation is pretty ugly and probably
     * needs updating by Michael.
     * 
     * @return
     */
    private ArrayList<String> getFactList() {
        ArrayList<String> factList = new ArrayList<String>();

        factList.add("near wp1 wp2");
        factList.add("near wp2 wp1");
        factList.add("near wp2 wp3");
        factList.add("near wp3 wp2");
        factList.add("near wp4 wp5");
        factList.add("near wp5 wp4");
        factList.add("near wp5 wp6");
        factList.add("near wp6 wp5");

        factList.add("left_of wp1 wp2");
        factList.add("left_of wp2 wp3");
        factList.add("left_of wp1 wp3");
        factList.add("left_of wp4 wp5");
        factList.add("left_of wp5 wp6");
        factList.add("left_of wp4 wp6");

        factList.add("right_of wp2 wp1");
        factList.add("right_of wp3 wp2");
        factList.add("right_of wp3 wp1");
        factList.add("right_of wp5 wp4");
        factList.add("right_of wp6 wp4");
        factList.add("right_of wp6 wp5");

        factList.add("colour movable1 red");
        factList.add("colour movable2 red");
        factList.add("colour movable3 blue");

        factList.add("pos movable1 wp2");
        factList.add("pos movable2 wp3");
        factList.add("pos movable3 wp4");

        return factList;

    }

    private PlanningProblem getPlanningProblem() {

        ArrayList<ObjectDeclaration> objects = getObjectList();
        ObjectDeclaration[] objectList =
                (ObjectDeclaration[]) objects
                    .toArray(new ObjectDeclaration[objects.size()]);

        ArrayList<String> facts = getFactList();
        String[] factList =
                (String[]) facts.toArray(new String[facts.size()]);

        return new PlanningProblem(getGoalString(), objectList,
            factList, m_domainFile);
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.PrivilegedManagedProcess#runComponent()
     */
    @Override
    protected void runComponent() {
        // create the planning problem

        PlanningProblem problem = getPlanningProblem();

        try {
            addToWorkingMemory(newDataID(),
                PlanningOntology.PLANNING_PROBLEM_TYPE, problem);
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
            System.exit(1);
        }

    }

}
