/**
 * 
 */
package manipulation;

import manipulation.autogen.Manipulation.PickAndPlaceCmd;
import manipulation.ontology.ManipulationOntology;
import manipulation.ontology.ManipulationOntologyFactory;
import planning.autogen.PlanningData.*;
import planning.ontology.PlanningOntology;
import planning.ontology.PlanningOntologyFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.*;
import cast.core.ontologies.CASTCompositeOntology;

/**
 * Dummy component to consume PickAndPlaceCmd structs and send success and
 * failure values back.
 * 
 * @author nah
 */
public class PretendManipulator extends ManagedProcess {

    /**
         * @param _id
         */
    public PretendManipulator(String _id) {
	super(_id);
	m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
	CASTCompositeOntology ontology = new CASTCompositeOntology();
	ontology.addOntology(ManipulationOntologyFactory.getOntology());
	ontology.addOntology(PlanningOntologyFactory.getOntology());
	setOntology(ontology);
    }

    /*
         * (non-Javadoc)
         * 
         * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
         */
    @Override
    protected void taskAdopted(String _goalID) {
    }

    /*
         * (non-Javadoc)
         * 
         * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
         */
    @Override
    protected void taskRejected(String _goalID) {
    }

    @Override
    public void start() {

	super.start();

	try {
	    addChangeFilter(PlanningOntology.ACTION_TYPE,
		    WorkingMemoryOperation.ADD, true,
		    new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(
				WorkingMemoryChange _wmc) {
			    try {
				log("ooooo oooooooooo i'm the great pretender.....");

				Action action = (Action) getWorkingMemoryEntry(
					_wmc.m_address).getData();
				assert (action.m_action.m_type
					.equals(ManipulationOntology.PICK_AND_PLACE_COMMAND_TYPE));
				PickAndPlaceCmd pnp = (PickAndPlaceCmd) getWorkingMemoryEntry(
					action.m_action.m_address).getData();


				for(int i = 30; i >= 0; i--) {
				    log(i + "...");
				    sleepProcess(1000);
				}				

				action.m_status = PlanningStatus.COMPLETE;
				action.m_succeeded = TriBool.triTrue;
				log("ok GO: id = " + _wmc.m_address.m_id
					+ " : sa = "
					+ _wmc.m_address.m_subarchitecture);
				
				overwriteWorkingMemory(_wmc.m_address.m_id,
					PlanningOntology.ACTION_TYPE, action);
				
				
			    } catch (SubarchitectureProcessException e) {
				e.printStackTrace();
			    }

			}
		    });
	} catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	}

    }

    /*
         * (non-Javadoc)
         * 
         * @see cast.architecture.subarchitecture.ManagedProcess#runComponent()
         */
    @Override
    protected void runComponent() {
	ActionRegistration reg = new ActionRegistration(getProcessIdentifier(),
		getSubarchitectureID(), "move");
	try {
	    log("registering action");
	    addToWorkingMemory(newDataID(),
		    PlanningOntology.ACTION_REGISTRATION_TYPE, reg);
	} catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	    System.exit(1);
	}

    }

}
