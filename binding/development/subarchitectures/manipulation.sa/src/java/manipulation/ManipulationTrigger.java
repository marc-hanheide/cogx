/**
 * 
 */
package manipulation;

import javax.swing.JOptionPane;

import manipulation.autogen.Manipulation.PickAndPlaceCmd;
import manipulation.ontology.ManipulationOntology;
import manipulation.ontology.ManipulationOntologyFactory;

import org.cognitivesystems.common.autogen.Math.Pose3D;
import org.cognitivesystems.common.autogen.Math.Vector3D;

import visionarch.global.VisionOntology;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.*;
import cast.core.CASTUtils;

/**
 * Java replacement for the UOL Matlab dialogue system. Basically a
 * placeholder to allow pre-integration exploration.
 * 
 * @author nah
 */
public class ManipulationTrigger extends ManagedProcess {

    private WorkingMemoryAddress m_lastSOAddress;

    /**
     * @param _id
     */
    public ManipulationTrigger(String _id) {
        super(_id);

        setOntology(ManipulationOntologyFactory.getOntology());

        // addChangeFilter(ManipulationOntology.ROI_TYPE,WorkingMemoryOperation.ADD);
        // addChangeFilter(VisionOntology.SCENE_OBJECT_TYPE,
        // WorkingMemoryOperation.ADD);

        m_lastSOAddress = null;

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

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#workingMemoryChanged(cast.corba.autogen.CAST.WorkingMemoryChange[])
     */
    @Override
    public void start() {
        super.start();

        try {
            addChangeFilter(VisionOntology.SCENE_OBJECT_TYPE,
                WorkingMemoryOperation.ADD, false,
                new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        m_lastSOAddress = _wmc.m_address;
                        println("last scene object at: "
                            + CASTUtils.toString(m_lastSOAddress));

                    }
                });
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        }

    }

    private String getInput() {
        String text = JOptionPane.showInputDialog(null,
            "Please type the speech input string",
            "Speech input string", JOptionPane.QUESTION_MESSAGE);
        return text;
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.core.components.CASTComponent#runComponent()
     */
    @Override
    protected void runComponent() {
//        try {
//            String input;
//            while (m_status == ProcessStatus.RUN) {
//
//                if (m_lastSOAddress != null) {
//                    // wait for input from user
//                    input = getInput();
//                    if (input != null) {
//                        println("input: " + input);
//
////                        PickAndPlaceCmd papc = new PickAndPlaceCmd(
////                            m_lastSOAddress.m_id, new Pose3D(
////                                new Vector3D(0.30f, -0.1f, 0f),
////                                new Vector3D(0, 0, 0)));
//
////                        addToWorkingMemory(
////                            newDataID(),
////                            ManipulationOntology.PICK_AND_PLACE_COMMAND_TYPE,
////                            papc);
//
//                    }
//                }
//
//                sleepProcess(200);
//
//            }
//        }
//        catch (SubarchitectureProcessException e) {
//            e.printStackTrace();
//        }
//    }
    }

}
