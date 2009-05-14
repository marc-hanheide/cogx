/**
 * 
 */
package spatial.taskmanagement;

import spatial.ontology.SpatialOntology;
import spatial.util.SpatialGoals;
import spatial.util.SpatialSubarchitectureException;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.architecture.subarchitecture.SubarchitectureTaskManager;
import cast.cdl.*;

/**
 * Prototype task manager for the spatial subarchitecture. Allows
 * proxity relationships to be calculated all the time, requires
 * something to happen (wtf?) to allow anything else.
 * 
 * @author nah
 */
public class SpatialSubarchitectureTaskManager
        extends
            SubarchitectureTaskManager {

    // HACK very ugly but fast to write... later use GJ's sync code
    private boolean m_bProximalProposed;
    private boolean m_bFrontProposed;
    private boolean m_bBackProposed;
    private boolean m_bLeftProposed;
    private boolean m_bRightProposed;

    private boolean m_bProximalWritten;
    private boolean m_bFrontWritten;
    private boolean m_bBackWritten;
    private boolean m_bLeftWritten;
    private boolean m_bRightWritten;

    private boolean m_bProximalReplied;
    private boolean m_bFrontReplied;
    private boolean m_bBackReplied;
    private boolean m_bLeftReplied;
    private boolean m_bRightReplied;

    private InformationProcessingTask m_proximalTask;
    private InformationProcessingTask m_rightTask;
    private InformationProcessingTask m_leftTask;
    private InformationProcessingTask m_backTask;
    private InformationProcessingTask m_frontTask;

    private String m_proximalID;
    private String m_frontID;
    private String m_backID;
    private String m_leftID;
    private String m_rightID;

    private void resetState() {
        m_bProximalProposed = false;
        m_bFrontProposed = false;
        m_bBackProposed = false;
        m_bLeftProposed = false;
        m_bRightProposed = false;
        m_bProximalWritten = false;
        m_bFrontWritten = false;
        m_bBackWritten = false;
        m_bLeftWritten = false;
        m_bRightWritten = false;
        m_bProximalReplied = false;
        m_bFrontReplied = false;
        m_bBackReplied = false;
        m_bLeftReplied = false;
        m_bRightReplied = false;
    }

    /**
     * @param _id
     */
    public SpatialSubarchitectureTaskManager(String _id) {
        super(_id);

    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.SubarchitectureTaskManager#taskCompleted(java.lang.String,
     *      cast.corba.autogen.CAST.TaskResult)
     */
    @Override
    protected void taskCompleted(String _src, TaskResult _data) {
        // log("complete: " + _src + " -> " + _data.m_id + " -> "
        // + _data.m_outcome.value());

        String taskID = _data.m_id;
        if (taskID.equals(m_proximalID)) {
            m_bProximalWritten = true;
        }
        else if (taskID.equals(m_frontID)) {
            m_bFrontWritten = true;
        }
        else if (taskID.equals(m_backID)) {
            m_bBackWritten = true;
        }
        else if (taskID.equals(m_leftID)) {
            m_bLeftWritten = true;
        }
        else if (taskID.equals(m_rightID)) {
            m_bRightWritten = true;
        }

    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.SubarchitectureTaskManager#taskRegistered(java.lang.String,
     *      cast.corba.autogen.CAST.TaskDescription[])
     */
    @Override
    protected void taskRegistered(String _src, TaskDescription[] _desc) {

    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.SubarchitectureTaskManager#taskProposed(java.lang.String,
     *      cast.corba.autogen.CAST.InformationProcessingTask)
     */
    @Override
    protected void taskProposed(String _src,
                                InformationProcessingTask _data) {
        // log("proposed: " + _data.m_taskName);

        // always allow processing
        if (isProcessingTask(_data)) {
            sendTaskDecision(_data, TaskManagementDecision.GOAL_ADOPTED);
        }
        else if (isWritingTask(_data)) {
            queueWritingTask(_data);
        }
        else {
            sendTaskDecision(_data, TaskManagementDecision.GOAL_ADOPTED);
        }

    }

    /**
     * @param _data
     */
    private void queueWritingTask(InformationProcessingTask _data) {
        String taskName = _data.m_taskName;
        if (taskName.equals(SpatialGoals.WRITE_PROXIMAL)) {
            m_bProximalProposed = true;
            m_proximalTask = _data;
            m_proximalID = _data.m_id;
        }
        else if (taskName.equals(SpatialGoals.WRITE_FRONT_PROJ)) {
            m_bFrontProposed = true;
            m_frontTask = _data;
            m_frontID = _data.m_id;
        }
        else if (taskName.equals(SpatialGoals.WRITE_BACK_PROJ)) {
            m_bBackProposed = true;
            m_backTask = _data;
            m_backID = _data.m_id;
        }
        else if (taskName.equals(SpatialGoals.WRITE_LEFT_PROJ)) {
            m_bLeftProposed = true;
            m_leftTask = _data;
            m_leftID = _data.m_id;
        }
        else if (taskName.equals(SpatialGoals.WRITE_RIGHT_PROJ)) {
            m_bRightProposed = true;
            m_rightTask = _data;
            m_rightID = _data.m_id;
        }
    }

    /**
     * @param _data
     * @return
     */
    private boolean isProcessingTask(InformationProcessingTask _data) {
        return _data.m_taskName.equals(SpatialGoals.PROCESS_PROXIMAL)
            || _data.m_taskName.equals(SpatialGoals.PROCESS_LEFT_PROJ)
            || _data.m_taskName.equals(SpatialGoals.PROCESS_RIGHT_PROJ)
            || _data.m_taskName.equals(SpatialGoals.PROCESS_FRONT_PROJ)
            || _data.m_taskName.equals(SpatialGoals.PROCESS_BACK_PROJ);
    }

    private boolean isWritingTask(InformationProcessingTask _data) {
        return _data.m_taskName.equals(SpatialGoals.WRITE_PROXIMAL)
            || _data.m_taskName.equals(SpatialGoals.WRITE_LEFT_PROJ)
            || _data.m_taskName.equals(SpatialGoals.WRITE_RIGHT_PROJ)
            || _data.m_taskName.equals(SpatialGoals.WRITE_FRONT_PROJ)
            || _data.m_taskName.equals(SpatialGoals.WRITE_BACK_PROJ);
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.SubarchitectureTaskManager#taskRetracted(java.lang.String,
     *      java.lang.String)
     */
    @Override
    protected void taskRetracted(String _src, String _taskID) {

    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.core.components.CASTComponent#runComponent()
     */
    @Override
    protected void runComponent() {
        InformationProcessingTask task;
        while (m_status == ProcessStatus.RUN) {

            try {
                
                waitForChanges();
                task = nextTask();
                if (task != null) {
                    sendTaskDecision(task,
                        TaskManagementDecision.GOAL_ADOPTED);
                }
            }
            catch (SpatialSubarchitectureException e) {
                e.printStackTrace();
            }

        }
    }

    /**
     * This defines the order in which things are done!!!
     * 
     * @return
     * @throws SpatialSubarchitectureException
     */
    private InformationProcessingTask nextTask()
            throws SpatialSubarchitectureException {
        InformationProcessingTask out = null;

        // if we're in the middle of processing
        if ((m_bProximalReplied && !m_bProximalWritten)
            || (m_bFrontReplied && !m_bFrontWritten)
            || (m_bBackReplied && !m_bBackWritten)
            || (m_bLeftReplied && !m_bLeftWritten)
            || (m_bRightReplied && !m_bRightWritten)) {
            return null;
        }

        if (m_bProximalProposed && !m_bProximalWritten) {
            if (!m_bProximalReplied) {
                out = m_proximalTask;
                m_proximalTask = null;
                return out;
            }
            else {
                return null;
            }
        }
        else if (m_bLeftProposed && !m_bLeftWritten) {
            if (!m_bLeftReplied) {
                out = m_leftTask;
                m_leftTask = null;
                return out;
            }
            else {
                return null;
            }
        }

        else if (m_bRightProposed && !m_bRightWritten) {
            if (!m_bRightReplied) {
                out = m_rightTask;
                m_rightTask = null;
                return out;
            }
            else {
                return null;
            }
        }
        else if (m_bFrontProposed && !m_bFrontWritten) {
            if (!m_bFrontReplied) {
                out = m_frontTask;
                m_frontTask = null;
                return out;
            }
            else {
                return null;
            }
        }
        else if (m_bBackProposed && !m_bBackWritten) {
            if (!m_bBackReplied) {
                out = m_backTask;
                m_backTask = null;
                return out;
            }
            else {
                return null;
            }
        }

        return null;
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
            addChangeFilter(SpatialOntology.SPATIAL_SCENE_TYPE,
                WorkingMemoryOperation.ADD, true,
                new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        // log("reset sync state");
                        rejectRemainingTasks();
                        resetState();

                    }
                });
            addChangeFilter(SpatialOntology.SPATIAL_SCENE_TYPE,
                WorkingMemoryOperation.OVERWRITE, true,
                new WorkingMemoryChangeReceiver() {

                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
			//don't do anything with it, just use this to
			//advance task processing
                    }
                });
        }
        catch (SubarchitectureProcessException e) {
            e.printStackTrace();
        }

    }

    /**
     * 
     */
    private void rejectRemainingTasks() {
        if (m_proximalTask != null) {
            sendTaskDecision(m_proximalTask,
                TaskManagementDecision.GOAL_REJECTED);
        }
        if (m_leftTask != null) {
            sendTaskDecision(m_leftTask,
                TaskManagementDecision.GOAL_REJECTED);
        }
        if (m_rightTask != null) {
            sendTaskDecision(m_rightTask,
                TaskManagementDecision.GOAL_REJECTED);
        }
        if (m_frontTask != null) {
            sendTaskDecision(m_frontTask,
                TaskManagementDecision.GOAL_REJECTED);
        }
        if (m_backTask != null) {
            sendTaskDecision(m_backTask,
                TaskManagementDecision.GOAL_REJECTED);
        }

    }

}
