package org.cognitivesystems.comsys.processing;

import java.util.HashMap;
import java.util.Properties;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.GRETask;

import binding.abstr.AbstractBindingReader;

import BindingFeatures.Concept;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ConsistencyException;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * The abstract class <b>AbstractGREProcess</b> defines an interface 
 * for components that implement algorithms for the
 * Generation of Referring Expressions (GRE) 
 * and also encapsulates the information flow processes.
 * 
 * @started 080811
 * @version 080814
 * @author Hendrik Zender (zender@dfki.de)
 * 
 */
public abstract class AbstractGREProcess extends AbstractBindingReader {
	
	/**
	 * Make sure to add entries for the feature to LF mapping!
	 * 
	 * @param _id
	 */
	public AbstractGREProcess(String _id) {
		super(_id);
        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
	}

	
    public void configure(Properties _config) {
    	super.configure(_config);
		// TODO Auto-generated method stub
    }


    /* (non-Javadoc)
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
     * This method registers the change filter for GRETasks
     * 
     */
    public void start() {
        super.start();
        try {
        	addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(GRETask.class, 
        			WorkingMemoryOperation.ADD),
        			new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        GRETask _myTask;
						try {
							_myTask = (GRETask) getWorkingMemoryEntry(_wmc.m_address).getData();
							GRETaskWrapper _wrappedTask = new GRETaskWrapper(_myTask, _wmc.m_address);                        
							processGRETask(_wrappedTask);
						} catch (SubarchitectureProcessException e) {
							e.printStackTrace();
							log("Error! Could not read GRETask struct. Aborting!");
							throw new RuntimeException(e);
						}                        
                    }
                });
        }
        catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			throw new RuntimeException("Error! Could not register change filter. Aborting!");
        }

    }

    
    /**
     * Instantiate this method!
     * This method gets a GRETaskWrapper whenever comsys puts a new GRETask on
     * comsys WM. Inside this method you should then generate the RefEx,
     * or at least, initiate its generation.
     * 
     * Make sure that after a successfully generated RefEx the resulting LF
     * is written back to the GRETaskWrapper before calling returnRefExLF.
     * 
     * @param _myWrappedTask
     */
    public abstract void processGRETask(GRETaskWrapper _myWrappedTask);
    
    /**
     * Make sure to call this after the GRE has been successfully generated.
     * This method will overwrite the original GRETask with the generated
     * logical form string.
     * 
     * @param _myWrappedTask
     */
    public void returnRefExLF(GRETaskWrapper _myWrappedTask) {
    	try {
			overwriteWorkingMemory(_myWrappedTask.m_originalGRETaskAdress, _myWrappedTask.getGRETaskStruct());
		} catch (DoesNotExistOnWMException e) {
			e.printStackTrace();
			log("Error! Could not overwrite GRETask. Aborting!");
			throw new RuntimeException(e);
		} catch (ConsistencyException e) {
			e.printStackTrace();
			log("Error! Could not overwrite GRETask. Aborting!");
			throw new RuntimeException(e);
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
			log("Error! Could not overwrite GRETask. Aborting!");
			throw new RuntimeException(e);
		}
    }
    
    
	@Override
	protected void taskAdopted(String _taskID) {
		// TODO Auto-generated method stub

	}

	@Override
	protected void taskRejected(String _taskID) {
		// TODO Auto-generated method stub

	}
	
	/**
	 * This helper class holds all information of a GRETask
	 * that is needed across the control flow of generating a
	 * referring expression.
	 * 
	 * @author zender
	 *
	 */
	protected class GRETaskWrapper {
		private GRETask m_myTask;
		private WorkingMemoryAddress m_originalGRETaskAdress;
			
		public GRETaskWrapper(GRETask _myTask, WorkingMemoryAddress _originalGRETaskAddress) {
			m_myTask = _myTask;
			m_originalGRETaskAdress = _originalGRETaskAddress;
			m_myTask.m_intendedReferentProxy.m_address.m_id = m_myTask.m_intendedReferentProxy.m_address.m_id.replace("_", ":");
		}
		
		/**
		 * This method returns the proxy address of the
		 * intended referent.
		 * 
		 * @return the <b>WorkingMemoryAddress</b> of the intended referent proxy.
		 */
		public WorkingMemoryAddress getIntendedRefProxyAdress() {
			return m_myTask.m_intendedReferentProxy.m_address;
		}
		
		/**
		 * Use this method to specify the resulting logical
		 * form (LF) of a referring expression (RefEx). The 
		 * flag m_done is set so it's easy to sync with CAST
		 * components that have written out the original task. 
		 * 
		 * @param _resultLF a <b>String</b> of the generated LF
		 */
		public void setResultLF(String _resultLF) {
			m_myTask.m_resultLF = _resultLF;
			m_myTask.m_done = true; 
		}
		
		/**
		 * This method returns the GRETask struct wrapped in this class.
		 * 
		 * @return the <b>GRETask</b> struct
		 */
		public GRETask getGRETaskStruct() {
			return m_myTask;
		}

		
		/**
		 * This method will be used by the returnRefExLF method
		 * to know the position of the original GRETask struct.
		 * 
		 * @return the <b>WorkingMemoryAddress</b> of the original GRETask.
		 */
		public WorkingMemoryAddress getOriginalGRETaskAddress () {
			return m_originalGRETaskAdress;
		}
		
	}

}
