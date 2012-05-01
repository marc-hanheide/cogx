package execution.components;

import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefs.utils.BeliefUtils;
import execution.util.LocalActionStateManager;

/**
 * Some things that seem common across all CogX action interfaces.
 * 
 * @author nah
 * 
 */
public abstract class AbstractActionInterface<BeliefType extends dBelief>
		extends ManagedComponent {

	protected LocalActionStateManager m_actionStateManager;

	protected final Class<BeliefType> m_beliefCls;

	protected AbstractActionInterface(Class<BeliefType> _beliefCls) {
		m_beliefCls = _beliefCls;
	}

	public void addFeature(WorkingMemoryAddress _beliefAddress,
			String _feature, boolean _value) throws DoesNotExistOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {

		BeliefUtils.addFeature(this, _beliefAddress, m_beliefCls, _feature,
				_value);
	}

	public void addFeature(WorkingMemoryAddress _beliefAddress,
			String _feature, String _value) throws DoesNotExistOnWMException,
			ConsistencyException, PermissionException,
			UnknownSubarchitectureException {

		BeliefUtils.addFeature(this, _beliefAddress, m_beliefCls, _feature,
				_value);
	}

	protected WorkingMemoryPointer getFirstAncestorOfBelief(
			WorkingMemoryAddress _beliefAddress)
			throws DoesNotExistOnWMException, UnknownSubarchitectureException {

		return BeliefUtils.getMostRecentAncestorPointer(this, _beliefAddress,
				m_beliefCls);
	}

	
	
}
