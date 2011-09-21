package execution.components;

import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import eu.cogx.beliefs.slice.GroundedBelief;
import execution.util.LocalActionStateManager;

/**
 * Some things that seem common across all CogX action interfaces.
 * 
 * @author nah
 *
 */
public abstract class AbstractActionInterface extends ManagedComponent {

	protected LocalActionStateManager m_actionStateManager;

	public void addBooleanFeature(WorkingMemoryAddress _beliefAddress, String _feature, boolean _value)
			throws DoesNotExistOnWMException, ConsistencyException,
			PermissionException, UnknownSubarchitectureException {
			
				GroundedBelief belief = getMemoryEntry(_beliefAddress,
						GroundedBelief.class);
				CASTIndependentFormulaDistributionsBelief<GroundedBelief> pb = CASTIndependentFormulaDistributionsBelief
						.create(GroundedBelief.class, belief);
			
				FormulaDistribution fd = FormulaDistribution.create();
				fd.add(_value, 1);
			
				pb.getContent().put(_feature, fd);
				overwriteWorkingMemory(_beliefAddress, pb.get());
			}

}
