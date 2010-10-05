/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package motivation.components.generators;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject;
import eu.cogx.beliefs.slice.GroundedBelief;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public abstract class AbstractEpistemicObjectMotiveGenerator<M extends Motive, T extends EpistemicObject>
		extends ManagedComponent {
	public static final String ROBOT_BELIEF_TYPE = "Robot";
	public static final String BINDER_SA = "binder";

	private static final int DEFAULT_MAX_EXECUTION_TIME = 60 * 5;

	private static final int DEFAULT_MAX_PLANNING_TIME = 60;
	final Map<WorkingMemoryAddress, WorkingMemoryAddress> bel2motiveMap = new HashMap<WorkingMemoryAddress, WorkingMemoryAddress>();
	final Class<M> motiveClass;
	final Class<T> epistemicClass;
	private WorkingMemoryAddress robotBeliefAddr = null;
	/**
	 * 
	 */
	protected AbstractEpistemicObjectMotiveGenerator(Class<M> motiveClass,
			Class<T> epistemicClass) {
		this.motiveClass = motiveClass;
		this.epistemicClass = epistemicClass;
	}

	protected <T2 extends Motive> T2 fillDefault(T2 result) {
		result.created = getCASTTime();
		result.correspondingUnion = "";
		result.maxExecutionTime = DEFAULT_MAX_EXECUTION_TIME;
		result.maxPlanningTime = DEFAULT_MAX_PLANNING_TIME;
		result.priority = MotivePriority.UNSURFACE;
		result.status = MotiveStatus.UNSURFACED;
		return result;
	}

	protected WorkingMemoryAddress getRobotBeliefAddr() {
		if (robotBeliefAddr == null) {
			List<CASTData<GroundedBelief>> groundedBeliefs = new ArrayList<CASTData<GroundedBelief>>();
			try {
				getMemoryEntriesWithData(GroundedBelief.class, groundedBeliefs,
						BINDER_SA, 0);
			} catch (UnknownSubarchitectureException e) {
				logException(e);
				return null;
			}

			for (CASTData<GroundedBelief> beliefEntry : groundedBeliefs) {
				if (beliefEntry.getData().type.equals(ROBOT_BELIEF_TYPE)) {
					robotBeliefAddr = new WorkingMemoryAddress(beliefEntry
							.getID(), BINDER_SA);
					break;
				}
			}
			getLogger().warn(
					"unable to find belief '" + ROBOT_BELIEF_TYPE + "'");
		}
		return robotBeliefAddr;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see motivation.components.generators.AbstractMotiveGenerator#start()
	 */
	@Override
	protected void start() {
	}

}
