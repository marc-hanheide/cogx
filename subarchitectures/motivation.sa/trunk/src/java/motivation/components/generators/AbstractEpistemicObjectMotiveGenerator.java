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
import cast.core.CASTUtils;
import eu.cogx.beliefs.slice.GroundedBelief;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public abstract class AbstractEpistemicObjectMotiveGenerator<M extends Motive, T extends Ice.Object>
		extends ManagedComponent {
	// TODO: BAAAAAAAD
	public static final String ROBOT_BELIEF_TYPE = "Robot"; 
	public static final String SPATIAL_SA = "spatial.sa";

	private static final int DEFAULT_MAX_EXECUTION_TIME = 60 * 5;

	private static final int DEFAULT_MAX_PLANNING_TIME = 60;
	final Map<WorkingMemoryAddress, List<WorkingMemoryAddress>> bel2motivesMap = new HashMap<WorkingMemoryAddress, List<WorkingMemoryAddress>>();
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

	public static <T2 extends Motive> T2 fillDefault(T2 result) {
		result.created = CASTUtils.getTimeServer().getCASTTime();
		result.maxExecutionTime = DEFAULT_MAX_EXECUTION_TIME;
		result.maxPlanningTime = DEFAULT_MAX_PLANNING_TIME;
		result.priority = MotivePriority.UNSURFACE;
		result.status = MotiveStatus.UNSURFACED;
		return result;
	}

	protected WorkingMemoryAddress getRobotBeliefAddr() {
		while (robotBeliefAddr == null) {
			List<CASTData<GroundedBelief>> groundedBeliefs = new ArrayList<CASTData<GroundedBelief>>();
			try {
				getMemoryEntriesWithData(GroundedBelief.class, groundedBeliefs,
						SPATIAL_SA, 0);
			} catch (UnknownSubarchitectureException e) {
				logException(e);
				return null;
			}

			for (CASTData<GroundedBelief> beliefEntry : groundedBeliefs) {
				if (beliefEntry.getData().type.equals(ROBOT_BELIEF_TYPE)) {
					robotBeliefAddr = new WorkingMemoryAddress(beliefEntry
							.getID(), SPATIAL_SA);
					break;
				}
				getLogger().warn(
						"unable to find belief '" + ROBOT_BELIEF_TYPE + "'");
			}
		}
		return robotBeliefAddr;
	}

	
	/**
	 * Returns true if this generator still has any motives on WM.
	 * @return
	 */
	protected boolean hasAvailableMotives() {
		return !bel2motivesMap.isEmpty();
	}
}
