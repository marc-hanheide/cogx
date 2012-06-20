/**
 * 
 */
package spatial.motivation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.StringTokenizer;

import motivation.slice.MotivePriority;
import motivation.slice.MotiveStatus;
import motivation.slice.PatrolMotive;
import cast.CASTException;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import eu.cogx.beliefs.slice.GroundedBelief;

/**
 * @author marc
 * 
 */
public class PatrolPlaceGenerator extends GotoPlaceGenerator {

	private Map<Integer, GroundedBelief> beliefs = new HashMap<Integer, GroundedBelief>();
	private WorkingMemoryAddress currentWMA = null;
	private List<Integer> targets = new ArrayList<Integer>();
	private Iterator<Integer> order = targets.iterator();

	@Override
	protected PatrolMotive checkForAddition(WorkingMemoryAddress addr,
			GroundedBelief newEntry) {
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		boolean isExplored = isExploredPlace(belief);
		if (isExplored) {
			beliefs.put(getPlaceId(belief), newEntry);
			manageMotive();
		}
		return null;
	}

	@Override
	protected PatrolMotive checkForUpdate(GroundedBelief newEntry,
			PatrolMotive existingMotive) {
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, newEntry);
		// do nothing
		boolean isExplored = isExploredPlace(belief);
		if (isExplored) {
			beliefs.put(getPlaceId(belief), newEntry);
			manageMotive();
		}
		return null;
	}

	@Override
	protected void configure(Map<String, String> _config) {
		super.configure(_config);
		String intStr = _config.get("--targets");
		if (intStr != null) {
			StringTokenizer st = new StringTokenizer(intStr, " ,");
			while (st.hasMoreTokens()) {
				targets.add(Integer.parseInt(st.nextToken()));
			}
		} else {
			targets.add(0);
		}
	}

	private synchronized void manageMotive() {

		if (currentWMA != null) {
			if (!order.hasNext())
				order = targets.iterator();
			int nextTarget = 0;
			while (order.hasNext()) {
				// if the next place does exist, we patrol it
				GroundedBelief bel = beliefs.get(nextTarget);
				if (bel != null) {
					currentWMA = submitNewGoal(bel);
					break;
				}
			}
		}
	}

	@Override
	protected synchronized void reactivateMotive(WorkingMemoryChange _wmc,
			PatrolMotive motive) {
		// flag that we have no active goal
		currentWMA = null;
		sleepComponent(1000);
		// trigger new one
		manageMotive();

	}

	@Override
	protected void runComponent() {
		// submit the first one if possible (very unlikely, but just to get
		// things properly started)
		manageMotive();
		super.runComponent();
	}

	private WorkingMemoryAddress submitNewGoal(GroundedBelief bel) {
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, bel);
		log("place is explored, so it is a goal to goto");
		PatrolMotive result = new PatrolMotive();
		result.created = getCASTTime();
		result.maxExecutionTime = MAX_EXECUTION_TIME;
		result.maxPlanningTime = MAX_PLANNING_TIME;
		result.priority = MotivePriority.UNSURFACE;
		result.referenceEntry = new WorkingMemoryAddress("dummy", "dummy");
		result.status = MotiveStatus.UNSURFACED;
		result.lastVisisted = result.created;
		fillValues(belief, result);
		WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(),
				getSubarchitectureID());
		try {
			addToWorkingMemory(wma, result);
		} catch (CASTException e) {
			logException(e);
		}
		return wma;
	}

	/**
	 * 
	 */

}
