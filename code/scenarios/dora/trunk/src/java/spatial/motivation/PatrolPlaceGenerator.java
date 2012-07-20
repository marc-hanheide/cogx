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
import castutils.castextensions.wmeditor.serializer.YAMLSerializer;
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
  private Iterator<Integer> order = null;

  @Override
  protected PatrolMotive checkForAddition(WorkingMemoryAddress addr,
      GroundedBelief newEntry) {
    log("got a new place belief to check for addtion...");
    CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
        .create(GroundedBelief.class, newEntry);
    boolean isExplored = isExploredPlace(belief);
    if (isExplored) {
      beliefs.put(getPlaceId(belief), newEntry);
      log("  it is explored so we can remember it and start managing our motives");
      manageMotive();
    }
    return null;
  }

  @Override
  protected PatrolMotive checkForUpdate(GroundedBelief newEntry,
      PatrolMotive existingMotive) {
    log("got a place belief overwrite");
    CASTIndependentFormulaDistributionsBelief<GroundedBelief> belief = CASTIndependentFormulaDistributionsBelief
        .create(GroundedBelief.class, newEntry);
    // do nothing
    boolean isExplored = isExploredPlace(belief);
    if (isExplored) {
      beliefs.put(getPlaceId(belief), newEntry);
      log("  it is explored so we can remember it and start managing our motives");
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
        String nextToken = st.nextToken();
        println("  added new target " + nextToken);
        targets.add(Integer.parseInt(nextToken));
      }
    } else {
      println("  added default target 0");
      targets.add(0);
    }
    order = targets.iterator();
  }

  private synchronized void manageMotive() {
    log("manageMotive()");
    if (currentWMA == null) {
      log("we have a new motive to submit as the old one has finished");
      if (!order.hasNext()) {
        order = targets.iterator();
        log("all targets done. start all over again");
      }
      int nextTarget = 0;
      while (order.hasNext()) {
        // if the next place does exist, we patrol it
        nextTarget = order.next();
        log("check if target " + nextTarget + " is valid");
        GroundedBelief bel = beliefs.get(nextTarget);
        if (bel != null) {
          log("  we found a new valid target, let's submit it");
          currentWMA = submitNewGoal(bel, nextTarget);
          break;
        } else {
          log("  no it isn't, wait for it");
        }
      }
    } else {
      log("currentWMA is non-null, so we have an active goal and nothing to do for now");
    }
  }

  @Override
  protected synchronized void reactivateMotive(WorkingMemoryChange _wmc,
      PatrolMotive motive) {
    // flag that we have no active goal
    log("reactivate called after a goal has been signalled as achieved. Check if it is the one we are responsible for.");
    if (_wmc.address.equals(currentWMA)) {
      log("reactivate called after a goal has been achieved. Signalling that we are free to issue the next one.");
      currentWMA = null;
      // sleepComponent(1000);
      // trigger new one
      manageMotive();
    }

  }

  @Override
  protected void runComponent() {
    // submit the first one if possible (very unlikely, but just to get
    // things properly started)
    println("start the generation of patrol motives");
    manageMotive();
    super.runComponent();
  }

  private WorkingMemoryAddress submitNewGoal(GroundedBelief bel, int target) {
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
    result.placeID = target;

    fillValues(belief, result);
    // WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(),
    // getSubarchitectureID());
    // result.thisEntry = wma;
    try {
      println("motive genereated as: " + (new YAMLSerializer()).dump(result));

      WorkingMemoryAddress wma = insertNewGoal(result);
      return wma;
    } catch (CASTException e) {
      logException(e);
      return null;
    }

  }

  /**
	 * 
	 */

}
