package verbalisation;

import java.util.HashMap;
import java.util.Map;

import autogen.Planner.Goal;
import autogen.Planner.PlanningTask;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.production.PlanVerbalizationRequest;
import de.dfki.lt.tr.dialogue.slice.produce.PEVTaskStatus;

/** Simple component to keep track of PlanningTasks and trigger verbalisation.
 * @author Graham Horn
*/
public class PlanVerbalisationController extends ManagedComponent
{
  private Map<Integer, PlanningTask> planningTaskMap = new HashMap<Integer, PlanningTask>();

  private final PlanVerbalisationControllerFrame m_gui;
  
  public PlanVerbalisationController()
  {
    super();
    m_gui = new PlanVerbalisationControllerFrame(this);
    m_gui.pack();
    m_gui.setSize(800, 200);
    m_gui.setVisible(true);
  }
  
  protected void start()
  {

    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
        PlanningTask.class, WorkingMemoryOperation.ADD),
        new WorkingMemoryChangeReceiver()
        {
          public void workingMemoryChanged(WorkingMemoryChange _wmc)
              throws CASTException
          {
            processAddedPlanningTask(_wmc);
          }
        });

    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
        PlanningTask.class, WorkingMemoryOperation.OVERWRITE),
        new WorkingMemoryChangeReceiver()
        {
          public void workingMemoryChanged(WorkingMemoryChange _wmc)
              throws CASTException
          {
            processOverwrittenPlanningTask(_wmc);
          }
        });
    
    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
        PEVTaskStatus.class, WorkingMemoryOperation.ADD),
        new WorkingMemoryChangeReceiver()
        {
          public void workingMemoryChanged(WorkingMemoryChange _wmc)
              throws CASTException
          {
            processPEVTaskStatus(_wmc);
          }
        });
    
    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
        PEVTaskStatus.class, WorkingMemoryOperation.OVERWRITE),
        new WorkingMemoryChangeReceiver()
        {
          public void workingMemoryChanged(WorkingMemoryChange _wmc)
              throws CASTException
          {
            processPEVTaskStatus(_wmc);
          }
        });
  }

  public boolean triggerPlanVerbalisation(int planning_task_id)
  {
    boolean result;
    if (planningTaskMap.keySet().contains(planning_task_id))
    {
      // new address for the request
      try {
        WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(),
            getSubarchitectureID());
        PlanVerbalizationRequest _request = new PlanVerbalizationRequest(
            planning_task_id);
        // add the action to wm
        addToWorkingMemory(wma, _request);
        log("Triggering Plan Verbalisation for planning task " + planning_task_id);
        result = true;
      }
      catch (Exception e)
      {
        log("Exception when triggering plan verbalisation " + e);
        result = false;
      }

    }
    else
    {
      result = false;
    }

    return result;
  }

  private void processPEVTaskStatus(WorkingMemoryChange _wmc)
  {
    PEVTaskStatus verbalisation_status;
    try
    {
      verbalisation_status = getMemoryEntry(_wmc.address, PEVTaskStatus.class);
      switch (verbalisation_status.status)
      {
        case INPROGRESS:
          log("PEVTaskStatus for task " + verbalisation_status.taskID + " is IN PROGRESS");
          break;
        case COMPLETED:
          log("PEVTaskStatus for task " + verbalisation_status.taskID + " is COMPLETED");
          break;
        case FAILED:
          log("PEVTaskStatus for task " + verbalisation_status.taskID + " is FAILED");
          break;
        default:
          log("PEVTaskStatus for task " + verbalisation_status.taskID + " is unknown");        
      }
      m_gui.updateVerbalisationStatus(verbalisation_status);
    }
    catch (DoesNotExistOnWMException e)
    {
      logException(e);
    }
    catch (UnknownSubarchitectureException e)
    {
      logException(e);
    }
    
  }
  
  private void processAddedPlanningTask(WorkingMemoryChange _wmc)
  {
    PlanningTask _newPlanningTask;
    try
    {
      _newPlanningTask = getMemoryEntry(_wmc.address, PlanningTask.class);
    }
    catch (DoesNotExistOnWMException e)
    {
      logException(e);
      return;
    }
    catch (UnknownSubarchitectureException e)
    {
      logException(e);
      return;
    }
    log("ADDED PlanningTask: " + planningTaskToString(_newPlanningTask));
    planningTaskMap.put(_newPlanningTask.id, _newPlanningTask);
    m_gui.addPlanningTask(_newPlanningTask);
  }

  private void processOverwrittenPlanningTask(WorkingMemoryChange _wmc)
  {
    PlanningTask _oldPlanningTask;
    try
    {
      _oldPlanningTask = getMemoryEntry(_wmc.address, PlanningTask.class);
    }
    catch (DoesNotExistOnWMException e)
    {
      logException(e);
      return;
    }
    catch (UnknownSubarchitectureException e)
    {
      logException(e);
      return;
    }
    log("OVERWRITTEN PlanningTask: " + planningTaskToString(_oldPlanningTask));

    planningTaskMap.put(_oldPlanningTask.id, _oldPlanningTask);
    m_gui.updatePlanningTask(_oldPlanningTask);
  }

  public static String planningTaskExecutionStatusToString(PlanningTask _planningTask)
  {
    String result;
    switch (_planningTask.executionStatus)
    {
    case PENDING:
    {
      result = "PENDING";
      break;
    }
    case SUCCEEDED:
    {
      result = "SUCCEEDED";
      break;
    }
    case ABORTED:
    {
      result = "ABORTED";
      break;
    }
    case FAILED:
    {
      result = "FAILED";
      break;
    }
    case INPROGRESS:
    {
      result = "IN PROGRESS";
      break;
    }
    default:
    {
      result = "unknown";
    }
    }
    return result;
  }

  public static String goalToString(Goal _goal)
  {
    StringBuilder return_sb = new StringBuilder();
    //TODO use regexp to replace goal string with something shorter
    //TODO define abbreviations in a data file
    if ("(forall (?p - place) (= (placestatus ?p) trueplace))".equals(_goal.goalString.trim()))
    {
      return_sb.append("Explore");
    }
    else
    {
      return_sb.append(_goal.goalString);
    }
    return return_sb.toString();
  }
  
  public static String planningTaskToString(PlanningTask _planningTask)
  {
    StringBuilder return_sb = new StringBuilder();
    return_sb.append("ID: ");
    return_sb.append(_planningTask.id);
    return_sb.append(" Goal: ");
    for (Goal goal : _planningTask.goals)
    {
      return_sb.append(goalToString(goal)); 
    }
    return_sb.append("Status: ");
    return_sb.append(planningTaskExecutionStatusToString(_planningTask));

    return return_sb.toString();
  }

}
