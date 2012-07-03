package verbalisation;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import autogen.Planner.PlanningTask;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.HypotheticalBelief;

/** A simple component to produce some verbalisation for explanations.
 * It assumes that the proposed object is in a room - an example verbalisation is:
 * "Perhaps the magazine is in a container which is in room 0 which is most likely a meetingroom".
 * @author Graham Horn
 *
 */
public class SimpleExplanationVerbalisation extends ManagedComponent 
{
 
  private Set<HypotheticalBelief> hypotheticalBeliefs;
  private String dialogueSA = "dialogue";
  private HypotheticalBelief proposedObject;
  private HypotheticalBelief proposedLocation;
 
  public SimpleExplanationVerbalisation()
  {
    super();
    hypotheticalBeliefs = new HashSet<HypotheticalBelief>();
  }
  
  /** 
   *  Constructor for use in testing.
   * @param initial_beliefs
   */
  protected SimpleExplanationVerbalisation(Set<HypotheticalBelief> initial_beliefs)
  {
    this();
    hypotheticalBeliefs.addAll(initial_beliefs);
  }
  
  protected void start()
  {
//    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
//        HypotheticalBelief.class, WorkingMemoryOperation.ADD),
//        new WorkingMemoryChangeReceiver()
//        {
//          public void workingMemoryChanged(WorkingMemoryChange _wmc)
//              throws CASTException
//          {
//            processAddedHypotheticalBelief(_wmc);
//          }
//        });
    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
        PlanningTask.class, WorkingMemoryOperation.OVERWRITE),
        new WorkingMemoryChangeReceiver()
        {
          public void workingMemoryChanged(WorkingMemoryChange _wmc)
              throws CASTException
          {
            processPlanningTask(_wmc);
          }
        });
  }
  
  private void processPlanningTask(WorkingMemoryChange _wmc)
  {
    try
    {
      PlanningTask planning_task = getMemoryEntry(_wmc.address, PlanningTask.class);
      if (planning_task.hypotheses.length > 0)
      {
        hypotheticalBeliefs.clear();
        for (WorkingMemoryAddress _wma : planning_task.hypotheses) 
        {
          HypotheticalBelief hypothetical_belief = getMemoryEntry(_wma, HypotheticalBelief.class);
          // for now, only store the beliefs of type visualobject
          if ("visualobject".equals(hypothetical_belief.type))
          {
            hypotheticalBeliefs.add(hypothetical_belief);
          }
          else
          {
            // discard
          }
        }
        
        if (hypotheticalBeliefs.size() == 2)
        {
          // TODO perhaps wait for the "I have now found an explanation message"
          verbaliseExplanation();
        }
        else
        {
          verbaliseCannedText("Please inspect the Hypothetical Beliefs in my working memory");
        }
      }
      else
      {
        // nothing to do
      }
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
  
//  private void processAddedHypotheticalBelief(WorkingMemoryChange _wmc)
//  {
//    try
//    {
//      HypotheticalBelief hypothetical_belief = getMemoryEntry(_wmc.address, HypotheticalBelief.class);
//      hypotheticalBeliefs.add(hypothetical_belief);
//      if (hypotheticalBeliefs.size() == 2)
//      {
//        // TODO wait for the "I have now found an explanation message"
//        verbaliseExplanation();
//      }
//      else
//      {
//        // wait for the other belief
//      }
//    }
//    catch (DoesNotExistOnWMException e)
//    {
//      logException(e);
//    }
//    catch (UnknownSubarchitectureException e)
//    {
//      logException(e);
//    }
//  }
  
  protected void findProposedObjectAndLocation()
  {
    Iterator<HypotheticalBelief> beliefs_itr = hypotheticalBeliefs.iterator();
    HypotheticalBelief belief1 = beliefs_itr.next();
    HypotheticalBelief belief2 = beliefs_itr.next();
    
    if (firstBeliefIsProposedObject(belief1, belief2))
    {
      proposedObject = belief1;
      proposedLocation = belief2;
    }
    else if (firstBeliefIsProposedObject(belief2, belief1))
    {
      proposedLocation = belief1;
      proposedObject = belief2;
    }
    else
    {
      proposedLocation = null;
      proposedObject = null;
    }
  }
  
  protected void verbaliseExplanation()
  {
    findProposedObjectAndLocation();
    
    String canned_text = generateText();
    if (isRunning())
    {
      verbaliseCannedText(canned_text);
    }
    else
    {
      System.out.println(canned_text);
    }
  }
  
  private String findLabel(CondIndependentDistribs ci_distribs)
  {
    BasicProbDistribution prob_distribution_label = (BasicProbDistribution) ci_distribs.distribs.get("label");
    FormulaValues formula_values_label = (FormulaValues) prob_distribution_label.values;
    ElementaryFormula elementary_formula_label = (ElementaryFormula) formula_values_label.values.get(0).val;
    return elementary_formula_label.prop;
  }
  
  private String findRelation(CondIndependentDistribs ci_distribs)
  {
    BasicProbDistribution prob_distribution_relation = (BasicProbDistribution) ci_distribs.distribs.get("relation");
    FormulaValues formula_values_relation = (FormulaValues) prob_distribution_relation.values;
    ElementaryFormula elementary_formula_relation = (ElementaryFormula) formula_values_relation.values.get(0).val;
    return elementary_formula_relation.prop;
  }
  
  private WorkingMemoryAddress findRelatedToPointer(CondIndependentDistribs ci_distribs)
  {
    BasicProbDistribution prob_distribution = (BasicProbDistribution) ci_distribs.distribs.get("related-to");
    FormulaValues formula_values = (FormulaValues) prob_distribution.values;
    PointerFormula pointer_formula = (PointerFormula) formula_values.values.get(0).val;
    
    return pointer_formula.pointer;
  } 
  
  private GroundedBelief getGroundedBelief(WorkingMemoryAddress _wma)
  {
    GroundedBelief result = null;
    try 
    {
      result = getMemoryEntry(_wma, GroundedBelief.class);
    }
    catch (DoesNotExistOnWMException e) 
    {
      logException(e);
    }
    catch (UnknownSubarchitectureException e) 
    {
      logException(e);
    }
    
    return result;
  }
  
  protected String generateRoomText(GroundedBelief grounded_belief)
  {
    StringBuilder result = new StringBuilder();
    
    if ("comaroom".equals(grounded_belief.type))
    {
      result.append("room ");
      CondIndependentDistribs ci_distribs = (CondIndependentDistribs) grounded_belief.content;
      result.append(findRoomId(ci_distribs));
      result.append(" which is most likely a ");
      
      result.append(findMostLikelyCategory(ci_distribs));
    }
    else
    {
      // unexpected type
    }
    
    return result.toString();
  }

  private int findRoomId(CondIndependentDistribs ci_distribs) 
  {
    BasicProbDistribution prob_distribution = (BasicProbDistribution) ci_distribs.distribs.get("RoomId");
    FormulaValues formula_values = (FormulaValues) prob_distribution.values;
    IntegerFormula room_id = (IntegerFormula) formula_values.values.get(0).val;
    return room_id.val;
  }
  
  private String findMostLikelyCategory(CondIndependentDistribs ci_distribs)
  {
    BasicProbDistribution prob_distribution = (BasicProbDistribution) ci_distribs.distribs.get("category");
    FormulaValues formula_values = (FormulaValues) prob_distribution.values;
    Iterator<FormulaProbPair> values_itr = formula_values.values.iterator();
    String most_likely_room_name = "unknown";
    float highest_probability = 0;
    while (values_itr.hasNext())
    {
      FormulaProbPair prob_pair = values_itr.next();
      if (prob_pair.prob > highest_probability)
      {
        highest_probability = prob_pair.prob;
        most_likely_room_name = ((ElementaryFormula) prob_pair.val).prop;
      }
    }
    return most_likely_room_name;
  }
  
  protected String generateText()
  {
    StringBuilder result = new StringBuilder();
    try
    {
      result.append("Perhaps the ");
      CondIndependentDistribs loc_distribs = (CondIndependentDistribs) proposedLocation.content;
      
      result.append(findLabel(loc_distribs));     
      result.append(" is ");
      result.append(findRelation(loc_distribs));
      
      result.append(" a ");
      
      CondIndependentDistribs obj_distribs = (CondIndependentDistribs) proposedObject.content;
      
      result.append(findLabel(obj_distribs));          
      result.append(" which is ");
      result.append(findRelation(obj_distribs));
    
      WorkingMemoryAddress _wma = findRelatedToPointer(obj_distribs);
      if (isRunning())
      {
        result.append(" ");
        result.append(generateRoomText(getGroundedBelief(_wma)));
      }
      else
      {
       result.append(" WM address ").append(_wma.id).append(" in SA ").append(_wma.subarchitecture);
      }
    }
    catch (NullPointerException npe)
    {
      logException(npe);
    }
    catch (ClassCastException cce)
    {
      logException(cce);
    }
    return result.toString();
  }
  
  protected boolean firstBeliefIsProposedObject(HypotheticalBelief belief1, HypotheticalBelief belief2)
  {
    boolean result = false;
    try
    {   
      if (belief2.content instanceof CondIndependentDistribs)
      {
        CondIndependentDistribs distribs2 = (CondIndependentDistribs) belief2.content;

        WorkingMemoryAddress pointer_address = findRelatedToPointer(distribs2);
        
        if (belief1.id.equals(pointer_address.id)) 
        {
          result = true;
        }     
        
      }
    }
    catch (NullPointerException npe)
    {
      logException(npe);
    }
    catch (ClassCastException cce)
    {
      logException(cce);
    }
    return result;
  }
  
  /**
   * Taken from {@link VerbalisationFacade}
   */
  protected LogicalForm lfForCannedText(String canned_text) 
  {
    if (canned_text.contains(" ")) 
    {
        canned_text = canned_text.replaceAll(" ", "_");
    }
    String lf_goal = CASTUtils.concatenate("@d1:dvp(c-goal ^ <CannedText>",
            canned_text, " )");
    log(lf_goal);
    return LFUtils.convertFromString(lf_goal);
  }

  /**
   * Taken from {@link VerbalisationFacade}  
   */
  public void verbaliseCannedText(String _text) 
  {
    if (!_text.isEmpty()) 
    {
      LogicalForm logical_form = lfForCannedText(_text);
      verbaliseLF(logical_form);
    }
  }

  /**
   * Taken from {@link VerbalisationFacade}
   */
  private void verbaliseLF(LogicalForm logical_form) 
  {
    ContentPlanningGoal cp_goal = new ContentPlanningGoal(newDataID(), logical_form, null);
    try 
    {
      addToWorkingMemory(new WorkingMemoryAddress(newDataID(), dialogueSA), cp_goal);
    } 
    catch (SubarchitectureComponentException e) 
    {
      e.printStackTrace();
    }
  }
  
}
