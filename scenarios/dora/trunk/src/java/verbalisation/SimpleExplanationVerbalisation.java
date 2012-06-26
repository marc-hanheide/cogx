package verbalisation;

import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

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
import castutils.castextensions.IceXMLSerializer;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.produce.ContentPlanningGoal;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import eu.cogx.beliefs.slice.HypotheticalBelief;

/** A simple component to produce some verbalisation for explanations.
 * 
 * @author Graham Horn
 *
 */
public class SimpleExplanationVerbalisation extends ManagedComponent 
{
 
  private Set<HypotheticalBelief> hypotheticalBeliefs;
  private String dialogueSA = "dialogue";
  
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
    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
        HypotheticalBelief.class, WorkingMemoryOperation.ADD),
        new WorkingMemoryChangeReceiver()
        {
          public void workingMemoryChanged(WorkingMemoryChange _wmc)
              throws CASTException
          {
            processAddedHypotheticalBelief(_wmc);
          }
        });
  }
  
  private void processAddedHypotheticalBelief(WorkingMemoryChange _wmc)
  {
    try
    {
      HypotheticalBelief hypothetical_belief = getMemoryEntry(_wmc.address, HypotheticalBelief.class);
      hypotheticalBeliefs.add(hypothetical_belief);
      if (hypotheticalBeliefs.size() == 2)
      {
        verbaliseExplanation();
      }
      else
      {
        // wait for the other belief
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
	
  protected void verbaliseExplanation()
  {
    Iterator<HypotheticalBelief> beliefs_itr = hypotheticalBeliefs.iterator();
    HypotheticalBelief belief1 = beliefs_itr.next();
    HypotheticalBelief belief2 = beliefs_itr.next();
    
    HypotheticalBelief proposed_object;
    HypotheticalBelief proposed_location;
    
    if (firstBeliefIsProposedObject(belief1, belief2))
    {
      proposed_object = belief1;
      proposed_location = belief2;
    }
    else if (firstBeliefIsProposedObject(belief2, belief1))
    {
      proposed_location = belief1;
      proposed_object = belief2;
    }
    else
    {
      proposed_location = null;
      proposed_object = null;
    }
    
    String canned_text = generateText(proposed_object, proposed_location);
    if (isRunning())
    {
      verbaliseCannedText(canned_text);
    }
    else
    {
      System.out.println(canned_text);
    }
  }
  
  protected String findLabel(CondIndependentDistribs ci_distribs)
  {
    BasicProbDistribution prob_distribution_label = (BasicProbDistribution) ci_distribs.distribs.get("label");
    FormulaValues formula_values_label = (FormulaValues) prob_distribution_label.values;
    ElementaryFormula elementary_formula_label = (ElementaryFormula) formula_values_label.values.get(0).val;
    return elementary_formula_label.prop;
  }
  
  protected String findRelation(CondIndependentDistribs ci_distribs)
  {
    BasicProbDistribution prob_distribution_relation = (BasicProbDistribution) ci_distribs.distribs.get("relation");
    FormulaValues formula_values_relation = (FormulaValues) prob_distribution_relation.values;
    ElementaryFormula elementary_formula_relation = (ElementaryFormula) formula_values_relation.values.get(0).val;
    return elementary_formula_relation.prop;
  }
  
  protected String generateText(HypotheticalBelief proposed_object, 
      HypotheticalBelief proposed_location)
  {
    StringBuilder result = new StringBuilder();
    try
    {
      result.append("Perhaps the ");
      CondIndependentDistribs loc_distribs = (CondIndependentDistribs) proposed_location.content;
      
      result.append(findLabel(loc_distribs));     
      result.append(" is ");
      result.append(findRelation(loc_distribs));
      
      result.append(" a ");
      
      CondIndependentDistribs obj_distribs = (CondIndependentDistribs) proposed_object.content;
      
      result.append(findLabel(obj_distribs));          
      result.append(" which is ");
      result.append(findRelation(obj_distribs));
      
      // TODO use the WM entry for the coma room
      
      BasicProbDistribution prob_distribution = (BasicProbDistribution) obj_distribs.distribs.get("related-to");
      FormulaValues formula_values = (FormulaValues) prob_distribution.values;
      PointerFormula pointer_formula = (PointerFormula) formula_values.values
          .get(0).val;
      
      WorkingMemoryAddress wma = pointer_formula.pointer;
      
      result.append(" WM address ").append(wma.id).append(" in SA ").append(wma.subarchitecture);
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

        Object related_to2 = distribs2.distribs.get("related-to");
        BasicProbDistribution prob_distribution2 = (BasicProbDistribution) related_to2;
        FormulaValues formula_values2 = (FormulaValues) prob_distribution2.values;
        PointerFormula pointer_formula2 = (PointerFormula) formula_values2.values
            .get(0).val;
        
        if (belief1.id.equals(pointer_formula2.pointer.id)) 
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
  
  public static void main(String[] argv)
  {
    String in1 =
        "<eu.cogx.beliefs.slice.HypotheticalBelief>" +
        "  <frame class=\"de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame\">" +
        "    <place></place> " +
        "    <existProb>0.0</existProb>" +
        "  </frame>" +
        "  <estatus class=\"de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus\">" +
        "    <agent>robot</agent>" +
        "  </estatus>" +
        "  <id>7:t</id>" +
        "  <type>visualobject</type>" +
        "  <content class=\"de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs\">" +
        "    <distribs>" +
        "      <entry>" +
        "        <string>relation</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>relation</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +    
        "            <values class=\"linked-list\">    " +
        "          <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula\">" +
        "                  <id>0</id>" +
        "                  <prop>in</prop>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "      <entry>" +
        "        <string>label</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>label</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +
        "            <values class=\"linked-list\">" +
        "              <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula\">" +
        "                  <id>0</id>" +
        "                  <prop>container</prop>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "      <entry>" +
        "        <string>related-to</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>related-to</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +
        "            <values class=\"linked-list\">" +
        "              <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula\">" +
        "                  <id>0</id>" +
        "                  <pointer>" +
        "                    <id>0:61</id>" +
        "                    <subarchitecture>coma</subarchitecture>" +
        "                  </pointer>" +
        "                  <type></type>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "    </distribs>" +
        "  </content>" +
        "  <hist class=\"de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory\">" +
        "    <ancestors/>" +
        "    <offspring/>" +
        "  </hist>" +
        "</eu.cogx.beliefs.slice.HypotheticalBelief>";
    
    String in2 =
        "<eu.cogx.beliefs.slice.HypotheticalBelief>" +
        "  <frame class=\"de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame\">" +
        "<place></place> " +
        "<existProb>0.0</existProb>" +
        "  </frame>" +
        "  <estatus class=\"de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus\">" +
        "    <agent>robot</agent>" +
        "  </estatus>" +
        "  <id>6:t</id>" +
        "  <type>visualobject</type>" +
        "  <content class=\"de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs\">" +
        "    <distribs>" +
        "      <entry>" +
        "        <string>relation</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>relation</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +    
        "            <values class=\"linked-list\">    " +
        "          <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula\">" +
        "                  <id>0</id>" +
        "                  <prop>in</prop>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "      <entry>" +
        "        <string>label</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>label</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +
        "            <values class=\"linked-list\">" +
        "              <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula\">" +
        "                  <id>0</id>" +
        "                  <prop>magazine</prop>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "      <entry>" +
        "        <string>related-to</string>" +
        "        <de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "          <key>related-to</key>" +
        "          <values class=\"de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues\">" +
        "            <values class=\"linked-list\">" +
        "              <de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "                <val class=\"de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula\">" +
        "                  <id>0</id>" +
        "                  <pointer>" +
        "                    <id>7:t</id>" +
        "                    <subarchitecture>coma</subarchitecture>" +
        "                  </pointer>" +
        "                  <type></type>" +
        "                </val>" +
        "                <prob>1.0</prob>" +
        "              </de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair>" +
        "            </values>" +
        "          </values>" +
        "        </de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution>" +
        "      </entry>" +
        "    </distribs>" +
        "  </content>" +
        "  <hist class=\"de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory\">" +
        "    <ancestors/>" +
        "    <offspring/>" +
        "  </hist>" +
        "</eu.cogx.beliefs.slice.HypotheticalBelief>";
    
    HashSet<HypotheticalBelief> initial_beliefs = new HashSet<HypotheticalBelief>();
    initial_beliefs.add(IceXMLSerializer.fromXMLString(in1, HypotheticalBelief.class));
    initial_beliefs.add(IceXMLSerializer.fromXMLString(in2, HypotheticalBelief.class));
    SimpleExplanationVerbalisation sev = new SimpleExplanationVerbalisation(initial_beliefs);
    sev.verbaliseExplanation();
  }
}
