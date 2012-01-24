/**
 * 
 */
package eu.cogx.perceptmediator.dora;

import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import cast.core.CASTUtils;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.formulas.PropositionFormula;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.data.specificproxies.FormulaDistribution;
import de.dfki.lt.tr.beliefs.util.BeliefException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.PerceptBelief;
import eu.cogx.perceptmediator.transferfunctions.abstr.DependentLinkingDiscreteTransferFunction;
import execution.slice.actions.ProcessConeGroupAction;

/**
 * @author marc
 * 
 */
public class VisualObjectTransferFunction
		extends
		DependentLinkingDiscreteTransferFunction<VisualObject, PerceptBelief, GroundedBelief> {

	public static final String LABEL_ID = "label";
	public static final String IS_IN = "related-to";
	public static final String CONE = "from-cone";
	public static final String RELATION = "relation";
	private static final double BLOODY_THRESHOLD_ACCORDING_TO_MICHI = 0.08;
    
    private WorkingMemoryAddress lastViewconeAddress = null;

	public VisualObjectTransferFunction(ManagedComponent component,
			WMView<GroundedBelief> allBeliefs) {
		super(component, allBeliefs, Logger
				.getLogger(VisualObjectTransferFunction.class),
				PerceptBelief.class);
	}

    public void start() {
		WorkingMemoryChangeReceiver receiver = new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				processConeActionAdded(_wmc);
			}
		};
		component.addChangeFilter(
				ChangeFilterFactory.createGlobalTypeFilter(ProcessConeGroupAction.class,
						WorkingMemoryOperation.ADD), receiver);
    }

    private void processConeActionAdded(WorkingMemoryChange _wmc) {
		try {
			CASTData<ProcessConeGroupAction> actionData = component.getMemoryEntryWithData(_wmc.address,
					ProcessConeGroupAction.class);
			ProcessConeGroupAction action = actionData.getData();
            lastViewconeAddress = action.coneGroupBeliefID;
		} catch (DoesNotExistOnWMException e) {
			component.logException(e);
		} catch (UnknownSubarchitectureException e) {
			component.logException(e);
		}

    }

	@Override
	protected Map<String, Formula> getFeatureValueMapping(
			WorkingMemoryChange wmc, VisualObject from) throws BeliefException {
		assert (from != null);
		Map<String, Formula> result = new HashMap<String, Formula>();
		// TODO: we should use a DoubleValue here!
		try {
            if (from.identDistrib[0] > BLOODY_THRESHOLD_ACCORDING_TO_MICHI) {
                result.put(LABEL_ID, PropositionFormula.create(from.identLabels[0])
                           .getAsFormula());
            }
		} catch (BeliefException e) {
			component.logException(e);
		}

		return result;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see eu.cogx.perceptmediator.transferfunctions.abstr.
	 * SimpleDiscreteTransferFunction#fillBelief(de.dfki.lt.tr.beliefs.data.
	 * CASTIndependentFormulaDistributionsBelief, cast.cdl.WorkingMemoryChange,
	 * Ice.ObjectImpl)
	 */
	@Override
	protected void fillBelief(
			CASTIndependentFormulaDistributionsBelief<PerceptBelief> belief,
			WorkingMemoryChange wmc, VisualObject from) {

        Formula related_to = null;
        Formula relation = null;
        Formula cone = null;
        log("check if we have pending cone actions");
        
        if (lastViewconeAddress != null) {
					CASTIndependentFormulaDistributionsBelief<GroundedBelief> coneGroupBelief = CASTIndependentFormulaDistributionsBelief
					  .create(GroundedBelief.class, allBeliefs.get(lastViewconeAddress));

					related_to = coneGroupBelief.getContent().get("cg-related-to").getDistribution().getMostLikely();
					relation = coneGroupBelief.getContent().get("cg-relation").getDistribution().getMostLikely();
//					label = coneGroupBelief.getContent().get("cg-label").getDistribution().getMostLikely();
                    cone = WMPointer.create(lastViewconeAddress, "conegroup").getAsFormula();

					log("we found the cone group that belongs to this cone: "
							+ CASTUtils.toString(lastViewconeAddress));
        }
        if (related_to == null) {
            throw new BeliefException();
        }
        component.log("size of from.identDistrib: "
                      + from.identDistrib.length);

        if (from.identDistrib[0] > BLOODY_THRESHOLD_ACCORDING_TO_MICHI) {
            FormulaDistribution fd1 = FormulaDistribution.create();
            fd1.add(related_to.get(), 1.0);
            FormulaDistribution fd2 = FormulaDistribution.create();
            fd2.add(relation.get(), 1.0);
            // FormulaDistribution fd3 = FormulaDistribution.create();
            // fd3.add(label.get(), 1.0);
            FormulaDistribution fd4 = FormulaDistribution.create();
            fd4.add(cone.get(), 1.0);
            
            belief.getContent().put(IS_IN, fd1);
            belief.getContent().put(RELATION, fd2);
            belief.getContent().put(CONE, fd4);
        }
	}
}
