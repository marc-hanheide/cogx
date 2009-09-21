package binder.components;

import binder.autogen.beliefmodel.ColorProperty;
import binder.autogen.beliefmodel.ComplexFormula;
import binder.autogen.beliefmodel.EntityDescription;
import binder.autogen.beliefmodel.LogicalOp;
import binder.autogen.beliefmodel.SuperFormula;
import binder.autogen.beliefmodel.UncertainSuperFormula;
import binder.autogen.core.Feature;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.featvalues.StringValue;
import binder.utils.BeliefModelUtils;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class BeliefModelTranslator extends ManagedComponent {
	
	@Override
	public void start() {

		// if the set of possible union configurations has been updated, update the
		// monitor accordingly
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(UnionConfiguration.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					UnionConfiguration config = 
					getMemoryEntry(_wmc.address, UnionConfiguration.class);
					translateIntoBeliefModel(config);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		log("Belief model translator successfully initialized");
	}
	
	
	public SuperFormula getFeatureValuesAsFormula (Feature feat, int featnumber) {
		
		SuperFormula formula;

		if (feat.alternativeValues.length == 1) {
			formula = BeliefModelUtils.createNewProperty(feat.featlabel, feat.alternativeValues[0]);			
		}
		else if (feat.alternativeValues.length >1) {
			ComplexFormula featurevalues = new ComplexFormula();
			SuperFormula[] featvalsArray = new UncertainSuperFormula[feat.alternativeValues.length];
			
			for (int k = 0 ; k < feat.alternativeValues.length ;k++) {
				featvalsArray[k] = BeliefModelUtils.createNewProperty(feat.featlabel, feat.alternativeValues[k]);
				featvalsArray[k].id = "featvalue-" + (k+1);
			}
			featurevalues.formulae = featvalsArray;
			featurevalues.op = LogicalOp.xor;
			formula = featurevalues;
		}
		else {
			formula = new UncertainSuperFormula();
		}
		
		formula.id = "feature-" + featnumber;
		return formula;
	}
	
	
	public SuperFormula translateIntoBeliefModel(UnionConfiguration config) {
		ComplexFormula formula = new ComplexFormula();
		
		int UnionConfigNb = 1;
		formula.id = "unionconfig-" + UnionConfigNb;
		formula.op = LogicalOp.and;
		formula.formulae = new SuperFormula[config.includedUnions.length];
	
		for (int i = 0 ; i < config.includedUnions.length ; i++) {
		
			Union union = config.includedUnions[i];
			
			if (union.features.length == 1) {
				Feature feat = union.features[0];
				formula.formulae[i] = getFeatureValuesAsFormula(feat, 1);
				
			}
			else if (union.features.length > 1) {
				ComplexFormula entity = new ComplexFormula();
				entity.id = newDataID();
				entity.op = LogicalOp.and;

				SuperFormula[] properties = new SuperFormula[union.features.length];
				for (int j = 0 ; j < union.features.length ; j++) {
				
					Feature feat = union.features[j];
					properties[j] = getFeatureValuesAsFormula(feat, (j+1));
			 		
				}
				entity.formulae = properties;
				formula.formulae[i] = entity;

			}
			formula.formulae[i].id = "union-"  + (i+1);		
			
		}
		log("Formula successfully built!");
		
		log("\n"+BeliefModelUtils.getFormulaPrettyPrint(formula));
		return formula;
	}

}
