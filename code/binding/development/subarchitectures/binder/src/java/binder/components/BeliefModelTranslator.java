package binder.components;

import binder.autogen.beliefmodel.ColorProperty;
import binder.autogen.beliefmodel.ComplexFormula;
import binder.autogen.beliefmodel.ComplexProperty;
import binder.autogen.beliefmodel.LogicalOp;
import binder.autogen.beliefmodel.Property;
import binder.autogen.beliefmodel.Entity;
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
	
	
	public SuperFormula translateIntoBeliefModel(UnionConfiguration config) {
		ComplexFormula formula = new ComplexFormula();
		formula.id = newDataID();
		formula.op = LogicalOp.and;
		formula.formulae = new Entity[config.includedUnions.length];
	
		for (int i = 0 ; i < config.includedUnions.length ; i++) {
			
			Union union = config.includedUnions[i];
			
			formula.formulae[i] = new Entity();			
			formula.formulae[i].id = newDataID();
			((Entity)formula.formulae[i]).properties = 
				new Property[union.features.length];
			
			for (int j = 0 ; j < union.features.length ; j++) {
				
				Feature feat = union.features[j];
				if (feat.alternativeValues.length == 1) {
				((Entity)formula.formulae[i]).properties[j] = 
					BeliefModelUtils.createNewProperty(feat.featlabel, feat.alternativeValues[0]);
					log("alter: " + ((StringValue)feat.alternativeValues[0]).val);
				}	
				else {
					ComplexProperty complexProp = new ComplexProperty();
					complexProp.alternativeProperties = 
						new Property[feat.alternativeValues.length];
					for (int k = 0 ; k < feat.alternativeValues.length ;k++) {
						complexProp.alternativeProperties[k] = 
							BeliefModelUtils.createNewProperty(feat.featlabel, feat.alternativeValues[k]);
					}
					((Entity)formula.formulae[i]).properties[j] = complexProp;
				}
				((Entity)formula.formulae[i]).properties[j].id = newDataID();
			}
			
		}
	
		log(BeliefModelUtils.getFormulaPrettyPrint(formula));
		return formula;
	}

}
