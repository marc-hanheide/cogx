package binder.components;

import binder.autogen.beliefmodel.ComplexFormula;
import binder.autogen.beliefmodel.LogicalOp;
import binder.autogen.beliefmodel.SuperFormula;
import binder.autogen.beliefmodel.UncertainSuperFormula;
import binder.autogen.core.Feature;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.utils.BeliefModelUtils;
import cast.DoesNotExistOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * Translate the union configurations on the binder working memory into belief models
 * 
 * @author Pierre Lison
 * @version 15/09/2009 (version 21/09/2009)
 */
public class BeliefModelTranslator extends ManagedComponent {
	
	

	// =================================================================
	// INITIALISATION
	// =================================================================
	
	
	/**
	 * Add a change filter for union configurations on the binder working memory
	 * 
	 * TODO: update from unionconfiguration --> alternativeunionconfigurations
	 */
	@Override
	public void start() {

		// if the set of possible union configurations has been updated, update the
		// monitor accordingly
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(UnionConfiguration.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					// Retrieve the union configuration
					UnionConfiguration config = 
					getMemoryEntry(_wmc.address, UnionConfiguration.class);
					
					// Translate the union configuration into a belief model
					SuperFormula formula = translateIntoBeliefModel(config);
					
					// And add the belief model to the binding working memory
					addFormulaToWM(formula);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		log("Belief model translator successfully initialized");
	}
	
	
	

	// =================================================================
	// METHODS FOR TRANSLATING UNION CONFIGURATIONS INTO BELIEF MODELS
	// =================================================================
	
	
	
	/**
	 * Translate a set of alternative feature values into a belief model superformula
	 * 
	 * @param feat a feature (as defined in BinderEssentials)
	 * @return a SuperFormula
	 */
	private SuperFormula getFeatureValuesAsFormula (Feature feat) {
		
		// The formula 
		SuperFormula formula;

		// If there a single feature value for the feature, create a simple property
		if (feat.alternativeValues.length == 1) {
			formula = BeliefModelUtils.createNewProperty(feat.featlabel, feat.alternativeValues[0]);			
		}
		
		// Else, create a complex formula includeing a set of disjunctive 
		// (mutually exclusive) properties
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
			log("WARNING: feature does not contain any feature value");
		}
		
		return formula;
	}
	
	
	/**
	 * Translate the union configuration into a belief model formula
	 * 
	 * @param config the union configuration
	 * @return the super formula
	 */
	public SuperFormula translateIntoBeliefModel(UnionConfiguration config) {
		ComplexFormula formula = new ComplexFormula();
		
		// BAD
		int UnionConfigNb = 1;
		
		// Create the root formula (defined as a conjunction of formulae)
		formula.id = "unionconfig-" + UnionConfigNb;
		formula.op = LogicalOp.and;
		formula.formulae = new SuperFormula[config.includedUnions.length];
	
		// loop on the included unions
		for (int i = 0 ; i < config.includedUnions.length ; i++) {
		
			Union union = config.includedUnions[i];
			
			// If there is only one feature, create a simple formula
			if (union.features.length == 1) {
				Feature feat = union.features[0];
				formula.formulae[i] = getFeatureValuesAsFormula(feat);
				formula.formulae[i].id = "feature-"+1;
				
			}
			
			// Else, create a complex conjunctive formula for the union
			else if (union.features.length > 1) {
				ComplexFormula entity = new ComplexFormula();
				entity.id = newDataID();
				entity.op = LogicalOp.and;

				SuperFormula[] properties = new SuperFormula[union.features.length];
				for (int j = 0 ; j < union.features.length ; j++) {
				
					Feature feat = union.features[j];
					properties[j] = getFeatureValuesAsFormula(feat);
					formula.formulae[i].id = "feature-"+(j+1);
			 		
				}
				entity.formulae = properties;
				formula.formulae[i] = entity;

			}
			formula.formulae[i].id = "union-"  + (i+1);		
			
		}
		log("Formula successfully built!");
		
		// Print the result
		log("\n"+BeliefModelUtils.getFormulaPrettyPrint(formula));
	
		return formula;
	}

	

	// =================================================================
	// METHODS FOR INSERTING/MODIFYING/DELETING FORMULAE IN THE WM
	// =================================================================
	

	
	/**
	 * Insert the formula in the binder working memory
	 * 
	 * @param formula
	 *            the belief model formula
	 */

	protected void addFormulaToWM(SuperFormula formula) {

		try {
			addToWorkingMemory(formula.id, formula);
			log("new belief model formula succesfully added to the binder working memory");

		} catch (Exception e) {
			e.printStackTrace();
		}
	}


	/**
	 * Overwrite an existing formula with a new one (the new formula needs to have
	 * the same id as the existing one)
	 * 
	 * @param formula
	 *            the belief model formula
	 */

	protected void overwriteFormulaInWM(SuperFormula formula) {
 
		try {
			overwriteWorkingMemory(formula.id, formula);
			log("existing belief model formula succesfully modified in the binder working memory");

		} catch (DoesNotExistOnWMException e) {
			log("Sorry, the belief model formula does not exist in the binder working memory");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Delete an existing formula
	 * 
	 * @param formula
	 *            the belief model formula
	 */

	protected void deleteFormulaInWM (SuperFormula formula) {

		try {
			deleteFromWorkingMemory(formula.id);
			log("existing belief model formula  succesfully deleted from the binder working memory");

		} catch (DoesNotExistOnWMException e) {
			log("Sorry, the belief model formula  does not exist in the binder working memory");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	
}
