package binder.abstr;

import java.util.Vector;

import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.SuperFormula;
import beliefmodels.domainmodel.cogx.UncertainSuperFormula;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.utils.BeliefModelUtils;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;


public class BeliefModelInterface extends ManagedComponent{

	
	ComplexFormula currentBeliefModel;
	
	 
	@Override
	public void start() {

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(ComplexFormula.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					currentBeliefModel = getMemoryEntry(_wmc.address, ComplexFormula.class);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
	}
	
	public ComplexFormula getCurrentBeliefModel ()  {
		return currentBeliefModel;
	}
	
	public SuperFormula getFormulaInBeliefModel (String formulaID) {
		
		SuperFormula result= getFormulaInComplexFormula(currentBeliefModel, formulaID);
		if (result != null) {
			log("WARNING: formula " + formulaID + " not found in belief model!");
		}
		return result;
	}
	
	private SuperFormula getFormulaInComplexFormula (ComplexFormula cform, String formulaID) {
		for (int i = 0; i < cform.formulae.length ; i++) {
			SuperFormula form = cform.formulae[i];
			
			if (form.id.equals(formulaID)) {
				return form;
			}
			else if (form instanceof ComplexFormula) {
				SuperFormula subresult = getFormulaInComplexFormula((ComplexFormula)form, formulaID);
				if (subresult != null) {
					return subresult;
				}
			}
		}
		return null;
	}
	
	public void addFormulaInBeliefModel (SuperFormula newFormula) {
		ComplexFormula newBeliefModel = new ComplexFormula();
		newBeliefModel.op = currentBeliefModel.op;
		newBeliefModel.prob = currentBeliefModel.prob;
		newBeliefModel.id = newBeliefModel.id;
		int nbFormulae = currentBeliefModel.formulae.length;
		newBeliefModel.formulae = new SuperFormula[nbFormulae + 1];
		for (int i = 0 ; i < nbFormulae ; i++) {
			newBeliefModel.formulae[i] = currentBeliefModel.formulae[i];
		}
		newBeliefModel.formulae[nbFormulae] = newFormula;
		try {
			overwriteWorkingMemory (newBeliefModel.id, "binder", newBeliefModel);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		currentBeliefModel = newBeliefModel;
	}
	
	
	public void updateFormulaInBeliefModel (SuperFormula newFormula) {
		
		updateFormulaInComplexFormula(currentBeliefModel, newFormula);
		try {
			overwriteWorkingMemory (currentBeliefModel.id, "binder", currentBeliefModel);
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	private void updateFormulaInComplexFormula (ComplexFormula cform, SuperFormula newFormula) {
		for (int i = 0; i < cform.formulae.length ; i++) {
			SuperFormula form = cform.formulae[i];
			
			if (form.id.equals(newFormula.id)) {
				cform.formulae[i] = newFormula;
			}
			else if (form instanceof ComplexFormula) {
				updateFormulaInComplexFormula ((ComplexFormula)form, newFormula);
			}
		}
	}
	
	@Override
	public void run() {
		
		int counter = 0;
		
		while (isRunning()) {
			try {
				sleepComponent(1000);
				ComplexFormula curForm = getCurrentBeliefModel();
				counter++;
				if (counter == 5) {
					UncertainSuperFormula newForm = new UncertainSuperFormula();
					newForm.id = "union-2:C";
					newForm.prob = 0.2f;
					addFormulaInBeliefModel(newForm);
				}
				log("current belief model: "  + BeliefModelUtils.getFormulaPrettyPrint(curForm));
			}
			
			catch (Exception e) {
				e.printStackTrace();
			}
		}
	}
}
