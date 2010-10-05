package de.dfki.lt.tr.dialmanagement.data.observations;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;

public class ObservationContent {
	
	dFormula content;
	int type;
	
	
	public static final int INTENTION = 0;
	public static final int EVENT = 1;


	public ObservationContent (String utterance) {		
		if (utterance.contains(" ")) {
			utterance = "\"" + utterance + "\"";
		}
		try {
			content = FormulaUtils.constructFormula(utterance);
		} catch (DialogueException e1) {
			e1.printStackTrace();
		}
		type = INTENTION;
	}

	public ObservationContent (dFormula formula, int type) {
		content = formula;
		this.type = type;
	}
	
	public boolean isUnderspecified () {
		return (content instanceof UnderspecifiedFormula);
	}
	
	public boolean equals(Object obj) {
		if (obj instanceof ObservationContent) {
			return FormulaUtils.isEqualTo(content, ((ObservationContent)obj).content);
		}
		
		// take underspecifiedformula into account!
		return false;
	}

	@Override
	public String toString () {
		if (type == INTENTION) {
			return "I[" + FormulaUtils.getString(content) + "]";
		}
		else if (type == EVENT) {
			return "E[" + FormulaUtils.getString(content) + "]";
		}
		return "";
	}
	
}
