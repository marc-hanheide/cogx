///////////////////////////////////////////////////////////////////////////////
//Copyright (C) 2003 Jason Baldridge and University of Edinburgh (Michael White)

//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public
//License as published by the Free Software Foundation; either
//version 2.1 of the License, or (at your option) any later version.

//This library is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU Lesser General Public License for more details.

//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.


package opennlp.ccg.grammar;

import opennlp.ccg.unify.*;
import opennlp.ccg.synsem.*;
import opennlp.ccg.hylo.*;

import java.util.*;

/**
 * Implements some default behavior for Rule objects.
 *
 * @author  Jason Baldridge
 * @author  Michael White
 * @version $Revision: 1.13 $, $Date: 2005/10/13 20:49:22 $
 */
public abstract class AbstractRule implements Rule {

	/** The name of this rule. */
	protected String _name;

	/** The rule group which contains this rule. */
	protected RuleGroup _ruleGroup;

	public boolean handleRobustness = true;


	/** Applies the rule to the given input signs, adding to the given list of results. */
	public void applyRule(Sign[] inputs, List<Sign> results) {
		if (!handleRobustness){
			applyRuleNormal(inputs, results);
		}
		else {
			applyRuleRobust(inputs, results);
		}
	}



	// NORMAL VERSION
	public void applyRuleNormal(Sign[] inputs, List<Sign> results) {

		if (inputs.length != arity()) { // shouldn't happen
			throw new RuntimeException("Inputs must have length " + arity());
		}

		Category[] cats = new Category[inputs.length];
		for (int i=0; i < cats.length; i++) {
			cats[i] = inputs[i].getCategory();
		}

		try {
			List<Category> resultCats = applyRule(cats);
			if (resultCats.isEmpty()) return;

			for (Category catResult : resultCats) {
				distributeTargetFeatures(catResult);
				Sign sign = Sign.createDerivedSign(catResult, inputs, this);
				results.add(sign);
			}
		} catch (UnifyFailure uf) {}
	}



	// ROBUST VERSION
	public void applyRuleRobust(Sign[] inputs, List<Sign> results) {

		if (inputs.length != arity()) { // shouldn't happen
			throw new RuntimeException("Inputs must have length " + arity());
		}

		int numberOfRulesType0Applied = 0;
		int numberOfRulesType1Applied = 0;
		int numberOfRulesType2Applied = 0;

		Category[] cats = new Category[inputs.length];
		for (int i=0; i < cats.length; i++) {
			cats[i] = inputs[i].getCategory();

			numberOfRulesType0Applied += 
				inputs[i].numberOfRulesType0Applied;
			numberOfRulesType1Applied += 
				inputs[i].numberOfRulesType1Applied;
			numberOfRulesType2Applied += 
				inputs[i].numberOfRulesType2Applied;

			FeatureStructure feats = cats[i].getTarget().getFeatureStructure();

			if (numberOfRulesType0Applied == 0 && 
					feats != null && feats.hasAttribute("ROBUST")) {
				inputs[i].numberOfRulesType0Applied = 1;
				numberOfRulesType0Applied ++;
			}

			if (name().contains("ROBUST")) {
				if (numberOfRulesType1Applied > 0) {
					return ;
				}
				else if (numberOfRulesType2Applied > 1) {
					return ;
				}    		
			} 

			if (numberOfRulesType0Applied > 1) {
				return ;
			}
		}

		if (name().contains("ROBUST1")) {
			numberOfRulesType1Applied++;
		}
		if (name().contains("ROBUST2")) {
			numberOfRulesType2Applied++;
		}
		
		try {
			List<Category> resultCats = applyRule(cats);
			if (resultCats.isEmpty()) return;

			for (Category catResult : resultCats) {
				distributeTargetFeatures(catResult);
				Sign sign = Sign.createDerivedSign(catResult, inputs, this);

				sign.numberOfRulesType0Applied = numberOfRulesType0Applied;
				sign.numberOfRulesType1Applied = numberOfRulesType1Applied;
				//     System.out.println( sign.numberOfRulesType1Applied);
				sign.numberOfRulesType2Applied = numberOfRulesType2Applied;
				
				results.add(sign);
			}
		} catch (UnifyFailure uf) {}
	}


	/** Propagates distributive features from target cat to the rest. */
	// nb: it would be nicer to combine inheritsFrom with $, but 
	//     this would be complicated, as inheritsFrom is compiled out
	protected void distributeTargetFeatures(Category cat) {
		if (_ruleGroup.grammar.lexicon.getDistributiveAttrs() == null) return;
		if (!(cat instanceof ComplexCat)) return;
		ComplexCat complexCat = (ComplexCat) cat;
		Category targetCat = (Category) complexCat.getTarget();
		targetFS = (GFeatStruc) targetCat.getFeatureStructure();
		if (targetFS == null) return;
		cat.forall(distributeTargetFeaturesFcn);
	}

	// target cat's feature structure
	private GFeatStruc targetFS = null;

	// copies ground distributive features from _targetFS to the rest
	private CategoryFcn distributeTargetFeaturesFcn = new CategoryFcnAdapter() {
		public void forall(Category c) {
			if (!(c instanceof AtomCat)) return;
			FeatureStructure fs = c.getFeatureStructure();
			if (fs == null) return;
			if (fs == targetFS) return;
			String[] distrAttrs = _ruleGroup.grammar.lexicon.getDistributiveAttrs();
			for (int i = 0; i < distrAttrs.length; i++) {
				Object targetVal = targetFS.getValue(distrAttrs[i]);
				if (targetVal != null && !(targetVal instanceof Variable)) {
					fs.setFeature(distrAttrs[i], UnifyControl.copy(targetVal));
				}
			}
		}
	};


	/**
	 * The number of arguments this rule takes.  For example, the arity of the
	 * forward application rule of categorial grammar (X/Y Y => Y) is 2.
	 *
	 * @return the number of arguments this rule takes
	 **/
	public abstract int arity();

	/**
	 * Apply this rule to some input categories.
	 *
	 * @param inputs the input categories to try to combine
	 * @return the categories resulting from using this rule to combine the
	 *         inputs
	 * @exception UnifyFailure if the inputs cannot be combined by this rule
	 **/
	public abstract List<Category> applyRule(Category[] inputs) throws UnifyFailure;


	/** Prints an apply instance for the given categories to System.out. */
	protected void showApplyInstance(Category[] inputs) {
		StringBuffer sb = new StringBuffer();  
		sb.append(_name).append(": ");

		for (int i=0; i < inputs.length; i++) {
			sb.append(inputs[i]).append(' ');
		}

		System.out.println(sb);
	}

	/** Prints an apply instance for the given categories to System.out. */
	protected void showApplyInstance(Category first, Category second) {
		Category[] ca = {first,second};
		showApplyInstance(ca);
	}


	/**
	 * Returns the name of this rule.
	 */
	public String name() {
		return _name;
	}

	/**
	 * Returns the rule group which contains this rule.
	 */
	public RuleGroup getRuleGroup() { return _ruleGroup; }

	/**
	 * Sets this rule's rule group.
	 */
	public void setRuleGroup(RuleGroup ruleGroup) { _ruleGroup = ruleGroup; }


	/** Appends, fills, sorts and checks the LFs from cats 1 and 2 into the result cat. */
	protected void appendLFs(Category cat1, Category cat2, Category result, Substitution sub) 
	throws UnifyFailure
	{
		LF lf = HyloHelper.append(cat1.getLF(), cat2.getLF());
		if (lf != null) {
			lf = (LF) lf.fill(sub);
			HyloHelper.sort(lf);
			HyloHelper.check(lf);
		}
		result.setLF(lf);
	}
}

