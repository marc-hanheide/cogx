///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003 Jason Baldridge and University of Edinburgh (Michael White)
// 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//////////////////////////////////////////////////////////////////////////////

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
 * @version $Revision: 1.14 $, $Date: 2007/06/13 22:02:51 $
 */
public abstract class AbstractRule implements Rule {

    /** The name of this rule. */
    protected String _name;
    
    /** The rule group which contains this rule. */
    protected RuleGroup _ruleGroup;

	public static boolean handleRobustness = true;
	public static boolean asrcorrectionRules = false;
	public static boolean disclevelcompositionRules = false;
	public static boolean disfluencycorrectionRules = false;

	public static int MAX_NB_COMPOSITIONS = 3;
	public static int MAX_LEXICAL_CORRECTIONS = 1;
	public static int MAX_DISFLUENCY_CORRECTIONS = 1;
	
	
	/** Applies the rule to the given input signs, adding to the given list of results. */
	public void applyRule(Sign[] inputs, List<Sign> results) {
		if (!handleRobustness){
			applyRuleNormal(inputs, results);
		}
		else {
			applyRuleRobust(inputs, results);
		}
	}
	
    /** Applies the rule to the given input signs, adding to the given list of results. */
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
    
    

    /** Applies the rule to the given input signs, adding to the given list of results. */
    public void applyRuleRobust(Sign[] inputs, List<Sign> results) {

        if (inputs.length != arity()) { // shouldn't happen
            throw new RuntimeException("Inputs must have length " + arity());
        }
        
		if (name().contains("disclevelcomposition") && 
				!disclevelcompositionRules) { return ; }   

		if (name().contains("correction") && 
				!disfluencycorrectionRules) { return ; }  
	
        
    	int NbLexicalCorrectionRulesApplied = 0;
		int NbTypeShiftingRulesApplied = 0;
		int NbDiscLevelCompositionRulesApplied = 0;
		int NbDisflCorrectionRulesApplied = 0;

        Category[] cats = new Category[inputs.length];
        
        for (int i=0; i < cats.length; i++) {
            cats[i] = inputs[i].getCategory();
            
            NbLexicalCorrectionRulesApplied += 
				inputs[i].getDerivationHistory().NbLexicalCorrectionRulesApplied;
			NbTypeShiftingRulesApplied += 
				inputs[i].getDerivationHistory().NbTypeShiftingRulesApplied;
			NbDiscLevelCompositionRulesApplied += 
				inputs[i].getDerivationHistory().NbDiscLevelCompositionRulesApplied;
			NbDisflCorrectionRulesApplied += 
				inputs[i].getDerivationHistory().NbDisflCorrectionRulesApplied; 
			
			FeatureStructure feats = cats[i].getTarget().getFeatureStructure();

			if (NbLexicalCorrectionRulesApplied < MAX_LEXICAL_CORRECTIONS && 
					feats != null && feats.hasAttribute("CORRECTED")) {
				if (asrcorrectionRules) {		
				inputs[i].getDerivationHistory().NbLexicalCorrectionRulesApplied = 1;
				NbLexicalCorrectionRulesApplied ++;
				}
				else { return; } 
			} 
			
			if (NbLexicalCorrectionRulesApplied > MAX_LEXICAL_CORRECTIONS) { 
				return ; }	 
		}
        
        if (disclevelcompositionRules) {
			if (name().equals("disclevelcomposition")) {
				if (NbDiscLevelCompositionRulesApplied > MAX_NB_COMPOSITIONS - 1) {
					return;
				}
				NbDiscLevelCompositionRulesApplied++;
			}
			else if (name().equals(">") && 
			 (NbDiscLevelCompositionRulesApplied > MAX_NB_COMPOSITIONS)) {
				return;
			}
		}
		
		
		if (disfluencycorrectionRules) {
			if (name().contains("correction")) {
				if (NbDisflCorrectionRulesApplied > MAX_DISFLUENCY_CORRECTIONS - 1) {
					return;
				}
				NbDisflCorrectionRulesApplied++;
			}
			else if (name().equals(">") && 
			 (NbDisflCorrectionRulesApplied > MAX_DISFLUENCY_CORRECTIONS)) {
				return;
			}
		} 
        

	
        try {
            List<Category> resultCats = applyRule(cats);
            if (resultCats.isEmpty()) return;
            
            for (Category catResult : resultCats) {
                distributeTargetFeatures(catResult);
                Sign sign = Sign.createDerivedSign(catResult, inputs, this);
                
				sign.getDerivationHistory().NbLexicalCorrectionRulesApplied = 
					NbLexicalCorrectionRulesApplied;
				sign.getDerivationHistory().NbTypeShiftingRulesApplied = 
					NbTypeShiftingRulesApplied;
				sign.getDerivationHistory().NbDiscLevelCompositionRulesApplied = 
					NbDiscLevelCompositionRulesApplied;
				sign.getDerivationHistory().NbDisflCorrectionRulesApplied = 
					NbDisflCorrectionRulesApplied; 
				
                results.add(sign);
            }
        } catch (UnifyFailure uf) {}
    }
    
    

    
    /** Propagates distributive features from target cat to the rest. */
    // nb: it would be nicer to combine inheritsFrom with $, but 
    //     this would be complicated, as inheritsFrom is compiled out
    protected void distributeTargetFeatures(Category cat) {
    	if (_ruleGroup == null) return;
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

