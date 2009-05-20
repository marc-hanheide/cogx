///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-4 Jason Baldridge, Gann Bierner and 
//                      University of Edinburgh (Michael White)
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

import opennlp.ccg.synsem.*;
import opennlp.ccg.hylo.*;
import opennlp.ccg.unify.*;
import opennlp.ccg.util.*;

import org.jdom.*;
import org.jdom.input.*;
//import org.jdom.output.*;

import java.io.*;
import java.net.*;
import java.util.*;

/**
 * A set of rules for combining categories.
 *
 * @author      Jason Baldridge
 * @author      Gann Bierner
 * @author      Michael White
 * @version     $Revision: 1.16 $, $Date: 2005/10/13 20:49:22 $
 */
public class RuleGroup {

    /** The grammar that this rule group is part of. */
    public final Grammar grammar;
    
    // rules
    protected List<Rule> unaryRules = new ArrayList<Rule>();
    protected List<Rule> binaryRules = new ArrayList<Rule>();

    // maps of type changing rules by their semantics
    private GroupMap<String,TypeChangingRule> predsToRules = new GroupMap<String,TypeChangingRule>();
    private GroupMap<String,TypeChangingRule> relsToRules = new GroupMap<String,TypeChangingRule>();
    
    // rule for use in applying coarticulations
    private BackwardApplication bapp = new BackwardApplication();
    
    
    /**
     * Constructs an empty rule group for the given grammar.
     */
    public RuleGroup(Grammar grammar) {
        this.grammar = grammar;
        bapp.setRuleGroup(this);
    }
    
    /**
     * Constructs a rule group from the given URL, for 
     * the given grammar.
     */
    public RuleGroup(URL url, Grammar grammar) throws IOException {

        this.grammar = grammar;
        bapp.setRuleGroup(this);
        
        SAXBuilder builder = new SAXBuilder();
        Document doc; 
        try {
            doc = builder.build(url); 
        }
        catch (JDOMException exc) {
            throw (IOException) new IOException().initCause(exc);
        }
        List entries = doc.getRootElement().getChildren();
        
        for (int i=0; i < entries.size(); i++) {
            Element ruleEl = (Element)entries.get(i);
            String active = ruleEl.getAttributeValue("active");
            if (active == null || active.equals("true")) {
                Rule r = readRule(ruleEl);
                addRule(r);
            }
        }
    }

    // reads in a rule
    private Rule readRule(Element ruleEl) throws IOException {
        Rule r;
        String type = ruleEl.getName();
        if (type.equals("application")) {
            String dir = ruleEl.getAttributeValue("dir");
            if (dir.equals("forward")) {
                r = new ForwardApplication();
            } else {
                r = new BackwardApplication();
            }
        } else if (type.equals("composition")) {
            String dir = ruleEl.getAttributeValue("dir");
            String harmonic = ruleEl.getAttributeValue("harmonic");
            boolean isHarmonic = new Boolean(harmonic).booleanValue();
            if (dir.equals("forward")) {
                r = new ForwardComposition(isHarmonic);
            } else {
                r = new BackwardComposition(isHarmonic);
            }
        } else if (type.equals("substitution")) {
            String dir = ruleEl.getAttributeValue("dir");
            String harmonic = ruleEl.getAttributeValue("harmonic");
            boolean isHarmonic = new Boolean(harmonic).booleanValue();
            if (dir.equals("forward")) {
                r = new ForwardSubstitution(isHarmonic);
            } else {
                r = new BackwardSubstitution(isHarmonic);
            }
        } else if (type.equals("typeraising")) {
            String dir = ruleEl.getAttributeValue("dir");
            String useDollar = ruleEl.getAttributeValue("useDollar");
            boolean addDollar = new Boolean(useDollar).booleanValue();
            Category arg = null;
            Element argElt = ruleEl.getChild("arg");
            if (argElt != null) {
                arg = CatReader.getCat((Element)argElt.getChildren().get(0));
            }
            Category result = null;
            Element resultElt = ruleEl.getChild("result");
            if (resultElt != null) {
                result = CatReader.getCat((Element)resultElt.getChildren().get(0));
            }
            if (dir.equals("forward")) {
                r = new ForwardTypeRaising(addDollar, arg, result);
            } else {
                r = new BackwardTypeRaising(addDollar, arg, result);
            }
        } else if (type.equals("typechanging")) {
            r = readTypeChangingRule(ruleEl);
        } else {
            throw (IOException) new IOException().initCause(
                new JDOMException("Invalid element in rules: " + type)
            );
        }
        return r;
    }
    
    // reads in a type changing rule
    private Rule readTypeChangingRule(Element ruleEl) {
        
        String rname = ruleEl.getAttributeValue("name");
        Element argCatElt = (Element)ruleEl.getChild("arg").getChildren().get(0);
        Category arg = CatReader.getCat(argCatElt);
        Element resultCatElt = (Element)ruleEl.getChild("result").getChildren().get(0);
        Element lfElt = resultCatElt.getChild("lf");
        Category result = CatReader.getCat(resultCatElt);
        LF firstEP = null;
        if (lfElt != null) {
            firstEP = HyloHelper.firstEP(HyloHelper.getLF(lfElt));
        }
        
        grammar.lexicon.propagateTypes(result, arg);
        grammar.lexicon.propagateDistributiveAttrs(result, arg);
        grammar.lexicon.expandInheritsFrom(result, arg);

        return new TypeChangingRule(arg, result, rname, firstEP);
    }

    
    /** Adds the given rule. */
    public void addRule(Rule r) {
        r.setRuleGroup(this);
        if (r instanceof TypeChangingRule) {
            unaryRules.add(r);
            index((TypeChangingRule)r);
        }
        else if (r.arity() == 1) { unaryRules.add(r); } 
        else if (r.arity() == 2) { binaryRules.add(r); } 
        else {
            // shouldn't happen
            throw new RuntimeException("Can't determine arity of rule: " + r);
        }
    }

    // indexes type changing rules by preds and rels
    private void index(TypeChangingRule rule) {
        LF firstEP = rule.getFirstEP();
        if (firstEP == null) { return; }
        String pred = HyloHelper.getLexPred(firstEP);
        if (pred != null) { 
            predsToRules.put(pred, rule); 
            return; 
        }
        String rel = HyloHelper.getRel(firstEP);
        if (rel != null) { 
            relsToRules.put(rel, rule);
        }
    }
    
    
    /** Returns the unary rules. */
    public List<Rule> getUnaryRules() { return unaryRules; }

    /** Returns the binary rules. */
    public List<Rule> getBinaryRules() { return binaryRules; }

    /** Returns the type changing rule with the given name, or null if none. */
    public TypeChangingRule getTypeChangingRule(String name) {
        for (Iterator<Rule> it = unaryRules.iterator(); it.hasNext(); ) {
            Object rule = it.next();
            if (rule instanceof TypeChangingRule) {
                TypeChangingRule tcr = (TypeChangingRule) rule;
                if (tcr.name().equals(name)) return tcr;
            }
        }
        return null;
    }
    
    /**
     * Returns the type changing rules indexed by the given lexical predicate. 
     * The type changing rules are indexed by their first elementary predication.
     */
    public Collection<TypeChangingRule> getRulesForPred(String pred) {
        return predsToRules.get(pred);
    }
    
    /**
     * Returns the type changing rules indexed by the given relation.
     * The type changing rules are indexed by their first elementary predication.
     */
    public Collection<TypeChangingRule> getRulesForRel(String rel) {
        return relsToRules.get(rel);
    }
    
    
    /** Applies the rules to given inputs, returning the list of results. */
    public List<Sign> applyRules(Sign[] inputs) {
        if (inputs.length != 1 && inputs.length != 2) {
            // shouldn't happen
            throw new RuntimeException("Inputs must have length 1 or 2");
        }

        List<Sign> results = new ArrayList<Sign>(2);
        Iterator<Rule> ruleIt = null;
        
        if (inputs.length == 1) {
            ruleIt = unaryRules.iterator();
        } else if (inputs.length == 2) {
            ruleIt = binaryRules.iterator();
        }

        for (; ruleIt.hasNext();) {
            AbstractRule ruleToUse = (AbstractRule)ruleIt.next();
            ruleToUse.applyRule(inputs, results);
        }
              
        return results;
    }
    
    
    /** Applies the coarticulation to the given sign, adding the result (if any) to the given ones. */
    public void applyCoart(Sign lexSign, Sign coartSign, List<Sign> results) {

        Category[] cats = new Category[] { lexSign.getCategory(), coartSign.getCategory() }; 

        try {
            List resultCats = bapp.applyRule(cats);
            if (resultCats.isEmpty()) return;
            
            for (Iterator it = resultCats.iterator(); it.hasNext();) {
                Category catResult = (Category) it.next();
                bapp.distributeTargetFeatures(catResult);
                Sign sign = Sign.createCoartSign(catResult, lexSign, coartSign);
                results.add(sign);
            }
        } catch (UnifyFailure uf) {}
    }
}
