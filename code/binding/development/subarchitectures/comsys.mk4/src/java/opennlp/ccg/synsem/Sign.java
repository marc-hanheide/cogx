///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-5 Jason Baldridge and University of Edinburgh (Michael White)
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

package opennlp.ccg.synsem;

import opennlp.ccg.parse.*;
import opennlp.ccg.util.*;
import opennlp.ccg.lexicon.*;
import opennlp.ccg.grammar.*;
import opennlp.ccg.hylo.*;

import org.jdom.*;
import gnu.trove.*;
import java.util.*;

/**
 * A CCG sign, consisting of a list of words paired with a category.
 *
 * @author      Jason Baldridge
 * @author      Michael White
 * @version     $Revision: 1.27 $, $Date: 2008/01/03 21:30:12 $
 */
public class Sign implements LexSemOrigin {
    
    /** The words. */
    protected List<Word> _words;
    
    /** The category. */
    protected Category _cat;
    
    /** The derivation history. */
    protected DerivationHistory _history;
    
    public int nbUses = 0;
    public boolean locked = false;
    
    /** Interner for categories. */
    protected static Interner<Category> catInterner = null;
    
    /** Interns the given category, if cat interning enabled, otherwise just returns it. */
    public static Category internCat(Category cat) {
        return (catInterner != null) ? catInterner.intern(cat) : cat;
    }

    /** Resets and enables/disables cat interning according to the given flag.
        Interning categories speeds up the search during realization by using a 
        canonical category for equivalent ones, thereby speeding up hashing 
        and reducing memory demands.
        The interner must be reset for each realization request, to avoid 
        problems with UnifyControl's sequencing.
        Cat interning should be disabled for parsing (at present), since 
        there are no constants to distinguish categories for two instances of 
        the same lexical item, unlike in realization. */
    public static void resetCatInterner(boolean on) { catInterner = (on) ? new Interner<Category>() : null; }
    
    
    /** Constructor for subclasses. */
    protected Sign() {}
    
    /** Constructor with derivation history. */
    @SuppressWarnings("unchecked")
	protected Sign(List<Word> words, Category cat, DerivationHistory dh) {
        _words = (List) Interner.globalIntern(words); 
        _cat = internCat(cat);
        _history = dh;
    }

    /** Constructor with no additional derivation history. */
    public Sign(List<Word> words, Category cat) {
        this(words, cat, null);
        _history = new DerivationHistory(this);
    }

    /** Constructor with no additional derivation history. */
    public Sign(Word word, Category cat) {
        this(new SingletonList<Word>(word), cat);
    }
    
    /** Factory method for creating a sign from a lexical sign plus a coarticulation one. */
    public static Sign createCoartSign(Category cat, Sign lexSign, Sign coartSign) {
        List words = lexSign.getWords();
        if (words.size() > 1) 
            throw new RuntimeException("Can't create coarticulation sign from multiple words.");
        Word word = (Word) words.get(0);
        Word coartWord = (Word) coartSign.getWords().get(0);
        Word wordPlus = Word.createWordWithAttrs(word, coartWord);
        Sign retval = new Sign(new SingletonList<Word>(wordPlus), cat, null);
        Rule coartRule = new Rule() {
            public String name() { return "coart"; }
            public int arity() { return 1; }
            public List<Category> applyRule(Category[] inputs) { throw new RuntimeException("Not supported."); }
            public RuleGroup getRuleGroup() { throw new RuntimeException("Not supported."); }
            public void setRuleGroup(RuleGroup ruleGroup) { throw new RuntimeException("Not supported."); }
        };
        retval._history = new DerivationHistory(new Sign[]{lexSign,coartSign}, retval, coartRule);
        return retval;
    }
    
    /** Factory method for creating derived signs with the given cat from the given inputs and rule. */
    public static Sign createDerivedSign(Category cat, Sign[] inputs, Rule rule) {
        return new Sign(cat, inputs, rule);
    }

    /** Factory method for creating derived signs from the given result cat, inputs and rule, 
        with a new LF constructed from the inputs.
        Note that unlike with rule applications, the result LF is constructed with 
        no var substitutions, so it is useful only for creating alternative signs during realization. */
    public static Sign createDerivedSignWithNewLF(Category cat, Sign[] inputs, Rule rule) {
        Category copyCat = cat.shallowCopy();
        LF lf = null;
        for (int i = 0; i < inputs.length; i++) {
            lf = HyloHelper.append(lf, inputs[i].getCategory().getLF());
        }
        if (rule instanceof TypeChangingRule) {
            TypeChangingRule tcr = (TypeChangingRule) rule;
            lf = HyloHelper.append(lf, tcr.getResult().getLF());
        }
        if (lf != null) { HyloHelper.sort(lf); }
        copyCat.setLF(lf);
        Category resultCat = internCat(copyCat);
        return new Sign(resultCat, inputs, rule);
    }
        
    /** Constructor with words and derivation history formed from the given inputs and rule. */
    protected Sign(Category cat, Sign[] inputs, Rule rule) {
        this(getRemainingWords(inputs, 0), cat, null);
        _history = new DerivationHistory(inputs, this, rule);
    }
    
    // returns the remaining words in a structure sharing way
    private static List<Word> getRemainingWords(Sign[] inputs, int index) {
        // if (inputs.length == 0) throw new RuntimeException("Error: can't make sign from zero inputs");
        if (index == (inputs.length - 1)) return inputs[index]._words;
        return new StructureSharingList<Word>(
            inputs[index]._words,
            getRemainingWords(inputs, index+1)
        );
    }

    
    /** Returns the words of the sign. */
    public List<Word> getWords() {
        return _words;
    }

    /** Returns the words as a string.  Delegates to the current tokenizer's getOrthography method. */
    public String getOrthography() {
        return Grammar.theGrammar.lexicon.tokenizer.getOrthography(_words);
    }

    /** Returns the sign's category. */
    public Category getCategory() {
        return _cat;
    }

    /** Sets the derivation history. */
    public void setDerivationHistory(DerivationHistory dh) {
        _history = dh;
    }
    
    /** Returns the derivation history. */
    public DerivationHistory getDerivationHistory() {
        return _history;
    }

    
    /** Returns a hash code for this sign. */ 
    public int hashCode() {
        return System.identityHashCode(_words) + _cat.hashCode();
    }
    
    /** Returns whether this sign equals the given object. */
    public boolean equals(Object obj) {
        if (obj == this) return true;
        if (!(obj instanceof Sign)) return false;
        Sign sign = (Sign) obj;
        return _words == sign._words && _cat.equals(sign._cat);
    }

    
    /** Returns a hash code for this sign with the words restricted to surface words. */ 
    public int surfaceWordHashCode() {
        int hc = 1;
        for (int i = 0; i < _words.size(); i++) {
            Word word = (Word) _words.get(i);
            hc = 31*hc + word.surfaceWordHashCode();
        }
        hc += _cat.hashCode();
        return hc;
    }
    
    /** Returns whether this sign and the given object have equal categories and 
        restrictions to surface words. */
    public boolean surfaceWordEquals(Object obj) {
        if (obj == this) return true;
        if (!(obj instanceof Sign)) return false;
        Sign sign = (Sign) obj;
        if (_words.size() != sign._words.size()) return false;
        for (int i = 0; i < _words.size(); i++) {
            Word word = (Word) _words.get(i); 
            Word signWord = (Word) sign._words.get(i);
            if (!word.surfaceWordEquals(signWord)) return false;
        }
        return _cat.equals(sign._cat);
    }

    
    /** Returns 'orthography :- category'. */
    public String toString() {
        return getOrthography() + " :- " + _cat.toString();
    }
 
    
    /** 
     * Returns the words in an XML doc, with no labeled spans for nominals. 
     */
    public Document getWordsInXml() { return getWordsInXml(Collections.EMPTY_SET); }
    
    /** 
     * Returns the words in an XML doc, with labeled spans for the given nominals, 
     * and with pitch accents and boundary tones converted to elements. 
     * Each orthographic word appears in a separate element, 
     * with multiwords grouped under a multiword element.
     * Attribute-value pairs for the word (if any) appear on the word 
     * or multiword element.
     * Words are also expanded using the grammar's tokenizer.
     */
    public Document getWordsInXml(Set nominals) {
        TObjectIntHashMap nominalsMap = new TObjectIntHashMap(); 
        setMaxOrthLengths(nominals, nominalsMap);
        Document doc = new Document();
        Element root = new Element("seg");
        doc.setRootElement(root);
        addWordsToXml(root, nominalsMap);
        return doc;
    }
    
    // finds the maximum orthography lengths for signs headed by the given nominals
    private void setMaxOrthLengths(Set nominals, TObjectIntHashMap nominalsMap) {
        // update map
        Nominal index = _cat.getIndexNominal();
        if (index != null && nominals.contains(index)) {
            int orthLen = getOrthography().length();
            if (!nominalsMap.containsKey(index) || orthLen > nominalsMap.get(index)) {
                nominalsMap.put(index, orthLen);
            }
        }
        // recurse
        Sign[] inputs = _history.getInputs();
        if (inputs == null) return;
        for (int i = 0; i < inputs.length; i++) {
            inputs[i].setMaxOrthLengths(nominals, nominalsMap); 
        }
    }
    
    // recursively adds orthographic words as XML to the given parent, 
    // using the nominals map to determine labeled spans
    private void addWordsToXml(Element parent, TObjectIntHashMap nominalsMap) {
        // check for matching nominal as index of target cat; 
        // if found, update parent to labeled span element
        Nominal index = _cat.getIndexNominal();
        if (index != null && nominalsMap.containsKey(index) && 
            nominalsMap.get(index) == getOrthography().length()) 
        {
            // remove index key from map, to avoid duplicate spans with the same length
            nominalsMap.remove(index);
            // make span element, update parent
            Element span = new Element("span");
            span.setAttribute("label", index.toString());
            parent.addContent(span);
            parent = span;
        }
        // process inputs from derivation history
        Sign[] inputs = _history.getInputs();
        if (inputs == null) {
            // in leaf case, word list must be a singleton
            Word word = (Word) _words.get(0); 
            // check for boundary tone
            if (Grammar.isBoundaryTone(word.getForm())) {
                // add element for boundary tone
                Element boundary = new Element("boundary");
                boundary.setAttribute("type", word.getForm());
                parent.addContent(boundary);
                return;
            }
            // check for pitch accent
            if (word.getPitchAccent() != null) {
                // add pitchaccent element containing word(s) with corresponding accent
                Element pitchaccent = new Element("pitchaccent");
                pitchaccent.setAttribute("type", word.getPitchAccent());
                addWords(pitchaccent, word);
                parent.addContent(pitchaccent);
                return;
            }
            // otherwise add word(s)
            addWords(parent, word);
            return;
        }
        if (inputs.length == 1) {
            inputs[0].addWordsToXml(parent, nominalsMap);
            return;
        }
        for (int i = 0; i < inputs.length; i++) {
            inputs[i].addWordsToXml(parent, nominalsMap);
        }
    }
    
    // adds one or more word elements after expanding surface form; 
    // multiwords are enclosed within a multiword element; 
    // any attribute-value pairs are added to the word or multiword element
    private void addWords(Element parent, Word word) {
        List orthWords = Grammar.theGrammar.lexicon.tokenizer.expandWord(word);
        Element child;
        if (orthWords.size() == 1) {
            Element wordElt = new Element("word");
            wordElt.addContent((String)orthWords.get(0));
            child = wordElt;
        }
        else {
            Element multiwordElt = new Element("multiword");
            for (int i = 0; i < orthWords.size(); i++) {
                Element wordElt = new Element("word");
                wordElt.addContent((String)orthWords.get(i));
                multiwordElt.addContent(wordElt);
            }
            child = multiwordElt;
        }
        for (Iterator it = word.getAttrValPairs(); it.hasNext(); ) {
            Pair p = (Pair) it.next();
            String attr = (String) p.a; String val = (String) p.b;
            child.setAttribute(attr, val);
        }
        parent.addContent(child);
    }
    

    /**
     * Returns a string showing the bracketings implied by the derivation.
     * See DerivationHistory.toString to see the complete derivation in 
     * vertical list form.
     */
    public String getBracketedString() {
        Sign[] inputs = _history.getInputs();
        if (inputs == null) return getOrthography();
        if (inputs.length == 1) return inputs[0].getBracketedString();
        StringBuffer sb = new StringBuffer();
        sb.append("(");
        for (int i = 0; i < inputs.length; i++) {
            sb.append(inputs[i].getBracketedString());
            if (i < (inputs.length - 1)) sb.append(" ");
        }
        sb.append(")");
        return sb.toString();
    }
    
    /**
     * Returns the category's supertag.
     */
    public String getSupertag() { return _cat.getSupertag(); }
    
    /**
     * Returns the POS tag of the first word.
     */
    public String getPOS() { return _words.get(0).getPOS(); }
    
    /**
     * Sets the origin of the elementary predications.
     */
    public void setOrigin() { HyloHelper.setOrigin(_cat.getLF(), this); }
}
