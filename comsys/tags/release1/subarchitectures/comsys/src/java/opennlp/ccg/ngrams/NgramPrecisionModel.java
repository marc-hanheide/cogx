///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-5 University of Edinburgh (Michael White)
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

package opennlp.ccg.ngrams;

import opennlp.ccg.grammar.*;
import opennlp.ccg.lexicon.*;
import opennlp.ccg.util.*;

import gnu.trove.*;
import java.util.*;

/**
 * N-gram precision scoring model, using a linear combination of 
 * n-grams with rank order centroid weights, and optionally replacing word forms with 
 * their semantic classes.
 * Words in the target strings are assumed to contain any desired delimiters.
 *
 * @author      Michael White
 * @version     $Revision: 1.11 $, $Date: 2005/10/20 18:49:42 $
 */
public class NgramPrecisionModel extends NgramScorer
{
    // n-grams in the target phrases
    @SuppressWarnings("unchecked")
	private Set<List<Word>> targetNgrams = new THashSet();
    
    // weights
    private double[] weights = null;
    
    /** Reusable list of reduced words. */
    protected List<Word> reducedWords = new ArrayList<Word>();
    
    /** Reusable word list, with identity equals. */
    protected List<Word> wordList = new ArrayListWithIdentityEquals<Word>();

    
    /**
     * Creates a new 4-gram precision model from the given target strings 
     * and with the combination weights determined by the rank order centroid method.  
     * Word forms are not replaced by their semantic classes.
     */
    public NgramPrecisionModel(String[] targets) { 
        this(targets, false);
    }
    
    /**
     * Creates a new n-gram precision model of the given order from the given target strings   
     * and with the combination weights determined by the rank order centroid method.  
     * Word forms are not replaced by their semantic classes.
     */
    public NgramPrecisionModel(String[] targets, int order) { 
        this(targets, order, false);
    }
    
    /**
     * Creates a new 4-gram precision model from the given target strings,  
     * with the given flag controlling whether word forms are replaced by their semantic classes, 
     * and with the combination weights determined by the rank order centroid method.  
     */
    public NgramPrecisionModel(String[] targets, boolean useSemClasses) { 
        this(targets, 4, useSemClasses);
    }
    
    /**
     * Creates a new n-gram precision model of the given order from the given target strings,  
     * with the given flag controlling whether word forms are replaced by their semantic classes, 
     * and with the combination weights determined by the rank order centroid method.  
     */
    public NgramPrecisionModel(String[] targets, int order, boolean useSemClasses) { 
        this(targets, order, useSemClasses, rankOrderCentroidWeights(order));
    }
    
    /**
     * Creates a new n-gram precision model of the given order from the given target strings,  
     * with the given flag controlling whether word forms are replaced by their semantic classes, 
     * and with the given combination weights, beginning with the 
     * highest-order weight and ending with the lowest-order (unigram) weight.
     */
    public NgramPrecisionModel(String[] targets, int order, boolean useSemClasses, double[] weights) {
        this.useSemClasses = useSemClasses;
        this.order = order;
        this.weights = new double[order];
        for (int i = 0; i < order; i++) {
            this.weights[order-(i+1)] = weights[i];
        }
        initTargetNgrams(targets);
    }
    
    /** Reduces the words in wordsToScore to reducedWords, before scoring. */
    protected void prepareToScoreWords() {
        reducedWords.clear();
        for (int i = 0; i < wordsToScore.size(); i++) {
            Word w = wordsToScore.get(i);
            reducedWords.add(reduceWord(w));
        }
    }
    
    /** Returns the given word reduced to a surface word, using the sem class, if apropos. */
    protected Word reduceWord(Word w) {
        if (useSemClasses && isReplacementSemClass(w.getSemClass())) 
            return Word.createSurfaceWordUsingSemClass(w);
        else return Word.createSurfaceWord(w);
    }
    
    /** 
     * Returns a score between 0 (worst) and 1 (best) for the words in wordsToScore.
     * In particular, returns the linear combination using the established weights 
     * of the various n-gram precision scores (from unigram up to the configured order), 
     * where the n-gram precision is the number of n-grams with a match in the target 
     * strings divided by the number of n-grams in the word sequence.
     * (Assumes words in wordsToScore have already been reduced into reducedWords, 
     * via call to prepareToScoreWords.)
     */
    protected double score() {
        // calc weighted precision score
        double retval = 0;
        for (int i = 0; i < order; i++) {
            retval += weights[i] * ngramPrecision(i+1);
        }
        return retval;
    }
    
    /** Not supported; throws an UnsupportedOperationException. */
    protected float logProbFromNgram(int i, int order) {
        throw new UnsupportedOperationException();
    }
    
    
    // returns the n-gram precision of the given order, or zero if too few words
    private double ngramPrecision(int order) {
        int numWords = reducedWords.size();
        int numNgrams = numWords - (order-1);
        if (numNgrams <= 0) return 0;
        int matches = 0;
        for (int i=0; i < numNgrams; i++) {
            setNgram(reducedWords, i, order);
            if (targetNgrams.contains(wordList))
                matches++;
        }
        return (matches * 1.0) / numNgrams;
    }

    /** Sets wordList to be the n-gram of the given order using words starting at pos i. */
    protected synchronized void setNgram(List<Word> words, int i, int order) {
        wordList.clear();
        for (int j = 0; j < order; j++) {
            wordList.add(words.get(i+j)); 
        }        
    }
    
    /** Makes a canonical n-gram of the given order using words starting at pos i. 
        Sublists are shared, a la a trie data structure. */
    @SuppressWarnings("unchecked")
	protected List<Word> makeNgram(List<Word> words, int i, int order) {
        // check for one already interned
        setNgram(words, i, order);
        List<Word> alreadyInterned = (List<Word>) Interner.getGlobalInterned(wordList);
        if (alreadyInterned != null) return alreadyInterned;
        // if order is 1, intern new singleton list
        if (order == 1) {
            return (List<Word>) Interner.globalIntern(new SingletonList<Word>(words.get(i)));
        }
        // otherwise, extend list for the first word with suffix list
        List<Word> firstOneList = makeNgram(words, i, 1);
        List<Word> suffixList = makeNgram(words, i+1, order-1); 
        return (List<Word>) Interner.globalIntern(new StructureSharingList<Word>(firstOneList, suffixList));
    }
    

    // initializes the n-grams from the target phrases
    private void initTargetNgrams(String[] targets) {
        for (int j = 0; j < targets.length; j++) {
            // parse or tokenize target phrase into words
            List<Word> words;
            if (useSemClasses) // use parsed words to get sem classes
                words = Grammar.theGrammar.getParsedWords(targets[j]);
            else
                words = Grammar.theGrammar.lexicon.tokenizer.tokenize(targets[j]);
            // add sentence delimiters, if not already present
            setWordsToScore(words, true);
            // reduce each word to a surface word, using the sem class if apropos
            int numWords = wordsToScore.size();
            for (int i = 0; i < numWords; i++) {
                Word w = wordsToScore.get(i);
                wordsToScore.set(i, reduceWord(w));
            }
            // make and store target n-grams
            for (int k=0; k < order; k++) {
                for (int i=0; i < numWords - k; i++) {
                    targetNgrams.add(makeNgram(wordsToScore, i, k+1));
                }
            }
        }
    }
}

