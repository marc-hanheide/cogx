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

import opennlp.ccg.synsem.Sign;
import opennlp.ccg.synsem.SignScorer;
import opennlp.ccg.grammar.Grammar;
import opennlp.ccg.lexicon.*;
import opennlp.ccg.util.*;

import java.util.*;
import java.io.*;

/**
 * Super class for n-gram scoring models.
 *
 * @author      Michael White
 * @version     $Revision: 1.27 $, $Date: 2007/12/21 05:13:37 $
 */
public abstract class NgramScorer implements SignScorer, Reversible
{
	protected NgramScorer() {}
	
	protected NgramScorer(int order) {
		this(order, false);
	}
	
	protected NgramScorer(int order, boolean useSemClasses) {
		this.order = order;
		this.useSemClasses = useSemClasses;
	}
	
    /** The n-gram order of the model. */
    protected int order;
    
    /** Returns the n-gram order of the model. */
    public int getOrder() { return order; }

    /** Flag for whether to reverse words before scoring (defaults to false). */
    protected boolean reverse = false;
    
    /** Get reverse flag. */
    public boolean getReverse() { return reverse; }
    
    /** Set reverse flag, and propagate to any reversible filters. */
    public void setReverse(boolean reverse) { 
        this.reverse = reverse; 
        if (ngramFilters != null) {
            for (int i = 0; i < ngramFilters.size(); i++) {
                NgramFilter filter = ngramFilters.get(i);
                if (filter instanceof Reversible) 
                    ((Reversible)filter).setReverse(reverse);
            }
        }
    }
    
    /** Root of the n-gram trie.  Nodes store NgramFloats instances. */
    protected TrieMap<Object,NgramFloats> trieMapRoot = new TrieMap<Object,NgramFloats>(null);

    /** An ngram data object, for holding the log prob and backoff weight. */
    public static class NgramFloats {
        /** The log prob. */
        public float logprob;
        /** The backoff weight. */
        public float bow;
        /** Constructor. */
        public NgramFloats(float logprob, float bow) {
            this.logprob = logprob; this.bow = bow; 
        }
        
        @Override
        public String toString() { return "logprob: " + logprob + ", bow: " + bow; }
    }

    
    /** The n-gram totals for different histories. */
    protected int[] numNgrams = null;
    
    /** Flag for open vocabulary, ie whether the unknown word &lt;unk&gt; is in the model. */
    protected boolean openVocab = false;
    
    /** Flag for whether to show scoring breakdown. */
    protected boolean debugScore = false;
    
    
    /** List of n-gram filters, for identifying unhappy sequences. */
    protected List<NgramFilter> ngramFilters = null;
    
    /** Adds an n-gram filter. */
    public void addFilter(NgramFilter filter) { 
        if (ngramFilters == null) { ngramFilters = new ArrayList<NgramFilter>(); }
        ngramFilters.add(filter);
    }

    
    
    /** Weak hash map for cached log probs, keyed from a sign's words. */
    protected Map<List,Float> cachedLogProbs = null;
    
    /** Reference to current sign to score. */
    protected Sign signToScore = null;
    
    /** Reusable list of words to score. */
    protected List<Word> wordsToScore = new ArrayList<Word>();

    /** Flag for whether start/end tags were added with the current words. */
    protected boolean tagsAdded = false;
    
    /** Reusable list of keys for n-gram lookups. */
    protected List<Object> keysList = new ArrayList<Object>();
    
    
    /** Gets a cached log prob for the given list of words (or null if none). */
    protected Float getCachedLogProb(List<Word> words) {
        if (cachedLogProbs == null) return null;
        return cachedLogProbs.get(words);
    }
    
    /** Caches a log prob for the given list of words. */
    protected void putCachedLogProb(List words, Float logprob) {
        if (cachedLogProbs == null) cachedLogProbs = new WeakHashMap<List,Float>();
        cachedLogProbs.put(words, logprob);
    }

    
    /** 
     * Returns a score between 0 (worst) and 1 (best) for the given sign 
     * and completeness flag, based on the n-gram score of the sign's words.
     * If the sign is complete, sentence delimiters are added before 
     * scoring the words, if not already present.
     * Returns 0 if any filter flags the n-gram for filtering, or if 
     * the sign has no words.
     * Otherwise, sets <code>signToScore</code>, calls <code>prepareToScoreWords</code>, 
     * and then returns the result of <code>score</code>.
     */
    public synchronized double score(Sign sign, boolean complete) {
        List<Word> words = sign.getWords(); 
        if (words == null) return 0;
        if (!complete) { // check cache
            Float logprob = getCachedLogProb(words);
            if (logprob != null) return convertToProb(logprob.floatValue());
        }
        signToScore = sign;
        setWordsToScore(words, complete);
        if (ngramFilters != null) {
            for (int i = 0; i < ngramFilters.size(); i++) {
                NgramFilter filter = ngramFilters.get(i);
                if (filter.filterOut(wordsToScore)) return 0;
            }
        }
        prepareToScoreWords();
        double retval = score();
        signToScore = null;
        return retval;
    }
    
    /** Sets wordsToScore to the given list, for sharing purposes. */
    protected void shareWordsToScore(List<Word> wordsToScore) {
        this.wordsToScore = wordsToScore;
    }
    
    /** Resets wordsToScore to the given ones, 
        reversing them when the reverse flag is true, 
        and adding sentence delimiters if not already present, 
        when the completeness flag is true. 
        Also sets the tagsAdded flag. */
    protected void setWordsToScore(List<Word> words, boolean complete) {
        wordsToScore.clear();
        tagsAdded = false; 
        if (complete && (reverse || words.get(0).getForm() != "<s>")) { 
            wordsToScore.add(Word.createWord("<s>"));
            tagsAdded = true;
        }
        if (reverse) {
            for (int j = words.size()-1; j >= 0; j--) {
                Word w = words.get(j);
                if (w.getForm() == "<s>" || w.getForm() == "</s>") continue; // skip <s> or </s>
                wordsToScore.add(w);
            }
        }
        else
            wordsToScore.addAll(words);
        if (complete && (reverse || words.get(words.size()-1).getForm() != "</s>")) {
            wordsToScore.add(Word.createWord("</s>"));
            tagsAdded = true;
        }
    }
    
    /** Optional step to do further preparation before scoring words. */
    protected void prepareToScoreWords() {}
    
    /** 
     * Returns a score between 0 (worst) and 1 (best) for the words 
     * in wordsToScore.
     * The default method returns the the probability of the word sequence 
     * as determined by this language model's <code>logProbFromNgram</code> method.
     * The probabilities for the first n-1 words are backed off to the  
     * lower order probabilities.
     * If the tagsAdded flag is false, the cache is checked to see whether 
     * the log prob of the words of signToScore's initial sign has already 
     * been calculated, and at the end the log prob of signToScore's words 
     * is stored in the cache.
     */
    protected double score() {
        float logProbTotal = 0;
        int numCached = 0;
        if (!tagsAdded) { // check cache for initial words
            Sign[] inputs = signToScore.getDerivationHistory().getInputs();
            if (inputs != null) {
                Sign initialSign = (!reverse) ? inputs[0] : inputs[inputs.length-1];
                List<Word> initialWords = initialSign.getWords();
                Float logprob = getCachedLogProb(initialWords);
                if (logprob != null) {
                    logProbTotal = logprob.floatValue();
                    numCached = initialWords.size();
                }
            }
        }
        for (int i = numCached; i < wordsToScore.size(); i++) {
            int orderToUse = Math.min(order, i+1);
            int startPos = i - (orderToUse-1);
            logProbTotal += logProbFromNgram(startPos, orderToUse);
        }
        if (!tagsAdded) { // add log prob to cache
            putCachedLogProb(signToScore.getWords(), new Float(logProbTotal));
        }
        return convertToProb(logProbTotal);
    }
    
    /** Returns the log prob of the ngram starting at the given index 
        in wordsToScore and with the given order, with backoff. */
    abstract protected float logProbFromNgram(int i, int order);
    
    
    /**
     * Flag whether to use sem classes in place of words.
     * Defaults to false.
     */
    protected boolean useSemClasses = false;
    
    // tokenizer reference
    private Tokenizer tokenizer = null;
    private Tokenizer getTokenizer() {
        if (tokenizer != null) return tokenizer;
        if (Grammar.theGrammar != null) tokenizer = Grammar.theGrammar.lexicon.tokenizer;
        else tokenizer = new DefaultTokenizer();
        return tokenizer;
    }
    
    /** Returns whether the given semantic class is a replacement one. */
    protected boolean isReplacementSemClass(String semClass) {
        return semClass != null && getTokenizer().isReplacementSemClass(semClass);
    }
    
    /** Returns the semantic class replacement value (the semantic class 
        uppercased and interned) for the given word, if apropos, otherwise null. */
    protected String semClassReplacement(Word w) {
        if (useSemClasses) {
            String semClass = w.getSemClass();
            if (isReplacementSemClass(semClass)) 
                return semClass.toUpperCase().intern();
        }
        // otherwise null
        return null;
    }
    
    
    /** Adds the TrieMap children, with their keys, under the given prefix, 
        then resets the lists. */
    protected void addTrieMapChildren(List<Object> prefix, List<Object> keys, List<TrieMap<Object,NgramFloats>> children) {
        if (!keys.isEmpty()) {
            TrieMap<Object,NgramFloats> prefixNode = trieMapRoot.findChildFromList(prefix);
            prefixNode.addChildren(keys, children);
        }
        prefix.clear(); keys.clear(); children.clear();
    }
    
    /** Returns the TrieMap node for the given sublist of keysList. */ 
    protected TrieMap<Object,NgramFloats> getNode(int pos, int len) {
        return trieMapRoot.getChildFromList(keysList.subList(pos, pos+len));
    }
    
    
    // from CMU-Cambridge Statistical Language Modeling Toolkit
    //
    // p(wd3|wd1,wd2)= if(trigram exists)           p_3(wd1,wd2,wd3)
                    // else if(bigram w1,w2 exists) bo_wt_2(w1,w2)*p(wd3|wd2)
                    // else                         p(wd3|w2)
    // 
    // p(wd2|wd1)= if(bigram exists) p_2(wd1,wd2)
                // else              bo_wt_1(wd1)*p_1(wd2)
    
    /** Returns the log prob (base 10) of the  given sublist of keysList, 
        with backoff, or -99 if not found. */
    protected float logProb(int pos, int len) {
        TrieMap<Object,NgramFloats> node = getNode(pos, len);
        if (node != null && node.data != null) return node.data.logprob;
        if (len == 1) return -99;
        float retval = logProb(pos+1, len-1);
        if (debugScore) System.out.print("(" + (len-1) + "-gram: " + retval + ") ");
        if (retval > -99) retval += backoffWeight(pos, len-1);
        return retval;
    }
    
    /** Returns the back-off weight (log base 10) of the given sublist of keysList, 
        or 0 if not found. */
    protected float backoffWeight(int pos, int len) {
        TrieMap<Object,NgramFloats> node = getNode(pos, len);
        if (node != null && node.data != null) {
            float retval = node.data.bow;
            // if (debugScore && retval != 0) System.out.print("(bow: " + retval + ") ");
            return retval;
        }
        return 0;
    }



    /**
     * Returns the rank order centroid weights for a ranked list of the given length. 
     * The weights go from highest to lowest, and sum to 1.
     */
    // ex:
    // weight 1 0.5208333333333333
    // weight 2 0.2708333333333333
    // weight 3 0.14583333333333331
    // weight 4 0.0625
    public static double[] rankOrderCentroidWeights(int length) {
        double[] retval = new double[length];
        for (int i = 0; i < length; i++) {
            double weight_i = 0;
            for (int j = i; j < length; j++) {
                weight_i += 1 / (double) (j+1);
            }
            weight_i = weight_i / (double) length;
            retval[i] = weight_i;
        }
        return retval;
    }
    
    
    /** Converts a base 10 log prob to an actual probability, checking for -99 (not found). */
    public static double convertToProb(double logProb) {
        if (logProb <= -99) { return 0; }
        else return Math.pow(10, logProb);
    }
    
    /** Converts a probability to a base 10 log prob, returning -99 if zero. */
    public static double convertToLogProb(double prob) {
        if (prob == 0) return -99;
        else return Math.log(prob) / Math.log(10);
    }
    
    /** Converts a base 10 log prob to the corresponding perplexity. */
    public static double convertToPPL(double logProb) {
        return Math.exp(- logProb * Math.log(10));
    }

    
    /** Sets up tokenizer for reading in language models. */ 
    protected static StreamTokenizer initTokenizer(Reader in) {
        StreamTokenizer tokenizer = new StreamTokenizer(in);
        tokenizer.resetSyntax();
        tokenizer.wordChars(0,255);
        tokenizer.whitespaceChars(' ',' ');
        tokenizer.whitespaceChars('\t','\t');
        tokenizer.whitespaceChars('\n','\n');
        tokenizer.whitespaceChars('\r','\r');
        tokenizer.eolIsSignificant(true);
        return tokenizer;
    }
    
    /** Reads a line of up to tokens.length tokens using the given tokenizer, with the remaining
        array elements set to null. */
    protected static void readLine(StreamTokenizer tokenizer, String[] tokens) throws IOException {
        int index = 0;
        int ttype;
        while ( (ttype = tokenizer.nextToken()) != StreamTokenizer.TT_EOF && ttype != StreamTokenizer.TT_EOL ) {
            if (index < tokens.length && ttype == StreamTokenizer.TT_WORD) { 
                tokens[index] = tokenizer.sval;
                index++;
            }
        }
        for (int i = index; i < tokens.length; i++) {
            tokens[i] = null;
        }
    }
}

