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

package opennlp.ccg.hylo;

import opennlp.ccg.synsem.*;
import opennlp.ccg.unify.*;
import opennlp.ccg.util.*;
import opennlp.ccg.grammar.*;
import opennlp.ccg.lexicon.Lexicon;
import org.jdom.*;

import java.util.*;

import gnu.trove.*;

/**
 * A utility class to help with certain global operations over hybrid logic
 * terms.
 *
 * @author      Jason Baldridge
 * @author      Michael White
 * @version     $Revision: 1.36 $, $Date: 2005/12/08 02:35:03 $
 **/
public class HyloHelper {

    //-----------------------------------------------------------------
    // XML functions
    
    /** 
     * Builds a Hylo term from the given element.
     * An "lf" element may be used to wrap one or more (implicitly conj-ed) terms.
     */
    public static LF getLF(Element e) {
        LF retval = null;
        String type = e.getName();
        if (type.equals("op")) {
            retval = new Op(e);
        } else if (type.equals("var")) {
            String name = getName(e);
            retval = new HyloVar(prefix(name), type(name));
        } else if (type.equals("nomvar")) {
            String name = getName(e); 
            boolean shared = "true".equals(e.getAttributeValue("shared"));
            retval = new NominalVar(prefix(name), type(name), shared);
        } else if (type.equals("nom")) {
            String name = getName(e);
            boolean shared = "true".equals(e.getAttributeValue("shared"));
            retval = new NominalAtom(prefix(name), type(name), shared);
        } else if (type.equals("prop")) {
            String name = getName(e);
            retval = new Proposition(name, existingType(name));
        } else if (type.equals("satop")) {
            retval = new SatOp(e);
        } else if (type.equals("box") || type.equals("b")) {
            retval = new Box(e);
        } else if (type.equals("diamond") || type.equals("d")) {
            retval = new Diamond(e);
        } else if (type.equals("mode")) {
            String name = getName(e);
            retval = new ModeLabel(name);
        } else if (type.equals("modevar")) {
            String name = getName(e);
            retval = new ModeVar(name);
        } else if (type.equals("lf")) {
            retval = getLF_FromChildren(e);
        } else {
            System.out.println("Invalid hybrid logic LF type: " + type);
        }
        // assign chunks
        if (retval != null) {
            String chunks = e.getAttributeValue("chunks");
            if (chunks != null) {
                retval.setChunks(convertChunks(chunks));
            }
        }
        // done
        return retval;
    }

    // returns the value of the attribute 'name' or 'n'
    private static String getName(Element e) { 
        String name = e.getAttributeValue("name");
        if (name == null) name = e.getAttributeValue("n");
        return name;
    }
    
    // returns the simple type with the given name, if it exists, or null if not
    private static SimpleType existingType(String name) {
        Types types = Grammar.theGrammar.types;
        if (types.containsSimpleType(name)) return types.getSimpleType(name);
        else return null;
    }
    
    /** Returns the prefix of the name, up to an optional colon. */
    protected static String prefix(String name) {
        int index = name.indexOf(":");
        if (index >= 0) return name.substring(0, index);
        else return name;
    }

    /** Returns the simple type given by the suffix of the name after the colon, or null if none. */
    protected static SimpleType type(String name) {
        int index = name.indexOf(":");
        String suffix = (index >=0 && index+1 < name.length()) ? name.substring(index+1) : null;
        if (suffix != null) return Grammar.theGrammar.types.getSimpleType(suffix);
        else return null;
    }
    
        
    /**
     * Returns a Hylo term from the children of the given element, 
     * adding an implicit CONJ op if necessary.
     */
    @SuppressWarnings("unchecked")
	public static LF getLF_FromChildren(Element e) {
        List<Element> children = e.getChildren();
        if (children.size() > 1) {
            List<LF> preds = new ArrayList<LF>(children.size());
            for (int i=0; i < children.size(); i++) {
                preds.add(getLF(children.get(i)));
            }
            Op conj = new Op(Op.CONJ, preds);
            return conj;
        }
        else return getLF(children.get(0));
    }

    /**
     * Returns an XML representation of the given LF, 
     * wrapped with an 'lf' element, 
     * removing CONJ ops that may be left implicit.
     */
    public static Element toXml(LF lf) {
        Element retval = new Element("lf");
        retval.addContent(lf.toXml());
        removeConjOps(retval);
        return retval;
    }

    
    //-----------------------------------------------------------------
    // process chunks
    
    /** 
     * Processes and removes any chunk elements.
     * Each chunk element is numbered, and all contained elements are marked 
     * as being contained by this chunk, via a "chunks" attribute.
     */
    public static void processChunks(Element e) {
        processChunks(e, null, 0);
        removeChunkElts(e);
    }
    
    // recursively processes chunks, threading count through calls
    @SuppressWarnings("unchecked")
	private static int processChunks(Element e, String chunks, int count) {
        // check for chunk
        if (e.getName().equals("chunk")) {
            // update chunks string and counter
            if (chunks == null) { chunks = "" + count; }
            else { chunks += " " + count; }
            count++;
        }
        // otherwise add chunks attr, if val non-null
        else if (chunks != null) {
            e.setAttribute("chunks", chunks);
        }
        // do children
        List<Element> children = e.getChildren();
        for (int i=0; i < children.size(); i++) {
            count = processChunks(children.get(i), chunks, count);
        }
        // return current count
        return count;
    }
    
    // converts chunk strings
    private static TIntArrayList convertChunks(String chunks) {
        String[] tokens = chunks.split("\\s+");
        TIntArrayList retval = new TIntArrayList(tokens.length);
        for (int i = 0; i < tokens.length; i++) {
            retval.add(Integer.parseInt(tokens[i]));
        }
        return retval;
    }
    
    //-----------------------------------------------------------------
    // recursively remove certain elements
    
    private static abstract class ElementTest {
        abstract boolean test(Element elt);
    }

    // recursively removes elements meeting given test
    @SuppressWarnings("unchecked")
	private static void removeElts(Element elt, ElementTest eltTest) {
        // nb: need to dump children into a new list, in order to get a list iterator 
        //     that will allow multiple adds
        List<Element> children = elt.getChildren();
        List<Element> newChildren = new ArrayList<Element>(children.size());
        newChildren.addAll(children);
        for (ListIterator<Element> li = newChildren.listIterator(); li.hasNext(); ) {
            Element nextElt = li.next();
            removeElts(nextElt, eltTest);
            if (eltTest.test(nextElt)) {
                li.remove();
                for (Iterator<Element> it = nextElt.getChildren().iterator(); it.hasNext(); ) {
                    Element childElt = it.next();
                    it.remove(); // removes childElt from nextElt's children, so it can become a child of elt
                    li.add(childElt);
                }
            }
        }
        elt.removeContent(); 
        elt.setContent(newChildren); 
    }
    
    // recursively removes conj ops
    private static void removeConjOps(Element lfElt) {
        removeElts(
            lfElt, 
            new ElementTest() {
                boolean test(Element elt) {
                    return elt.getName().equals("op") && 
                           elt.getAttributeValue("name").equals(Op.CONJ);
                }
            }
        );
    }
    
    // recursively removes chunk elements
    private static void removeChunkElts(Element lfElt) {
        removeElts(
            lfElt, 
            new ElementTest() {
                boolean test(Element elt) {
                    return elt.getName().equals("chunk");
                }
            }
        );
    }
    

    //-----------------------------------------------------------------
    // functions for elementary predications

    /**
     * Returns whether the given LF is an elementary predication, 
     * ie a lexical predication, relation predication or attribute-value predication.
     */
    public static boolean isElementaryPredication(LF lf) {
        return isLexPred(lf) || isRelPred(lf) || isAttrPred(lf);
    }
    
    /**
     * Returns whether the given elementary predication is a lexical predication, 
     * ie one of the form @x(prop).
     */
    public static boolean isLexPred(LF pred) {
        if (!(pred instanceof SatOp)) return false;
        SatOp satOp = (SatOp) pred;
        LF arg = satOp.getArg();
        return (arg instanceof Proposition);
    }

    /**
     * Returns whether the given elementary predication is a relation predication, 
     * ie one of the form @x(&lt;Rel&gt;y).
     */
    public static boolean isRelPred(LF pred) {
        if (!(pred instanceof SatOp)) return false;
        SatOp satOp = (SatOp) pred;
        LF arg = satOp.getArg();
        if (!(arg instanceof Diamond)) return false;
        Diamond d = (Diamond) arg;
        return (d.getArg() instanceof Nominal);
    }

    /**
     * Returns whether the given elementary predication is an attribute-value predication, 
     * ie one of the form @x(&lt;Rel&gt;prop).  Note that the prop is also allowed to be 
     * a HyloVar.
     */
    public static boolean isAttrPred(LF pred) {
        if (!(pred instanceof SatOp)) return false;
        SatOp satOp = (SatOp) pred;
        LF arg = satOp.getArg();
        return isAttr(arg);
    }
    
    /**
     * Returns whether the given arg is an attribute-value pair, 
     * ie one of the form &lt;Rel&gt;prop.  Note that the prop is also allowed to be 
     * a HyloVar.
     */
    public static boolean isAttr(LF arg) {
        if (!(arg instanceof Diamond)) return false;
        Diamond d = (Diamond) arg;
        LF dArg = d.getArg();
        return ( dArg instanceof Proposition || 
                 (dArg instanceof HyloVar && !(dArg instanceof NominalVar)) );
    }
    
    /**
     * Returns the name of the lexical predicate of the given elementary predication, 
     * or null, if the given LF is not a lexical predicate.
     */
    public static String getLexPred(LF lf) {
        if (!isLexPred(lf)) return null;
        LF arg = ((SatOp)lf).getArg();
        return ((Proposition)arg).toString();
    }
    
    /**
     * Returns the name of the relation of the given elementary predication, 
     * or null, if the given LF is not a relation or attribute-value predicate.
     */
    public static String getRel(LF lf) {
        if (!isRelPred(lf) && !isAttrPred(lf)) return null;
        LF arg = ((SatOp)lf).getArg();
        return ((Diamond)arg).getMode().toString();
    }
    
    /**
     * Returns the principal nominal the given elementary predication, 
     * or null, if the given LF is not an elementary predication.
     */
    public static Nominal getPrincipalNominal(LF lf) {
        if (!isElementaryPredication(lf)) return null;
        return ((SatOp)lf).getNominal();
    }

    /**
     * Returns the secondary nominal of the given elementary predication, 
     * or null, if the given LF is not a relation predication.
     */
    public static Nominal getSecondaryNominal(LF lf) {
        if (!isRelPred(lf)) return null;
        LF arg = ((SatOp)lf).getArg();
        return (Nominal) ((Diamond)arg).getArg(); 
    }

    
    //-----------------------------------------------------------------
    // flattening 

    /**
     * Returns a flattened, sorted list of elementary preds from the given LF 
     * as a conjunction op, or as a single LF, if there is only one. 
     * LF chunks are preserved on satops, as are alts (exclusive disjunctions) 
     * and opts (optional parts).
     * A runtime exception is thrown if the LF cannot be flattened.
     */
    @SuppressWarnings("unchecked")
	public static LF flattenLF(LF lf) {
        List<?> preds = flatten(lf);
        if (preds.size() == 1) {
            return (LF) preds.get(0);
        }
        else {
        	return new Op(Op.CONJ, (List<LF>)preds);
        }
    }
    
    /**
     * Returns a list of predications from the given LF, which is assumed to be either 
     * a conjunction of elementary predications or a single elementary predication.
     */
    public static List<SatOp> getPreds(LF lf) {
        if (lf instanceof Op && ((Op)lf).getName().equals(Op.CONJ)) {
            List<LF> args = ((Op)lf).getArguments();
            List<SatOp> retval = new ArrayList<SatOp>(args.size());
            for (LF arg : args) retval.add((SatOp)arg);
            return retval;
        }
        else { 
            List<SatOp> retval = new ArrayList<SatOp>(1);
            retval.add((SatOp)lf);
            return retval;
        }
    }
    
    /**
     * Returns a flattened, sorted list of elementary preds from the given LF 
     * as a list.
     * LF chunks are preserved on satops, as are alts (exclusive disjunctions) 
     * and opts (optional parts).
     * A runtime exception is thrown if the LF cannot be flattened.
     */
    public static List<SatOp> flatten(LF lf) { 
        List<SatOp> retval = new ArrayList<SatOp>(5);
        flatten(lf, null, retval);
        sort(retval);
        return retval;
    }
    
    /**
     * Returns the first elementary predication in the flattened LF.
     * A runtime exception is thrown if the LF cannot be flattened.
     */
    public static LF firstEP(LF lf) { 
        List<SatOp> preds = new ArrayList<SatOp>(5);
        flatten(lf, null, preds);
        return preds.get(0);
    }
    
    // flatten, converting then propagating alts and opts
    private static void flatten(LF lf, Nominal currentNominal, List<SatOp> preds) {
        int[] counts = new int[]{0,0};
        flatten(lf, currentNominal, preds, new Stack<Alt>(), new TIntArrayList(), counts);
        if (counts[0] > 0 || counts[1] > 0) {
            List<Nominal> rootNoms = new ArrayList<Nominal>();
            findRoots(lf, rootNoms);
            propAltsOptsChunks(preds, rootNoms);
        }
    }

    // recursive flattening, with conversion of alts and opts
    private static void flatten(
        LF lf, Nominal currentNominal, List<SatOp> preds, 
        Stack<Alt> alts, TIntArrayList opts, int[] counts
    ) {
        if (lf instanceof SatOp) {
            // flatten arg with new current nominal
            SatOp satOp = (SatOp) lf;
            flatten(satOp.getArg(), satOp.getNominal(), preds, alts, opts, counts);
        }
        else if (lf instanceof Op) {
            Op op = (Op) lf;
            if (op._name.equals(Op.XOR)) {
                // introduce new alt set; add alt for each item
                int altSet = counts[0]++;
                for (int i = 0; i < op._args.size(); i++) {
                    alts.push(new Alt(altSet, i));
                    LF arg = op._args.get(i);
                    flatten(arg, currentNominal, preds, alts, opts, counts);
                    alts.pop();
                }
            }
            else if (op._name.equals(Op.OPT)) {
                // introduce new opt index for arg
                opts.add(counts[1]++);
                LF arg = op._args.get(0);
                flatten(arg, currentNominal, preds, alts, opts, counts);
                opts.remove(opts.size()-1);
            }
            else {
                // otherwise just flatten each item
                for (Iterator<LF> it = op.getArguments().iterator(); it.hasNext(); ) {
                    flatten(it.next(), currentNominal, preds, alts, opts, counts);
                }
            }
        }
        else if (lf instanceof Proposition) {
            // add SatOp for lf
            if (currentNominal == null) {
                throw new RuntimeException("No current nominal in trying to flatten " + lf);
            }
            SatOp satOp = new SatOp(currentNominal, lf);
            addSatOp(satOp, lf, preds, alts, opts);
        }
        else if (lf instanceof HyloVar) {
            // just skip for now
        }
        else if (lf instanceof Diamond) {
            Diamond diamond = (Diamond) lf;
            LF arg = diamond.getArg();
            if (arg instanceof Proposition || arg instanceof Nominal || arg instanceof HyloVar) {
                // add SatOp for diamond
                SatOp satOp = new SatOp(currentNominal, lf);
                addSatOp(satOp, lf, preds, alts, opts);
            }
            else if (arg instanceof Op && ((Op)arg)._name.equals(Op.CONJ)) {
                // add SatOp for diamond with first nominal arg, 
                // and flatten the rest of the args with the first nominal arg as the 
                // new current nominal
                Op argOp = (Op) arg;
                Iterator<LF> args = argOp._args.iterator();
                LF firstArg = args.next();
                if (!(firstArg instanceof Nominal)) {
                    throw new RuntimeException("First arg of diamond is not a nominal: " + firstArg);
                }
                Nominal firstNominalArg = (Nominal) firstArg;
                // add SatOp for diamond, with nominal marked as shared
                firstNominalArg.setShared(true);
                SatOp satOp = new SatOp(currentNominal, new Diamond(diamond.getMode(), firstNominalArg));
                addSatOp(satOp, lf, preds, alts, opts);
                // flatten rest of list
                for (; args.hasNext(); ) {
                    flatten(args.next(), firstNominalArg, preds, alts, opts, counts);
                }
            }
            else if (arg instanceof Op && ((Op)arg)._name.equals(Op.XOR)) {
                Op argOp = (Op) arg;
                // as before, process xor by introducing new alt set and adding alt for each disjunct; 
                // this time, also assume each disjunct is a conj op or nominal, and add a diamond satop 
                // to the disjunct nominal
                int altSet = counts[0]++;
                for (int i = 0; i < argOp._args.size(); i++) {
                    alts.push(new Alt(altSet, i));
                    LF disjunct = argOp._args.get(i);
                    if (!(disjunct instanceof Op && ((Op)disjunct)._name.equals(Op.CONJ)) && !(disjunct instanceof Nominal)) {
                        throw new RuntimeException("Disjunct of diamond is not a conj op or nominal: " + disjunct);
                    }
                    // conj op case
                    if (disjunct instanceof Op) {
                        Op disjunctOp = (Op) disjunct;
                        Iterator<LF> args = disjunctOp._args.iterator();
                        LF firstArg = args.next();
                        if (!(firstArg instanceof Nominal)) {
                            throw new RuntimeException("First arg of conj op under xor op is not a nominal: " + firstArg);
                        }
                        // add SatOp for diamond, with nominal marked as shared
                        Nominal disjunctNominal = (Nominal) firstArg;
                        disjunctNominal.setShared(true);
                        SatOp satOp = new SatOp(currentNominal, new Diamond(diamond.getMode(), disjunctNominal));
                        addSatOp(satOp, lf, preds, alts, opts);
                        // flatten rest of list
                        for (; args.hasNext(); ) {
                            flatten(args.next(), disjunctNominal, preds, alts, opts, counts);
                        }
                    }
                    // nominal case
                    else {
                        // just add SatOp for diamond
                        Nominal disjunctNominal = (Nominal) disjunct;
                        SatOp satOp = new SatOp(currentNominal, new Diamond(diamond.getMode(), disjunctNominal));
                        addSatOp(satOp, lf, preds, alts, opts);
                    }
                    alts.pop();
                }
            }
            else { 
                throw new RuntimeException("Arg of diamond is not a proposition, nominal or list: " + arg);
            }
        }
        else throw new RuntimeException("Unable to flatten " + lf);
    }

    private static void addSatOp(SatOp satOp, LF lf, List<SatOp> preds, Stack<Alt> alts, TIntArrayList opts) {
        satOp.setChunks(lf.getChunks());
        preds.add(satOp);
        if (!alts.empty()) satOp.alts = new ArrayList<Alt>(alts);
        if (opts.size() > 0) satOp.opts = new TIntArrayList(opts.toNativeArray());
    }
    
    private static void findRoots(LF lf, List<Nominal> rootNoms) {
        if (lf instanceof SatOp) {
            rootNoms.add(((SatOp)lf).getNominal()); return;
        }
        else if (lf instanceof Op) {
            for (Iterator it = ((Op)lf).getArguments().iterator(); it.hasNext(); ) {
                findRoots((LF) it.next(), rootNoms);
            }
        }
        else {
            throw new RuntimeException("Unable to find root nominals: " + lf);
        }
    }

    // propagates alts, opts and chunks down from root noms through preds
    private static void propAltsOptsChunks(List<SatOp> preds, List<Nominal> rootNoms) {
        // map preds by nominals
        // nb: can't use a group map as == on preds is required here, 
        //     given that alts and opts don't figure in SatOp.equals
        Map<Nominal,List<SatOp>> predMap = new HashMap<Nominal,List<SatOp>>();
        for (SatOp satOp : preds) {
            Nominal nom = satOp.getNominal();
            List<SatOp> nomPreds = predMap.get(nom);
            if (nomPreds == null) {
                nomPreds = new ArrayList<SatOp>(5);
                predMap.put(nom, nomPreds);
            }
            nomPreds.add(satOp);
        }
        // propagate for each root nom
        Stack<Nominal> history = new Stack<Nominal>();
        List<Alt> alts = new ArrayList<Alt>(0);
        TIntArrayList opts = new TIntArrayList(0);
        TIntArrayList chunks = new TIntArrayList(0);
        for (Nominal nom : rootNoms) {
        	propAltsOptsChunks(nom, predMap, history, alts, opts, chunks);
        }
    }
    
    // recursive prop on nom through preds in map
    @SuppressWarnings("unchecked")
	private static void propAltsOptsChunks(
        Nominal nom, Map<Nominal,List<SatOp>> predMap, Stack<Nominal> history, 
        List<Alt> alts, TIntArrayList opts, TIntArrayList chunks
    ) { 
        history.push(nom);
        List<SatOp> preds = predMap.get(nom);
        if (preds == null) { history.pop(); return; }
        for (SatOp satOp : preds) {
            // prop alts and opts
            if (!alts.isEmpty()) {
                if (satOp.alts == null) satOp.alts = new ArrayList<Alt>(3);
                for (Alt alt : alts) {
                    if (!satOp.alts.contains(alt)) satOp.alts.add(alt);
                }
                Collections.sort(satOp.alts);
            }
            if (!opts.isEmpty()) {
                if (satOp.opts == null) satOp.opts = new TIntArrayList(3);
                for (int i=0; i < opts.size(); i++) {
                    int opt = opts.get(i);
                    if (!satOp.opts.contains(opt)) satOp.opts.add(opt);
                }
                satOp.opts.sort();
            }
            if (!chunks.isEmpty()) {
                if (satOp.chunks == null) satOp.chunks = new TIntArrayList(3);
                for (int i=0; i < chunks.size(); i++) {
                    int chunk = chunks.get(i);
                    if (!satOp.chunks.contains(chunk)) satOp.chunks.add(chunk);
                }
                satOp.chunks.sort();
            }
            // recurse, if apropos
            Nominal nom2 = getSecondaryNominal(satOp);
            if (nom2 != null && nom2.isShared() && !history.contains(nom2)) {
                List<Alt> alts2 = (satOp.alts != null) ? satOp.alts : alts;
                TIntArrayList opts2 = (satOp.opts != null) ? satOp.opts : opts;
                TIntArrayList chunks2 = (satOp.chunks != null) ? satOp.chunks : chunks;
                propAltsOptsChunks(nom2, predMap, history, alts2, opts2, chunks2);
            }
        }
        history.pop();
    }
    
    
    //-----------------------------------------------------------------
    // compacting 
    
    /** Composes compact and convertNominals. */
    public static LF compactAndConvertNominals(LF lf, Nominal root) {
        LF retval = compact(lf, root);
        convertNominals(retval);
        return retval;
    }
    
    /**
     * Returns a compacted LF from the given flattened one. 
     * A root nominal may also be given (otherwise null). 
     * Nominals with multiple parents are kept separate.
     */
    public static LF compact(LF lf, Nominal root) {
        // get preds, make copies
        List<SatOp> preds = getPreds(lf);
        for (int i=0; i < preds.size(); i++) {
            SatOp pred = preds.get(i);
            preds.set(i, (SatOp) pred.copy());
        }
        
        // check for single pred
        if (preds.size() == 1) return preds.get(0);
        
        // find unique parents and multiple parents
        Map<Nominal,Nominal> parents = new HashMap<Nominal,Nominal>();
        GroupMap<Nominal,Nominal> multipleParents = new GroupMap<Nominal,Nominal>();
        for (int i = 0; i < preds.size(); i++) {
            SatOp pred = preds.get(i);
            // get principal nominal as nom1
            Nominal nom1 = getPrincipalNominal(pred);
            // get secondary nominal
            Nominal nom2 = getSecondaryNominal(pred);
            // skip if none or nom2 equal to root
            if (nom2 == null) continue;
            if (root != null && nom2.equals(root)) continue;
            // if nom2 already in group map, add nom1 as another parent
            if (multipleParents.containsKey(nom2)) {
                multipleParents.put(nom2, nom1);
            }
            // if nom2 already in parent map, add existing parent and nom1 to group map, 
            // record pred, then remove nom2 from parent map
            else if (parents.containsKey(nom2)) {
                multipleParents.put(nom2, parents.get(nom2));
                multipleParents.put(nom2, nom1);
                parents.remove(nom2);
            }
            // otherwise put in nom1 as parent
            else {
                parents.put(nom2, nom1);
            }
        }

        // check multiple parent nominals for cycles
        int prevSize = -1;
        List<Nominal> history = new ArrayList<Nominal>();
        while (multipleParents.size() != prevSize) {
            prevSize = multipleParents.size();
            for (Iterator<Nominal> it = multipleParents.keySet().iterator(); it.hasNext(); ) {
            	Nominal nom = it.next();
                Set<Nominal> nomParents = multipleParents.get(nom);
                for (Iterator<Nominal> it2 = nomParents.iterator(); it2.hasNext(); ) {
                	Nominal parent = it2.next();
                    history.clear();
                    history.add(nom);
                    while (parent != null && !history.contains(parent)) { 
                        history.add(parent);
                        parent = parents.get(parent);
                    }
                    // remove if cycle found
                    if (parent != null) it2.remove();
                }
                // switch to single parent if others removed
                if (nomParents.size() == 1) {
                	Nominal parent = nomParents.iterator().next();
                    parents.put(nom, parent);
                    it.remove();
                }
            }
        }
        
        // break any remaining cycles in parent relationships
        for (Iterator<Nominal> it = parents.keySet().iterator(); it.hasNext(); ) {
        	Nominal nom = it.next();
        	Nominal parent = parents.get(nom);
            history.clear();
            history.add(nom);
            while (parent != null && !history.contains(parent)) {
                history.add(parent);
                parent = parents.get(parent);
            }
            if (parent != null) { it.remove(); } 
        }
        
        // ensure sorted
        sort(preds);
        
        // combine preds on same nominal
        List<SatOp> combinedPreds = new ArrayList<SatOp>(preds.size());
        SatOp currentSatOp = preds.get(0);
        Nominal currentNominal = currentSatOp.getNominal();
        combinedPreds.add(currentSatOp);
        for (int i = 1; i < preds.size(); i++) {
            SatOp satOp = preds.get(i);
            // skip if equal to previous
            if (satOp.equals(preds.get(i-1))) continue;
            // check for different nominal
            Nominal nominal = satOp.getNominal();
            if (!nominal.equals(currentNominal)) {
                // add to combined preds, update current refs, 
                currentSatOp = satOp;
                currentNominal = nominal;
                combinedPreds.add(currentSatOp);
            }
            // otherwise combine
            else {
                combine(currentSatOp, satOp);
            }
        }

        // compact preds with unique parent
        for (int i = 0; i < combinedPreds.size(); i++) {
            SatOp satOp1 = combinedPreds.get(i);
            Nominal nom1 = satOp1.getNominal();
            if (!parents.containsValue(nom1)) continue;
            for (int j = 0; j < combinedPreds.size(); j++) {
                SatOp satOp2 = combinedPreds.get(j);
                Nominal nom2 = satOp2.getNominal();
                if (nom1.equals(nom2)) continue;
                if (!parents.containsKey(nom2)) continue;
                if (nom1.equals(parents.get(nom2))) {
                    subst(satOp1, satOp2, nom2, null);
                }
            }
        }
        
        // get root nominals, root preds, and multiple parent preds
        List<Nominal> roots = new ArrayList<Nominal>();
        List<SatOp> rootPreds = new ArrayList<SatOp>();
        List<SatOp> multipleParentPreds = new ArrayList<SatOp>();
        for (int i = 0; i < combinedPreds.size(); i++) {
            SatOp pred = combinedPreds.get(i);
            Nominal nom = pred.getNominal();
            if (!parents.containsKey(nom)  && !multipleParents.containsKey(nom)) {
                roots.add(nom);
                rootPreds.add(pred);
            }
            if (multipleParents.containsKey(nom)) {
                multipleParentPreds.add(pred);
            }
        }
        
        // compact preds with multiple parents, using parent that is closest to a root
        prevSize = -1;
        while (multipleParentPreds.size() != prevSize) {
            prevSize = multipleParentPreds.size();
            // for each nominal with multiple parents
            for (Iterator<SatOp> it = multipleParentPreds.iterator(); it.hasNext(); ) {
                SatOp pred = it.next();
                Nominal nom = pred.getNominal();
                // find parent closest to root, but checking for a parent not below a root
                Set<Nominal > nomParents = multipleParents.get(nom);
                Nominal parentClosestToRoot = null;
                int closestDist = 0;
                int closestRootIndex = -1;
                for (Iterator<Nominal > it2 = nomParents.iterator(); it2.hasNext(); ) {
                    Nominal parent = it2.next();
                    int dist = 0;
                    // trace parents to top ancestor
                    Nominal topAncestor = parent;
                    while (parents.containsKey(topAncestor)) {
                        topAncestor = parents.get(topAncestor);
                        dist++;
                    }
                    // if top ancestor a root, update closest parent
                    if (roots.contains(topAncestor)) {
                        if (parentClosestToRoot == null || dist < closestDist) {
                            parentClosestToRoot = parent; 
                            closestDist = dist;
                            closestRootIndex = roots.indexOf(topAncestor);
                        }
                    }
                    // otherwise set closest dist to -1, to indicate that not all ancestors are roots
                    else { closestDist = -1; }
                }
                // check for a parent not below a root, or no closest root, and skip this nom if so
                if (closestDist == -1 || closestRootIndex == -1) { continue; }
                // otherwise compact under root pred of parent closest to root
                SatOp closestRootPred = rootPreds.get(closestRootIndex);
                subst(closestRootPred, pred, nom, parentClosestToRoot);
                // update parents map
                parents.put(nom, parentClosestToRoot);
                // and remove from iterator
                it.remove();
            }
        }
        
        // return
        List<LF> retPreds = new ArrayList<LF>();
        retPreds.addAll(rootPreds);
        retPreds.addAll(multipleParentPreds);
        if (retPreds.size() == 1) { return retPreds.get(0); }
        else { return new Op(Op.CONJ, retPreds); }
    }
    
    
    // combines two preds for the same nominal into the first pred, 
    // where either both preds are elementary, 
    // or the first is the result of an earlier combination
    private static void combine(SatOp satOp1, SatOp satOp2) {
        // get args
        LF arg1 = satOp1.getArg();
        LF arg2 = satOp2.getArg();
        // check if arg1 already conj op
        if (arg1 instanceof Op && ((Op)arg1).getName().equals(Op.CONJ)) {
            List<LF> args = ((Op)arg1).getArguments();
            args.add(arg2);
        }
        // or make it one
        else {
            List<LF> args = new ArrayList<LF>(2);
            args.add(arg1); args.add(arg2);
            satOp1.setArg(new Op(Op.CONJ, args));
        }
    }
    
    
    // substitutes the second satop into the first lf at nom2, optionally 
    // respecting the given parent constraint (if non-null)
    private static void subst(LF lf, SatOp satOp2, Nominal nom2, Nominal requiredParent) {
        subst(lf, null, satOp2, nom2, requiredParent);
    }
    
    // recursive implementation that tracks the current parent and 
    // returns whether the substitution has been made
    private static boolean subst(LF lf, Nominal currentParent, SatOp satOp2, Nominal nom2, Nominal requiredParent) {
        // recurse to nom2, then append if requiredParent constraint met
        if (lf instanceof SatOp) {
            SatOp satOp = (SatOp) lf;
            return subst(satOp.getArg(), satOp.getNominal(), satOp2, nom2, requiredParent);
        }
        else if (lf instanceof Diamond) {
            Diamond d = (Diamond) lf;
            LF arg = d.getArg();
            // check for nom2, and that requiredParent constraint met
            if (arg.equals(nom2) && (requiredParent == null || requiredParent.equals(currentParent))) {
                // make substitution
                d.setArg(append(arg, satOp2.getArg()));
                return true;
            }
            else {
                return subst(arg, currentParent, satOp2, nom2, requiredParent);
            }
        }
        else if (lf instanceof Op) {
            List<LF> args = ((Op)lf).getArguments();
            for (int i = 0; i < args.size(); i++) {
                LF arg = args.get(i);
                if (arg instanceof Nominal) {
                    currentParent = (Nominal) arg;
                    continue;
                }
                boolean madeSubst = subst(arg, currentParent, satOp2, nom2, requiredParent);
                if (madeSubst) return true;
            }
        }
        return false;
    }
    
    
    //-----------------------------------------------------------------
    // convert nominals
    
    /** Converts nominal vars to atoms, renaming them based on lexical propositions. */
    public static void convertNominals(LF lf) {
        Map<Nominal,Nominal> nominalMap = new HashMap<Nominal,Nominal>();
        Map<String,Integer> nameMap = new HashMap<String,Integer>();
        // traverse twice, skipping absent props the first time
        boolean skipAbsentProp = true;
        convertNominals(lf, nominalMap, nameMap, skipAbsentProp);
        skipAbsentProp = false;
        convertNominals(lf, nominalMap, nameMap, skipAbsentProp);
    }

    // recurse through lf, converting nominals
    private static void convertNominals(LF lf, Map<Nominal,Nominal> nominalMap, Map<String,Integer> nameMap, boolean skipAbsentProp) {
        if (lf instanceof SatOp) {
            SatOp satOp = (SatOp) lf;
            Nominal oldNom = satOp.getNominal();
            Proposition prop = null;
            LF arg = satOp.getArg();
            if (arg instanceof Proposition) { prop = (Proposition) arg; }
            else if (arg instanceof Op) {
                Op op = (Op) arg;
                LF first = (LF) op.getArguments().get(0);
                if (first instanceof Proposition) { prop = (Proposition) first; }
            }
            Nominal convertedNom = convertNominal(oldNom, prop, nominalMap, nameMap, skipAbsentProp);
            satOp.setNominal(convertedNom);
            convertNominals(arg, nominalMap, nameMap, skipAbsentProp);
        }
        else if (lf instanceof Diamond) {
            Diamond d = (Diamond) lf;
            LF arg = d.getArg();
            if (arg instanceof Nominal) {
                Nominal oldNom = (Nominal) arg;
                Nominal convertedNom = convertNominal(oldNom, null, nominalMap, nameMap, skipAbsentProp);
                d.setArg(convertedNom);
            }
            else if (arg instanceof Op) {
                Op op = (Op) arg;
                List<LF> args = op.getArguments();
                LF first = args.get(0);
                if (first instanceof Nominal) {
                    Nominal oldNom = (Nominal) first;
                    LF second = args.get(1);
                    Proposition prop = null;
                    if (second instanceof Proposition) { prop = (Proposition) second; }
                    Nominal convertedNom = convertNominal(oldNom, prop, nominalMap, nameMap, skipAbsentProp);
                    args.set(0, convertedNom);
                }
                convertNominals(arg, nominalMap, nameMap, skipAbsentProp);
            }
        }
        else if (lf instanceof Op) {
            List<LF> args = ((Op)lf).getArguments();
            for (int i = 0; i < args.size(); i++) {
                convertNominals(args.get(i), nominalMap, nameMap, skipAbsentProp);
            }
        }
    }
    

    // returns a nominal atom based on the old nominal, prop and maps, 
    // which are updated accordingly; the skipAbsentProp flag controls 
    // whether to skip a null prop, so that a meaningful name might 
    // be created later
    private static Nominal convertNominal(
        Nominal oldNom, Proposition prop, 
        Map<Nominal,Nominal> nominalMap, Map<String,Integer> nameMap, boolean skipAbsentProp) 
    {

        // check for an atom
        if (oldNom instanceof NominalAtom) return oldNom;
        // skip absent props according to flag
        if (prop == null && skipAbsentProp) return oldNom;
        // check if already converted, and return copy
        Nominal alreadyConvertedNom = nominalMap.get(oldNom);
        if (alreadyConvertedNom != null) {
            return (Nominal) alreadyConvertedNom.copy();
        }
        // otherwise create new atom, with name based on prop (if possible)
        String nameBase = "x";
        if (prop != null) { 
            nameBase = prop.toString().toLowerCase();

            // use "n" if not a letter
            if (!Character.isLetter(nameBase.charAt(0))) nameBase = "n";
        }
        int ext=1;
        String name = nameBase + ext;
        if (prop!=null && prop.getWordPosition() != -1) {
        	name = nameBase + prop.getWordPosition() + "_" + prop.getUtteranceIncrement();
        }
        else { 
        	Integer baseCount = nameMap.get(nameBase);
        	if (baseCount != null) { 
        		ext = baseCount.intValue() + 20;
        		name = nameBase + (new Integer(ext)).toString() + "_" + utteranceIncrement; 
        	}
        	else    
        		name = nameBase + 1 + "_" + utteranceIncrement; 
        		ext++;
        }
        nameMap.put(nameBase, new Integer(ext));
        Nominal retval = new NominalAtom(name, oldNom.getType());
        nominalMap.put(oldNom, retval);
        return retval;
    }


    //-----------------------------------------------------------------
    // append 

    /**
     * Returns a the conjunction of the two LFs, either 
     * as a conjunction op, or as a single LF, if one is null.
     * If either LF is itself a conj op, its elements are appended  
     * instead of the conj op itself.
     * If both LFs are null, null is returned.
     */
    public static LF append(LF lf1, LF lf2) {
        
        // set up new list
        int size = 0;
        List<LF> args1 = null;
        if (lf1 instanceof Op && ((Op)lf1).getName().equals(Op.CONJ)) {
            args1 = ((Op)lf1).getArguments();
            size += args1.size();
        } else if (lf1 != null) {
            size++;
        } 
        List<LF> args2 = null;
        if (lf2 instanceof Op && ((Op)lf2).getName().equals(Op.CONJ)) {
            args2 = ((Op)lf2).getArguments();
            size += args2.size();
        } else if (lf2 != null) {
            size++;
        }
        List<LF> combined = new ArrayList<LF>(size);
        
        // add to new list
        if (args1 != null) { 
            combined.addAll(args1);
        } else if (lf1 != null) {
            combined.add(lf1);
        }
        if (args2 != null) { 
            combined.addAll(args2);
        } else if (lf2 != null) {
            combined.add(lf2);
        }
        
        // return
        if (combined.isEmpty()) { return null; }
        else if (combined.size() == 1) { return combined.get(0); }
        else { return new Op(Op.CONJ, combined); }
    }

    
    //-----------------------------------------------------------------
    // sort 

    /**
     * Sorts the list of elementary predications in a conj op, 
     * or does nothing if the LF is not a conj op.
     */
    public static void sort(LF lf) {
        if (lf instanceof Op && ((Op)lf).getName().equals(Op.CONJ)) {
            sort(((Op)lf).getArguments());
        }
    }
    
    /**
     * Sorts a list of elementary predications.
     */
    public static void sort(List<? extends LF> preds) {
        Collections.sort(preds, predComparator);
    }

    // compares elementary predications
    private static final Comparator<LF> predComparator = new Comparator<LF>() {
        public int compare(LF lf1, LF lf2){
            // sort first on principal nominal
            int nomCompare = getPrincipalNominal(lf1).compareTo(getPrincipalNominal(lf2));
            if (nomCompare != 0) return nomCompare;
            // sort next on type of elementary predication
            int typeCompare = epType(lf1).compareTo(epType(lf2));
            if (typeCompare != 0) return typeCompare;
            // then on lex pred
            if (isLexPred(lf1)) {
                return getLexPred(lf1).compareToIgnoreCase(getLexPred(lf2));
            }
            // then rels
            String rel1 = getRel(lf1);
            String rel2 = getRel(lf2);
            Lexicon theLexicon = Grammar.theGrammar.lexicon;
            Integer rel1Index = theLexicon.getRelationSortIndex(rel1);
            Integer rel2Index = theLexicon.getRelationSortIndex(rel2);
            int relIndexCompare = rel1Index.compareTo(rel2Index);
            if (relIndexCompare != 0) return relIndexCompare;
            int relCompare = rel1.compareToIgnoreCase(rel2);
            if (relCompare != 0) return relCompare;
            // then secondary nominal
            if (isRelPred(lf1)) {
                return getSecondaryNominal(lf1).compareTo(getSecondaryNominal(lf2));
            }
            // otherwise 0
            return 0;
        }
    };
    
    // order of elementary predication type
    private static Integer epType(LF lf) {
        if (isLexPred(lf)) return LEX_PRED;
        else if (isAttrPred(lf)) return ATTR_PRED;
        else if (isRelPred(lf)) return REL_PRED;
        // shouldn't happen
        else return null;
    }
    
    private static Integer LEX_PRED = new Integer(1);
    private static Integer ATTR_PRED = new Integer(2);
    private static Integer REL_PRED = new Integer(3);

    
    //-----------------------------------------------------------------
    // check

    /**
     * Checks the list of elementary predications in a conj op 
     * for well-formedness, or does nothing if the LF is not a conj op.
     * A UnifyFailure exception is thrown if the check fails.
     * The only current check is that there is no more than one lexical 
     * predication per nominal.  
     * The list of predications is assumed to be already sorted.
     */
    public static void check(LF lf) throws UnifyFailure {
        if (lf instanceof Op && ((Op)lf).getName().equals(Op.CONJ)) {
            check(((Op)lf).getArguments());
        }
    }
    
    private static void check(List preds) throws UnifyFailure {
        for (int i = 0; i < preds.size()-1; i++) {
            LF lf1 = (LF) preds.get(i);
            LF lf2 = (LF) preds.get(i+1);
            if (isLexPred(lf1) && isLexPred(lf2) &&
                getPrincipalNominal(lf1).equals(getPrincipalNominal(lf2))) 
            {
                throw new UnifyFailure();
            }
        }
    }
    
    
    // utterance increment (used for labelling the nominal variable identifiers)
    static private int utteranceIncrement = 0;
    
    public static void setUtteranceIncrement(int utteranceIncr) {
    	utteranceIncrement = utteranceIncr ;
    }
}
