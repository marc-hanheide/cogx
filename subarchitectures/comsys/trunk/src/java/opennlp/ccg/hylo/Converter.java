///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-6 Michael White (University of Edinburgh, The Ohio State University)
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
import java.util.*;

/**
 * A class implementing conversion of nominal variables to nominal atoms.
 *
 * @author      Michael White
 * @version     $Revision: 1.1 $, $Date: 2006/09/04 14:09:10 $
 **/
public class Converter {

	// map to already converted nominals 
    private Map<Nominal,Nominal> nominalMap = new HashMap<Nominal,Nominal>();
    
    // map to int for names
    private Map<String,Integer> nameMap = new HashMap<String,Integer>();
	
    // flag for whether to skip absent props
    private boolean skipAbsentProp = true;
    
    /** Converts nominal vars to atoms, renaming them based on lexical propositions. */
    static void convertNominals(LF lf) {
        // traverse twice, skipping absent props the first time
    	Converter converter = new Converter();
    	converter.convertNoms(lf);
        converter.skipAbsentProp = false;
    	converter.convertNoms(lf);
    }

    // recurse through lf, converting nominals
    private void convertNoms(LF lf) {
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
            Nominal convertedNom = convertNominal(oldNom, prop);
            satOp.setNominal(convertedNom);
            convertNoms(arg);
        }
        else if (lf instanceof Diamond) {
            Diamond d = (Diamond) lf;
            LF arg = d.getArg();
            if (arg instanceof Nominal) {
                Nominal oldNom = (Nominal) arg;
                Nominal convertedNom = convertNominal(oldNom, null);
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
                    Nominal convertedNom = convertNominal(oldNom, prop);
                    args.set(0, convertedNom);
                }
                convertNoms(arg);
            }
        }
        else if (lf instanceof Op) {
            List<LF> args = ((Op)lf).getArguments();
            for (int i = 0; i < args.size(); i++) {
            	convertNoms(args.get(i));
            }
        }
    }

    // returns a nominal atom based on the old nominal, prop and maps, 
    // which are updated accordingly; the skipAbsentProp flag controls 
    // whether to skip a null prop, so that a meaningful name might 
    // be created later
    private Nominal convertNominal(Nominal oldNom, Proposition prop) {
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
    //    String name = nameBase + ext;
        Nominal retval = new NominalAtom(name, oldNom.getType());
        nominalMap.put(oldNom, retval);
        return retval;
    }
    
//  utterance increment (used for labelling the nominal variable identifiers)
    static private int utteranceIncrement = 0;
    
    public static void setUtteranceIncrement(int utteranceIncr) {
    	utteranceIncrement = utteranceIncr ;
    }
}
