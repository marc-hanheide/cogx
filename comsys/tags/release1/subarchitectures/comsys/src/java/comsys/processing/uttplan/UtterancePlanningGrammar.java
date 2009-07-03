//=================================================================
// Copyright (C) 2005-2009 Geert-Jan M. Kruijff (gj@acm.org)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
//=================================================================

package comsys.processing.uttplan;

//=================================================================
// IMPORTS
//=================================================================

import comsys.datastructs.lf.*;

import java.io.*;
import java.util.*; 

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 * 
 *  @version 050208 (started 050208)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UtterancePlanningGrammar {
    
      
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    private Vector choosersfiles;
    private Vector systemsfiles;

    private HashMap systems;
    private HashMap choosers;

    private String grammarname;

    boolean logging = true;

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     * The basic constructor initializes the internal variables
     */

    public UtterancePlanningGrammar () {
	grammarname = "";
	choosersfiles = new Vector ();
	systemsfiles = new Vector();
	systems = new HashMap();
	choosers = new HashMap ();
    } // end constructor


    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     *  The method <i>getApplicableSystems</i> returns a Vector with
     *  the names of those systems whose conditions are satisfied
     *  given the locus <tt>l</tt>. The Vector contains String objects. 
     */

    public Vector getApplicableSystems (UPLocus l) { 
	Vector names = new Vector ();
	for (Iterator sIter = systems.values().iterator(); sIter.hasNext(); ) { 
	    UPGSystem sys = (UPGSystem) sIter.next();
	    if (sys.isApplicable(l)) { 
		if (sys.getRegion().equals("STRUCTURE")) { 
		    names.add(0,sys.getId()); 
		} else { 
		    names.addElement(sys.getId()); 
		} // end if.. check for STRUCTURE: always move up front!
	    } // end if.. check for applicable
	} // end for over systems
	return names;
    } // end getApplicableSystems

    /**
     *  The method <i>getChooser</i> returns the chooser with the
     *  given identifier, provided it exists in the grammar. If it
     *  does not exist, an exception of type <tt>UPGException</tt> is
     *  thrown.
     */

    public UPGChooser getChooser (String chid) throws UPGException { 
	if (choosers.containsKey(chid)) { 
	    return (UPGChooser) choosers.get(chid);
	} else {
	    throw new UPGException("Unknown chooser id: "+chid+" not found in grammar <"+grammarname+">");
	} // end if..else
    } // end getChooser

    /**
     *  The method <i>getSystem</i> returns the system with the given
     *  identifier, provided it exists in the grammar. If it does not
     *  exist, an exception of type <tt>UPGException</tt> is thrown. 
     */

    public UPGSystem getSystem (String sysid) throws UPGException { 
	if (systems.containsKey(sysid)) { 
	    return (UPGSystem) systems.get(sysid); 
	} else {
	    throw new UPGException("Unknown system id: "+sysid+" not found in grammar <"+grammarname+">");
	} // end if..else
    } // end getSystem


	/** Returns an iterator over all the stored systems */

	public Iterator getChoosers () { 
		return choosers.values().iterator();
	} // end getChoosers

	/** Returns an iterator over all the stored systems */

	public Iterator getSystems () { 
		return systems.values().iterator();
	} // end getSystems


    public void setChoosersFiles (Vector cfs) { choosersfiles = cfs; }

    public void setSystemsFiles (Vector sfs) { systemsfiles = sfs; }

    //=================================================================
    // I/O METHODS
    //=================================================================

    public void log (String m) { if (logging) System.out.println("[UCG] "+m); } 


    public void prettyPrint () { 
	for (Iterator sIter = systems.values().iterator(); sIter.hasNext(); ) { 
	    UPGSystem sys = (UPGSystem) sIter.next();
	    System.out.println(sys.toString()+"\n");
	} // end for over systems
	for (Iterator cIter = choosers.values().iterator(); cIter.hasNext(); ) { 
	    UPGChooser ch = (UPGChooser) cIter.next();
	    System.out.println(ch.toString()+"\n");
	} // end for over systems

    } // end prettyPrint


    /**
     *  The method <i>read</i> reads in the utterance planning
     *  grammar, starting with the given filename. This should be the
     *  location of a "grammar.xml" file, which specifies what systems
     *  definitions files and what choosers definitions files should
     *  be loaded.
     */

    public void read (String fn) {  
	try { 
	    UtterancePlanningGrammarReader upgr = new UtterancePlanningGrammarReader();
	    upgr.read(fn);
	    choosersfiles = upgr.getChoosersFiles();
	    systemsfiles  = upgr.getSystemsFiles();
	    grammarname   = upgr.getName();
	    // First, read all the systems
	    for (Iterator siter = systemsfiles.iterator(); siter.hasNext(); ) { 
		String sfn = (String) siter.next();
		UPGSystemsReader upgsr = new UPGSystemsReader();
		upgsr.read(sfn);
		HashMap sys = upgsr.getSystems(); 
		for (Iterator syIter = sys.keySet().iterator(); syIter.hasNext(); ) { 
		    String sysid = (String) syIter.next();
		    UPGSystem s  = (UPGSystem) sys.get(sysid);
		    if (!systems.containsKey(sysid)) { 
			systems.put(sysid,s);

		    } else { 
			System.err.println("Warning: overriding previous version of system with id "+sysid);
			systems.put(sysid,s);
		    } // end if..else
		} // end for over systems 
	    } // end for over system files

	    // Next, read all the choosers from the various files
	    for (Iterator cIter = choosersfiles.iterator(); cIter.hasNext(); ) { 
		String cfn = (String) cIter.next();
		UPGChoosersReader upgcr = new UPGChoosersReader ();
		upgcr.read(cfn); 
		HashMap chsrs = upgcr.getChoosers (); 
		for (Iterator chIter = chsrs.keySet().iterator(); chIter.hasNext(); ) { 
		    String chId = (String) chIter.next();
		    UPGChooser c = (UPGChooser) chsrs.get(chId);
		    if (!choosers.containsKey(chId)) { 
			choosers.put(chId,c);
		    } else { 
			System.err.println("Warning: overriding previous version of chooser with id "+chId);
			choosers.put(chId,c);
		    } // end if..else check for uniqueness
		} // end for
	    } // end for over chooser files
	} catch (Exception e) { 
	    System.err.println(e.getMessage());
	} // end try..catch
    } // end read


    //=================================================================
    // MAIN METHOD
    //=================================================================

    public static void main (String[] args) {
	UtterancePlanningGrammar upg = new UtterancePlanningGrammar (); 
	System.out.println("Reading from "+args[0]);
	upg.read(args[0]);
	System.out.println("\n\n");

	upg.prettyPrint();

    }  // end main

} // end class definition 
