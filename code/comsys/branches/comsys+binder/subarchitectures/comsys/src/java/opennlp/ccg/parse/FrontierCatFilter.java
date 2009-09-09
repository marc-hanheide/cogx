// =================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION
// =================================================================

package opennlp.ccg.parse;

// =================================================================
// IMPORTS
// =================================================================

// -----------------------------------------------------------------
// CAST IMPORTS
// -----------------------------------------------------------------

// -----------------------------------------------------------------
// COMSYS IMPORTS
// -----------------------------------------------------------------
import comsys.arch.ComsysException;
import comsys.processing.parse.SignHashParseResults;

// -----------------------------------------------------------------
// JAVA IMPORTS
// -----------------------------------------------------------------
import java.util.Collection;
import java.util.Iterator;
import java.util.StringTokenizer;
import java.util.Vector;

// -----------------------------------------------------------------
// OPENCCG IMPORTS
// -----------------------------------------------------------------
import opennlp.ccg.synsem.AtomCat;
import opennlp.ccg.synsem.Category;
import opennlp.ccg.synsem.Sign;
import opennlp.ccg.synsem.SignHash;
import opennlp.ccg.unify.GFeatStruc;
import opennlp.ccg.unify.GUnifier;
import opennlp.ccg.unify.UnifyFailure;

// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
The class <b>FrontierCatFilter</b> implements a "bar-level" filter on 
what kind(s) of categories can appear on the right-most frontier of 
an incrementally computed analysis. This enables a qualitative 
characterization of the incremental steps an incremental parser
should take ("parse until you're after an NP" instead of "parse 
until string position X"). 

@version 070814
@since	 070814
@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

public class FrontierCatFilter 
	implements IncrFrontierFilter
{

	// =================================================================
    // GLOBAL DATA STRUCTURES
    // =================================================================
	
	// Vector with the categories present in the filter
	Vector<Category> cats; 
	
	// Unifier used for checking categories in a SignHash against the filter
	GUnifier unifier; 
	
	boolean logging;
	
	// =================================================================
    // CONSTRUCTOR 
    // =================================================================
	
	public FrontierCatFilter () { 
		init();
	} // end constructor
	
	public FrontierCatFilter (Vector<Category> cvs) { 
		init();
		cats = cvs; 
	} // end constructor
	
	private void init () { 
		cats = new Vector<Category>();
		logging = false; 
	} // end init

	// =================================================================
    // ACCESS METHODS 
    // =================================================================
		
	/**
		The method <i>add</i> takes a Category object and adds it to the 
		filter.
		
		@param cat The category to be added
	*/ 
	
	public void add (Category cat) { 
		cats.addElement(cat);
	} // end add

	/**
		The method <i>add</i> takes a collection of Category objects, and
		adds them to the filter. 
		
		@param cc The collection of Category objects to be added
	*/ 

	public void add (Collection<Category> cc) { 
		for (Iterator cIter=cc.iterator(); cIter.hasNext(); ) { 
			Category cat = (Category)cIter.next(); 
			cats.addElement(cat);
		} // end for over categories in the collection
	} // end add

	/**
		The method <i>add</i> takes a String array of atomic category labels, 
		constructs the corresponding (atomic) categories, and adds these to the
		filter. The syntax for atomic category labels is identical to that of
		the OpenCCG output: <p>
		
		ATOMCAT = catlabel | catlabel{FSTRUCT} <br>
		FSTRUCT = {FV} <br>
		FV		= flabel=alabel | flabel=alabel, FV <p>

		with catlabel, flabel, and alabel being Strings. 
		
		@param ac The atomic category labels, represented as String array
		@throws ComsysException Thrown if there is a syntax error in a specified category
	*/

	public void add (String[] ac) 
		throws ComsysException 	
	{ 
		for (int i=0; i < ac.length; i++) { 
			String catLabel = ac[i]; 
			int fsStart = catLabel.indexOf("{");
			if (fsStart == -1) { 
				// no feature structures
				AtomCat cat = new AtomCat(catLabel);
				cats.addElement(cat);
			} else { 
				// category label with feature structures
				if (fsStart > 0) { 
					String atomCat = catLabel.substring(0,fsStart); // get the atomic category label
					int fsEnd = catLabel.indexOf("}");
					if (fsEnd != -1 && fsEnd > fsStart) { 
						String fsStr = catLabel.substring(fsStart+1,fsEnd);
						StringTokenizer fsStrTZ = new StringTokenizer(fsStr,", ");
						GFeatStruc fs = new GFeatStruc();
						while (fsStrTZ.hasMoreTokens()) { 
							String fa = fsStrTZ.nextToken();
							int eqPos = fa.indexOf("=");
							if (eqPos != -1) { 
								String feature = fa.substring(0,eqPos); 
								String value   = fa.substring(eqPos+1); 
								fs.setFeature(feature,value);
							} else { 
								throw new ComsysException("Trying to add illegal category to FrontierCatFilter: ["+catLabel+"]");
							} // end if.. check for proper feature-value pair syntax
						} // end while over feature-value assignments
						AtomCat cat = new AtomCat(catLabel,fs);
						cats.addElement(cat);
					} else { 
						throw new ComsysException("Trying to add illegal category to FrontierCatFilter: ["+catLabel+"]");
					} // end if..else check for valid feature structure
				} else { 
					throw new ComsysException("Trying to add illegal category to FrontierCatFilter: ["+catLabel+"]");
				} // end if..else check for proper category
			} // end if..else check for feature structures
		} // end for over atomic category labels
	} // end add

	// =================================================================
    // COMPUTATION METHODS 
    // =================================================================

	/**
		The method <i>eligibleFrontierReached</i> checks for a provided
		SignHash object whether it contains one or more signs that would
		pass the specified filter -- making the SignHash an eligible frontier 
		in an incremental analysis. For a Sign to "pass the filter" it means
		it should have a category that unifies with a "frontier category"
		specified in the filter.  
		
		@param hash The SignHash object to be checked 
		@return boolean Indicating whether the SignHash includes one or more Signs constituting frontier categories
		@since  071014
	*/
	
	public boolean eligibleFrontierReached (ParseResults pr) {
		boolean result = false; 
		// cast the parse results to a SignHashParseResults
		SignHash hash = ((SignHashParseResults)pr).hash;
		if (hash == null) { 
			log("Incr. frontier reached because hash is null."); 
			result = true; 
		} // end if check for empty filter
		else if (cats.isEmpty()) { 
			log("Incr. frontier reached because no categories defined for frontier"); 
			result = true; 
		} // end if check for empty filter		
		else { 
			Iterator signIter=hash.iterator();
			while(signIter.hasNext() && !result) {
				Category cat = ((Sign)signIter.next()).getCategory();
				Iterator filterIter = cats.iterator();
				while (filterIter.hasNext() && !result) { 
					Category filterCat = (Category)filterIter.next();
					try { 
						unifier.unify(filterCat,cat);
						// if we're still here, then they unified; 
						// else we've had to catch the unify failure
						result=true;
					} catch (UnifyFailure e) { 
						// no need to do anything
					} // end try..catch
				} // end while
			} // end while over categories in the signs in a SignHash
		} // end if..else checking for whether to do something
		return result;
	} // end eligibleFrontierReached
	
	
	public void setLogging (boolean l) { logging = l; } 
	
	public void log (String msg) { 
		if (logging) System.out.println("[FrontierCatFilter] "+msg);
	
	} 
	





} // end class
