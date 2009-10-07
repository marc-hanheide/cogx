//=================================================================
// Copyright (C) 2005-2008 Geert-Jan M. Kruijff
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

import java.util.*;

import comsys.lf.utils.*;


//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 *  The class <b>UtterancePlanner</b> implements a planner for
 *  generating logical forms, using an Utterance Planning
 *  Grammar. After having been initialized with a grammar, the planner
 *  can take a logical form as input, and expand that logical form
 *  based on meaning- and context-driven decisions modeled in the
 *  grammar. 
 * 
 *  @started 050210
 *  @version 080822
 *  @author  Geert-Jan M. Kruijff, Maria Staudte, Ivana Kruijff-Korbayova
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UtterancePlanner {
    
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    UPLocus locus; 
    UPAgenda agenda; 
    public UPChart chart;
    UPChartItem chitem;
    UtterancePlanningGrammar upg;

    HashMap global;

    // Handler for communication methods
    UPReqHandler upAgent;
	GREHandler   greHandler;
    
    boolean logging = false; 
    int     logginglvl = 0;

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor initializes the internal variables. 
     */

    public UtterancePlanner () {
		locus  = new UPLocus (); 
		agenda = new UPAgenda ();
		chart = new UPChart ();
		upg = new UtterancePlanningGrammar(); 
		global = new HashMap();
		upAgent = null; 
		greHandler = new DummyGREHandler(); 
    } // end constructor

	public UtterancePlanner (GREHandler ge) {
		locus  = new UPLocus (); 
		agenda = new UPAgenda ();
		chart = new UPChart ();
		upg = new UtterancePlanningGrammar(); 
		global = new HashMap();
		upAgent = null; 
		greHandler = ge; 
		
		System.err.println("GRE HANDLER HAS BEEN SET <<<<<<<<<<<<*********************************");
		
    } // end constructor
	
	
    /** 
     *  The unary constructor passes the filename for the utterance
     *  planning grammar, and initializes the remaining internal
     *  variables.
     */

    public UtterancePlanner (String f, UPReqHandler agent) {
	locus  = new UPLocus (); 
	agenda = new UPAgenda ();
	chart = new UPChart ();
	upg = new UtterancePlanningGrammar();
	global = new HashMap();
	upAgent = agent;
		greHandler = null; 
	try { 
	    upg.read(f);
	} catch (Exception e) { 
	    log(100,"error while reading UPG file: "+e.getMessage());
	} 
    } // end constructor

    
    //=================================================================
    // ACCESSOR METHODS
    //=================================================================
    
    /**
     *  The method <i>registerGREHandler</i> registers a plug-in
	 component for handling non-built-in inquiries.
     */
	
    public void registerGREHandler (GREHandler ag) { 
		System.out.println("Registering GREHandler for utterance planner");
		greHandler = ag; 
	}
	
	/**
     *  The method <i>registerInqHandler</i> registers a plug-in
        component for handling non-built-in inquiries.
     */

    public void registerInqHandler (UPReqHandler ag) { upAgent = ag; }

    /**
     *  The method <i>setGrammar</i> sets the utterance planning
     *  grammar used by the planner to the provided grammar.
     */

    public void setGrammar (UtterancePlanningGrammar g) { upg = g; } 

    /**
     *  Sets whether the class should output logging messages to
     *  <tt>System.out</tt>
     */

    public void setLogging (boolean l) { logging = l; }

    /**
     *  Sets the minimal level of the logging message(s) to be printed
     *  out to <tt>System.out</tt>. The higher the level, to more restrictive. 
     */

    public void setLoggingLevel (int l) { logginglvl = l; }

    //=================================================================
    // COMPUTATION METHODS
    //=================================================================

    /**
     *  The method <i>plan</i> takes a logical form <tt>lf</tt>, and
     *  expands that logical form based on decisions modeled in the
     *  utterance planning grammar. The method returns the expanded
     *  logical form.
     */

    public LogicalForm plan (LogicalForm lf) { 
		LogicalForm planlf = lf;
		locus = new UPLocus();
		
		// CHART
		chart.newItem();
		chitem = chart.next();
		
		// Take the root of the logical form
		LFNominal root = planlf.root;

		// Initialize the locus with the root nominal
		locus.setNomvar(root);
		locus.setLF(planlf);

		// Register the request handler with the locus
		locus.setReqHandler(upAgent);
		
		// CHART
		chitem.add(locus.getNominal());

		// Initialize the agenda
		agenda.reset();
		Vector items = createAgendaItems(locus);
		agenda.addItems(items);
		log(3,agenda.toString()); 	

		// Check whether the root of the LF has any embedded items
		// as per the plan specification, and add agenda items based on those
		Vector subtreeNoms = LFUtils.lfCollectNomvars(root,lf);
		for (Iterator<String> subtreeIter = subtreeNoms.iterator(); subtreeIter.hasNext(); ) { 
			String stNomvar = subtreeIter.next();
			LFNominal stNode = LFUtils.lfGetNominal(lf,stNomvar);
			Vector newitems = createAgendaItems(stNode);
			log("Created ["+newitems.size()+"] agenda items for nominal "+LFUtils.lfNominalToString(stNode));
			agenda.addItems(newitems);			
		} // end for over nominal variables	

		log(0,"Added agenda items based on subtree nodes "+subtreeNoms);
		log(0,"Agenda: \n"+agenda.toString());
		
		
		// Cycle over the agenda items 

		while (agenda.hasNext()) { 
			UPAgendaItem item = agenda.next();

			// check whether the item still is indexed to the current
			// locus, otherwise we change the locus to the nominal on
			// the item.

			String locnom = locus.getNomvar();
			String ainomv = item.getNomvar(); 
			
			if (!ainomv.equals(locnom)) { 
				log(0,"Change of locus needed: locus "+locnom+" not equal to agenda item for "+ainomv);
				// CHART
				chitem.add(locus.getNominal());
				LFNominal ainom = LFUtils.lfGetNominal(planlf, ainomv);
				HashMap gvi = locus.getGlobalVariables();  // store a copy of the global vars
				locus = new UPLocus ();                    // create a new locus
				locus.setGlobalVariables(gvi);             // transfer the global vars to the new locus
				locus.setNomvar(ainom);                    // change the locus to the ainom nominal
				locus.setLF(planlf);                       // store the global lf with the locus
				locus.setReqHandler(upAgent);              // register the req handler with the locus
				locnom = ainomv;
				log(2,"Changed locus to: "+locus.toString());
			} // end check whether to automatically change the locus
			// WARNING: not checking whether the item still applies!
			try { 
			String sysid  = item.getSystemId(); 
			UPGSystem sys = upg.getSystem(sysid);
			UPResult res  = runSystem(locus,planlf,sys);
			
			// Check for nominals that have been added to the LF;
			// if any, then add any applicable systems based on those
			if (res.getLocus().hasInsertedNominals()) { 
				Iterator nIter = locus.getInsertedNominals();
				while (nIter.hasNext()) { 
					LFNominal n = (LFNominal) nIter.next();
					Vector newitems = createAgendaItems(n);
					agenda.addItems(newitems);
				} // end while over inserted nominals
				locus.resetInsertedNominals();
			} // end if check whether new inserted nominals

			// Check whether there are new systems that have
			// become applicable
			LFNominal locn = locus.getNominal();
			Vector newitems =  createAgendaItems(locn);
			Iterator nwIter = newitems.iterator();
			while (nwIter.hasNext()) { 
				UPAgendaItem nwitem = (UPAgendaItem) nwIter.next();
				String sysId = nwitem.getSystemId();
				if (!agenda.hasItem(sysId,locnom) && !agenda.hasDoneItem(sysId,locnom)) { agenda.addItem(nwitem); }
			} // end while

			if (!res.getLocus().getNomvar().equals(locnom)) { 
				log(0,"Change of locus? ["+res.getLocus().getNomvar()+"] not equal to ["+locnom+"]");
				items = createAgendaItems(res.getLocus());
				agenda.addItems(items);
			} // end if check whether locus has changed
			log(agenda.toString()); 

			} catch (UPGException e) {
			if (!logging) { System.err.println(e.getMessage());
			} else { log(e.getMessage()); } 
			} // end try..catch
		} // end while over agenda items

	//	System.out.println("------------------------------------------------------------------------------------");
	//	System.out.println(chitem.toString());

		return planlf; 
    } // end plan


    /**
     *  The method <i>createAgendaItems</i> creates the agenda items for
     *  the given locus. The method returns them together as a Vector
     *  object.
     */

    public Vector createAgendaItems (LFNominal nom) { 
	Vector items = new Vector();
	UPLocus lcs = new UPLocus (nom);
	items = createAgendaItems(lcs);
	return items;
    } // end createAgendaItems


    /**
     *  The method <i>createAgendaItems</i> creates the agenda items
     *  for the given nominal. The method returns them together as a
     *  Vector object.
     */

    public Vector createAgendaItems (UPLocus l) { 
		Vector items = new Vector();
		// Given the locus, get the names of the applicable systems
		// from the grammar.

		Vector sys = upg.getApplicableSystems(l); 
		LFNominal nom = l.getNominal();
		String nomvar = nom.nomVar;
		// Create agenda items
		for (Iterator sIter = sys.iterator(); sIter.hasNext(); ) {
			String sysname = (String) sIter.next();
			UPAgendaItem item = new UPAgendaItem (nomvar,sysname);
			items.addElement(item);
		} // end for over names of applicablesystems
		
		
		
		
		return items;
    } // end createAgendaItems


	/** Returns an iterator over the choosers in the utterance planning grammar */ 

	public Iterator getChoosers () { 
		return upg.getChoosers();
	} // end getSystems


	/** Returns an iterator over the systems in the utterance planning grammar */ 

	public Iterator getSystems () { 
		return upg.getSystems();
	} // end getSystems



    //=================================================================
    // GRE METHODS
    //=================================================================

    //=================================================================
    // I/O METHODS
    //=================================================================

    public void load (String filename) { 
	upg = new UtterancePlanningGrammar();
	try { 
	    upg.read(filename);
	} catch (Exception e) { 
	    log(100,"error while reading UPG file: "+e.getMessage());
	} 	
    } // end 


    public void log (String m) { 
	if (logging) { 
	    System.out.println("[UPL] "+m); 
	} // end if
    } // end log


    public void log (int lvl, String m) {
	if (logging && lvl >= logginglvl) { 
	    System.out.println("[UPL] "+m);
	} // end if
    } // end log

    //=================================================================
    // PLANNING COMPUTATION METHODS
    //=================================================================

    /**
     *  The method <i>runSystem</i> takes a locus, a logical form, and
     *  a system. It then runs the chooser, and executes the steps
     *  under the action associated with the result obtained from the
     *  chooser. The method returns an object with the resulting
     *  (possibly updated) logical form and locus.
     */ 

    public UPResult runSystem (UPLocus lcs, LogicalForm logf, UPGSystem sys) throws UPGException {  
	try { 
	    log(2,"Running system "+sys.getId());
	    UPResult result = new UPResult (lcs,logf);
	    String answer;
	    String chsrid  = sys.getChooserId(); 
	    if (chsrid.equals("default")) { 
			answer = "default";
	    } else { 
			UPGChooser chsr = upg.getChooser(chsrid);
			if (chsrid.equals("random")) {
				answer = runChooserRandom(chsr,logf,lcs); 
			} else {
				answer = runChooser(chsr,logf,lcs);
			} // end if..else check for random
	    } // end if..else check whether to run chooser, return a random choice or default
	    log(2,"In system "+sys.getId()+" the chooser "+chsrid+" returns as answer \""+answer+"\"");
	    UPGAction action = sys.getAction(answer);
	    if (action == null) { // if there is no action for the
				  // answer, do nothing
		result.setLocus(lcs);
		result.setLF(logf);
		log(100,"There is no action associated with the answer \""+answer+"\"");
	    } else { // if there is an action associated with the
		     // answer, carry out its steps
		
		result = runActionSteps(action,result);
	    } // end if..else check whether action steps to perform
	    return result;
	} catch (UPGException e) { 
	    throw new UPGException ("Unable to run system "+sys.getId()+": "+e.getMessage()); 
	} // end try..catch
    } // end runSystem


    /** 
     *  The method <i>runActionSteps</i> carries out the action steps
     *  that constitute the given action. This is done until we either
     *  (a) get to the end of the list of action steps, or (b)
     *  encounter a move-locus step. The method returns a
     *  UPResult object that reflects the changes. 
     */

    public UPResult runActionSteps (UPGAction action, UPResult imres) throws UPGException { 
		log(0,"Running action steps");
		UPLocus cloc   = imres.getLocus();
		LogicalForm lf = imres.getLF();
		UPResult result = new UPResult (cloc,lf);
		Vector steps = action.getSteps();
		Iterator sIter = steps.iterator();
		boolean nomovelocus = true;
		while ( sIter.hasNext() && nomovelocus) {
			UPGActionStep step = (UPGActionStep) sIter.next();
			String stepid = step.getId();
			log(0,"Handling stepid "+stepid);


			// The following steps have an effect on the logical form 

			if (stepid.equals("add-feature")) { result = stepcodeAddFeature(step,cloc,lf); }
			if (stepid.equals("add-lf")) { result = stepcodeAddLF(step,cloc,lf); }
			if (stepid.equals("add-proposition")) { result = stepcodeAddProposition(step,cloc,lf); }
			if (stepid.equals("add-relation")) { result = stepcodeAddRelation(step,cloc,lf); }
			if (stepid.equals("adjoin-lf")) { result = stepcodeAdjoinLF(step,cloc,lf); }
			if (stepid.equals("assign-type")) { result = stepcodeAssignType(step,cloc,lf); }
			if (stepid.equals("copy-feature")) { result = stepcodeCopyFeature(step,cloc,lf); }		
			if (stepid.equals("copy-lf")) { result = stepcodeCopyLF(step,cloc,lf); }				
			
			if (stepid.equals("generate-rfx")) { result = stepcodeGenerateRfx(step,cloc,lf); }

			// The following steps have an effect on the locus
			if (stepid.equals("identify-nomvar")) { result = stepcodeIdentifyNomvar(step,cloc,lf); }
			if (stepid.equals("move-locus")) { result = stepcodeMoveLocus(step,cloc,lf); 
				cloc = result.getLocus(); // nomovelocus = false;
			}
			if (stepid.equals("replace-relation")) { result = stepcodeReplaceRelation(step,cloc,lf);}

			log(2,result.getLocus().toString());
			log(2,LFUtils.lfToString(result.getLF()));
			lf = result.getLF();
		} // end for over steps
		return result;
    } // end runActionSteps

    /**
     *  The method <i>runChooser</i> runs the given chooser, and
     *  returns the answer. The method does not internally execute any
     *  inquiry code; this is done by the method
     *  <i>runInquiryCode</i>.
     * 
     *  @see runInquiryCode
     */

    public String runChooser (UPGChooser chsr, LogicalForm lf, UPLocus lcs) throws UPGException {  
		try { 
			String result = "";
			UPGDecTree dtree = chsr.getDecTree(); 
			UPGChoiceNode nd = dtree.getRoot();
			while (!nd.isLeaf()) { 
			try { 
				// Get the inquiry, and run it to get an answer
				UPGInquiry inq = nd.getInquiry(); 
				String answer = runInquiryCode(inq,lf,lcs);
				log("Received answer is ["+answer+"]");
				// Get the child choice node that the answer triggers
				nd = nd.getChild(answer);
			} catch (UPGException e) { 
				throw new UPGException (e.getMessage()); 
			} // end try..catch
			} // while descending down the decision tree
			// out of the while we have a leaf node
			// return the result of the leaf node
			result = nd.getResult();
			return result;
		} catch (UPGException e) { 
			throw new UPGException ("Unable to execute chooser: "+e.getMessage());
		} // end try..catch
    } // end runChooser

	
	public String runChooserRandom (UPGChooser chsr, LogicalForm lf, UPLocus lcs) 
		throws UPGException 
	{  
		String result = "";
		UPGDecTree dtree = chsr.getDecTree();
		Vector dtreeAnswerSet = dtree.getAnswerSet(dtree.getRoot());
		Random generator = new Random();
		int randomIndex = generator.nextInt(dtreeAnswerSet.size());
		result = (String) dtreeAnswerSet.elementAt(randomIndex);
		log(1,"In random chooser, return answer ["+result+"] at index ["+randomIndex+"] in answer set "+dtreeAnswerSet);
		return result; 
	} // end runChooserRandom
	
    //=================================================================
    // BUILT-IN INQUIRY CODE IMPLEMENTATIONS
    //=================================================================

    /**
     *  The method <i>runInquiryCode</i> calls the function associated
     *  with the inquiry code id, and returns the answer returned from
     *  the inquiry.
     */

    public String runInquiryCode (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
		try { 
			String inqcode = inq.getCodeId();
			log(1,"Inquiry code for inquiry "+inq.getId()+": "+inqcode);
			String answer = "<unknowncodeid>";

			// Random choice 
			
			if (inq.getId().equals("random"))	{ answer = inqcodeRandom(inq,lf,lcs); } 
			
			// Boolean queries:

			if (inqcode.equals("q-deprel-type"))     { answer = inqcodeQDepRelType(inq,lf,lcs); }		
			if (inqcode.equals("q-rel-eq"))  { answer = inqcodeQRelEq(inq,lf,lcs); }
			if (inqcode.equals("q-rel-num")) { answer = inqcodeQRelNum(inq,lf,lcs); }
			if (inqcode.equals("q-ex-feat")) { answer = inqcodeQExFeat(inq,lf,lcs); }	
			if (inqcode.equals("q-ex-fv"))   { answer = inqcodeQExFV(inq,lf,lcs); }
			if (inqcode.equals("q-ex-prop")) { answer = inqcodeQExProp(inq,lf,lcs); }
//			if (inqcode.equals("q-prop"))	 { answer = inqcodeQProp(inq,lf,lcs); } assuming a nominal always has one
			if (inqcode.equals("q-ex-rel"))  { answer = inqcodeQExRel(inq,lf,lcs); }
			if (inqcode.equals("q-mod-maxevid-bool")) { answer = inqcodeQModMaxevidBool(inq,lf,lcs); }	    
			
			// Fetch 

			if (inqcode.equals("f-mod-salience")) { answer = inqcodeFModSalience(inq,lf,lcs); }
			if (inqcode.equals("f-mod-maxevid"))  { answer = inqcodeFModMaxevid(inq,lf,lcs); }
			if (inqcode.equals("f-rel-type"))     { answer = inqcodeFRelType(inq,lf,lcs); }			
			if (inqcode.equals("f-relation"))     { answer = inqcodeFRelation(inq,lf,lcs); }
			if (inqcode.equals("f-featval"))	  { answer = inqcodeFFeatVal(inq,lf,lcs); }

			if (answer.equals("<unknowncodeid>")) { 
			String[] args = new String[2];
			args[0] = lcs.getNomvar();
			args[1] = inq.getAnswerset();
			if (upAgent != null) { 
				answer = upAgent.solveRequest(inq.getId(),args);
				return answer;
			} else { 
				throw new UPGException("Unknown inquiry code id: "+inqcode);
			} // end if..else check for plug-in to handle tailored inquiry code
			} else { 
			return answer;
			} // end if..else check whether we got an answer 
		} catch (UPGException e) { 
			throw new UPGException ("Unable to run inquiry code: "+e.getMessage());
		} 
    } // end runInquiryCode

	/**
	 The method <i>inqcodeRandom</i> returns a random selection from the answerset provided in the inquiry
	*/
	
	public String inqcodeRandom (UPGInquiry inq, LogicalForm lf, UPLocus lcs) 
		throws UPGException 
	{ 
		log(2,"Running random inquiry");
		String randomAnswer = "";
		String answerSet = inq.getAnswerset();
		// Create a vector from the answer set
	    Vector answerVector = new Vector();
	    StringTokenizer st = new StringTokenizer(answerSet,"@");
	    while (st.hasMoreTokens()) { 
			String answer = (String) st.nextToken();
			answer = strip(answer);
			answerVector.addElement(answer);
	    } // end while over propositions		
		// Take a random pick from the vector with answers
		Random generator = new Random();
		int randomIndex = generator.nextInt(answerVector.size());
		randomAnswer = (String) answerVector.elementAt(randomIndex);
		log(1,"In random inquiry, return answer ["+randomAnswer+"] at index ["+randomIndex+"] in answer set "+answerSet);		
		return randomAnswer;
	} // end inqcodeRandom
	

    /**
     *  The method <i>inqcodeQRelEq</i> checks for an aspect of the
     *  nominal whether a given value holds. The "focus" parameter
     *  (stored with the inquiry) specifies what aspect (type,
     *  proposition, relation) needs to be checked. 
     *  The answerset for this inquiry is <tt>{true, false}</tt>. 
     */

    public String inqcodeQRelEq (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
	log(2,"Running inquiry code for q-rel-eq");
	String answer; 
	boolean answerbool = false;
	LFNominal nom;
	String nomvar;
	LFNominal lcn;
	// First, check whether we have to look at the current
	// nominal, or at another nominal identified by a variable
	// name in the nomvar parameter.
	String nomid = inq.getParameterValue("nomvar"); 
	if (nomid.equals("locus")) {
	    nom = lcs.getNominal();
	    nomvar = nom.nomVar;
	} else {
	    nomvar = lcs.getVariableId(nomid);
	} // end if..else check for locus/other nominal
	// Get the nominal from the global logical form
	lcn = LFUtils.lfGetNominal(lf, nomvar);
	if (lcn == null) { throw new UPGException("Nul locus in incodeQRelQ!"); }	
	if (lcn.nomVar.equals("unknown")) { 
	    throw new UPGException("Unknown locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
	} else { 
	    String focus = inq.getParameterValue("focus");
	    String val   = inq.getParameterValue("val");
	    String nomval;
	    if (focus.equals("type")) { answerbool = lcn.sort.equals(val); 
	    // System.out.println("Checking "+val+" against type of nom "+nomvar+": "+lcn.getType());
	    } 
	    if (focus.equals("prop")) { answerbool = lcn.prop.prop.equals(val); } 
	    if (focus.equals("rel"))  { 
	    	for (int i=0; i < lcn.rels.length ; i++) {
	    		if (lcn.rels[i].mode.equals(val)) {
	    			answerbool = true;
	    		}
	    	}
	    } 
	    if (answerbool) { answer = "true"; } else { answer = "false"; } 
	    log(2,"Returned answer: "+answer);
	    return answer;
	} // end if..else check whether nominal in logical form
    } // end inqcodeQRelEq

    /**
     *  The method <i>inqcodeQRelNum</i> implements a numerical
     *  comparison on feature values. The "rel" feature can specify as
     *  comparator &lt; or &gt;. The "feat" value describes the
     *  feature whose value is to be compared against the value given
     *  by the "val" parameter. The method assumes that we can
     *  translate the String values to <tt>Integer</tt> values.  The
     *  answerset for this inquiry is <tt>{true, false}</tt>.
     */
    
    public String inqcodeQRelNum (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
	log(2,"Running inquiry code for q-rel-num");
	String answer; 
	boolean answerbool = false;
	LFNominal nom = lcs.getNominal();
	String nomvar = nom.nomVar;
	LFNominal lcn = LFUtils.lfGetNominal(lf, nomvar);
	if (lcn.nomVar.equals("unknown")) { 
	    throw new UPGException("Unknown locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
	} else { 
	    try { 
		String rel  = inq.getParameterValue("rel");
		String feat = inq.getParameterValue("feat");
		String val  = inq.getParameterValue("val");
		String nomval = null;
		for (int i=0; i < lcn.feats.length ; i++) {
			if (lcn.feats[i].feat.equals(feat)) {
				nomval = lcn.feats[i].value;
			}
		}

		if (nomval != null) { 
		    int nomint = (new Integer(nomval)).intValue();
		    int valint = (new Integer(val)).intValue();
		    if (rel.equals("<")) { answerbool = (nomint < valint); } 
		    if (rel.equals(">")) { answerbool = (nomint > valint); } 
		} else {
		    throw new UPGException ("Unable to execute q-rel-num: value for feature "+feat+" unknown for locus "+nomvar);
		} // end if..else ensure value is known
		if (answerbool) { answer = "true"; } else { answer = "false"; } 
		log(2,"Returned answer: "+answer);
		return answer;
	    } catch (Exception e) {  
		throw new UPGException ("Unable to execute q-rel-num: "+e.getMessage()); 
	    } // end try catch
	} // end if..else check whether nominal in logical form
    } // end inqcodeQRelNum

    /**
     *  The method <i>inqcodeQExFeat</i> checks whether the locus
     *  nominal has a feature of the name specified by the "feat"
     *  parameter.  The answerset for this inquiry is <tt>{true,
     *  false}</tt>.
     */

    public String inqcodeQExFeat (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
	log(2,"Running inquiry code for q-ex-feat");
	String answer; 
	boolean answerbool = false;
	LFNominal nom = lcs.getNominal();
	String nomvar = nom.nomVar;
	LFNominal lcn = LFUtils.lfGetNominal(lf, nomvar);
	if (lcn.nomVar.equals("unknown")) { 
	    throw new UPGException("Unknown locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
	} else { 
	    String feat = inq.getParameterValue("feat");
	   log(1,"Feature: "+feat);
	    for (int i=0; i < lcn.feats.length ; i++) {
			if (lcn.feats[i].feat.equals(feat)) {
				answerbool = true;
			}
		}
	    if (answerbool) { answer = "true"; } else { answer = "false"; } 
	    log(2,"Returned answer: "+answer);
	    return answer;
	} // end if..else check whether nominal in logical form
    } // end inqcodeQExFeat

    /**
     *  The method <i>inqcodeQExFV</i> checks whether a nominal has a
     *  feature of the name specified by the "feat" parameter. If the
     *  nomvar attribute for the inquiry specifies "locus", then the
     *  locus nominal is inspected; otherwise we retrieve the nominal
     *  identified by the given variable name. If the nominal is
     *  known, and has the feature, the method checks whether the
     *  value for that feature equals the value specified by the
     *  inquiry's "val" parameter. The answerset for this inquiry is
     *  <tt>{true, false, unknown}</tt>. The value "unknown" is returned if the feature has no value. 
     */

    public String inqcodeQExFV (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
		log(3,"Running code for q-ex-fv");
		String answer; 
		boolean answerbool = false;
		LFNominal nom = null;
		if (!inq.getParameterValue("nomvar").equals("locus")) { 
			String nomID = lcs.getVariableId(inq.getParameterValue("nomvar"));
			log(3,"Apply inquiry q-ex-fv to nominal "+nomID);
			if (nomID.equals("unknown")) {
			throw new UPGException("Unknown nominal for variable "+inq.getParameterValue("nomvar")+" in inquiry q-ex-fv");
			} else { 
			LogicalForm glf = lcs.getLF();
			nom = LFUtils.lfGetNominal(glf, nomID);
			} // end if..else check whether variable is known. 
		} else { 
			nom = lcs.getNominal();
		} // end if..else check what nominal to take
		String nomvar = nom.nomVar;
		LFNominal lcn = LFUtils.lfGetNominal(lf, nomvar);
		if (lcn.nomVar.equals("unknown")) { 
			throw new UPGException("Unknown locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
		} else { 
			// feat is the feature asked after in the inquiry, val is the value to be checked for
			String feat = inq.getParameterValue("feat");
			String val = inq.getParameterValue("val");
			// retrieve the actual value for the feature, and store it in the variable "nomval"
		   log(1,"Feature: "+feat+"; Value: "+val);
			String nomval = null;
			for (int i=0; i < lcn.feats.length ; i++) {
				if (lcn.feats[i].feat.equals(feat)) {
					nomval = lcn.feats[i].value;
				}
			} // end for cycling over feature array 
			// first check whether we retrieved any value; null indicates "no such feature"
			if (nomval == null) { 
				// throw new UPGException("Unknown feature "+feat+" for locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
				answer = "false";  // changed from "unknown"
				return answer;
			} else { 
				// we do have a value for the feature, check whether it's equal
				answerbool = (nomval.equals(val));
				// if it's equal, return true, else return false
				if (answerbool) { answer = "true"; } else { answer = "false"; } 
				log(2,"Returned answer: "+answer);
				//  return the answer, silly. 
				return answer;
				// for those who are too lazy to read even the most basic java documentation: 
				// if you want to return the actual bloody value, just say [return nomval; ] because that's your answer. 
			} // end if..else check whether feature present!
		} // end if..else check whether nominal in logical form
    } // end inqcodeQExFV
    
   /**
     *  The method <i>inqcodeFFeatVal</i> checks whether a nominal has a
     *  feature of the name specified by the "feat" parameter. If the
     *  nomvar attribute for the inquiry specifies "locus", then the
     *  locus nominal is inspected; otherwise we retrieve the nominal
     *  identified by the given variable name. If the nominal is
     *  known, and has the feature, the method retrieves the
     *  value for that feature. The answerset for this inquiry is
     *  <tt>{"val", unknown}</tt>. The value "unknown" is returned if the feature has no value. 
     */

    public String inqcodeFFeatVal (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
		log(3,"Running code for f-featval");
		String answer; 
		boolean answerbool = false;
		LFNominal nom = null;
		if (!inq.getParameterValue("nomvar").equals("locus")) { 
			String nomID = lcs.getVariableId(inq.getParameterValue("nomvar"));
			log(3,"Apply inquiry f-featval to nominal "+nomID);
			if (nomID.equals("unknown")) {
			throw new UPGException("Unknown nominal for variable "+inq.getParameterValue("nomvar")+" in inquiry f-featval");
			} else { 
			LogicalForm glf = lcs.getLF();
			nom = LFUtils.lfGetNominal(glf, nomID);
			} // end if..else check whether variable is known. 
		} else { 
			nom = lcs.getNominal();
		} // end if..else check what nominal to take
		String nomvar = nom.nomVar;
		LFNominal lcn = LFUtils.lfGetNominal(lf, nomvar);
		if (lcn.nomVar.equals("unknown")) { 
			throw new UPGException("Unknown locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
		} else { 
			// feat is the feature asked after in the inquiry
			String feat = inq.getParameterValue("feat");
			// retrieve the actual value for the feature, and store it in the variable "nomval"
			String nomval = null;
			for (int i=0; i < lcn.feats.length ; i++) {
				if (lcn.feats[i].feat.equals(feat)) {
					nomval = lcn.feats[i].value;
				}
			} // end for cycling over feature array 
			// first check whether we retrieved any value; null indicates "no such feature"
			if (nomval == null) { 
				// throw new UPGException("Unknown feature "+feat+" for locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
				answer = "unknown";
				log(2,"Returned feature value: "+answer);
				return answer;
			} else { 
				// we do have a value for the feature, return it
				log(2,"Returned feature value: "+nomval);
				return nomval;
			} // end if..else check whether feature present!
		} // end if..else check whether nominal in logical form
    } // end inqcodeFFeatVal

	/**
     *  The method <i>inqcodeFProp</i> checks whether the locus
     *  nominal has some proposition. The answerset for this inquiry is <tt>{true,
     *  false}</tt>.
	 * taken out; assuming there always is one
     */
	
/*    public String inqcodeQProp (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
		log(2,"Running inquiry code for q-prop");
		String answer; 
		boolean answerbool = false;
		LFNominal nom = null; 
		String nomvar = inq.getParameterValue("nomvar"); 
		if (nomvar.equals("locus") || nomvar.equals("")) {
			nom = lcs.getNominal();
			nomvar = nom.nomVar;
		} else {
			nomvar = lcs.getVariableId(nomvar);
		} // end if..else check for locus/other nominal		
		
		LFNominal lcn = LFUtils.lfGetNominal(lf, nomvar);
		if (lcn.nomVar.equals("unknown")) { 
			throw new UPGException("Unknown nominal "+nomvar+" in logical form "+LFUtils.lfToString(lf));
		} else { 
			// String val = inq.getParameterValue("val");
			log(0,"Checking for any proposition at nominal ["+lcn.nomVar+"]");
			answerbool = lcn.prop.prop.equals("");
			log(0,"Does proposition eqal empty string? "+answerbool);
			answerbool = lcn.prop.prop.equals("");
			log(0,"Does proposition eqal null? "+answerbool);
			if (answerbool) { answer = "true"; } else { answer = "false"; } 
			log(2,"Returned answer: "+answer);
			return answer;
		} // end if..else check whether nominal in logical form
    } // end inqcodeQExProp
*/ 
	
	
    /**
     *  The method <i>inqcodeQExProp</i> checks whether the
     *  proposition given by the "val" parameter holds at the locus
     *  nominal. The answerset for this inquiry is <tt>{true,
     *  false}</tt>.
     */

    public String inqcodeQExProp (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
		log(2,"Running inquiry code for q-ex-prop");
		String answer; 
		boolean answerbool = false;
		LFNominal nom = null; 
		String nomvar = inq.getParameterValue("nomvar"); 
		if (nomvar.equals("locus") || nomvar.equals("")) {
			nom = lcs.getNominal();
			nomvar = nom.nomVar;
		} else {
			nomvar = lcs.getVariableId(nomvar);
		} // end if..else check for locus/other nominal		
		
		LFNominal lcn = LFUtils.lfGetNominal(lf, nomvar);
		if (lcn.nomVar.equals("unknown")) { 
			throw new UPGException("Unknown nominal "+nomvar+" in logical form "+LFUtils.lfToString(lf));
		} else { 
			String val = inq.getParameterValue("val");
			log(0,"Checking at nominal ["+lcn.nomVar+"] with propositional value ["+lcn.prop.prop+"] against inquiry value ["+val+"]");
			answerbool = lcn.prop.prop.equals(val);
			if (answerbool) { answer = "true"; } else { answer = "false"; } 
			log(2,"Returned answer: "+answer);
			return answer;
		} // end if..else check whether nominal in logical form
    } // end inqcodeQExProp

    /**
     *  The method <i>inqcodeQExRel</i> checks whether the relation
     *  given by the "val" parameter holds at the locus nominal. The
     *  answerset for this inquiry is <tt>{true, false}</tt>.
     */

    public String inqcodeQExRel (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
	log(2,"Running inquiry code for q-ex-rel");
	String answer; 
	boolean answerbool = false;
        LFNominal nom = null;
        if (!inq.getParameterValue("nomvar").equals("locus")) {
            String nomID = lcs.getVariableId(inq.getParameterValue("nomvar"));
            log(3,"Apply inquiry q-ex-rel to nominal "+nomID);
            if (nomID.equals("unknown")) {
                throw new UPGException("Unknown nominal for variable "+inq.getParameterValue("nomvar")+" in inquiry q-rel-ex");
            } else {
                LogicalForm glf = lcs.getLF();
                nom = LFUtils.lfGetNominal(glf, nomID);
            } // end if..else check whether variable is known.
        } else {
            nom = lcs.getNominal();
        } // end if..else check what nominal to take
	String nomvar = nom.nomVar;
	LFNominal lcn = LFUtils.lfGetNominal(lf, nomvar);
	if (lcn.nomVar.equals("unknown")) { 
	    throw new UPGException("Unknown locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
	} else { 
	    String val = inq.getParameterValue("val");
    	for (int i=0; i < lcn.rels.length ; i++) {
    		if (lcn.rels[i].mode.equals(val)) {
    			answerbool = true;
    		}
    	}
	    if (answerbool) { answer = "true"; } else { answer = "false"; } 
	    log(2,"Returned answer: "+answer);
	    return answer;
	} // end if..else check whether nominal in logical form
    } // end inqcodeQExRel


    /**
     *  The method <i>inqcodeQDepRelType</i> checks whether the
     *  type of dependency relation of  the given nominal is the given type. 
	 *  The inquiry looks for an argument "nomvar"
     * 
     */
	
    public String inqcodeQDepRelType (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
		log(2,"Running inquiry code for q-deprel-type");
		String answer = "unknown"; 
		LFNominal nom = lcs.getNominal();
		if (nom == null) { throw new UPGException("Unknown locus in inquiry q-deprel-type"); }
		String depMode = inq.getParameterValue("mode");
		String nomvar = inq.getParameterValue("nomvar");
		LFNominal depNom = LFUtils.lfGetNominal(lf, nomvar);
		if (depNom == null) { 
			depNom = LFUtils.lfGetNominal(lf,lcs.getVariableId(nomvar));
			if (depNom == null) { 
				// then go and take the locus
				nomvar = nom.nomVar;
				log(0,"Checking for the relation type ["+depMode+"] to ["+nom.nomVar+"]");								
			} else { 
				log(0,"Checking for the relation type ["+depMode+"] to ["+depNom.nomVar+"]");				
			} 
		}
		boolean depFound = false; 		
		for(Iterator<LFNominal> nomsIter = LFUtils.lfGetNominals(lf); nomsIter.hasNext(); ) { 
			LFNominal lfNom = nomsIter.next();

			for (Iterator<LFRelation> relsIter = LFUtils.lfNominalGetRelations(lfNom); relsIter.hasNext(); ) { 
				LFRelation rel = relsIter.next(); 
				// log(0,"Checking against ["+nomvar+"] against relation dependent ["+rel.dep+"]");
				if (rel.dep.equals(nomvar)) { 
					depFound = true; 
					answer = rel.mode; 
					break;
				} // end if.. check whether found
			} // end for over relations
			if (depFound) { break; } 
		} // end for
		if (depFound) { 
			if (answer.equals(depMode)) { 
				answer = "true"; 
			} else { 
				answer = "false";
			} // check whether if found then correct
		} else { 
			answer = "false";
		} // end if..else check whether any found at all
	    log(2,"Returned answer: "+answer);			
		return answer;		
    } // end inqcodeFRelType
	
	
	
	
    /**
     *  The method <i>inqcodeFRelType</i> inquires after the
     *  type of dependency relation  the given nominal is under some head. 
	 *  The inquiry looks for an argument "nomvar"
     * 
     */
	
    public String inqcodeFRelType (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
		log(2,"Running inquiry code for f-rel-type");
		String answer = "unknown"; 
		LFNominal nom = lcs.getNominal();
		if (nom == null) { throw new UPGException("Unknown locus in inquiry f-rel-type"); }
		String nomvar = inq.getParameterValue("nomvar");
		LFNominal depNom = LFUtils.lfGetNominal(lf, nomvar);		
		if (depNom == null) { 
			depNom = LFUtils.lfGetNominal(lf,lcs.getVariableId(nomvar));
			if (depNom == null) { 
				// then go and take the locus
				nomvar = nom.nomVar;
				log(0,"Checking for the type of relation to ["+nom.nomVar+"]");								
			} else { 
				log(0,"Checking for the type of relation to ["+depNom.nomVar+"]");				
			} 
		}

		for(Iterator<LFNominal> nomsIter = LFUtils.lfGetNominals(lf); nomsIter.hasNext(); ) { 
			LFNominal lfNom = nomsIter.next();
			boolean depFound = false; 
			for (Iterator<LFRelation> relsIter = LFUtils.lfNominalGetRelations(lfNom); relsIter.hasNext(); ) { 
				LFRelation rel = relsIter.next(); 
				// log(0,"Checking against ["+nomvar+"] against relation dependent ["+rel.dep+"]");
				if (rel.dep.equals(nomvar)) { 
					depFound = true; 
					answer = rel.mode; 
					break;
				} // end if.. check whether found
			} // end for over relations
			if (depFound) { break; } 
		} // end for
	    log(2,"Returned answer: "+answer);			
		return answer;		
    } // end inqcodeFRelType
	
	
	
    /**
     *  The method <i>inqcodeFModSalience</i> inquires after the
     *  salience of the given nominal in the modality specified by the
     *  "mod" parameter. The answerset for this inquiry is an integer
     *  converted to a String.
     * 
     *  <h4>Warning</h4> This code has not yet been implemented!! Defaults to "0";
     */

    public String inqcodeFModSalience (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
	log(2,"Running inquiry code for f-mod-salience");
	String answer; 
	boolean answerbool = false;
	LFNominal nom = lcs.getNominal();
	String nomvar = nom.nomVar;
	LFNominal lcn = LFUtils.lfGetNominal(lf, nomvar);
	if (lcn.nomVar.equals("unknown")) { 
	    throw new UPGException("Unknown locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
	} else { 
	    // System.out.println("WARNING: Code for f-mod-salience not yet implemented; default answer is 0");
	    // ==============================
	    // WARNING: Default answer!!
	    // ------------------------------

	    String[] args = new String[1];
	    args[0] = nomvar;
	    answer = upAgent.solveRequest("f-mod-salience",args);

	    // ==============================
	    // log(2,"Returned answer: "+answer+" (*DEFAULTING*)");
	    return answer;
	} // end if..else check whether nominal in logical form
    } // end inqcodeQModSalience

    /**
     *  The method <i>inqcodeFModMaxevid</i> inquires after the
     *  maximal ("strongest") evidencing modality for the given
     *  nominal. The answerset for this inquiry is a String.
     * 
     *  <h4>Warning</h4> This code has not yet been implemented!! Defaults to "vision";
     */

    public String inqcodeFModMaxevid (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
	log(2,"Running inquiry code for f-mod-maxevid");
	String answer; 
	boolean answerbool = false;
	LFNominal nom = lcs.getNominal();
	String nomvar = nom.nomVar;
	LFNominal lcn = LFUtils.lfGetNominal(lf, nomvar);
	if (lcn.nomVar.equals("unknown")) { 
	    throw new UPGException("Unknown locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
	} else { 
	    // System.out.println("WARNING: Code for f-mod-maxevid not yet implemented; default answer is \"vision\"");
	    // ==============================
	    // WARNING: Default answer!!
	    // ------------------------------

	    String[] args = new String[1];
	    args[0] = nomvar;
	    answer = upAgent.solveRequest("f-mod-maxevid",args);

	    // ==============================
	    log(2,"Returned answer: "+answer+" (*DEFAULTING*)");
	    return answer;
	} // end if..else check whether nominal in logical form
    } // end inqcodeFModMaxevid

    /**
     *  The method <i>inqcodeQModMaxevidBool</i> inquires whether the
     *  modality given by the "mod" parameter stored at the inquiry is
     *  the maximal ("strongest") evidencing modality for the given
     *  nominal. The answerset for this inquiry is <tt>{true,false}</tt>. 
     * 
     *  <h4>Warning</h4> This code has not yet been implemented!! Defaults to "true";
     */

    public String inqcodeQModMaxevidBool (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
	log(2,"Running inquiry code for q-mod-maxevid-bool");
	String answer; 
	boolean answerbool = false;
	LFNominal nom = lcs.getNominal();
	String nomvar = nom.nomVar;
	LFNominal lcn = LFUtils.lfGetNominal(lf, nomvar);
	if (lcn.nomVar.equals("unknown")) { 
	    throw new UPGException("Unknown locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
	} else { 
	    // System.out.println("WARNING: Code for q-mod-maxevid-bool not yet implemented; default answer is \"true\"");
	    // ==============================
	    // WARNING: Default answer!!
	    // ------------------------------

	    String[] args = new String[2];
	    args[0] = nomvar;
	    args[1] = "modality";
	    answer = upAgent.solveRequest("q-mod-maxevid-bool",args);

	    // ==============================
	    log(2,"Returned answer: "+answer+" (*DEFAULTING*)");
	    return answer;
	} // end if..else check whether nominal in logical form
    } // end inqcodeQModMaxevidBool


    public String inqcodeFRelation (UPGInquiry inq, LogicalForm lf, UPLocus lcs) throws UPGException { 
	log(2,"Running inquiry code for f-relation");
	String answer; 
	boolean answerbool = false;
	LFNominal nom = lcs.getNominal();
	String nomvar = nom.nomVar;
	LFNominal lcn = LFUtils.lfGetNominal(lf, nomvar);
	if (lcn.nomVar.equals("unknown")) { 
	    throw new UPGException("Unknown locus "+nomvar+" in logical form "+LFUtils.lfToString(lf));
	} else { 
	    Iterator rIter = Arrays.asList(nom.rels).iterator();
	    answer = "unknown";
	    // Just get the first relation on the list
	    while (answer.equals("unknown") && rIter.hasNext()) {
		LFRelation arel = (LFRelation) rIter.next();
		answer = arel.mode;
	    } // end while
	    return answer;
	} // end if..else check whether nominal in logical form
    } // end inqcodeFRelation



    //=================================================================
    // BUILT-IN ACTION STEP CODE IMPLEMENTATIONS
    //=================================================================


    /**
     *  The method <i>stepcodeAddLF</i> looks whether the step
     *  provides a logical form as value for the "lf" attribute, or
     *  specifies a source modality ("src") whose results specify a
     *  logical form that needs to be used. The logical form (from
     *  "lf" or "src") is then added to the logical form being
     *  planned, and we unify the root of the added logical form with
     *  the nominal variable specified as value for the "dest"
     *  attribute. The method returns a <tt>UPResult</tt> object that
     *  includes the new logical form. If the destination specified
     *  the locus as the variable to unify the root of the logical
     *  form to be added, then the locus is changed to this root (else
     *  the locus would no longer correspond to a nominal that is in
     *  the logical form!); else, the locus remains unchanged (modulo
     *  noting the new nominals being inserted through the logical
     *  form).
     */

    public UPResult stepcodeAddLF (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException {  
	log(1,"Running code for action step add-lf");
	UPResult result = new UPResult (lcs,lf); 

	// check whether we have an lf, or whether we need to retrieve
	// one through the request handler
	LogicalForm lfadd = null; 
	String lfstr = step.getAttributeValue("lf"); 
	log(1,"Add-lf: lf attribute has value ["+lfstr+"]");
	if (lfstr.equals("unknown") || !lfstr.startsWith("@")) { 
	    String src = step.getAttributeValue("src"); 
	    if (!src.equals("unknown")) { 
		String[] args = new String[1];
		args[0] = src;
		if (!lfstr.equals("unknown") && lfstr.startsWith("*")) { 
		    String varname = lfstr.substring(1,lfstr.lastIndexOf("*")-1);
		    String var = lcs.getVariableId(varname);
		    log("Parametrizing getLF with variable ["+var+"]");
		    args[1] = var;
		} // end if.. check for 
		log(1,"Add-lf: src attribute has value ["+args[0]+"] and lf attribute ["+lfstr+"]");
		lfstr = upAgent.solveRequest("getLF",args);
		log(1,"Add-lf: retrieved lf has value ["+lfstr+"]");
		if (!lfstr.equals("")) { 
		    lfadd = LFUtils.convertFromString(lfstr);
		    log(1,"Constructed logical form: "+lfadd.toString());
		} else { 
		    throw new UPGException("Unable to execute stepcode add-lf: No lf for source ["+src+"] with the request handler");
		} // end if..else check an lf has been retrieved
	    } else { 
		throw new UPGException("Unable to execute stepcode add-lf: Empty src and lf attributes");
	    } // end if..else check for src
	} else { 
	    lfadd = LFUtils.convertFromString(lfstr);
	} // end if..else check for lf
	// If all has gone well, we now have a logical form -- else
	// things will have exited through exceptions, or crashed ...
	// Get the destination, add the logical form to the existing
	// LF in the result, and unify the root of the added LF with
	// the nominal identified as destination. Check whether this
	// is the locus (*LOCUS*) or another variable.

	// Get the destination
	String dest = step.getAttributeValue("dest");
	log(1,"Value of the attribute dest is ["+dest+"]");
	if (dest.equals("unknown")) { 
	    throw new UPGException("Unable to execute stepcode add-lf: Empty dest attribute for attachment destination");
	} else { 
	    // Add the nominals
	    Iterator nomIter = Arrays.asList(lfadd.noms).iterator();
	    while (nomIter.hasNext()) { 
		LFNominal nom = (LFNominal) nomIter.next();
		
		for (int i=0; i < lf.noms.length ; i++) {
			if (lf.noms[i].nomVar.equals(nom.nomVar)) { 
			    log(3,"Warning: lf already contains nominal "+nom.nomVar+" to be added! Old nominal will be overridden!");	
			}
		} // end if.. check for double lf!
		lf.noms = LFUtils.lfAddNominal(lf, nom);
		lcs.addInsertedNominal(nom);
	    } // end while over nominals to be added
	    log(1,"LF after adding nominals: "+LFUtils.lfToString(lf));



	    if (dest.equals("*LOCUS*")) { 
			log(1,"Destination is locus!");
			// Replace the mention of the locus name with the root name
			String locnom  = lcs.getNomvar();
			LFNominal addroot = lfadd.root;
			String rootvar = addroot.nomVar;
			log(1,"Replacing "+locnom+" with "+rootvar+" in the lf");
			LFUtils.lfReplaceNomvar(lf.noms, locnom, rootvar);
			log(1,"LF after replacing nominal variables: "+LFUtils.lfToString(lf));
			log(1,"Removing "+locnom+" from the lf");		
			LFUtils.lfRemoveNominal(lf.noms, locnom);
			log(1,"LF after removing nominal variable "+locnom+": "+LFUtils.lfToString(lf));
			// Note that the locus now no longer corresponds to an existing nominal!
			// So we need to change the locus to the root nominal
			lcs.setNomvar(addroot);
	    } else { 
			String destnom = lcs.getVariableId(dest);
			log(1,"Destination resolves to the nominal variable "+destnom);
			if (destnom.equals("unknown")) { 
				throw new UPGException("Variable "+dest+" for destination is unknown in the locus");
			} else {
				// this is where we replace the destination variable with the root of the provided LF
				String addroot = lfadd.root.nomVar;
				LFUtils.lfReplaceNomvar(lf.noms,destnom,addroot);
				LFUtils.lfRemoveNominal(lf.noms, destnom);
				// make sure that the variable name now points to the new root
				lcs.addVariableId(dest,addroot,"global");	
			} // end if..else check for destination variable
	    } // end if..else check for kind of destination
	} // end if.. check for destination set
	

	// At this point, we have added the logical form to the main
	// logical form, and have replaced the destination with the
	// root of the logical form to be added. If the destination
	// was the locus, then that means that at this point, the
	// locus has been changed to point to the root of the added
	// logical form. Furthermore, the locus has been updated with
	// the new nominals as "inserted" nominals, to ensure that
	// systems can -if necessary- be put onto the agenda.

	result.setLF(lf);
	result.setLocus(lcs);
	log(1,"New LF: "+LFUtils.lfToString(lf));

	return result;
    } // end stepcodeAddLF

    /**
     *  The method <i>stepcodeAssignType</i> looks at the type stored
     *  in the "type" parameter with the step, and assigns this type
     *  in the logical form to the nominal that is the current
     *  locus. The method returns a <tt>UPResult</tt> object that
     *  includes the new logical form, and the (unchanged) locus.
     */

    public UPResult stepcodeAssignType (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException {  
	log(1,"Running code for action step assign-type");
	UPResult result = new UPResult (lcs,lf); 
	// Get the destination
	String lnomv = step.getAttributeValue("dest");
	log(1,"Destination for assigning type is ["+lnomv+"]");
	if (lnomv.equals("unknown")) {  // destination not given
	    log(0,"Empty dest attribute for type assignment; taking locus as default");  
     	// Get the name of the nominal that is currently the locus
		lnomv  = lcs.getNomvar(); 
		// Get the LFNominal object for this name from the LF
	}
		
	// Get the LFNominal object for this name from the LF
	LFNominal nom = LFUtils.lfGetNominal(lf, lnomv);
	if (nom.nomVar.equals("unknown")) { 
	    throw new UPGException ("Unable to execute stepcode assign-type: Unknown nominal "+lnomv+" as locus");
	} // end if check whether nominal is known 
	String type = step.getAttributeValue("type");
	if (type.equals("unknown")) { 
	    throw new UPGException ("Unable to execute stepcode assign-type: Attribute \"type\" not assigned in step");
	} else { 
	    // Update the type of the nominal, and update the logical form
	    nom.sort = type;
	    lf.noms = LFUtils.lfAddNominal(lf.noms, nom);
	    result.setLF(lf);
	    log(1,"New LF: "+LFUtils.lfToString(lf));
	} // end if..else check for type
	return result;
    } // end stepcodeAssignType

    /**
     *  The method <i>stepactionAddProposition</i> looks at the
     *  proposition stored in the "propositions" parameter with the
     *  step, and assigns the propositions in the logical form with
     *  the nominal that is the current locus. The method returns a
     *  <tt>UPResult</tt> object that includes the new logical form,
     *  and the (unchanged) locus.
     */

    public UPResult stepcodeAddProposition (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException {  
		log(1,"Running code for action step add-proposition");
		UPResult result = new UPResult (lcs,lf); 
		// Get the destination
		String lnomv = step.getAttributeValue("dest");
		log(1,"Destination for adding proposition(s) is ["+lnomv+"]");
		if (lnomv.equals("unknown")) {  // destination not given
			log(0,"Empty dest attribute for attachment destination in add-proposition; taking locus as default");  
			// Get the name of the nominal that is currently the locus
			lnomv  = lcs.getNomvar(); 
			// Get the LFNominal object for this name from the LF
		} 
		LFNominal nom = LFUtils.lfGetNominal(lf, lnomv);
		if (nom == null) { 	
			lnomv = lcs.getVariableId(lnomv);
			nom = LFUtils.lfGetNominal(lf, lnomv);
		}	
		if (nom == null || nom.nomVar.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode add-proposition: Unknown nominal "+lnomv+" given locus");
		} // end if check whether locus nominal is known 

		// Now the NOM has been properly initialized, either to DEST or to LOCUS

		String props = step.getAttributeValue("propositions");
		if (props.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode add-proposition: Attribute \"propositions\" not assigned in step");
		} else { 
			log(0,"Parsing a list of propositions ["+props+"]");
			// Create a Vector from the list of propositions
			Vector pvec = new Vector();
			StringTokenizer st = new StringTokenizer(props,"@");
			while (st.hasMoreTokens()) { 
			String prop = (String) st.nextToken();
			log(0,"Add proposition \""+prop+"\"");
			prop = strip(prop);
			pvec.addElement(prop);
			} // end while over propositions
			// Add the elements from the vector to the nominal, and update the logical form
			for (Iterator pIter = pvec.iterator(); pIter.hasNext(); ) { 
			String prop = (String) pIter.next();
			nom.prop.prop = prop;   // HERE IS/WAS THE ERROR
			} // end for over propositions
			lf.noms = LFUtils.lfAddNominal(lf, nom);
			result.setLF(lf);
			log(1,"New LF: "+LFUtils.lfToString(lf));
		} // end if..else check for propositions parameter set
		return result;
    } // end stepcodeAddProposition


    /**
     *  The method <i>stepactionAddFeature</i> looks at the
     *  feature in the "feature" parameter stored with the
     *  step, and the value in the "value" parameter. It then assigns
        the feature with the given value in the logical form to
     *  the nominal that is the current locus. The method returns a
     *  <tt>UPResult</tt> object that includes the new logical form,
     *  and the (unchanged) locus.
     */

    public UPResult stepcodeAddFeature (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException {  
	log(1,"Running code for action step add-feature");
	UPResult result = new UPResult (lcs,lf); 

/** orig:
*	// Get the name of the nominal that is currently the locus
*	String lnomv  = lcs.getNomvar(); 
*	// Get the LFNominal object for this name from the LF
*	LFNominal nom = LFUtils.lfGetNominal(lf, lnomv);
*	if (nom.nomVar.equals("unknown")) { 
*	    throw new UPGException ("Unable to execute stepcode add-feature: Unknown nominal "+lnomv+" as locus");
*	} // end if check whether nominal is known 
*/


	// Get the destination
	String lnomv = step.getAttributeValue("dest");

	log(1,"Destination for adding a feature is ["+lnomv+"]");
	if (lnomv.equals("unknown")) {  // destination not given
	    log(0,"Empty dest attribute for attachment destination in add-feature; taking locus as default");  
     	// Get the name of the nominal that is currently the locus
		lnomv  = lcs.getNomvar(); 
		// Get the LFNominal object for this name from the LF
	}
	LFNominal nom = LFUtils.lfGetNominal(lf, lnomv);
	// if null then try to get the nominal through the assignment to the variable
	if (nom == null) { 
		nom = LFUtils.lfGetNominal(lf, lcs.getVariableId(lnomv));		
	} 
	log(1,"nom:"+nom);
	if (nom.nomVar.equals("unknown")) { 
	    throw new UPGException ("Unable to execute stepcode add-feature: Unknown nominal "+lnomv+" as locus");
	} // end if check whether locus nominal is known 

	// Now the NOM has been properly initialized, either to DEST or to LOCUS

	String feat = step.getAttributeValue("feature");
	if (feat.equals("unknown")) { 
	    throw new UPGException ("Unable to execute stepcode add-feature: Attribute \"feature\" not assigned in step");
	} else { 
	    String val = step.getAttributeValue("value");
	    if (val.equals("unknown")) { 
		throw new UPGException ("Unable to execute stepcode add-feature: Attribute \"value\" not assigned in step");
	    } else { 
	    int oldSize = nom.feats.length ;
	    nom.feats = (Feature[]) LFUtils.resizeArray(nom.feats, oldSize+1);
		nom.feats[oldSize] = new Feature();
		nom.feats[oldSize].feat = feat;
		nom.feats[oldSize].value = val;
		lf.noms = LFUtils.lfAddNominal(lf, nom);
		result.setLF(lf);
		log(1,"New LF: "+LFUtils.lfToString(lf));
	    } // end if..else check whether feature and value set
	} // end if..else check for feature parameter set
	return result;
    } // end stepcodeAddFeature 

    /**
     *  The method <i>stepactionAddRelation</i> looks at the relation
     *  in the "relation" parameter stored with the step, and the
     *  variable in the "nomvar" parameter.  The method then checks
     *  whether the variable is known: it needs to have been
     *  identified before with a nominal in the logical form. This
     *  information is stored with the current locus. If the nominal
     *  can be identified, the method adds the relation. Depending on
     *  the orientation of the relation, this can be from the given
     *  nominal in the logical to the nominal that is the current
     *  locus, or the other way round. The method returns a
     *  <tt>UPResult</tt> object that includes the new logical form,
     *  and the (unchanged) locus.
     */

    public UPResult stepcodeAddRelation (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException {  
		log(1,"Running code for action step add-relation");
		UPResult result = new UPResult (lcs,lf); 
		// Get the name of the nominal that is currently the locus
		String lnomv  = lcs.getNomvar(); 
		// Get the LFNominal object for this name from the LF
		LFNominal nom = LFUtils.lfGetNominal(lf, lnomv);
		if (nom.nomVar.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode add-relation: Unknown nominal "+lnomv+" as locus");
		} // end if check whether nominal is known 
		// First, get the variable
		String var = step.getAttributeValue("nomvar");
		if (var.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode add-relation: Attribute \"nomvar\" not assigned in step");
		} else { 
			// okay, we have a variable. now we need to check whether
			// it is a variable (type="*VAR*") or whether we should
			// introduce a new nominal in the logical form.
			String type = step.getAttributeValue("type");
			log("Type: "+type);
			if (type.equals("*VAR*") || type.equals("unknown")) { 
				// We are dealing with a variable. 
				// Next, check whether the variable is known
				String nomvar = lcs.getVariableId(var);
				if (nomvar.equals("unknown")) { 
					throw new UPGException ("Unable to execute stepcode add-relation: variable \""+var+"\" unknown in current locus");
				} else { 
					// Finally, check whether we have a relation
					String rel = step.getAttributeValue("mode");
					if (rel.equals("unknown")) { 
					throw new UPGException ("Unable to execute stepcode add-relation: Attribute \"mode\" not assigned in step");
					} else { 
					// Now check the orientation of the relation to be
					// introduced, and with which nominal to add the
					// relation                                                                                                                                 
					String orientation = step.getAttributeValue("orientation");
					if (orientation.equals("src")) {
						LFRelation relation = new LFRelation();
						relation.head = nomvar;
						relation.mode = rel;
						relation.dep = nom.nomVar;
						LogicalForm glf = lcs.getLF();
						LFNominal headnom = LFUtils.lfGetNominal(glf, nomvar);
						if (headnom.nomVar.equals("unknown")) { 
						throw new UPGException("Unable to execute stepcode add-relation: Nominal variable "+nomvar+" unknown in LF");
						} else { 
							int oldSize = headnom.rels.length ;
						headnom.rels = (LFRelation[]) LFUtils.resizeArray(headnom.rels, oldSize + 1);
						headnom.rels[oldSize] = relation;
							
							log("Added a new relation: ["+relation.head+"/"+relation.mode+"/"+relation.dep+"]");													
						} // end if..else check that nominal is known
					} else {
						LFRelation relation = new LFRelation ();
						relation.head = nom.nomVar;
						relation.mode = rel;
						relation.dep = nomvar;
						int oldSize = nom.rels.length ;
						nom.rels = (LFRelation[]) LFUtils.resizeArray(nom.rels, oldSize + 1);
						nom.rels[oldSize] = relation;
						
						log("Added a new relation: ["+relation.head+"/"+relation.mode+"/"+relation.dep+"]");						
						
					} // end if..else check for orientation   
					lf.noms = LFUtils.lfAddNominal(lf, nom);
					result.setLF(lf);
					log(1,"New LF: "+LFUtils.lfToString(lf));
					} // end if..else check whether relation has been set in mode parameter
				} // end if..else check whether nomvar is known in the locus
			} else { 
				// We are dealing with a new nominal. 
				LFNominal newnom = LFUtils.newLFNominal ();
				newnom.nomVar = var;
				newnom.sort = type;
				log("Introduce new variable "+var+" of type "+type);
				// Finally, check whether we have a relation
				String rel = step.getAttributeValue("mode");
				if (rel.equals("unknown")) { 
					throw new UPGException ("Unable to execute stepcode add-relation: Attribute \"mode\" not assigned in step");
				} else { 
					// Now check the orientation of the relation to be
					// introduced, and with which nominal to add the
					// relation
					String orientation = step.getAttributeValue("orientation");
					log(3,"Orientation of the relation: "+orientation);
					if (orientation.equals("src")) { 
						LFRelation relation = new LFRelation ();
						relation.head = var;
						relation.mode = rel;
						relation.dep = nom.nomVar;
						
						int oldSize = newnom.rels.length ;
						newnom.rels = (LFRelation[]) LFUtils.resizeArray(newnom.rels, oldSize + 1);
						newnom.rels[oldSize] = relation;
						
						// System.out.println("Newnom: "+newnom.toString());
						lf.noms = LFUtils.lfAddNominal(lf, nom);
						lf.noms = LFUtils.lfAddNominal(lf, newnom);
						// now check whether we need to adjust the root!
						if (lf.root.nomVar.equals(nom.nomVar)) { 
							lf.root = newnom;
						} // end if.. check whether the nom is the root
						
						log("Added a new relation: "+relation);
						
					} else { 
						LFRelation relation = new LFRelation ();
						relation.head = nom.nomVar;
						relation.mode = rel;
						relation.dep = var;
						log(1,"Adding new relation: ["+relation.mode+"] from ["+relation.head+"] to ["+relation.dep+"] ");
						int oldSize = nom.rels.length ;
						nom.rels = (LFRelation[]) LFUtils.resizeArray(nom.rels, oldSize + 1);
						nom.rels[oldSize] = relation;
						lf.noms = LFUtils.lfAddNominal(lf, nom);
						lf.noms = LFUtils.lfAddNominal(lf, newnom);
						log(1,"Adding new nominal: "+LFUtils.lfNominalToString(newnom));
						} // end if..else check for orientation
						// System.out.println("Lf: "+LFUtils.lfToString(lf));
					
						lcs.addInsertedNominal(newnom);
						result.setLF(lf);
						result.setLocus(lcs);
						log(1,"New LF: "+LFUtils.lfToString(lf));		    
				} // end if..else check whether relation has been set in mode parameter
			} // end if..else check whether we are dealing with a variable or a new nominal
		} // end if..else check for nomvar parameter set
		return result;
    } // end stepcodeAddRelation


    /**
     *  The method <i>stepcodeAdjoinLF</i> adjoins an auxiliary
     *  logical form LFA, given in the "lf" parameter of the step,
     *  into the logical form of the planner LFP. The point at which
     *  LFA is adjoined into LFP is given by the "rel" parameter. This
     *  relation identifies the nominal where LFA is going to be
     *  adjoined into LFP. The step also identifies the foot of LFA,
     *  which will be attached to the point where the remainder of LFP
     *  attaches.
     *
     * <p>
     * 
     * We need to make several checks and steps: 
     * 
     * <ol> 
     * <li> Get the dependent nominal of the given relation under the locus in the LFP </li>
     * <li> Reconstruct the LFA from its string representation </li>
     * <li> Get the nominal in the LFA identified as the foot </li>
     * <li> <b>Check</b>: Ensure that the foot nominal and the dependent nominal are of the same type </li>
     * <li> Add the foot identifier as variable name pointing to the dependent nominal, in the locus </li>
     * <li> Get the root of the nominal in the LFA </li> 
     * <li> <b>Check</b>: Ensure that the root nominal and the dependent nominal have the same (super) type </li>
     * <li> Detach the dependent from the current locus, and have the relation to the root of the LFA instead </li>

     * <li> Identify the dependent with the foot in the LFA: this
     * means (i) check that the foot is a nominal without features,
     * propositions, or further relations, and if that is the case,
     * (ii) replace any mention of this nominal in the LFA by a
     * pointer to the dependent</li>

     * <li> Update the LF with all the nominals from the auxiliary
     * LF. We first check whether any of the names assigned to
     * nominals in the namespace of the auxiliary logical form are
     * also used in the namespace of the (main) logical form. If this
     * is the case, then we rename all the nominals in the auxiliary
     * logical form before adding them. </li> 
     * 
     * </ol>
     */

    public UPResult stepcodeAdjoinLF (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException {  
		log(1,"Running code for action step adjoin-lf");
		UPResult result = new UPResult (lcs,lf); 
		// Get the name of the nominal that is currently the locus
		String lnomv  = lcs.getNomvar(); 
		// Get the LFNominal object for this name from the LF
		LFNominal nom = LFUtils.lfGetNominal(lf, lnomv);
		if (nom.nomVar.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode adjoin-lf: Unknown nominal "+lnomv+" as locus");
		} // end if check whether nominal is known 
		// Next, get the adjunction node through the relation ("rel"),
		// the auxiliary LF to be adjoined into the lf ("lf"), and the
		// foot of the auxiliary ("foot"). Throw exceptions if they're
		// not set.
		String lfa = step.getAttributeValue("lf");
		if (lfa.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode adjoin-lf: Parameter \"lf\" not set");
		} // end if.. check whether the auxiliary LF has been provided
		String footid = step.getAttributeValue("foot");
		if (footid.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode adjoin-lf: Parameter \"foot\" not set");
		} // end if.. check whether foot has been set
		String relid = step.getAttributeValue("mode");
		if (relid.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode adjoin-lf: Parameter \"rel\" not set");
		} // end if.. check whether relation has been set
		// Now we have the lfa, the foot in the lfa, and the relation
		// under the locus in the current lf.

		// 1) Get the dependent nominal of the given relation under
		// the locus in the LFP

		LFNominal dep = LFUtils.newLFNominal();
		LFRelation depRel = LFUtils.lfNominalGetRelation(nom,relid);
		if (depRel == null) { 
			throw new UPGException("Relation \""+relid+"\" unknown under current locus "+lnomv);
		} else { 
			dep = LFUtils.lfGetNominal(lf, depRel.dep);
		} // end if..else check whether the relation was present
		if (dep == null) { 
			throw new UPGException("Cannot retrieve dependent nominal under relation ["+relid+"] under current locus "+lnomv);
		} 
		// Now the dependent nominal is known, and has been set (else an exception is thrown)

		// 2) Reconstruct the LFA from its string representation 
		LogicalForm auxlf = LFUtils.convertFromString(lfa);

		log(0,"Auxiliary logical form: "+LFUtils.lfToString(auxlf));
		
		
		// 3) Get the nominal in the LFA identified as the foot 
		LFNominal auxfoot = LFUtils.lfGetNominal(auxlf, footid);
		if (auxfoot == null) { 
			throw new UPGException("Foot id \""+footid+"\" unknown as nominal in ["+lfa+"]");
		} // end if.. check whether foot is identifiable in the auxiliary LF
		
		// 4) Check: Ensure that the foot nominal and the dependent nominal are of the same sort 
		String foottype = auxfoot.sort;
		String deptype  = dep.sort;
		if (!foottype.equals(deptype)) { 
			throw new UPGException("Auxiliary LF foot type ["+foottype+"] does not equal dependent type ["+deptype+"]");	
		} // end if.. check for type equality between foot and dependent

		// 5) Add the foot identifier as variable name pointing to the dependent nominal, in the locus 
		//lcs.addVariableId(footid,dep.nomVar);

		// 6) Get the root of the nominal in the LFA 
		LFNominal auxroot = auxlf.root;
		
		// 7) Check: Ensure that the root nominal and the dependent nominal are of the same sort -- THIS SHOULD NOT BE CHECKED
		/**
		String roottype = auxroot.sort; 
		if (!roottype.equals(deptype)) { 
			log(0,"Warning: The auxiliary LF has a root of type \""+roottype+"\" different from dependent type \""+deptype+"\"");
		} // end if check for equality
		*/
		 
		 
		// 8) Detach the dependent from the current locus, and have the relation to the root of the LFA instead
		depRel.dep = auxroot.nomVar;
		// update the relation in the nominal -- remove the old occurrence, put in the new relation
		nom.rels = LFUtils.lfNominalRemoveRelation(nom,depRel.mode);
		nom.rels = LFUtils.lfNominalAddRelation(nom,depRel);
		// update the nominal in the logical form
		lf.noms = LFUtils.lfUpdateNominal(lf.noms,nom);
		
		
		
		// 9) Identify the dependent with the foot in the LFA: this
		// means (i) check that the foot is a nominal without
		// features, propositions, or further relations, and if that
		// is the case, (ii) replace any mention of this nominal in
		// the LFA by a pointer to the dependent

		if (auxfoot.feats.length > 0) {
			log(0,"Non-empty auxiliary node will be replaced: nominal \""+auxfoot.nomVar+"\" has features!");
		} // end if check whether features
		if (!auxfoot.prop.prop.equals("")) {
			log(0,"Non-empty auxiliary node will be replaced: nominal \""+auxfoot.nomVar+"\" has propositions!");
		} // end if check whether propositions
		if (auxfoot.rels.length > 0) {
			log(0,"Non-empty auxiliary node will be replaced: nominal \""+auxfoot.nomVar+"\" has relations!");
		} // end if check whether relations
		
		
		log(0,"Would need to replace nomvar ["+auxfoot.nomVar+"] with ["+dep.nomVar+"]");
		auxlf.noms = LFUtils.lfReplaceNomvar(auxlf.noms, auxfoot.nomVar, dep.nomVar);
		
		log(0,"Auxiliary LF: "+LFUtils.lfToString(auxlf));
		
		
		// 10) Update the LF with all the nominals from the auxiliary LF

		boolean namespaceclash = false;
		Iterator nIter = Arrays.asList(auxlf.noms).iterator(); 
		while (nIter.hasNext()) { 
			LFNominal auxnv = (LFNominal) nIter.next(); 
			if (LFUtils.lfHasNomvar(lf, auxnv.nomVar)) { 
			namespaceclash = true; 
			} // end if check for namespace clash 
		} // end while over nominals in auxlf
		if (namespaceclash) { 
			log(0,"Warning: Undealt with namespace clash between AUXLF and LF!"); 
		} // end if rename nominals
		nIter = Arrays.asList(auxlf.noms).iterator(); 
		while (nIter.hasNext()) { 
			LFNominal auxnv = (LFNominal) nIter.next(); 
			if (!auxnv.nomVar.equals(auxfoot.nomVar) && !auxnv.nomVar.equals(dep.nomVar)) { 
				lf.noms = LFUtils.lfAddNominal(lf, auxnv);
				lcs.addInsertedNominal(auxnv);
				log("Adding nominal "+LFUtils.lfNominalToString(auxnv));
			}
		} // end while over nominals in auxlf
		
		log(0,"Resulting LF after adjoining: "+LFUtils.lfToString(lf));
		
		result.setLF(lf);
		result.setLocus(lcs);
		return result;
    } // end stepcodeAdjoinLF

    /**
     *  The method <i>stepcodeGenerateRfx</i> calls a function
     *  <i>generateRfx</i> which deals with the generation of
     *  referring expressions. 
     */

    public UPResult stepcodeGenerateRfx (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException {  
		log(1,"Running code for action step generate-rfx");
		UPResult result = new UPResult (lcs,lf); 
		LFNominal nom = null; 
		String nomVar = step.getAttributeValue("nomvar");
		log(0,"Retrieving nomvar attribute: ["+nomVar+"]");
		if (nomVar.equals("")) { 
			// Get the name of the nominal that is currently the locus
			nomVar  = lcs.getNomvar(); 
			// Get the LFNominal object for this name from the LF
			nom = LFUtils.lfGetNominal(lf, nomVar);
			if (nom == null | nom.nomVar.equals("unknown")) { 
				throw new UPGException ("Unable to execute stepcode generate-rfx: Unknown nominal "+nomVar+" as locus");
			} // end if check whether nominal is known 
		} else { 
			nomVar = lcs.getVariableId(nomVar);
			// Get the LFNominal object for this name from the LF
			nom = LFUtils.lfGetNominal(lf, nomVar);	
			if (nom == null || nom.nomVar.equals("unknown")) { 
				throw new UPGException ("Unable to execute stepcode generate-rfx: Unknown nominal variable ["+nomVar+"] in nomvar attribute");
			} // end if check whether nominal is known 	
		} // end if..else
			
		log("Calling the GRE Handler now");
		String rfxString = greHandler.produceRFX(nom.nomVar);
			
		LogicalForm rfx = LFUtils.convertFromString(rfxString);
		log(1,"Generated rfx "+LFUtils.lfToString(rfx)+"/ converted from ["+rfxString+"]");
		
		LFNominal rfxRoot = rfx.root;
		// Copy all features and relations from the RFX nominal to the GRE
		for (Iterator<Feature> featsIter = LFUtils.lfNominalGetFeatures(nom); featsIter.hasNext(); ) { 
			rfxRoot.feats = LFUtils.lfNominalAddFeature(rfxRoot,featsIter.next());
		}
		for (Iterator<LFRelation> relsIter = LFUtils.lfNominalGetRelations(nom); relsIter.hasNext(); ) { 
			rfxRoot.rels = LFUtils.lfNominalAddRelation(rfxRoot,relsIter.next());
		} 
		//	Update the GRE lf with the updated root
		rfx.noms = LFUtils.lfUpdateNominal(rfx.noms,rfxRoot);
		// Remove the original RFX nominal from the LF
		lf.noms = LFUtils.lfRemoveNominal(lf.noms,nomVar);
		// Check whether to update the root of the LF
		if (lf.root.nomVar.equals(nomVar)) { lf.root = rfxRoot; }
		
		//log(0,"Removed rfx origin: "+LFUtils.lfToString(lf));
		
		// Replace all mentions in the LF of the old nomVar with the new nomvar
		lf.noms = LFUtils.lfReplaceNomvar(lf.noms,nomVar,rfx.root.nomVar);
		// Add the (updated) GRE
		for (Iterator nIter = Arrays.asList(rfx.noms).iterator() ; nIter.hasNext(); ) { 
			LFNominal nv = (LFNominal) nIter.next();
			lf.noms = LFUtils.lfAddNominal(lf, nv);
			log(0,"generate-rfx: adding nominal "+LFUtils.lfNominalToString(nv));
			lcs.addInsertedNominal(nv);
		} // end for over new/updated nominals
		
		
		log(0,"The logical form with the new GRE is as follows:"+LFUtils.lfToString(lf));
		
		result.setLF(lf);
		// result.setLocus(lcs);
		return result;
    } // end stepcodeGenerateRfx


	/** 
	The method <i>stepcodeCopyFeature</i> copies the value of a feature F under a source nominal, to another feature F' under a destination nominal. 
	The names of the two features need not be identical; if the destination feature name F' is not provided, it's assumed to be identical to the source feature name. 
	If no source or destination is given, take the locus. This enables to copy within the locus.
	*/ 
	
	public UPResult stepcodeCopyFeature (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException { 
		log(1,"Running code for action step copy-feature");
		UPResult result = new UPResult (lcs,lf); 
		// Get the name of the nominal that is currently the locus
		String lnomv  = lcs.getNomvar(); 
		// Get the source and destination.
		String srcVar = step.getAttributeValue("srcnom");
		String destVar = step.getAttributeValue("destnom");

	    //  if no source is given, take the locus
		if (srcVar.equals("unknown")) { 
			srcVar = lnomv; 
		}
		
/** Make it possible for both src and dest to be locus 
			if (destVar.equals("unknown")) { 
				throw new UPGException ("Unable to execute stepcode copy-feature: Unknown source and destination nominals");
			} 

		} else {  
		
			// source is set, check whether destnom should be locus
			if (destVar.equals("unknown")) {   
				destVar = lnomv;
			} // end if.. check for destination
		} // end if..else check for source, destination
**/

		// if no destination is given, take the locus
		if (destVar.equals("unknown")) {   
		destVar = lnomv;
		}
		
		// check whether the destination is a variable, if so resolve it to the coresponding nominal variable
		if (!lcs.getVariableId(destVar).equals("unknown")) { destVar = lcs.getVariableId(destVar); }
		// check whether the source is a variable, if so resolve it to the coresponding nominal variable
		if (!lcs.getVariableId(srcVar).equals("unknown")) { srcVar = lcs.getVariableId(srcVar); }
		
		// Now the source and the destination variables are set. Next, get the nominals
		LFNominal srcNom = LFUtils.lfGetNominal(lf, srcVar);
		LFNominal destNom = LFUtils.lfGetNominal(lf, destVar);		
		
		log(2,"Source nominal: "+srcNom+"; "+srcVar);
		log(2,"Destination nominal: "+destNom+"; "+destVar);
        log(2,"Locus: "+lnomv+"; Source: "+srcVar+"; Destination: "+destVar); 	
		
		if (srcNom.nomVar.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode copy-feature: Unknown nominal "+srcVar+" as source nominal");
		} // end if check whether source nominal is known 		
		if (destNom.nomVar.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode copy-feature: Unknown nominal "+destVar+" as destination nominal");
		} // end if check whether destination nominal is known 	
        log(2,"Locus: "+lnomv+"; Source: "+srcVar+"; Destination: "+destVar); 	
		// Destination and source nominals are now set
		String srcFeat = step.getAttributeValue("srcfeat");
		if (srcFeat.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode copy-feature: Attribute \"srcfeat\" not assigned in step");
		} // end if.. check for source feature
		// Set the destination feature name
		String destFeat = step.getAttributeValue("destfeat");		
		if (destFeat.equals("unknown")) { 
			destFeat = srcFeat;
		} // end if.. set destFeat
		// Get the value from the source feature
		String srcValue = LFUtils.lfNominalGetFeature(srcNom,srcFeat);
		log(1,"Returned source value in copy-feature is ["+srcValue+"]");
		// Now check what to do: two modes, "feature" and "relation"
		String mode = step.getAttributeValue("mode");
		// In the case we have the relation mode, we need to check whether there is a relation of the given destFeat type, and then add the value as proposition there
		// Here is the problem: we need to get the relation that goes to the destNom, not just any relation of the given type, because there may be multiple ones 
		log(1,"Copy feature mode: "+mode);

		if (mode.equals("unknown") || mode.equals("feature")) { 
			log(2,"Copy feature: mode=feature or unknown mode");
			// Add the feature
			Feature newFeat = new Feature();
			newFeat.feat = destFeat;
			newFeat.value = srcValue;
			destNom.feats = LFUtils.lfNominalAddFeature(destNom,newFeat);
			lf.noms = LFUtils.lfAddNominal(lf, destNom);
		} else if (mode.equals("relation")) { 
	        log(2,"Copy feature: mode=relation");		
			if (LFUtils.lfNominalHasRelation(srcNom,destFeat)) { 
				for (Iterator<LFRelation> depsIter = LFUtils.lfNominalGetRelations(srcNom); depsIter.hasNext(); ) { 
					LFRelation depRel = depsIter.next();
					if (depRel.mode.equals(destFeat)) {
						if (depRel.dep.equals(destNom.nomVar)) { 
							LFNominal depNom = LFUtils.lfGetNominal(lf, depRel.dep);
							if (depNom != null) { 
								depNom.prop = LFUtils.lfNominalAddProposition(depNom,srcValue);
								lf.noms = LFUtils.lfAddNominal(lf,depNom);
							} else { 
								throw new UPGException ("Unable to execute stepcode copy-feature: No nominal with nomvar ["+depRel.dep+"] as dependent of ["+destFeat+"] under ["+destVar+"]");
							} // end if..else 
							break; 
						} // end if.. check whether the relation is pointing to the right dependent nominal
					} // end if.. check whether looking at the right type of relation
				} // end for.. cycling over the relations under the source nominal
			/**
			LFRelation depRel = LFUtils.lfNominalGetRelation(srcNom,destFeat);
			if (depRel != null) {
				LFNominal depNom = LFUtils.lfGetNominal(lf, depRel.dep);
				if (depNom != null) { 
					depNom.prop = LFUtils.lfNominalAddProposition(depNom,srcValue);
					lf.noms = LFUtils.lfAddNominal(lf,depNom);
				} else { 
					throw new UPGException ("Unable to execute stepcode copy-feature: No nominal with nomvar ["+depRel.dep+"] as dependent of ["+destFeat+"] under ["+destVar+"]");
				} // end if..else 
			*/	
			} // end if			
			} // end else if relation
			 else if (mode.equals("proposition")) { 
			  log(2,"Copy feature: mode=proposition"); 	
			  destNom.prop.prop = srcValue; 
			  lf.noms = LFUtils.lfAddNominal(lf, destNom);
			  }
			   else {
				throw new UPGException ("Unable to execute stepcode copy-feature: No relation of type ["+destFeat+"] under nominal ["+destVar+"]");
			     } // end if..else check for availability of dependent
//		       } // 
		result.setLF(lf);
		log(1,"New LF: "+LFUtils.lfToString(lf));	
		return result;
	} // end stepcodeCopyFeature

	/** 

	The method <i>stepcodeCopyLF</i> copies a specified node, or a subgraph starting from that node, 
	to a destination (variable) node in the logical form. For the subgraph, an exclusion set can be specified 
	of relations under the node not to be included in the subgraph. The attribute "mode" specifies the 
	copy mode, "dest" specifies the destination, "excludes" the (optional) exclusion set, and "src" the (optional) copy source. 
	By default, the LOCUS is used as copy source. 
	
	*/ 
	
	public UPResult stepcodeCopyLF (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException {  
		log(1,"Running code for action step copy-lf");
		UPResult result = new UPResult (lcs,lf); 
		// Get the name of the nominal that is currently the locus
		String lnomv  = lcs.getNomvar(); 
		// Get the LFNominal object for this name from the LF
		LFNominal sourceNom = LFUtils.lfGetNominal(lf, lnomv);
			if (sourceNom.nomVar.equals("unknown")) { 
				throw new UPGException ("Unable to execute stepcode copy-lf: Unknown nominal "+lnomv+" as locus");
		} // end if check whether nominal is known 	
		// Check whether the source has been set
		String source = step.getAttributeValue("src");
		if (source.equals("unknown")) { 
			source = lnomv; 
		} else { 
			// get the nominal, either through direct naming or as variable
			sourceNom = LFUtils.lfGetNominal(lf, lnomv);
			if (sourceNom.nomVar.equals("unknown")) { 
				source = lcs.getVariableId(source);
				if (source.equals("unknown")) { 		
					throw new UPGException ("Unable to execute stepcode copy-lf: Unknown nominal "+source+" as source");	
				} else { 
					sourceNom = LFUtils.lfGetNominal(lf, source);
				} // end if..else
			} // end if.. check for existence 
		} // end if..else check for source
		// Now we have the source, and the source nominal
		// Next check the destination
		String destination = step.getAttributeValue("dest");
        log(1,"Locus: "+lnomv+" Source: "+source+" Destination:"+destination);
		boolean destVariable = false;
		if (destination.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode copy-lf: No nominal specified as copy destination");
		} else { 
			LFNominal destinationNom = LFUtils.lfGetNominal(lf, destination);
			if (destinationNom.nomVar.equals("unknown")) {   // null pointer error here!!!
				destination = lcs.getVariableId(destination);
				if (destination.equals("unknown")) { 		
					throw new UPGException ("Unable to execute stepcode copy-lf: Unknown nominal "+destination+" as destination");	
				} else { 
					destinationNom = LFUtils.lfGetNominal(lf, destination);
					destVariable = true;
				} // end if..else		
			} // end if.. check for existence 
		} // end if..else check for existence destination
		// Now we have the source, and the destination
		// Check for the copy mode
		String mode = step.getAttributeValue("mode");
		if (mode.equals("unknown") || mode.equals("node")) { 
			// The source nominal provides all the information to be copied to the destination
			lf.noms = LFUtils.lfReplaceNomvar(lf.noms, destination, source);
			// Update the destination variable name to be pointing to the source nominal
			if (destVariable) { 
				String destVarName = step.getAttributeValue("dest");
				lcs.addVariableId(destVarName,source);
			} // end if.. 
		} else if (mode.equals("subgraph")) { 
			LogicalForm subgraph = LFUtils.lfConstructSubtree(sourceNom,lf);
			String excludes = step.getAttributeValue("excludes");
			if (!excludes.equals("unknown")) { 
				// Create a Vector from the list of excludes
				Vector xvec = new Vector();
				StringTokenizer st = new StringTokenizer(excludes,"@");
				while (st.hasMoreTokens()) { 
					String xRel = (String) st.nextToken();
					log(0,"Add exclude relation ["+xRel+"]");
					xRel = strip(xRel);
					xvec.addElement(xRel);
				} // end while over excludes
				// Remove the relations on the excludes vector from the root in the subgraph
				// subgraph = LFUtils.lfNominalRemoveRelations(subgraph,sourceNom,xvec);
			} // end if.. check whether we have a list of excludes
			// Set the subgraph at the destination 
			
			
		} else {
			throw new UPGException("Unable to execute stepcode copy-lf: Unknown mode ["+mode+"] as copy mode");
		} // end if.. check 
		// set the logical form
		result.setLF(lf);
		// set the locus
		result.setLocus(lcs);
		// return the result
		return result;
	} // end stepcodeCopyLF
	


    /**
     *  The method <i>stepactionIdentifyNomvar</i> looks at the
     *  relation in the "relation" parameter stored with the
     *  step, and the variable in the "nomvar" parameter.
     *  The method then checks whether the nominal that is the current
     *  locus has a relation as mentioned. If this is the case, the
     *  nominal to which this relation holds is identified with the
     *  variable, and the (variable,nominal) pair is stored with the
     *  current locus. The method returns a <tt>UPResult</tt>
     *  object that includes the (unchanged) logical form, and the
     *  changed  locus.
     * 
     *  <h4>Warning</h4> The first instance of the relation stored
     *  with the locus is taken; if there are more relations with the
     *  given name (e.g. <tt>Property</tt>) then these are currently
     *  ignored.
     */

    public UPResult stepcodeIdentifyNomvar (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException {  
	log(1,"Running code for action step identify-nomvar");
	UPResult result = new UPResult (lcs,lf); 
	// Get the name of the nominal that is currently the locus
	String lnomv  = lcs.getNomvar(); 
	// Get the LFNominal object for this name from the LF
	LFNominal nom = LFUtils.lfGetNominal(lf, lnomv);
	if (nom.nomVar.equals("unknown")) { 
	    throw new UPGException ("Unable to execute stepcode identify-nomvar: Unknown nominal "+lnomv+" as locus");
	} // end if check whether nominal is known 
	// First, get the relation
	String mode = step.getAttributeValue("mode");
	String root = step.getAttributeValue("root");
		log(0,"Value of root variable: ["+root+"]");
	if (mode.equals("unknown")) { 
	    throw new UPGException ("Unable to execute stepcode identify-nomvar: Attribute \"mode\" not assigned in step");
	} else if (!root.equals("unknown") && !root.equals("")) { 
		// We actually have a root, so get the nominal assigned to this variable
		String rootVar = lcs.getVariableId(root);
		if (rootVar.equals("unknown")) { throw new UPGException ("Unable to execute stepcode identify-nomvar: Attribute \"root\" ["+root+"] unknown variable in locus"); }
		// Get the nominal for that root
		LFNominal rootNom = LFUtils.lfGetNominal(lf,rootVar);
		if (rootNom.equals("unknown")) { throw new UPGException ("Unable to execute stepcode identify-nomvar: Variable for \"root\" ["+rootVar+"] points to unknown nominal in locus"); }
		// Get the dependent of the given type/mode from under the root
		LFRelation modeRel = LFUtils.lfNominalGetRelation(rootNom,mode);
		if (rootNom == null) { throw new UPGException ("Unable to execute stepcode identify-nomvar: Variable for \"mode\" ["+mode+"] points to unknown relation under ["+rootVar+"]");	}
		// Set the provided variable name in DEST to the nominal variable just retrieved
		String scope = step.getAttributeValue("scope");
		String dest  = step.getAttributeValue("nomvar");
		lcs.addVariableId(dest,modeRel.dep,scope);
		log("Setting ["+dest+"] to ["+modeRel.dep+"] with scope ["+scope+"]");	
		result.setLocus(lcs);
		log(0,"New locus: "+lcs.toString());
	} else { 
	    // Next, check whether the variable is set
	    String nomvar = step.getAttributeValue("nomvar");
	    if (nomvar.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode identify-nomvar: Attribute \"nomvar\" not assigned in step");
	    } else { 
		// First, check whether the mode is "*LOCUS*" or not;
		// if it is the locus, we just need to identify the
		// nominal of the current locus with the given
		// variable name, i.e. no need to go looking.

		String idnom = "unknown";
		if (mode.equals("*LOCUS*")) {
		    idnom = lcs.getNomvar();
		} else { 
		    // Now we need to check: depending on the direction of
		    // the relation, we need to look for a relation from
		    // the locus to a nominal ("dest"), or we need to look
		    // for a nominal that has the given relation to the
		    // current locus.
		    String orientation = step.getAttributeValue("orientation");
		    if (orientation.equals("dest")) { 
			log(3,"Orientation is dest ["+orientation+"] so look under current locus");
			Iterator rIter = Arrays.asList(nom.rels).iterator();
			boolean notfound = true; 
			while (notfound && rIter.hasNext()) { 
			    LFRelation rel = (LFRelation) rIter.next();
			    if (rel.mode.equals(mode)) { 
				idnom = rel.dep;
			    } // end if
			} // end while over relations
		    } else { 
			log(3,"Orientation is src ["+orientation+"] so look for a nominal relating to the current locus");
			LogicalForm globallf = lcs.getLF();
			Iterator nomIter = Arrays.asList(globallf.noms).iterator();
			boolean found = false; 
			while (!found & nomIter.hasNext()) { 
			    LFNominal nomi = (LFNominal) nomIter.next();
			    Iterator rIter = Arrays.asList(nom.rels).iterator();
			    boolean notfound = true;
			    while (notfound && rIter.hasNext()) {
				LFRelation rel = (LFRelation) rIter.next();
				if (rel.mode.equals(mode)) {
				    String depnom = rel.dep;
				    if (depnom.equals(nom.nomVar)) {
					idnom = rel.head;
					notfound = false; // exit cycle over relations
					found = true;     // exit cycle over nominals
				    } // end if..check whether nomvar is dependent
				} // end if.. check for relation
			    } // end while over relations     
			} // end while over nominals in the lf
		    } // end if..else.. check whether src or dest
		} // end if..else check what type of mode
		if (idnom.equals("unknown")) { 
		    // *********************************  save global nomvars and nominal (locus) *************************
		    // if Relation doesn't exist, check whether there is such feature (SCOPE)
		    String scope = "";
		    for (int i=0; i < nom.feats.length; i++) {
		    	if (nom.feats[i].feat.equals(mode)) {
		    		scope = nom.feats[i].value;
		    	}
		    }
		    if (scope.equals("global")) global.put(lnomv,nom);
		    // ****************************************************************************************************
		    System.err.println("Cannot find relation \""+mode+"\" to/from under current locus "+lnomv);
		} else { 
		    // CHART
		    chitem.add(lcs.getNominal());
		    
		    // Next, deal with the variable, and its
		    // scoping.
		    String scope = step.getAttributeValue("scope");
		    lcs.addVariableId(nomvar,idnom,scope);	    
		    result.setLocus(lcs);
		    log(0,"New locus: "+lcs.toString());
		} // end if..else check whether the relation was present
	    } // end if..else check whether nomvar parameter is set
	} // end if..else check for mode parameter set
	return result;
    } // end stepcodeIdentifyNomvar

    /**
     *  The method <i>stepactionMoveLocus</i> looks at the
     *  variable in the "nomvar" parameter stored with the
     *  step.The method then checks whether the variable is known: it needs
     *  to have been identified before with a nominal in the logical
     *  form. This information is stored with the current locus. If
     *  the nominal can be identified, the method changes the locus to 
     *  the given nominal in the logical form that is identified by
     *  the variable. The method returns a <tt>UPResult</tt>
     *  object that includes the (unchanged) logical form, and the
     *  new locus.
     */

    public UPResult stepcodeMoveLocus (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException {  
	log(1,"Running code for action step move-locus");
	UPResult result = new UPResult (lcs,lf); 
	// Get the name of the nominal that is currently the locus
	String lnomv  = lcs.getNomvar(); 
	// Get the LFNominal object for this name from the LF
	LFNominal nom = LFUtils.lfGetNominal(lf, lnomv);
	if (nom.nomVar.equals("unknown")) { 
	    throw new UPGException ("Unable to execute stepcode move-locus: Unknown nominal "+lnomv+" as locus");
	} // end if check whether nominal is known 
	// First, get the variable
	String var = step.getAttributeValue("nomvar");
	if (var.equals("unknown")) { 
	    throw new UPGException ("Unable to execute stepcode add-relation: Attribute \"nomvar\" not assigned in step");
	} else { 
	    // Next, check whether the variable is known
	    String nomvar = lcs.getVariableId(var);
	    if (nomvar.equals("unknown")) { 
		throw new UPGException ("Unable to execute stepcode add-relation: variable \""+var+"\" unknown in current locus");
	    } else { 
		LFNominal newlnom = LFUtils.lfGetNominal(lf, nomvar);
		lcs.setNomvar(newlnom);
		lcs.resetVariableIds();
		
		 // CHART
		chitem.add(lcs.getNominal());
		
		log(0,"New locus: "+lcs.toString());
	    } // end if..else check whether nomvar is known in the locus
	} // end if..else check for nomvar parameter set
	return result;
    } // end stepcodeMoveLocus


	/**
		"root" can be left unassigned, or assigned "*LOCUS*", to point to the locus; otherwise, a variable name is assumed
	
	*/ 
	 
	public UPResult stepcodeReplaceRelation (UPGActionStep step, UPLocus lcs, LogicalForm lf) throws UPGException {  
		log(1,"Running code for action step replace-relation");
		UPResult result = new UPResult (lcs,lf); 
		// Get the name of the nominal that is currently the locus
		String lnomv  = lcs.getNomvar(); 
		log(1,"Locus: "+lnomv);
		// First, get the root
		String root = step.getAttributeValue("root");		
		log(1,"Root from input: "+root);
		// Check whether it is to be the locus, or a variable name to be retrieved
		if (root.equals("unknown") || root.equals("") || root.equals("*LOCUS*")) { 
			root = lnomv; 
		} else { 
			root = lcs.getVariableId(root);
		log(1,"Root to be used: "+root);
			if (root.equals("unknown")) { throw new UPGException ("Unable to execute stepcode replace-relation: Attribute \"root\" ["+root+"] unknown variable in locus"); }
		} // end if..else check for locus or variable name
		// Get the LFNominal object for the root from the LF
		LFNominal rootnom = LFUtils.lfGetNominal(lf, root);
		// Check we have a nominal
		if (rootnom.nomVar.equals("unknown")) { 
			throw new UPGException ("Unable to execute stepcode add-relation: Unknown nominal "+lnomv+" as root");
		} // end if check whether nominal is known 
		// Now we have a root, retrieve the mode
		String mode = step.getAttributeValue("mode"); 
		log(1,"Mode to replace: "+mode);
		if (mode.equals("unknown")) { throw new UPGException ("Unable to execute stepcode replace-relation: Attribute \"mode\" ["+mode+"] not set"); }
		// Get the corresponding LFRelation object
		LFRelation modeRel = null; // LFUtils.lfNominalGetRelation(rootnom,mode);
		for (Iterator<LFRelation> relsIter = LFUtils.lfNominalGetRelations(rootnom); relsIter.hasNext(); ) { 
			LFRelation rel = relsIter.next(); log(0,"Relation present: ["+rel.mode+"]"); 
			if (rel.mode.startsWith(mode)) { modeRel = rel; }
		}
		if (modeRel == null) { throw new UPGException ("Unable to execute stepcode replace-relation: Relation \"mode\" ["+mode+"] not present under nominal ["+rootnom.nomVar+"]");  } 		
		
		// Check whether we are relabeling the relation, or replacing it with a complex LF
		String newmode = step.getAttributeValue("newmode");
		log(1,"New mode: "+newmode);
		if (!(newmode.equals("unknown") || newmode == null || newmode.equals("") )) { 
		log(1,"Doing simple replacement version");
			// Construct the new relation
			LFRelation newRel = new LFRelation();
			newRel.head = modeRel.head;
			newRel.mode = newmode;
			newRel.dep  = modeRel.dep;
		    log(1,"New relation: "+newRel.head+";   "+newRel.mode);
			// Update the nominal, replacing the old relation with the new one
			rootnom.rels = LFUtils.lfNominalReplaceRelation(rootnom,modeRel,newRel);
			// Update the logical form with the updated nominal
			lf.noms = LFUtils.lfUpdateNominal(lf.noms,rootnom);
		} else { 
		log(1,"Doing complex replacement version");
			String relLF = step.getAttributeValue("lf");
			if (relLF.equals("unknown")) { throw new UPGException ("Unable to execute stepcode replace-relation: Neither \"newmode\" ["+newmode+"] nor \"lf\"  ["+lf+"] set");  } 
			String foot  = step.getAttributeValue("foot");
			if (foot.equals("unknown")) { throw new UPGException ("Unable to execute stepcode replace-relation: Argument \"foot\" ["+foot+"] nor set for \"lf\"  ["+lf+"]");  } 			
			// Create the logical form to be inserted
			LogicalForm replaceLF = LFUtils.convertFromString(relLF);
			// Replace the foot with the actual dependent of the relation to be replaced
			replaceLF.noms = LFUtils.lfReplaceNomvar(replaceLF.noms,foot,modeRel.dep);
			// Get the root from the replace LF
			LFNominal replaceRoot = replaceLF.root;	
			// Remove the relation from the existing root in the lf
			rootnom.rels = LFUtils.lfNominalRemoveRelation(rootnom,modeRel.mode);
			// Add all the relations from the replace LF's rootnode to the rootnom 
			for (int i=0; i < java.lang.reflect.Array.getLength(replaceRoot.rels); i++) {  
				rootnom.rels = LFUtils.lfNominalAddRelation(rootnom,(LFRelation)replaceRoot.rels[i]);
			} // end for over the relations in the replacement LF
			// Remove the root from the replacement LF
			replaceLF.noms = LFUtils.lfRemoveNominal(replaceLF.noms,replaceRoot.nomVar);
			// Remove the foot from the replacement LF
			replaceLF.noms = LFUtils.lfRemoveNominal(replaceLF.noms,modeRel.dep);
			// Add all the other nominals to the LF
			for (Iterator<LFNominal> newNomsIter = LFUtils.lfGetNominals(replaceLF); newNomsIter.hasNext(); ) { 
				LFNominal newNom = newNomsIter.next();
				log(0,"Adding nominal when replacing relation: "+LFUtils.lfNominalToString(newNom));
				
				lf.noms = LFUtils.lfAddNominal(lf.noms, newNom);
			} // end for over the nominals to be added
		} // end if..else check for newmode or lf/foot	
		// Set the logical form, locus of the result
		result.setLF(lf);
		result.setLocus(lcs);		
		// Return the result
		return result;
	} // end replace-relation
	
	
    //=================================================================
    // MISCELLANEOUS METHODS
    //=================================================================


    private String strip (String s) {
        StringTokenizer tok = new StringTokenizer(s);
        String result = "";
        while (tok.hasMoreTokens()) {
            String c = tok.nextToken();
            if (!c.equals(" ")) { result=result+c; }
        }
        return result;
    }

    //=================================================================
    // MAIN METHOD
    //=================================================================

    public static void main (String[] args) { 
	// 	String filename = args[0];
 	//UtterancePlanner up = new UtterancePlanner(filename,new UtterancePlannerAgent()); 
	
 	// LogicalForm lf = new LogicalForm("(@x1:disc-vantagepoint(<Acknowledgment>p1:process) ^ @p1:process(<Patient>(b1:phys-obj ^ ball)))"); 

 	//LogicalForm lf = new LogicalForm("(@dvp0:dvp(body ^ <Modality>vision ^ <RhetRel>query-wh ^ <Novelty>known ^ <Grounding>ack ^  <ContentBody>(b1:phys-obj ^ ball ^  <Property>(r1:color ^ red))))"); 

 	//System.out.println(LFUtils.lfToString(lf)+"\n\n"); 

 	//LogicalForm planlf = up.plan(lf);
	
 	//System.out.println(planLFUtils.lfToString(lf)); 

    } // end main


} // end class definition 
