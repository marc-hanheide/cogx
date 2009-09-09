// =================================================================
// Copyright (C) 2006 Geert-Jan M. Kruijff (gj@dfki.de)
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

package comsys.components.dialogue;

// =================================================================
// IMPORTS
// =================================================================

// -----------------------------------------------------------------
// JAVA IMPORTS
// -----------------------------------------------------------------
import java.util.*;

import comsys.datastructs.comsysEssentials.*;
import comsys.arch.*;

import comsys.datastructs.lf.*;
import comsys.lf.utils.ArrayIterator;
import comsys.lf.utils.LFUtils;
import comsys.utils.SDRSUtils;
import comsys.utils.DialogueMoveUtils;
import comsys.processing.parse.ActiveIncrCCGParser;



import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.ManagedComponent;
import cast.SubarchitectureComponentException;
import cast.cdl.*;
import cast.core.CASTData;

// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
 * The class <b>DialInt</b> implements a goal-driven process for
 * dialogue interpretation -- i.e. the interpretation of an utterance
 * against a linguistic model of the dialogue context.
 * <h4>Sub-architecture specifics</h4>
 * The process proposes several types of information processing goals.
 *
 *
 * <h4>CAST specifics</h4>
   The class has been refactored to deal with changes in CAST up to release 9:
   addToWorkingMemory, 
 
  
 * @version 080424 (started 060925)
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
 */

public class cc_DialogueStructureManager extends ManagedComponent {

    // =================================================================
    // CLASS-GLOBAL DATA STRUCTURES
    // =================================================================

    // ----------------------------------------------------------------
    // INFORMATION PROCESSING DATA STRUCTURES
    // ----------------------------------------------------------------

    // Hashtable used to record the tasks we want to carry out. For each
    // taskID we store a Vector with the data it is to work on
    private Hashtable<String, ProcessingData> m_proposedProcessing;

    // Hashtable linking data IDs to goal IDs
    private Hashtable<String, String> m_dataToProcessingGoalMap;

    // Hashtable linking task IDs to task types
    private Hashtable<String, String> m_taskToTaskTypeMap;

    // Vector with objects to be processed,
    // can be ComSys:PhonString,...
    private Vector<ProcessingData> m_dataObjects;

    // ----------------------------------------------------------------
    // DIALOGUE INTERPRETATION DATA STRUCTURES
    // ----------------------------------------------------------------

    // Counter for DiscRefBindings identifiers
    private int bindingsIdCounter;

    // Counter for LFBinding identifiers
    private int lfBindingIdCounter;

    // Counter for ProcessingData identifiers
    private int pdIdCounter;

    // Counter for StructInt identifiers
    private int strintIdCounter;

    Cache uncommittedCache;
    
    // ----------------------------------------------------------------
    // DIALOGUE INTERPRETATION ENGINES
    // ----------------------------------------------------------------

    // Engine for rhetorical relation resolution
//    RhetRelResolver rrresolver;

    // Engine for grammar network-based dialogue interpretation
 //   private GNProcessor dialIntEngine;

    // Inquiry request handler for dialogue interpretation
  //  private GNReqHandlerDialInt dialIntInqHandler;

    // =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================

    /**
     * The unary constructor
     * 
     * @param _id
     */
 /**   public cc_DialogueStructureManager(String _id) {
        init();
    } // constructor/1 */

    /**
     * The method <i>init</i> initializes the global data structures,
     * and sets the ontology for this component.
     * 
     * @see #configure
     */

    private void init() {
        log("Initializing dialogue structure manager");

        // nah: making all the comsys queue changes... don't want to
        // miss a thing
        m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

        // general information processing structures
        m_proposedProcessing = new Hashtable<String, ProcessingData>();
        m_dataToProcessingGoalMap = new Hashtable<String, String>();
        m_taskToTaskTypeMap = new Hashtable<String, String>();
        m_dataObjects = new Vector<ProcessingData>();
        pdIdCounter = 0;
        // dialogue interpretation data structures
        bindingsIdCounter = 0;
        lfBindingIdCounter = 0;
        strintIdCounter = 0;
        
    } // init

    // =================================================================
    // ACCESSOR METHODS
    // =================================================================

    /**
     * Returns a ParsingLF with the highest preference score in the
     * collection.
     */
/**
    private ParsingLF getMostPreferredLF(ParsingLF[] lfs) {
        ParsingLF result = null;
        float maxPrefScore = 0.0f;
        Iterator lfsIter = new ArrayIterator(lfs);
        while (lfsIter.hasNext()) {
            ParsingLF lf = (ParsingLF) lfsIter.next();
            if (lf.preferenceScore >= maxPrefScore) {
                if (lf.STATUS_COMPLETENESS
                    .equals(CompletenessStatus.COMPLETE)) {
                    maxPrefScore = lf.preferenceScore;
                    result = lf;
                }
            } // end if
        } // end while
        return result;
    } // end getMostPreferredLF
*/
    // =================================================================
    // TASK METHODS
    // =================================================================

    /**
     * The method <i>taskAdopted</i> processes a dialogue-level
     * interpretation task once the task manager has informed the
     * component it can run. The method pushes the processing data for a
     * given task onto the m_dataObjects queue, so that the
     * runComponent() method can spot something needs to be done. The
     * method does not distinguish between different types of tasks.
     * <p>
     * This method does <b>not</b> inform the task manager, whether the
     * task has been succesfully completed. This notification happens in
     * the <i>runComponent</i> method.
     * 
     * @see #runComponent
     * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _goalID) {
        // get the data we stored for this goal
        ProcessingData pd = m_proposedProcessing.remove(_goalID);
        if (pd != null) {
            // add the data item to the data objects queue
            m_dataObjects.addElement(pd);
            // get the identifier of the processing data object
            String pdID = pd.getID();
            // link the data ID to the goal ID, for future reference
            // on task completion (done in runComponent)
            m_dataToProcessingGoalMap.put(pdID, _goalID);
        }
        else {
            log("ERROR: Goal without data: " + _goalID);
        } // end if..else
    } // end taskAdopted

    /**
     * The method <i>taskRejected</i> removes a rejected task from the
     * list of proposed processing tasks.
     * 
     * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _goalID) {
        log("WARNING: The goal with ID [" + _goalID
            + "] has been rejected.");
        m_proposedProcessing.remove(_goalID);
    } // end taskRejected

    @Override
    public void start() {

        super.start();
        
        init();
        
     // test
        SDRS sdrs = new SDRS();
        sdrs.A = new String[0];
        sdrs.F = new LabelFormulaMapping();
        sdrs.F.mapping = new LabelFormulaPair[0];
        sdrs.LAST = "none";
        String id = newDataID();
        try {
            //addToWorkingMemory(id, ComsysOntology.SDRS_TYPE, sdrs);
			addToWorkingMemory(id, sdrs); // refactored, data type automatically established for the provided object
        }
        catch (Exception e) {
            e.printStackTrace();
        } // end try..catch overwriting working memory
        sdrs = null;
        log("SDRS discourse model initialized");
        
        try {
        	/**
        addChangeFilter(ComsysOntology.PACKEDLFS_TYPE,
            WorkingMemoryOperation.ADD, true,
            new WorkingMemoryChangeReceiver() {
            public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                    addedPackedLF(_wmc);
                }
            });
        	 */
           addChangeFilter(
        		   ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.ADD),
            new WorkingMemoryChangeReceiver() {
                public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                	addedPackedLF(_wmc);
                }
            }); 
        
        addChangeFilter(
        		ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.OVERWRITE),
                new WorkingMemoryChangeReceiver() {
                public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        addedPackedLF(_wmc);
                    }
                });

        addChangeFilter(
        		ChangeFilterFactory.createLocalTypeFilter(Cache.class,  WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        addedDiscRefBindings(_wmc);
                    }
                });
        
        addChangeFilter(
        		ChangeFilterFactory.createLocalTypeFilter(DialogueMove.class,  WorkingMemoryOperation.ADD),
                new WorkingMemoryChangeReceiver() {
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        addedDialogueMove(_wmc);
                    }
                });

        }
        catch (Exception e) {
            e.printStackTrace();
        }

    } // end start

    
    /**
     * @param _wmc
     * @param i
     */
    private void addedPackedLF(WorkingMemoryChange _wmc) {
    	
        // immediately process this logical form,
        // add it to the context model
        try {
        	// get the id of the working memory entry
            String id = _wmc.address.id;
            // get the data from working memory and store it
            // with its id
            CASTData plfWM = getWorkingMemoryEntry(id);
            PackedLFs plf = (PackedLFs) plfWM.getData();
            executeContextUpdateTask(plf);
        }
        catch (SubarchitectureComponentException e) {
            e.printStackTrace();
        } // end try.. catch
    } 
    
   
    /**
     * @param _wmc
     * @param i
     */
   synchronized private void addedDiscRefBindings(WorkingMemoryChange _wmc) {
    	// immediately process the discourse referent bindings
    	try {
            // get the id of the working memory entry
            String id = _wmc.address.id;
            // get the data from working memory and store it
            // with its id
            CASTData cacheWM = getWorkingMemoryEntry(id);
            Cache cache = (Cache) cacheWM.getData();
            addCacheToDiscourseModel(cache);
           
        }
        catch (SubarchitectureComponentException e) {
            e.printStackTrace();
        } // end try.. catch
    }
        
        
   synchronized private void addedDialogueMove(WorkingMemoryChange _wmc) {
	   
        	// immediately process the discourse referent bindings
        	try {
                // get the id of the working memory entry
                String id = _wmc.address.id;
                // get the data from working memory and store it
                // with its id
                CASTData dmWM = getWorkingMemoryEntry(id);
                DialogueMove dm = (DialogueMove) dmWM.getData();
                addDialogueMoveToDiscourseModel(dm);
               
            }
            catch (SubarchitectureComponentException e) {
                e.printStackTrace();
            } // end try.. catch
        }


        static Vector<String> alreadyGeneratedPLF = new Vector<String>();

        synchronized public void addCacheToDiscourseModel(Cache cache) {
        	log("adding the cache to the discourse model");
        	try {
        		CASTData[] Data = getWorkingMemoryEntries(SDRS.class, 0);
        		SDRS sdrs = (SDRS) Data[0].getData() ;
        		Iterator<SDRSFormula> it = SDRSUtils.getFormulas(sdrs);
        		SDRSFormula associatedFormula = null;
        		while (it.hasNext()) {
        			SDRSFormula f = it.next();
        			if (SDRSUtils.getFormulaType(f).type.equals(SDRSUtils.PLF_TYPE) &&
        					f.type.plf.id.equals(cache.plf.id)) {
        				associatedFormula = f;
        			}
        		}
        		if (associatedFormula != null && cache.CacheId.equals("discRef")) {
        			if (associatedFormula.caches.length > 0) {
        				SDRSUtils.replaceCacheInFormula(associatedFormula, cache);
        			}
        			else {
        				SDRSUtils.addCacheToFormula(associatedFormula, cache);
        			}
        				if (cache.plf.finalized == ActiveIncrCCGParser.FINAL_PARSE && 
        						!alreadyGeneratedPLF.contains(associatedFormula.label)) {
        					alreadyGeneratedPLF.add(associatedFormula.label);
        					overwriteWorkingMemory(Data[0].getID(),sdrs);
        					log("Context model updated with new cache associated to the SDRS Formula labelled " + associatedFormula.label );
        				}
        				else {
        					log("label: " + associatedFormula.label);
        				}
        			
        		}
        		else if (cache.CacheId.equals("discRef") &&
        				cache.plf.finalized == ActiveIncrCCGParser.FINAL_PARSE) {
        			uncommittedCache = cache;
        			log("setting the cache as being temporarily uncommitted");
        		}
        }
        catch (Exception e) {
        	e.printStackTrace();
        } // end try..catch overwriting working memory	
}

        synchronized public void addDialogueMoveToDiscourseModel (DialogueMove dm) {
       	log("adding the dialogue move to the discourse model");
    	try { 
     		CASTData[] Data = getWorkingMemoryEntries(SDRS.class);
     		SDRS sdrs = (SDRS) Data[0].getData() ;
     		SDRSFormula form = new SDRSFormula();
     		form.label = SDRSUtils.generateLabel();
     		form.type = new SDRSType();
     		SDRSRelation relation = new SDRSRelation();
     		String[] args= {dm.SDRSFormulaId1, dm.SDRSFormulaId2};
     		relation.args = args;
     		relation.relType = DialogueMoveUtils.convertMoveTypeToString(dm.mType);
     		form.type = new SDRSType();
     		form.type.type = SDRSUtils.RELATION_TYPE;
     		form.type.relation = relation;
     		SDRSUtils.addFormula(sdrs, form);
    		overwriteWorkingMemory(Data[0].getID(), sdrs);
     		log("Context model updated with the dialogue move " + relation.relType) ;
    	}
     	catch (Exception e) {
     		e.printStackTrace();
     	} // end try..catch overwriting working memory	
    }
    
    
    /**
     * The method <i>executeContextUpdateTask</i> adds a produced
     * logical form to the context model.
     */

        synchronized public void executeContextUpdateTask(PackedLFs plf) {
        
    	 if (plf.finalized == ActiveIncrCCGParser.FINAL_PARSE) {
         	try {
         		log("Updating the discourse model with a new (finalized) packed logical form...");
         		CASTData[] Data = getWorkingMemoryEntries(SDRS.class, 0);
         		SDRS sdrs = (SDRS) Data[0].getData() ;
         		SDRSFormula form = new SDRSFormula();
         		form.label = SDRSUtils.generateLabel();
         		form.type = new SDRSType();
         		form.type.type = SDRSUtils.PLF_TYPE;
         		form.type.plf = plf;
         		if (uncommittedCache != null) {
    				SDRSUtils.addCacheToFormula(form, uncommittedCache);
    				uncommittedCache = null;
         		}
         		sdrs = SDRSUtils.addFormula(sdrs, form);
         		overwriteWorkingMemory(Data[0].getID(),sdrs);
         		log("Context model updated with new packed logical form");
         	}
         	catch (Exception e) {
         		e.printStackTrace();
         	} // end try..catch overwriting working memory
         }

    	 /**
        // ---!!!!!---- WARNING -----
        // Something is funny with adding / removing nominals, we end up
        // with duplicates for one-time roots
        // right now, do a quick pruning.
        LFNominal[] prunedNoms = new LFNominal[0];
        Iterator nIter = LFUtils.lfGetNominals(blf);
        Vector mentions = new Vector();
        while (nIter.hasNext()) {
            LFNominal nom = (LFNominal) nIter.next();
            String nomvar = nom.nomVar;
            if (mentions.contains(nomvar)) {
                // do nothing
            }
            else {
                prunedNoms = LFUtils.lfAddNominal(prunedNoms, nom);
                mentions.add(nomvar);
            }
        } // end while
        blf.noms = prunedNoms;
        // ---!!!!--- END OF THE FUNNY STUFF

        // Now resolve the rhetorical relations
        // Operate on the stinLF variable
        log("Resolving rhetorical relations");
        HashMap resolution = rrresolver.resolve(sdastruct);
        if (resolution.containsKey("lf")) {
            blf = (LogicalForm) resolution.get("lf");
            sdastruct.setLf(blf);
            log("RRResolved produced LF: " + LFUtils.lfToString(blf));
        } // end if.. check for resolved logical form

        // Finally, add the structure to the context model
        dialctxt.addFormula(sdastruct, lfBindings);
    	  */
    	 
    } // end

    
        
        
        
        /**
         * The method <i>executeContextUpdateTask</i> adds a produced
         * logical form to the context model.
         */

       synchronized public void executeContextUpdateTask(SpokenOutputItem spoi) {

    	   log("new spoken output item received");       	 

   /**    	try {
     		log("Updating the discourse model with a new (finalized) packed logical form...");
     		CASTData[] Data = getWorkingMemoryEntries(SDRS.class, 0);
     		SDRS sdrs = (SDRS) Data[0].getData() ;
     		SDRSFormula form = new SDRSFormula();
     		form.label = SDRSUtils.generateLabel();
     		form.type = new SDRSType();
     		form.type.plf(plf);
     		if (uncommittedCache != null) {
				SDRSUtils.addCacheToFormula(form, uncommittedCache);
				uncommittedCache = null;
     		}
     		sdrs = SDRSUtils.addFormula(sdrs, form);
     		overwriteWorkingMemory(Data[0].getID(),sdrs, OperationMode.BLOCKING);
     		log("Context model updated with new packed logical form");
     	}
     	catch (Exception e) {
     		e.printStackTrace();
     	} // end try..catch overwriting working memory
     	
     }
       */
        } // end

            
    // =================================================================
    // COMPUTATION METHODS
    // =================================================================

  
    
    /** Returns a new identifier for a DiscRefBindings object */

    private String newDRBindingsId() {
        String result = "" + bindingsIdCounter + "";
        bindingsIdCounter++;
        return result;
    } // end newDRBindingsId

    /** Returns a new identifier for a ProcessingData object */

    private String newProcessingDataId() {
        String result = "pd" + pdIdCounter;
        pdIdCounter++;
        return result;
    } // end newProcessingDataId

    /** Returns a new identifier for a StructInt object */

    private String newStructIntId() {
        String result = "stint" + strintIdCounter;
        strintIdCounter++;
        return result;
    } // end newStructIntId

    // =================================================================
    // RUN METHODS
    // =================================================================

    
    /**
     * The <i>configure</i> method overrides the method from
     * CASTProcessingComponent (though calls the super methods to ensure
     * any higher-level configuration is done), and looks for a
     * command-line argument <tt>--digrammar</tt> specifying the
     * grammar networks for dialogue interpretation. The grammar is
     * loaded in this method.
     * 
     * @see cast.core.components.CASTProcessingComponent#configure(java.util.Properties)
     */

    public void configure(Properties _config) {
//        _config.list(System.out);
        String diGrammarFile = "grammars/grnetworks/dialint/grammar.xml";
        if (_config.containsKey("--digrammar")) {
            diGrammarFile = _config.getProperty("--digrammar");
        }
        else if (_config.containsKey("--log")) {
            String logFlag = _config.getProperty("--log");
            if (logFlag.equals("") | logFlag.equals("true")) {
                m_bLogOutput = true;
            }
            else {
                m_bLogOutput = false;
            } // end if..else
        } // end if..else check for command-line arguments
        log("DialInt configuration:\n\tDialogue interpretation grammar file: ["
            + diGrammarFile + "]");
        // Load the grammar
      //  GrammarNetwork dialIntGN = new GrammarNetwork();
     //   dialIntGN.read(diGrammarFile);
       // dialIntEngine.setGrammar(dialIntGN);

     //   rrresolver.setLogging(m_bLogOutput);
     //  ctxtRefResolver.setLogging(m_bLogOutput);
     //   dialIntEngine.setLogging(m_bLogOutput);
    } // end configure

} // end class

