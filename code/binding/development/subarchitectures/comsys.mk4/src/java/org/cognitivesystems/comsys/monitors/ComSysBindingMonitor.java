//=================================================================
// Copyright (C) 2007-2008 Geert-Jan M. Kruijff (gj@dfki.de)
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

package org.cognitivesystems.comsys.monitors;

//=================================================================
// IMPORTS
//=================================================================

// ----------------------------------------------------------------
// BINDING imports
// ----------------------------------------------------------------

import binding.BindingException;
import binding.abstr.AbstractMonitor;
import binding.abstr.AbstractMonitorState;
import binding.abstr.GlobalIdentifier;
import binding.abstr.LocalIdentifier;
import binding.abstr.ProxyRelation;
import BindingData.BindingProxy;
import BindingData.BindingProxyType;
import BindingData.BindingUnion;
import BindingData.FeaturePointer;
import BindingData.TriggerDotViewer;
import BindingFeatures.*;
import BindingFeaturesCommon.*;
import binding.common.BindingComponentException;
import BindingQueries.MakeProxyUnavailable;


// import binding.ontology.CommunicativeGoalsOntology;

// ----------------------------------------------------------------
// CAST imports
// ----------------------------------------------------------------
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.OperationMode;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;
import cast.core.CASTUtils;



// ----------------------------------------------------------------
// COMSYS imports
// ----------------------------------------------------------------
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Cache;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.ContextInfo;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Event;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.EventDiscRefRelation;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.InterpretationSupport;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Nucleus;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.State;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.StateDiscRefRelation;
import org.cognitivesystems.comsys.general.CacheWrapper;
import org.cognitivesystems.comsys.general.EventStructureUtils;

import org.cognitivesystems.comsys.monitors.proxyfactories.ActionMotionProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.ActionNonMotionProxyFactory;
// import org.cognitivesystems.comsys.monitors.proxyfactories.AnimateProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.AscriptionProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.CognitionProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.CommunicationProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.DefaultProxyFactory;
// import org.cognitivesystems.comsys.monitors.proxyfactories.DeicticPronounProxyFactory;
// import org.cognitivesystems.comsys.monitors.proxyfactories.DisConnectiveProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.DUnitsProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.ELocationProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.EPlaceProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.ERegionProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.EntityProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.ETimeUnitProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.EventProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.GreetingProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.MCommentProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.MDirectionProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.MLocationProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.MFrequencyProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.MMannerProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.MTimeProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.MTimePointProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.ModalProxyFactory;
// import org.cognitivesystems.comsys.monitors.proxyfactories.MWhereToProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.PerceptionProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.PersonProxyFactory;
// import org.cognitivesystems.comsys.monitors.proxyfactories.PhysicalProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.QColorProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.QLocationProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.QPhysicalProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.QShapeProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.QSizeProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.QStateProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.SymbolicProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.ThingProxyFactory;
import org.cognitivesystems.comsys.monitors.proxyfactories.WHProxyFactory;


// ----------------------------------------------------------------
// JAVA imports
// ----------------------------------------------------------------
import java.util.*;

// ----------------------------------------------------------------
// LF imports
// ----------------------------------------------------------------
import org.cognitivesystems.repr.lf.autogen.LFEssentials.*;
import org.cognitivesystems.repr.lf.autogen.LFPacking.*;
import org.cognitivesystems.repr.lf.utils.LFUtils;

// import com.sun.tools.example.debug.expr.ExpressionParser.GetFrame;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
The class <b>ComSysBindingMonitor</b> monitors the ComSys.mk4 working 
memory, and proposes binding proxies for cross-modal content
interconnectivity on the basis of packed logical forms, possibly also 
even structure interpretations as represented by event nuclei. The 
monitor uses a monitor state to keep track of the proxies and proxy
relations that have been introduced. 
<p>
The monitor state records the relations between nominal variables in the namespace of a PLF and discourse referents, 
and between discourse referents and proxy addresses. To maintain different global namespaces, we prefix discourse 
referents with a subarchitecture ID when linking them to proxy addresses. 


<p>
Configure options for the monitor are: 
<ul> 
<li> <tt>--incrementalBinding</tt>: whether the binder should bind 
     content generated at each incremental parsing step, or not. 
	 the option takes boolean values "true" and "false", and int
	 values "0", "1", and "2". "0" means incremental binding, 
	 "1" means to wait for completed parses, and "2" completed, 
	 completely pruned parses. </li> 

<li> <tt>--unionMinSize</tt>: int value, indicating the minimal size of 
	 the union in which a proxy participates, for the content represented
	 by the proxy to be considered "supported." This value needs to be 
	 "1" or higher.
<li> <tt>--syncModel</tt>: the model to be adopted for synchronizing 
     the exchange of information between binding and dialogue processing, 
	 about what interpretations	are currently supported. The monitor currently 
	 implements two synchronization models: <tt>SYNC_ALL</tt>, in which support 
	 information about all the relations in a packed logical form is gathered 
	 (via change notifications) before a <tt>ContextInfo</tt> struct is being 
	 written out; and, <tt>SYNC_ONE</tt>, in which support information about 
	 relations in a packed logical form is provided as a <tt>ContextInfo</tt> 
	 struct on an individual basis, as this information becomes available (via change notifications). 
	The flag takes values "sync_all" or "sync_one". The default value for the flag is "sync_one". </li>
	 
	 
</ul>
 
@version 080906
@started 070907
@author  Geert-Jan M. Kruijff (gj@dfki.de)
@author  Henrik Jacobsson     (henrik.jacobsson@dfki.de)
@author  Hendrik Zender       (zender@dfki.de)
*/ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class ComSysBindingMonitor 
  extends AbstractMonitor {
    
    
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================


	/** The monitor state */
	public AbstractMonitorState monitorState; 

	public Object m_lockObject;

	String currentPLFId;
		
	Hashtable changeListeners; 
	
	/** The mapping from local structures to binding SA objects */
	LocalToBindingMapping localMapping;
	
	// ------------------------------------------------------------
	// DATA STRUCTURES FOR TRACKING PROXY RELATIONS
	// ------------------------------------------------------------			

	/**
		The table storedRelProxies maintains, for each relational 
		proxy there is on the binder, a map from its address (used when 
		notifying about changes) to an StoredProxyRelation object. This
		object stores information about the source of the relation
	*/ 
	Hashtable<String,StoredProxyRelation> storedRelProxies; 
	
	/**
		The table storedInterpretationSupport maintains, for each packed 
		logical form (by plfId), a hashtable with InterpretationSupport
		objects for the different (relational) proxies that the current 
		version of the packed logical form has introduced. The support
		hashtable for a packed logical form is reset every time a new
		version is retrieved (by change notification). 
	*/ 
	Hashtable<String,Hashtable> storedInterpretationSupport;
	
	/** The "current" (i.e. last seen) packed logical form */ 
	// String currentPLFId; 
	
	Hashtable<String,PackedLogicalForm> plfsTable;
	
	/**
	The table localProxyStructures maintains the proxy structures which
	have been produced for the current packed logical form. The structures
	are indexed by discourse referent (for the root of the proxy structure). 
	*/ 
	
	Hashtable<String,LocalProxyStructure> localProxyStructures; 

	
	
	// ------------------------------------------------------------
	// PROXY FACTORIES
	// ------------------------------------------------------------	

	/** The table proxyFactoryTable maintains a list of proxy factories, 
		each factory keyed by the ontological sort of the packed nominal on which 
		it operates. 
	*/ 
	Hashtable<String,ProxyFactory> proxyFactoryTable; 
	
	/** The default factory for producing proxy content */ 
	//DefaultProxyFactory defaultProxyFactory; 
	DefaultProxyFactory defaultProxyFactory;
	
	// ------------------------------------------------------------
	// CONFIGURATION FLAGS
	// ------------------------------------------------------------	
	
	/** The flag to set whether binding should be incremental, or only done on complete PLFs.
		This flag can be set using the boolean command-line option "--incrementalBinding". 
	*/
	boolean incrementalBinding;
	
	/** The flag which sets which completeness level a parse should have to be considered "complete". 
		This allows for a more flexible "non-incremental" binding interaction -- either after parsing
		is finished, or only after pruning is completely done. By default this level is set to 2 (
		complete after pruning). This flag can be set as numerical value to the 
		"--incrementalBinding" option (instead of "false", use "1" or "2"). 
	*/ 
	int	plfCompletenessLevel; 
	
	
	
	/**
		This flag, set by the command-line option "--syncModel", sets the model to be adopted for 
		synchronizing the exchange of information between binding and dialogue processing, about what interpretations
		are currently supported. The monitor currently implements two synchronization models: 

		<ol> 
			<li> <tt>SYNC_ALL</tt>, in which support information about all the relations in a packed logical form 
				 is gathered (via change notifications) before a <tt>ContextInfo</tt> struct is being written out. </li> 
			<li> <tt>SYNC_ONE</tt>, in which support information about relations in a packed logical form is 
				 provided as a <tt>ContextInfo</tt> struct on an individual basis, as this information becomes 
				 available (via change notifications). </li> 
		</ol>
		
		The flag takes values "sync_all" or "sync_one". The default value for the flag is "sync_one". 
	*/ 
	int syncModel;
	
	public final int SYNC_ALL = 0;
	public final int SYNC_ONE = 1;
	


	boolean intentionalityDiff; 
	String	motivSA; 
	String  bindingSA;
	  
	  
	  boolean intentionalProxies = false;		

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *  The basic constructor calls the super class, and triggers
	 *  initialization of the internal variables. 
     */

    public ComSysBindingMonitor (String _id) {
		super(_id);
		init();
    } // end constructor

	/** 
		Initializes the internal variables 
	*/ 
	
	private void init () { 

		localMapping = new LocalToBindingMapping();

		// MONITOR STATE
		monitorState = new AbstractMonitorState();
		monitorState.setLogging(m_bLogOutput);
		currentPLFId = "";
		changeListeners = new Hashtable();

		// INTERPRETATION SUPPORT
		storedRelProxies = new Hashtable();
		storedInterpretationSupport = new Hashtable();
		plfsTable = new Hashtable<String,PackedLogicalForm>();

		// CONFIGURABEL FLAGS
		incrementalBinding = true;
		plfCompletenessLevel = 0; 
		syncModel = SYNC_ONE;
		
		intentionalityDiff = false;
		motivSA = null; 
		bindingSA = null;

		// FACTORIES
		proxyFactoryTable = new Hashtable();
		defaultProxyFactory = new DefaultProxyFactory();
		this.registerProxyFactory(new ActionMotionProxyFactory());						
		this.registerProxyFactory(new ActionNonMotionProxyFactory());								
		// this.registerProxyFactory(new AnimateProxyFactory());				
		this.registerProxyFactory(new AscriptionProxyFactory());
		this.registerProxyFactory(new CognitionProxyFactory());				
		this.registerProxyFactory(new CommunicationProxyFactory());						
		// this.registerProxyFactory(new DeicticPronounProxyFactory());		
		// this.registerProxyFactory(new DisConnectiveProxyFactory());										
		this.registerProxyFactory(new DUnitsProxyFactory());												
		this.registerProxyFactory(new ELocationProxyFactory());
		this.registerProxyFactory(new EPlaceProxyFactory());								
		this.registerProxyFactory(new ERegionProxyFactory());				
		this.registerProxyFactory(new EntityProxyFactory());		
		this.registerProxyFactory(new ETimeUnitProxyFactory());				
		this.registerProxyFactory(new EventProxyFactory());										
		this.registerProxyFactory(new GreetingProxyFactory());						
		this.registerProxyFactory(new MCommentProxyFactory());			
		this.registerProxyFactory(new MDirectionProxyFactory());						
		this.registerProxyFactory(new MLocationProxyFactory());				
		this.registerProxyFactory(new MFrequencyProxyFactory());
		this.registerProxyFactory(new MMannerProxyFactory());									
		this.registerProxyFactory(new MTimePointProxyFactory());					
		this.registerProxyFactory(new MTimeProxyFactory());							
		this.registerProxyFactory(new ModalProxyFactory());								
		// this.registerProxyFactory(new MWhereToProxyFactory());			
		this.registerProxyFactory(new PerceptionProxyFactory());								
		this.registerProxyFactory(new PersonProxyFactory());								
		// this.registerProxyFactory(new PhysicalProxyFactory());			
		this.registerProxyFactory(new QColorProxyFactory());
		this.registerProxyFactory(new QLocationProxyFactory());
		this.registerProxyFactory(new QPhysicalProxyFactory());
		this.registerProxyFactory(new QShapeProxyFactory());
		this.registerProxyFactory(new QSizeProxyFactory());
		this.registerProxyFactory(new QStateProxyFactory());
		// this.registerProxyFactory(new RestrictedEntityProxyFactory());				
		this.registerProxyFactory(new SymbolicProxyFactory());		
		this.registerProxyFactory(new ThingProxyFactory());				
		this.registerProxyFactory(new WHProxyFactory());				

		localProxyStructures = new Hashtable<String,LocalProxyStructure>();


	} // end init
	
    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

	/** 
		The method <i>registerProxyFactory</i> registers a proxy factory 
		with the binding monitor. 
		
		@param factory The factory to be registered
	*/ 

	public void registerProxyFactory (ProxyFactory factory) { 
		String sort = factory.getRootSort();
		if (sort == null) { 
			System.out.println("Trying to register a proxy factory ["+(Object)factory.getClass().getName()+"] with an empty rootsort!"); 
		} else { 
			log("Registering a proxy factory for sort ["+sort+"]");
			proxyFactoryTable.put(sort,factory); 
		} // end if..else check for available sort 
	} // end registerProxyFactory
 

    //=================================================================
    // CAST METHODS
    //=================================================================

    /** 
     The <i>start</i> method registers change filters with the working
	 memory, checking for added or overwritten packed logical forms. 
	 Change filters for event nuclei are added if the monitor runs in
	 non-incremental mode, looking for complete logical forms. 
	 	 
	 @see #handleWorkingMemoryChange(WorkingMemoryChange _wmc)
	 @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
     */
    @Override
    public void start() {
	super.start();
	try {
	    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Cache.class,								       
								       WorkingMemoryOperation.ADD), 
								       new WorkingMemoryChangeReceiver() {
									   public void workingMemoryChanged(
													    WorkingMemoryChange _wmc) {
									       handleCacheChange(_wmc);
									   }
								       });
	    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Cache.class,								       
								       WorkingMemoryOperation.OVERWRITE), 
								       new WorkingMemoryChangeReceiver() {
									   public void workingMemoryChanged(
													    WorkingMemoryChange _wmc) {
									       handleCacheChange(_wmc);
									   }
								       }
			    );
	    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Nucleus.class,
								       WorkingMemoryOperation.ADD),
								       new WorkingMemoryChangeReceiver() {
									   public void workingMemoryChanged(
													    WorkingMemoryChange _wmc) {
									       handleNucleusChange(_wmc);
									   }
								       }
			    );
	    addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Nucleus.class,
								       WorkingMemoryOperation.OVERWRITE),
								       new WorkingMemoryChangeReceiver() {
									   public void workingMemoryChanged(
													    WorkingMemoryChange _wmc) {
									       handleNucleusChange(_wmc);
									   }
								       }
			    );

	} catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	} // end try..catch
    } // end method

	/**
         * The method <i>addBindingWMChangeFilters</i> adds two change filters
         * that listens for changes to a proxy with the given address (ADD,
         * OVERWRITE). This method is called in <i>createAndStoreBindingProxies</i>
         * before updating the binding working memory.
         */ 

	protected void addBindingWMChangeFilters (String proxyAddr) { 
		try { 
			if (!changeListeners.contains(proxyAddr)) { 
				log("Adding an OVERWRITE filter to binding WM for proxy ["+proxyAddr+"]");
				WorkingMemoryChangeReceiver wmcChange = new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
									handleBindingWMChange(_wmc);
								}
						}; 
				addChangeFilter(ChangeFilterFactory.createAddressFilter(proxyAddr,super.getBindingSA(),WorkingMemoryOperation.OVERWRITE),
							wmcChange
				); 	
				changeListeners.put(proxyAddr, wmcChange);	
			} // end if.. check whether already added listener
		} catch (SubarchitectureProcessException e) { 
			log("Failed to add a change filter: "+e.getMessage());
            e.printStackTrace();	
		} 	
	} // end addBindingWMChangeFilters

	/**
		The method <i>removeBindingWMChangeFilters</i> removes the change filters that 
		have been specified to listen for changes to a proxy with the given address 
		(ADD, OVERWRITE). This method is called in <i>createAndStoreBindingProxies</i> 
		before updating	the binding working memory (removing the proxy). 
		
		@param proxyAddr	The address of the proxy for which the change filter should be removed
	*/ 

	protected void removeBindingWMChangeFilters (String proxyAddr) { 
			try { 
				WorkingMemoryChangeReceiver wmcChange = (WorkingMemoryChangeReceiver) changeListeners.get(proxyAddr); 
				changeListeners.remove(proxyAddr);	
				removeChangeFilter(wmcChange);
			} catch (SubarchitectureProcessException e) {
				log("Failed to remove a change filter: "+e.getMessage());
				e.printStackTrace();
			} 
	} // end removeBindingWMChangeFilters

	/** 
		The method <i>removeBindingWMChangeFilters</i> takes an iterator over
		local identifiers, and removes for 
		each proxy address the change filters registered on the binding working 
		memory. 
		
	*/

	protected void removeBindingWMChangeFilters (Iterator localIdIter, String spaceId) { 
		while (localIdIter.hasNext()) { 
			String locId = (String) localIdIter.next();
			try { 
				LocalIdentifier locVar = monitorState.getLocalSpaceIdentifier(locId,spaceId);
				removeBindingWMChangeFilters (locVar._proxyAddress); 
			} catch (BindingException be) { 
				System.err.println(be.getMessage());
			} // end 
		} // end while over local identifiers
	} // end removeBindingWMChangeFilters


    /**
     * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _goalID) {

    }

    /**
     * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _goalID) {}

    //=================================================================
    // CAST WMC METHODS
    //=================================================================
    
    
    private void handleCacheChange(WorkingMemoryChange _wmc) {
	
		try {

			// get the id of the working memory entry
			String dataId = _wmc.m_address.m_id;
			String subArchId = _wmc.m_address.m_subarchitecture;

			// get the data from working memory
			CASTData data = getWorkingMemoryEntry(dataId, subArchId);

			log("Received notification for item [" + "Cache"
				+ "] with address [" + _wmc.m_address + "] and id ["
				+ dataId + "]");

			Cache discRefs = (Cache) data.getData();

			// get the packed logical forms
			PackedLFs plfs = discRefs.plf;

			boolean createAndStore = true;

			int finalized = plfs.finalized; 

			if (!incrementalBinding) { 
				log("Packed logical form completeness level is ["+finalized+"], compare against ["+plfCompletenessLevel+"]");
				if (finalized < plfCompletenessLevel) { 
					createAndStore = false; 
					log("PLF not complete yet, so don't create and store proxies");
				} 
			}

			if (createAndStore) { 
				// Reset the supported interpretations table for the PLF
				storedInterpretationSupport.put(plfs.packedLF.packedLFId, new Hashtable());
				// Create and store and bind the proxies
				createAndStoreBindingProxies(_wmc, plfs, discRefs, subArchId);
			}
		} catch (SubarchitectureProcessException e) {
			println(e.getLocalizedMessage());
			e.printStackTrace();
		} // end try .. catch
    } // end method




    private void handleNucleusChange(WorkingMemoryChange _wmc) {
	try {
	    // get the id of the working memory entry
	    String dataId = _wmc.m_address.m_id;
	    String subArchId = _wmc.m_address.m_subarchitecture;
	    // get the data from working memory
	    CASTData data = getWorkingMemoryEntry(dataId, subArchId);

	    log("Received notification for item [" + "Nucleus"
		    + "] with address [" + _wmc.m_address + "] and id ["
		    + dataId + "]");

	    Nucleus nucleus = (Nucleus) data.getData();
	    // create the event structure for the packed logical
	    // form
	    createEventStructure(nucleus);
	} catch (SubarchitectureProcessException e) {
	    println(e.getLocalizedMessage());
	    e.printStackTrace();
	}
    }// end method

	/**
         * The method <i>handleBindingWMChange</i> is called whenever a proxy,
         * representing content of the current utterance (packed LF), is being
         * modified (ADD, OVERWRITE).
         * 
         * @param _wmc
         *                The change in working memory
         */ 


	public void handleBindingWMChange (WorkingMemoryChange _wmc) { 
		try {
	
			// get the id of the working memory entry
            String dataId  = _wmc.m_address.m_id;
            String subArchId = _wmc.m_address.m_subarchitecture;
            // get the data from working memory
            CASTData data = getWorkingMemoryEntry(dataId, subArchId);
			// get the type of data
			String dType = data.getType();
			// log("Received change on data ["+dType+"] with id ["+dataId+"]");
			if (dType.equals(CASTUtils.typeName(BindingProxy.class))){//BindingOntology.BINDING_PROXY_TYPE)) {
				BindingProxy proxy = (BindingProxy) data.getData();
				// check whether it's a relation 
				switch (proxy.m_type.value()) { 
					case BindingProxyType._BASIC :		log("received change on BASIC proxy ["+dataId+"]"); break;
					case BindingProxyType._GROUP :		break; 
					case BindingProxyType._RELATION :	log("received change on RELATION proxy ["+dataId+"]"); break;										
				} 
				// update the comsys WM with information on the stored relation 
				
			
			} // end if check for proxy type
		} catch (SubarchitectureProcessException e) {
            println(e.getLocalizedMessage());
            e.printStackTrace();
		} // end try .. catch
	} // end handleBindingWMChange

	  @Override
	  protected void makeProxyUnavailable(MakeProxyUnavailable _makeProxyUnavailable)
		throws DoesNotExistOnWMException, SubarchitectureProcessException {
			// deleteExistingProxy(_makeProxyUnavailable.m_proxyID);
			log("Ignoring trigger to delete existing proxy with ID ["+_makeProxyUnavailable.m_proxyID+"]");
			
	  }	  
	  
	  
    //=================================================================
    // COMPUTATION METHODS
    //=================================================================

	/** 
		The method <i>createEventStructure</i> takes a nucleus that has 
		recently been added to the comsys working memory, and creates the
		corresponding (compacted) event structure on binding working memory. 
		The event structure on binding WM is tied to the proxies for the packed 
		logical form on the basis of which the nucleus has been formulated. 
	*/ 

	private void createEventStructure (Nucleus nucleus) 
		throws SubarchitectureProcessException
	{ 
		// Get the packed logical form id
		String plfId = nucleus.plfId;
		log("Creating event structure for packed logical form ["+plfId+"]");
		// Get the discourse referent bindings
		Cache drBindings = nucleus.discRefs;
		CacheWrapper drWrapper = new CacheWrapper (drBindings);
		// Get the state, state-type
		State state = EventStructureUtils.getFirstState(nucleus);
		String stateType = state.type; 
		// Depending on state type, determine what the participant is
		// FROM where the state relation should start, and TO what participant 
		// it should go. 
		String fromRel = ""; 
		String toRel = "";
		String stateRel = "";
		if (stateType.equals("HAS")) { 
			fromRel = "Actor";
			toRel = "Patient";
			stateRel = stateType;
		} else if (stateType.startsWith("IS-")) { 
			fromRel = "Actor|Patient"; // underspecified role
			toRel = "Anchor"; 
			stateRel = stateType.substring(3,stateType.length());
		} else { 
			System.err.println("[ERROR:ComsysBindingMonitor] Invalid nucleus type for comsys binding monitor: ["+stateType+"]");
		} // end if.. else check for state-type
		// Now cycle over the participants in the state, and get the discourse referents
		String fromDiscRef = "";
		String toDiscRef   = "";
		Iterator partIter = EventStructureUtils.getParticipants(state);
		while (partIter.hasNext()) { 
			StateDiscRefRelation sdRel = (StateDiscRefRelation) partIter.next();
			if (fromRel.indexOf(sdRel.mode) != -1) { 
				fromDiscRef = sdRel.discRefId;
			} else if (toRel.indexOf(sdRel.mode) != -1) { 
				toDiscRef   = sdRel.discRefId;
			} // end if..else check for mode
		} // end while over participants
		log("Referents for relations: from ["+fromDiscRef+"] and to ["+toDiscRef+"]");		
		// Finally, get the proxy addresses for the discrefs, and introduce the proxy relation
		try { 
			String fromProxyAddress = monitorState.getGlobalSpaceProxyAddress(fromDiscRef);
			String toProxyAddress = monitorState.getGlobalSpaceProxyAddress(toDiscRef);
			// Add the compacted state relation 
			String relAddr = addSimpleRelation(fromProxyAddress, toProxyAddress, stateRel, TemporalFrameType.DESIRED);
			// Get the event, and create the compacted temporal-aspect relation 
			Event event = EventStructureUtils.getFirstEvent(nucleus);
			EventDiscRefRelation eventRel = EventStructureUtils.getParticipant(event,"event");
			String eventProxyAddress = monitorState.getGlobalSpaceProxyAddress(eventRel.discRefId);
			String consequenceAddr = addSimpleRelation(eventProxyAddress, relAddr, "Consequence", TemporalFrameType.DESIRED);
		} catch (BindingException be) { 
			System.err.println("[ERROR:ComsysBindingMonitor] "+be.getMessage());
			be.printStackTrace();
		} // end try..catch
	} // end createEventStructure

	/**
		The method <i>createBindingProxies</i> creates binding proxies. We cycle over 
		the packing nodes in a packed logical form, and within each node, we cycle 
		over the packing nominals stored there. From this we then first of all create
		a tree map, so that we have quick access to all the nominals in the packed 
		representation (nominal variables are unique). Then, for each packing nominal we create a 
		proxy structure. These proxy structures are stored in a map, indexed by discourse referent. 
		Cycling over these map, we check whether the proxy structures generated now are new or updates to 
		previously generated ones, or whether they are identical. Only if they are new or updates we put them
		onto a queue. Subsequently, the proxy structures on this queue are stored as proxies on the
		binding working memory. The methods finally stores the proxy relations, and updates the monitor
		state with the references to proxy addresses. 
				
		@param _wmc The received change
		@param plfs The packed logical form
		@param subArchId The id of the originating subarchitecture
	
	*/
	private void createAndStoreBindingProxies (WorkingMemoryChange _wmc, PackedLFs plfs, Cache discRefCache, String subArchId) 
		throws SubarchitectureProcessException
    {   
		CacheWrapper discRefs = new CacheWrapper(discRefCache);
		log("Starting to create and store binding proxies");
		// Get the actual representation
		PackedLogicalForm plf = plfs.packedLF;
		log("Storing PLF ["+plf.packedLFId+"] to establish interpretation support ");
		plfsTable.put(plf.packedLFId,plf);
		
		long finalized = plfs.finalized;
		if (plf != null) { 
			// create the treemap of the nominals
			TreeMap packedNoms = LFUtils.createNomTreeMap(plf);
			// check for incremental binding -- true is fine, or if we have a PLF which has the required completeness level
			if (incrementalBinding || finalized > 0) { 
				// Reset the local proxy structures table
				localProxyStructures = new Hashtable<String,LocalProxyStructure>();
				// Create the proxy structures; these are stored on the Hashtable localProxyStructures
				log("Create the local proxy structures");
			    Vector<PendingProxyRelation> pendingRelations = this.addNominalProxies(plfs.id, plf,packedNoms,discRefs);
				// Create a filter, based on what properties may be asked after
				
				TreeSet bindingFilter = this.createBindingFilter(pendingRelations);
				
				
				// Create a queue with proxy structures which updates to existing structures, or are new
				log("Create queue with new/updated proxy structures");
				Vector proxyQueue = createProxyQueue();
				// Store the queued proxy structures
				storeQueuedProxies(proxyQueue,plfs.id,discRefs,bindingFilter,pendingRelations);
			    // Store the pending proxy relations
				this.addPendingProxyRelations(plfs.id, pendingRelations,plf,bindingFilter);
				// Update the discourse referents, i.e. global identifiers, in the monitor state
				// addDiscourseReferents(discRefs, plfs.id, packedNoms);
				// trigger DOT file creation for the motivation WM
				triggerDOTgeneration(motivSA);
				
			} else {
				log("Not yet updating binding working memory -- incrementalBinding mode ["+incrementalBinding+"] and finalized ["+finalized+"]");
			} // end if..else check for incrementally updating binding working memory
		} // end check for non-null plf packing nodes
    } // end method

	  
	  /**
		The method <i>createBindingFilter</i> returns a TreeSet with features and/or relation labels which are being questioned, 
	    and which therefore should not be generated for proxy structures on binding working memory. To ensure that this constraints
	    only apply to the proxies under consideration (and not wholesale), we add the nominal variables for the proxies to the filter 
	    as well. We assume that features, relations and nominal variables are disjoint namespaces. 
	   
	  */ 
	  
	  private	TreeSet createBindingFilter(Vector<PendingProxyRelation> prs) { 
		  TreeSet filter = new TreeSet();
		  for (Iterator<PendingProxyRelation> prsIter = prs.iterator(); prsIter.hasNext(); ) { 
			  PendingProxyRelation pr = prsIter.next();
			  if (pr.relMode.indexOf("-KNOWS") != -1) { 
				  int colonPos = pr.relMode.indexOf(":");
				  String knowLabel = pr.relMode.substring(colonPos+1,pr.relMode.length());
				  filter.add(knowLabel);
				  filter.add(pr.depNomVar);
			  } // end if.
		  } // end for over relations
		  log("Returning binding WM filter: "+filter);
		  return filter; 
	  }	// end createBindingFilter
	  
	/** 
		The method <i>addDiscourseReferents</i> adds discourse referents for the local identifiers, based on cycling over the 
		packed nominals in the packed logical form. 
		
		@param discRefs	The discourse references 
		@param plfsId	The id of the packed logical form, serving as local name space
		@param packedNoms	The packed nominals in the packed logical form
	*/ 

	private void addDiscourseReferents (CacheWrapper discRefs, String plfsId, TreeMap packedNoms) { 
		log("Adding discourse referents");
		// cycle over the nominal variables
		Iterator nomsIter = packedNoms.keySet().iterator();
		while (nomsIter.hasNext()) { 
			String nomVar = (String) nomsIter.next();
			try { 
				if (!nomVar.startsWith("rootNom")) {
					// Get the discourse referent
					String discRef = discRefs.getDiscRef(nomVar);
					log("Attempt storing discref ["+discRef+"] for nomvar ["+nomVar+"]");
					if (!discRef.equals("none") && !discRef.equals("unknown")) {
						if (!monitorState.isExcluded(nomVar,plfsId)) { 
							storeDiscourseReferent(nomVar,plfsId,discRef);
						} // end if.. check whether to actually add a discourse referent			
					} else { 		
						System.err.println("[ERROR:ComsysBindingMonitor] In addDiscourseReferents, cannot find discourse referent for nomvar ["+nomVar+"]");
					} // end if..else check for discourse referent
				} // end if.. check the nomvar is not excluded
			} catch (BindingException be) { 
				// do nothing 
			} // end try..catch
		} // end while over nominal variables
	} // end addDiscourseReferents

	/** 
		The method <i>addNominalProxies</i> takes a list of (packed) nominals from a packed logical form, and creates 
		proxy structures for them. The method returns a vector with pending proxy relations. 
		
		@param	plfsId		The identifier of the packed logical form
		@param	plf			The packed logical form 
		@param	packedNoms	A map with all the (unique) nominal variables in the packed logical form
		@param  discRefs	A map with discourse referents for the nominal variables in the packed logical form
		@return Vector		A vector with the PendingProxyRelation objects
	*/
	private Vector addNominalProxies(String plfsId, PackedLogicalForm plf, TreeMap packedNoms, CacheWrapper discRefs) 
	{
		log("Adding nominal proxies");
		if (plf == null) { System.err.println("[ERROR:ComsysBindingMonitor] received null PackedLogicalForm object! (l.850)"); }
		// Set up the set with pending relations
		Vector pendingRelations = new Vector();
		// Cycle over the packing nodes
		Iterator pnomsIter = LFUtils.getNominalsIterator(plf,"DF",true);
		while (pnomsIter.hasNext()) { 
			PackedNominal nom = (PackedNominal) pnomsIter.next();
			String nomVar = nom.nomVar;
			boolean propOK = true;
			if (nom.prop.prop == null || nom.prop.prop.equals("")) { propOK = false; } 
			if (!monitorState.isExcluded(nomVar,plfsId) && !nomVar.equals("rootNom") && propOK) {
				String sort = getPackedNominalSort(nom);
				
				if (!sort.equals("") && sort != null) {
					ProxyFactoryResults pfResults = new ProxyFactoryResults();
					ProxyFactory factory = defaultProxyFactory; 
					if (proxyFactoryTable.containsKey(sort)) { 
						factory = (ProxyFactory) proxyFactoryTable.get(sort);
						log("Applying proxy factory for the observed nominal sort ["+sort+"], nomvar ["+nomVar+"] and proposition ["+nom.prop.prop+"]");					
					} else {
						System.err.println("[ERROR:ComsysBindingMonitor] No proxy factory for the observed nominal sort ["+sort+"]");
					} // end 
					// Initialize the factory with discrefs
					factory.setDiscRefs(discRefs);
					log("Now applying the factory to create the content for the proxy for ["+sort+"], nomvar ["+nomVar+"] and proposition ["+nom.prop.prop+"]");					
					
					pfResults = factory.produce(nom,plf,packedNoms); 
					if (pfResults != null) { 
						log("The results returned the following pending proxy relations: \n"+pfResults.getPendingProxyRelations().toString());
					}
					
					if (pfResults == null) { 					
						log("The proxy factory returned a NULL result");
					} else if (!pfResults.generateProxy()) { 
						// we haven't even started a proxy, so nothing to cancel, but add the pending relations
						log("No proxy generation, just updating the set of pending proxy relations with ["+pfResults.getPendingProxyRelations().size()+"] relations");
						pendingRelations.addAll(pfResults.getPendingProxyRelations());
					} else {
						log("Updating the monitor state with excludes "+pfResults.getExcludes());
						// Update the state with information about excludes
						monitorState.excludeIdentifiers(pfResults.getExcludes().iterator(),plfsId);
						// Update the set with pending proxy relations
						log("Updating the set of pending proxy relations with ["+pfResults.getPendingProxyRelations().size()+"] relations");
						pendingRelations.addAll(pfResults.getPendingProxyRelations());
						// Store the proxy using the discourse referent for the root of the proxy structure
						if (pfResults.getAddedNominalsSize() > 1) { 
							log("WARNING: trying to add ["+pfResults.getAddedNominalsSize()+"] nominals for a proxy structure of sort ["+sort+"]"); 
						} else if (pfResults.getAddedNominalsSize() < 0) { 
							log("WARNING: trying to NO ["+pfResults.getAddedNominalsSize()+"] nominals for a proxy structure of sort ["+sort+"]"); 
						}						
						log("Starting to add ["+pfResults.getAddedNominalsSize()+"] nominals for a proxy structure of sort ["+sort+"]");
						for (Iterator<String> addedNomsIter = pfResults.getAddedNominals(); addedNomsIter.hasNext(); ) { 
							String addedNom = addedNomsIter.next();
							String discRef = discRefs.getDiscRef(addedNom);
							LocalProxyStructure locPRX = pfResults.getProxyStructure();
							log("Adding a proxy factory structure for disc ref ["+discRef+"] given nomvar ["+addedNom+"] to the local proxy structures map\n"+locPRX.toString());
							localProxyStructures.put(discRef,locPRX);
						} // end for over added nominals
						// COMMGOAL: at this point we could store the communicative goal if present: [ pfResults.getCommGoals().size()>0 ] 
					} // end if.. else check how to update binding WM
					// -----------------------------------------------------------------
				} // end if.. check for non-empty sort
			} // end if.. check whether the nominal hasn't already been processed
		} // end while over packed nominals
		// Return the results
		return pendingRelations;
	} // end addNominalProxies

	/** 
		The method <i>addPendingProxyRelations</i> takes a list of pending proxy relations, and then adds these relations using the 
		proxy-addresses in the provided map (linking nominal variables with proxy addresses). 
		
		@param pendingRelations A vector with PendingProxyRelation objects
	*/

	private void addPendingProxyRelations (String plfsId, Vector pendingRelations, PackedLogicalForm plf, TreeSet bindingFilter) 
		throws SubarchitectureProcessException, BindingComponentException	
	{ 
		boolean intentionalContent = false;
		setBindingSA(bindingSA);
		log("Adding pending proxy relations");
		

		
		Iterator pendingRelsIter = pendingRelations.iterator();
		while (pendingRelsIter.hasNext()) { 
			PendingProxyRelation rel = (PendingProxyRelation) pendingRelsIter.next();
			if (rel.hasContentStatus("intentional")) { intentionalContent = true; }
			if (!rel.relMode.equals("")) {  
				String bindingHead = bindingSA+"_"+rel.headNomVar;
				String bindingDep  = bindingSA+"_"+rel.depNomVar;
				boolean filterPass = true;
				if (bindingFilter.contains(rel.relMode)) { filterPass = false; }
				int colonPos = rel.relMode.indexOf(":");
				if (colonPos != -1) { 
					String prefix = rel.relMode.substring(0,colonPos);
					if (bindingFilter.contains(prefix)) { filterPass = false; }
					log("Check whether prefix ["+prefix+"] is in filter "+bindingFilter+": ["+!filterPass+"] so pass is ["+filterPass+"]");
				} 	
				if (filterPass) { 
					if (rel.hasContentStatus("indexical") || !rel.isSetContentStatus()) { 
						log("Pending indexical relation of type ["+rel.relMode+"] between ["+rel.headNomVar+"] and ["+rel.depNomVar+"], given filterPass ["+filterPass+"]");
						if (monitorState.existsLocalSpaceProxyAddress(bindingHead,plfsId)) { 
							String fromAddress = monitorState.getLocalSpaceProxyAddress(bindingHead,plfsId);
							if (monitorState.existsLocalSpaceProxyAddress(bindingDep,plfsId)) { 
								
								// CHANGING LOCATION INTO POSITION
								String replaced = rel.relMode.replaceAll("Location","Position");
								rel.relMode = replaced;
								
								String toAddress = monitorState.getLocalSpaceProxyAddress(bindingDep,plfsId);				
								ProxyRelation proxyRel = new ProxyRelation("",replaced,fromAddress,toAddress);
								if (!monitorState.existsLocalSpaceProxyRelation(plfsId,proxyRel)) { 
									log ("Adding relation ["+replaced+"] between ["+bindingHead+"] and ["+bindingDep+"]");
									// Add the relation, get its address, and store the address on the list for this nominal
									if (toAddress != null && fromAddress != null) { 
										TemporalFrameType tmpFT = (TemporalFrameType) localMapping.map("TemporalFrameType",rel.tempFrameType) ; 
										proxyRel._proxyAddress = addSimpleRelation(fromAddress, toAddress, rel.relMode, tmpFT);			
										monitorState.addLocalSpaceProxyRelation(proxyRel,plfsId);
										// Add a change filter, so that we can monitor whether it is getting bound
										log("Adding a change filter for a relation proxy.");
										addBindingWMChangeFilters(proxyRel._proxyAddress);
										// Retrieve the id's of logical forms that introduce this particular relation
										Vector matchedLFs = LFUtils.getMatchedLFs(plf,bindingHead,bindingDep,rel.relMode);
										// Store the id's with the relation address etc. 
										StoredProxyRelation storedRel = new StoredProxyRelation (bindingHead,bindingDep,rel.relMode, plf.packedLFId, matchedLFs);
										storedRelProxies.put(proxyRel._proxyAddress,storedRel);
									} else { 
										System.err.println("[ERROR:ComSysBindingMonitor] Invalid proxy addresses for ["+rel.headNomVar+"/"+fromAddress+"] and/or ["+rel.depNomVar+"/"+toAddress+"]");
									} // end  		
								} else {
									// System.err.println ("[ERROR:ComSysBindingMonitor] Already introduced relation ["+rel.relMode+"] between ["+rel.headNomVar+"] and ["+rel.depNomVar+"]");						
								} // end if..else
							} else { 
								System.err.println("Proxy address not known for dependent TO proxy with nominal variable ["+rel.depNomVar+"]");
							} // end if.. check for available TO proxy
						} // end if .. check for available FROM proxy
					}
				} else { 
					log("filter pass is ["+filterPass+"] on relmode label ["+rel.relMode+"]");
				} 
				// end if..check for indexical
			} else { 
				System.err.println ("[ERROR:ComSysBindingMonitor] Empty relation label for relation between ["+rel.headNomVar+"] and ["+rel.depNomVar+"]");
			} // end if..else check for mode defined
		} // end while over pending relations
		bindNewProxies();
		
		if (intentionalContent || intentionalProxies) { 
			setBindingSA(motivSA);
			for (Iterator<PendingProxyRelation> intentsIter = pendingRelations.iterator(); intentsIter.hasNext(); ) { 
				PendingProxyRelation rel = (PendingProxyRelation) intentsIter.next();
				log("Pending intentional relation of type ["+rel.relMode+"] between ["+rel.headNomVar+"] and ["+rel.depNomVar+"]");
				if (!rel.relMode.equals("")) {  
					String motivHead = motivSA+"_"+rel.headNomVar;
					String motivDep  = motivSA+"_"+rel.depNomVar;
					String fromAddress = null;
					if (monitorState.existsLocalSpaceProxyAddress(motivHead,plfsId)) { 
						fromAddress = monitorState.getLocalSpaceProxyAddress(motivHead,plfsId);
					} else if (monitorState.existsGlobalSpaceProxyAddress(motivHead)) { 
						fromAddress = monitorState.getGlobalSpaceProxyAddress(motivHead);
					}
					if (fromAddress != null) { 
						if (monitorState.existsLocalSpaceProxyAddress(motivDep,plfsId)) { 
							String toAddress = monitorState.getLocalSpaceProxyAddress(motivDep,plfsId);				
							ProxyRelation proxyRel = new ProxyRelation("",rel.relMode,fromAddress,toAddress);
							if (!monitorState.existsLocalSpaceProxyRelation(plfsId,proxyRel)) { 
								log ("Adding relation ["+rel.relMode+"] between ["+motivHead+"] and ["+motivDep+"]");
								// Add the relation, get its address, and store the address on the list for this nominal
								if (toAddress != null && fromAddress != null) { 
									TemporalFrameType tmpFT = (TemporalFrameType) localMapping.map("TemporalFrameType",rel.tempFrameType) ; 
									proxyRel._proxyAddress = addSimpleRelation(fromAddress, toAddress, rel.relMode, tmpFT);			
									monitorState.addLocalSpaceProxyRelation(proxyRel,plfsId);
									// Add a change filter, so that we can monitor whether it is getting bound
									log("Adding a change filter for a relation proxy.");
									addBindingWMChangeFilters(proxyRel._proxyAddress);
									// Retrieve the id's of logical forms that introduce this particular relation
									Vector matchedLFs = LFUtils.getMatchedLFs(plf,motivHead,motivDep,rel.relMode);
									// Store the id's with the relation address etc. 
									StoredProxyRelation storedRel = new StoredProxyRelation (motivHead,motivDep,rel.relMode, plf.packedLFId, matchedLFs);
									storedRelProxies.put(proxyRel._proxyAddress,storedRel);
								} else { 
									System.err.println("[ERROR:ComSysBindingMonitor] Invalid proxy addresses for ["+rel.headNomVar+"/"+fromAddress+"] and/or ["+rel.depNomVar+"/"+toAddress+"]");
								} // end  		
							} else {
								// System.err.println ("[ERROR:ComSysBindingMonitor] Already introduced relation ["+rel.relMode+"] between ["+rel.headNomVar+"] and ["+rel.depNomVar+"]");						
							} // end if..else
						} else { 
							System.err.println("Proxy address not known for dependent TO proxy with nominal variable ["+rel.depNomVar+"]");
						} // end if.. check for available TO proxy
					} else { 
						System.err.println("Proxy address not known for head proxy with nominal variable ["+rel.headNomVar+"] looking for ["+motivHead+"]");
					} // end if .. check for available FROM proxy
				} else { 
					System.err.println ("[ERROR:ComSysBindingMonitor] Empty relation label for relation between ["+rel.headNomVar+"] and ["+rel.depNomVar+"]");
				} // end if..else check for mode defined
				
		
		
			} // end for
			bindNewProxies();
		} // end if.. check for intentional content
		
	} // end addPendingProxyRelations


	/** 
	The method <i>createProxyQueue</i> cycles over the proxy structures in the Hashtable localProxyStructures, and checks whether they are new or present updates to already 
	generated local structures. These local structures are stored in (and retrieved from) the monitor state. New or updated proxy structures are put onto a queue (Vector). 
	The method returns this queue as a result. 

	@return	Vector<LocalProxyStructure>		A queue with new or updated proxy structures 
	
	*/ 
	
	private Vector<LocalProxyStructure> createProxyQueue () { 
		Vector<LocalProxyStructure> queue = new Vector<LocalProxyStructure>();
		for (Iterator<String> localPRXIter = localProxyStructures.keySet().iterator(); localPRXIter.hasNext(); ) { 
			String discRef = localPRXIter.next();
			log("Queue creation, checking for existence of proxy structure for discourse referent ["+discRef+"]");
			// Get the created local proxy structure, and ensure that it has a discourse referent as global namespace identifier
			LocalProxyStructure createdPRX = (LocalProxyStructure) localProxyStructures.get(discRef);
			createdPRX.setGlobalID(discRef);
			if (monitorState.existsProxyStructure(discRef)) { 
				log("Proxy structure exists, check for equality");
				try { 
					// Get the proxy structure from the monitor state
					LocalProxyStructure storedPRX = (LocalProxyStructure) monitorState.getProxyStructure(discRef); 
					// Check for equality
					short prxEquality = createdPRX.equals(storedPRX); 
					if (prxEquality == 2) { 
						// Complete equality, so do not store on the queue
						log("Stored and created proxy structures are equal, so do not queue");
					} else { 
						// Inequality, so store the updated proxy structure and add it to the queue
						log("Created proxy structure is an update, so store in the monitor state and add it to the queue");
						queue.addElement(createdPRX);
						monitorState.updateProxyStructure(discRef,(Object)createdPRX);						
					} // end if..else check for equality
				} catch (BindingException be) { 
					log("ERROR while creating a queue with local proxy structures: "+be.getMessage()); 
				} // end try..catch
			} else { 
				try { 
					// Proxy structure new, so add to queue and store in monitor state
					log("Proxy structure new, so add to queue and store in monitor state");
					queue.addElement(createdPRX);
					monitorState.addProxyStructure(discRef,(Object)createdPRX);
				} catch (BindingException be) { 
					log("ERROR while trying to add a proxy structure ["+discRef+"] to the monitor state:\n"+be.getMessage());
				} // end try..catch
			} // end if..else check for existence of a local proxy structure
		} // end for
		return queue; 
	} // end createProxyQueue

	/** 
	The method <i>storeQueuedProxies</i> cycles over the proxy structures in the queue. For each proxy structure, it uses the LocalToBindingMapping class to map 
	content from a local proxy structure representation to a binding SA proxy. 

	@param	queue	The queue with LocalProxyStructure objects
	*/ 

	  private void storeQueuedProxies (Vector<LocalProxyStructure> queue, String plfsId, CacheWrapper discRefs, TreeSet bindingFilter, Vector<PendingProxyRelation> pendingRels) { 
		try { 
			boolean intentionalContent = false;
			intentionalProxies = false; 
			// Set for basic binding SA
			setBindingSA(bindingSA);
			log("Storing indexical proxy structures onto WM of ["+bindingSA+"]");
			// Cycle over the proxies for indexical stuff, to store them
			for (Iterator<LocalProxyStructure> localPRXIter = queue.iterator(); localPRXIter.hasNext(); ) { 
				LocalProxyStructure localPRX = localPRXIter.next();
				// check right away for the intentionality flag
				if (localPRX.hasContentStatus("intentional")) { intentionalContent = true; intentionalProxies=true;}
				// Check for indexical content, only store to indexical if set, or nothing has been set (default)
				if (localPRX.hasContentStatus("indexical") || !localPRX.isSetContentStatus()) { 
					String wmc_id = storeProxyStructure(bindingSA,localPRX,bindingFilter);
						// Update the monitor state with local namespace information
						for (Iterator<String> addedNVsIter = localPRX.getCoveredNodes(); addedNVsIter.hasNext(); ) { 
							String addedNV = addedNVsIter.next();
							String addedNomVar = bindingSA+"_"+ addedNV;
							monitorState.updateLocalSpaceIdentifier(addedNomVar,plfsId,wmc_id);
							storeDiscourseReferent(addedNomVar,plfsId,bindingSA+"_"+discRefs.getDiscRef(addedNV));
							// Next we need to check whether we have added but also excluded a nominal 
							// If so, we need to add a global identifier -- this case is not covered by the addDiscourseReferents method
							if (monitorState.isExcluded(addedNomVar,plfsId)) { 
								storeDiscourseReferent(addedNomVar,plfsId,discRefs.getDiscRef(addedNV));
							} // end if.. check whether added but also excluded				
						} // end for		
				} // end if.. check
			} // end for	
			// first bind the new proxies
			bindNewProxies();
			// Should the queue be empty, then the above for-loop will not have run, 
			// meaning the flag intentionalProxies has not be properly set. 
			if (queue.size() == 0) { 
				log("The queue for (indexical) proxies to be stored is empty, so check whether there is any intentional content");
				for (LocalProxyStructure localPRX : localProxyStructures.values()) { 
					if (localPRX.hasContentStatus("intentional")) { 
						intentionalContent = true; 
						intentionalProxies=true;
						break; 
					}
				} // end for
				// Should there be any intentional content, we will now need to copy the created proxy structures to the queue
				queue.addAll(localProxyStructures.values());
			} // end check for queue size
			log("Intentional content? ["+intentionalContent+"]");
			// Check whether the intentional flag has been set; if so, store everything there
			if (intentionalContent) { 
				// change the binding SA
				setBindingSA(motivSA);
				log("Storing intentional proxy structures onto WM of ["+motivSA+"]");
				// for (Iterator<LocalProxyStructure> intentPRXIter = queue.iterator(); intentPRXIter.hasNext(); ) { 
				for (Iterator<LocalProxyStructure> intentPRXIter = localProxyStructures.values().iterator(); intentPRXIter.hasNext(); ) { 
					LocalProxyStructure localPRX = intentPRXIter.next();
					String wmc_id = storeProxyStructure(motivSA,localPRX, new TreeSet());
					// Update the monitor state with local namespace information
					for (Iterator<String> addedNVsIter = localPRX.getCoveredNodes(); addedNVsIter.hasNext(); ) { 
						String addedNV = addedNVsIter.next();
						String addedNomVar = motivSA+"_"+addedNV;
						monitorState.updateLocalSpaceIdentifier(addedNomVar,plfsId,wmc_id);
						storeDiscourseReferent(addedNomVar,plfsId,motivSA+"_"+discRefs.getDiscRef(addedNV));				
						// Next we need to check whether we have added but also excluded a nominal 
						// If so, we need to add a global identifier -- this case is not covered by the addDiscourseReferents method
						if (monitorState.isExcluded(addedNomVar,plfsId)) { 
							storeDiscourseReferent(addedNomVar,plfsId,discRefs.getDiscRef(addedNV));
						} // end if.. check whether added but also excluded				
					} // end for
				} // end for over proxies
				// Bind the new proxies
				bindNewProxies();
			} // end if.. check 
				
		} catch (BindingComponentException be) {
			System.err.println("ERROR while storing queued proxies:\n"+be.getMessage());
		} catch (SubarchitectureProcessException spe) { 
			System.err.println("ERROR while storing queued proxies:\n"+spe.getMessage());		
		} // end try.. catch
	} // end storeQueuedProxies

	/**
	The method <i>storeProxyStructure</i> takes the given local proxy structure and maps its features to binding proxy Objects, 
	which are subsequently stored with a proxy on the binding WM. The methdo assumes that a proxy (shell) has been created for
	storing the features in -- if not, exceptions will be thrown. The method returns the working memory identifier of the proxy 
	on the binding WM. 

	@param	prx		The local proxy structure to be stored as a proxy on binding WM
	@return String	The working memory ID of the proxy on binding WM
	*/ 

	private String storeProxyStructure (String subarch, LocalProxyStructure prx, TreeSet bindingFilter) 
		throws BindingComponentException, SubarchitectureProcessException
	{ 
		// Check whether we have a proxy address for the global namespace identifier of the proxy structure
		String prxGID = subarch+"_"+prx.getGlobalID();
		if (monitorState.existsGlobalSpaceProxyAddress(prxGID) && (!subarch.equals(motivSA))) { 
			// Get the proxy address to which the global namespace identifiers refers
			String proxyAddress = monitorState.getGlobalSpaceProxyAddress(prxGID); 
			// Initialize to update an existing proxy
			this.changeExistingProxy(proxyAddress);
		} else {
			// Start a new proxy
			this.startNewProxy();
		} // end if..else for starting a proxy shell on the binding WM. 
		// Initialize the mapping
		LocalToBindingMapping mapping = new LocalToBindingMapping(subarch);
		// Check whether to add a SourceData feature
		if (!subarch.equals(bindingSA)) { 
			String bindingID = bindingSA+"_"+prx.getGlobalID();
			if (monitorState.existsGlobalSpaceProxyAddress(bindingID)) { 
				// Get the proxy address to which the global namespace identifiers refers
				String proxyAddress = monitorState.getGlobalSpaceProxyAddress(bindingID); 
				if (proxyAddress == null) { 
					log("ERROR !!! Adding NULL proxy address ["+proxyAddress+"] as binding source data for ["+prx.getGlobalID()+"/"+bindingID+"]");
					System.exit(0);
				} else { 
					log("Adding proxy address ["+proxyAddress+"] as binding source data for ["+prx.getGlobalID()+"/"+bindingID+"]");
					Object sourceDataFeature = mapping.map("BindingSource",proxyAddress);
					this.addFeatureToCurrentProxy(sourceDataFeature);
				} 
			} else { 
				log("No source data provided for ["+prx.getGlobalID()+"], no global space proxy address for ["+bindingID+"]");
			
			} 
		} // end 
		// Cycle over the features, map each feature to an Object, and add it to the current proxy
		boolean nullValue = false; 
		for (Iterator<String> featsIter = prx.getFeatures(); featsIter.hasNext(); ) { 
			String feature = featsIter.next();
			if (!bindingFilter.contains(feature)) { 
				try {	
						String value = prx.getFeatureValue(feature);
						Object proxyFeature = mapping.map(feature,value);
						this.addFeatureToCurrentProxy(proxyFeature);
				} catch (NullPointerException npe) { 
					nullValue = true;
				} // end try..catch
			} else { 
				if (!bindingFilter.contains(prx.getGlobalID())) { 
					log("Not omitting feature ["+feature+"] for binding structure for nomvar ["+prx.getGlobalID()+"]");
					String value = prx.getFeatureValue(feature);
					Object proxyFeature = mapping.map(feature,value);
					this.addFeatureToCurrentProxy(proxyFeature);
				} else { 
					log("Omitting feature ["+feature+"] for binding structure for nomvar ["+prx.getGlobalID()+"]");
				} // end if.. else 
			} 
		} // end for over features
		// Add salience to the proxy
		addSalienceToCurrentProxy(); 
		log("Stored proxy ["+prxGID+"] / ["+prx.getFeatureValue("Concept")+"] at WM ["+subarch+"]");
		return storeCurrentProxy();

	} // end storeProxyStructure






	/** 
		The method <i>getPackedNominalSort</i> returns the (stable) ontological sort of the given nominal. 
		
		@param nom The packed nominal
		@return String	The ontological sort
	*/ 

	private String getPackedNominalSort (PackedNominal nom) { 
		TreeSet addedSorts = new TreeSet();
		ArrayList<PackedOntologicalSort> packedSorts = new ArrayList<PackedOntologicalSort>(Arrays.asList(nom.packedSorts));
		Iterator sortsIter = packedSorts.iterator();
		while (sortsIter.hasNext()) { 
			PackedOntologicalSort pSort = (PackedOntologicalSort) sortsIter.next();
			String sort = pSort.sort;
			if (!addedSorts.contains(sort)) { 
				addedSorts.add(sort);
			} // end if..check whether already added
		} // end while over sorts
		return (String) addedSorts.first();
	} // end getPackedNominalSort


	protected PackedNominal getDependentNominal (PackedNominal head, String relation, TreeMap packedNoms) { 
		PackedNominal result = null; 
		boolean depFound = false; 
		ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(head.rels));
		if (relation.equals("") && relations.size() > 0) { 
			LFRelation rel = (LFRelation) relations.get(0); 
			result = (PackedNominal) packedNoms.get(rel.dep);
		} else { 
			Iterator relsIter = relations.iterator();
			while (relsIter.hasNext() && !depFound) { 
				LFRelation rel = (LFRelation) relsIter.next();
				if (rel.mode.equals(relation)) { 
					// Get the dependent nominal
					String depVar = rel.dep;
					if (packedNoms.containsKey(depVar)) { 
						result = (PackedNominal) packedNoms.get(depVar);
						depFound = true;
					} // end if check for availability of the nominal in the map
				} // end if.. check whether the right relation 
			} // end while over relations
			// If we still haven't found a dependent, cycle over
			// the packing edges
			if (!depFound) { 
				if (head.pEdges != null) { 
					ArrayList<PackingEdge> packingEdges = new ArrayList<PackingEdge>(Arrays.asList(head.pEdges));
					Iterator peIter = packingEdges.iterator();
					while (peIter.hasNext() && !depFound) { 
						PackingEdge packingEdge = (PackingEdge) peIter.next();
						if (packingEdge.mode.equals(relation)) { 
							ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
							if (targets.size() > 0) { 
								PackingNodeTarget target = targets.get(0);
								String targetNV = target.pnId.substring(0,target.pnId.indexOf("_PN"));
								result = (PackedNominal) packedNoms.get(targetNV);
								log("[AbstractProxyFactory] Getting dependent nominal: scanning packing edge targets under head node ["+head.nomVar+"], taking the first of a total ["+targets.size()+"] with id ["+targetNV+"]");						
							} // end if.. check for available targets
						} // end if .. check for mode of packing edge
					} // end while
				} // end check for there being packing edges
			} // end if.. check whether to check the packing edges 
		} // end if..else check for type
		return result;
	} // end getDependentNominal



	/** 
		Returns whether the given packed nominal has dependents (LFRelation or PackingEdge)
	*/ 

	/** 
		The method <i>hasDependent</i> returns a boolean indicating whether the nominal has a dependent of the given type. 
		
		@param head		The head nominal
		@param relation	The type being looked for
		@return boolean	Indicating whether the head has a relation of the given type
	*/ 
	protected boolean hasDependent (PackedNominal head, String relation) { 
		boolean result = false; 
		if (head != null) { 
			if (head.rels != null) { 
				ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(head.rels));
				Iterator relsIter = relations.iterator();
				while (relsIter.hasNext() && !result) { 
					LFRelation rel = (LFRelation) relsIter.next();
					log("Checking relation ["+rel.mode+"] against ["+relation+"]");
					if (rel.mode.equals(relation)) { 
							result = true;
					} // end if.. check whether the right relation 
				} // end while over relations
			} 
			if (head.pEdges != null) { 
				boolean depFound = false;
				ArrayList<PackingEdge> packingEdges = new ArrayList<PackingEdge>(Arrays.asList(head.pEdges));
				Iterator peIter = packingEdges.iterator();
				while (peIter.hasNext() && !depFound) { 
					PackingEdge packingEdge = (PackingEdge) peIter.next();
					log("Checking packing edge relation ["+packingEdge.mode+"] against ["+relation+"]");					
					if (packingEdge.mode.equals(relation)) { 
						ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
						if (targets.size() > 0) { 
							result = true;
						} // end if.. check for available targets
					} // end if .. check for mode of packing edge
				} // end while
			} else { 
				log("The nominal has neither packing edges nor lf relations");
			} // end check for there being packing edges
		} else { 
			System.err.println("ERROR [AbstractProxyFactory] cannot determine relation because head ["+head+"]");
		} // end check for presence of relations
		return result;
	} // end hasDependent


	/** 
		Returns whether the given packed nominal has dependents (LFRelation or PackingEdge)
	*/ 
	
	private boolean hasDependents (PackedNominal nom) { 
		boolean result = false;
		log("For nominal ["+nom.nomVar+"] rels are ["+nom.rels+"] and packing edges ["+nom.pEdges+"]");
		if (nom.rels != null || nom.pEdges != null) { 
			ArrayList<LFRelation> rels = new ArrayList<LFRelation>(Arrays.asList(nom.rels));
			log("For nominal ["+nom.nomVar+"] rels are size ["+rels.size()+"]");
			if (rels.size() > 0) { 
				result = true;
			} else { 
				if (nom.pEdges != null) { 
					ArrayList<PackingEdge> perels = new ArrayList<PackingEdge>(Arrays.asList(nom.pEdges));						
					log("For nominal ["+nom.nomVar+"] packing edges are size ["+perels.size()+"]");				
					if (perels.size() > 0) { 
						result = true;
					} // end if check for packing edges
				}
			} // end if check for lf relations
		} // end if.. check for there being relations or packing edges
		return result;
	} // end hasDependents


	/** 
		The method <i>initializeProxyAddress</i> checks whether we have already introduced a proxy for the given nominal 
		variable. If so, we return the address; otherwise, <tt>null</tt> is returned. 

		
	*/ 

	private String initializeProxyAddress (String plfsId, String nomVar, CacheWrapper discRefs) { 
		String nomVarProxyAddress = null;
		try { 
			// check whether there is a discourse referent
			String discRef = discRefs.getDiscRef(nomVar);
			// check whether there is a proxy for that referent (it may be new)
			if (monitorState.existsGlobalSpaceProxyAddress(discRef)) { 
				nomVarProxyAddress = monitorState.getGlobalSpaceProxyAddress(discRef);
				log("Using discourse referent for initializing proxy  for ["+nomVar+"], ["+nomVarProxyAddress+"]");					
			} else { 
				nomVarProxyAddress = monitorState.getLocalSpaceProxyAddress(nomVar,plfsId);
				log("Reusing existing local proxy address for initializing proxy for ["+nomVar+"], ["+nomVarProxyAddress+"]");			
			} // end if..else check for global/discref or local proxy address 
		} catch (BindingException be) {
			// System.err.println("[ERROR: ComSysBindingMonitor] When trying to initialize proxy address: "+be.getMessage()); 
		} // end try.. catch thrown if no such identifier
		/*
		// if we have an address, we need to see whether we can already update the proxy
		if (nomVarProxyAddress != null) { 
			try { 
				log("Registering a proxy monitor on address ["+nomVarProxyAddress+"]");
				ProxyMonitor monitor = ProxyMonitor.newProxyMonitor(this, nomVarProxyAddress, getBindingSA());
				// create an object that we will use to sync with
				Object lockObject = new Object();
				// now lets register a callback that calls notify on this object
				monitor.registerRebindCallback(new NotifyOnRebindCallback(lockObject));
				// now wait for the monitor to inform us that the proxy has been bound
				while (!monitor.proxyIsBound()) {
					try { 
						// get monitor ownership
						synchronized (lockObject) {
							lockObject.wait();
						} // end synchronized wait
					} catch (InterruptedException ie) { 
						System.err.println("[ERROR: ComSysBindingMonitor] IE While trying to initialize a proxy address for updating: "+ie.getMessage());
					} // end try.. catch
				} // end while
			} catch (BindingComponentException be) { 
				System.err.println("[ERROR: ComSysBindingMonitor] BE While trying to initialize a proxy address for updating: "+be.getMessage());			
			} catch (SubarchitectureProcessException se) {
				System.err.println("[ERROR: ComSysBindingMonitor] SE While trying to initialize a proxy address for updating: "+se.getMessage());						
			} // end try..catch
		} // end check for updating an existing address
		*/
		log("Exiting proxy address initialization");
		return nomVarProxyAddress;
	} // end initializeProxyAddress


	/**
	
	
	*/ 

	private void storeDiscourseReferent (String nomVar, String plfsId, String discRef) 
		throws BindingException
	{ 
		LocalIdentifier localId = monitorState.getLocalSpaceIdentifier(nomVar,plfsId);
		log("Updating global name space ["+plfsId+"] with global identifier ["+discRef+"]");
		GlobalIdentifier gId = new GlobalIdentifier();
		if (monitorState.existsGlobalSpaceProxyAddress(discRef)) { 
			// Get the global identifier
			gId = monitorState.getGlobalSpaceIdentifier(discRef);
			// update the local identifier
			localId._globalIdentifier = discRef;
			// update the global identifier
			gId.addLocalIdentifier(localId);
		} else { 
			// if not, create a new global identifier
			gId._varId = discRef;		
			gId._proxyAddress = localId._proxyAddress;
			gId.addLocalIdentifier(localId);
		} // end if..else
		// Store the updated local and global identifiers
		monitorState.updateLocalSpaceIdentifier(localId,plfsId);
		monitorState.updateGlobalSpaceIdentifier(gId);		
	} // end storeDiscourseReferent

	
    //=================================================================
    // I/O METHODS
    //=================================================================

	/** Adds the given feature to the proxy */
	public void addProxyFeature (Object feat) 
		throws BindingComponentException, SubarchitectureProcessException
	{
		addFeatureToCurrentProxy(feat);
	} // end addProxyFeature

	public void cancelProxy() 
		throws BindingComponentException , SubarchitectureProcessException
	{
		cancelCurrentProxy();
	} 


	/** Flags the current proxy as hypothetical */ 
	public void makeProxyHypothetical () 
		throws BindingComponentException
	{ 
		makeCurrentProxyHypothetical();
	} // end makeProxyHypothetical

	/** Starts a new proxy */ 
	public void startNewProxy () 
		throws BindingComponentException , SubarchitectureProcessException
	{ 
		startNewBasicProxy();
	} // end startNewProxy

	/** Indicates to the monitor we should update an already present proxy */
	public void updateExistingProxy(String nomVarProxyAddress,HashSet delFeats) 
		throws BindingComponentException, SubarchitectureProcessException 
	{ 
		changeExistingProxy(nomVarProxyAddress,delFeats);
	} // end updateExistingProxy
	
	
	/** makes the current proxy a group proxy */
	
	public void makeCurrentProxyGroup (short size) 
		throws BindingComponentException, SubarchitectureProcessException 	
	{
		makeCurrentProxyAGroup(size); 
	}
	
	
	public void triggerDOTgeneration (String subarch) { 
		try { 
		TriggerDotViewer trigger = new TriggerDotViewer();
		setBindingSA(subarch);
        addToWorkingMemory(newDataID(), 
			   getBindingSA(),
			   trigger, 
			   OperationMode.BLOCKING);
		} catch (Exception e) { 
			System.out.println("ERROR when trigger DOT view on ["+subarch+"]"+e.getMessage());
		} // end try..catch
	} 
	
	
    //=================================================================
    // MAIN METHOD
    //=================================================================

    // Methods that Henrik insists on:
    @Override
    public void configure(Properties _config) {
        super.configure(_config);
    
		try { 
			bindingSA = getBindingSA();
		} catch (BindingComponentException bce) { 	
			System.out.println(bce.getMessage()); 
			System.exit(0);
		} 	
	
		if (_config.containsKey("--incrementalBinding")) { 
			String incrValue = _config.getProperty("--incrementalBinding");
			if (incrValue.equals("false")) { 
				incrementalBinding = false;
				plfCompletenessLevel = 1;
			} else if (incrValue.equals("0") || incrValue.equals("true")) {
				incrementalBinding = true;
				plfCompletenessLevel = 0;
			} else if (incrValue.equals("1")) { 
				incrementalBinding = false;
				plfCompletenessLevel = 1;
			} else if (incrValue.equals("2")) { 
				incrementalBinding = false;
				plfCompletenessLevel = 1;			
			} else {
				incrementalBinding = true;
			} // end 
		} // end if.. check for incremental binding 
		
		log("incremental binding: " + incrementalBinding);
		log("plfCompletenessLevel: " + plfCompletenessLevel);
		
		if (_config.containsKey("--syncModel")) { 
			String syncValue = _config.getProperty("--syncModel");	
			if (syncValue.equals("sync_all")) { 
				syncModel = SYNC_ALL;
			} else if (syncValue.equals("sync_one")) { 
				syncModel = SYNC_ONE;
			} else { 
				syncModel = SYNC_ONE;
			} // end if check for true
		} // end if.. check for hypothetical proxies


		if (_config.containsKey("--motivation")) { 
			String motivationValue = _config.getProperty("--motivation");	
			intentionalityDiff = true;
			motivSA = motivationValue;
		} // end if.. check for motivation	

		if (_config.containsKey("--msa") || _config.containsKey("-msa")) { 
			String motivationValue = _config.getProperty("--msa");	
			intentionalityDiff = true;
			motivSA = motivationValue;
		} // end if.. check for motivation	


		log("Binding SA: ["+bindingSA+"]\n Motivation SA: ["+motivSA+"]");

		if (bindingSA == null || motivSA == null) { 
			System.out.println("ERROR: Binding SA or Motivation SA not set.");
			System.exit(0);
		} 

        //set the source id to be this subarch id
        m_sourceID = m_subarchitectureID;
    }


} // end class definition 
