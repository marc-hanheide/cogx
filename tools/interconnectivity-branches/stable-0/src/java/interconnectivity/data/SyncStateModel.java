//=================================================================
// Copyright (C) 2006 Geert-Jan M. Kruijff (gj@dfki.de)
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

package interconnectivity.data;

//=================================================================
// IMPORTS
//=================================================================

//-----------------------------------------------------------------
// CAST IMPORTS
//-----------------------------------------------------------------

import java.io.IOException;
import java.util.*;

import interconnectivity.io.*;

import cast.core.CASTData;

//=================================================================
// CLASS DOCUMENTATION 
//=================================================================

/** 
The class <b>SyncStateModel</b> implements a state-based synchronization 
model. The model is initialized with a specification of the involved 
processes and the synchronization constraints, which either can be 
read from a file (using a <tt>ConfigurationReader</tt>), or be 
directly provided using accessor methods.  
<p>
To construct a model, a collection of states is generated from the 
processes, with each state corresponding to a process. Then, for every
synchronization constraint for a process A, it is checked that there is 
a process B (state) which generates the data type in the constraint. A and B 
may, but need not be, different processes (!). If these checks are satisfied, 
a transition from B to A is put into place. 
<p>
Once all processes and synchronization constraints have been processed, the 
resulting graph is checked for several properties: 
<ul> 
<li> <b>Start:</b> There has to be at least one state, based on a data-driven process, which 
	 generates an input to another state. </li>
<li> <b>End:</b> There has to be at least one state, based on a goal-driven process, which 
	 generates ouput but which has no outgoing transitions. </li>
<li> <b>Cycles:</b> Check whether there are cycles, and that each cycle has at least one
		transition that leads to a state outside the cycle. 
</ul> 

<p>
State-transitions can be triggered in two ways. The model defines a method <i>updateOnData</i> which 
takes a data type as argument, and then computes the appropriate transition(s). The resulting state(s) 
can then be inquired for runnable processes using <i>getRunnableProcesses</i>. Alternatively, an 
<b>external</b> controller can trigger an update on time-out, should the current state(s) involve 
timing-out transitions. This update is triggered using a call to <i>updateOnTimeOut()</i>. 
<p>
(The other strategy, letting the model transit on time-out and then inform registered controllers, has not
yet been implemented.) 

@see #updateOnDate
@see #updateOnTimeOut
@see #getRunnableProcesses
@see org.cognitivesystems.interconnectivity.io.ConfigurationReader
@version 061020 (started 061019) 
@author  Geert-Jan M. Kruijff (gj@dfki.de) 
*/ 

public class SyncStateModel { 

    //=================================================================
    // GLOBAL VARIABLES
    //=================================================================

	// HashMap with states: state-id (key), state (SyncState)
	HashMap<Integer,SyncState> states; 

	// HashMap connecting process-names to state-id's
	HashMap<String,Integer> processToStateMap; 

	// Vector with the identifiers for the start state(s) in the model
	Vector startStates; 

	// Vector with the identifiers for the end state(s) in the model
	Vector endStates; 

	// Vector with the identifiers for the current state(s) in the model (as automaton)
	Vector currentStates; 
	
	// Boolean indicating whether in the current state(s) there is a 
	// time-out
	boolean currentTimingOut; 
	
	// Timing-out value (in milliseconds)
	int currentTimeOutValue;
	
	// Identifier of state reached when timing out
	int timeOutConsequentState; 
	
	boolean logOutput = true; 

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public SyncStateModel () { 
		init();
	} // end constructor
	

	private void init() { 
		states = new HashMap(); 
		startStates = new Vector();
		endStates = new Vector();
		currentStates = new Vector();
		processToStateMap = new HashMap();
		currentTimingOut = false;
		currentTimeOutValue = -1;
		timeOutConsequentState = -1;
	} // end init


    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

	public void setLogging (boolean l) { logOutput = l; }



    //=================================================================
    // CONSTRUCTION METHODS
    //=================================================================

	/** The method <i>construct(String)</i> takes a filename for a 
		synchronization configuration as argument, and constructs a 
		state-based model from that.  
		
		@param  String the filename for the synchronization configuration file 
		@throws IOException if the provided file is not found (thrown by the ConfigurationReader)
	*/

	public void construct (String filename) 
		throws IOException 
	{ 
		log("Constructing from file ["+filename+"]");
		ConfigurationReader reader = new ConfigurationReader();
		reader.read(filename);
		// get the processes, as iterator over SyncProcess objects, 
		// and construct the states from these objects. 
		try { 
			this.constructStates(reader.getSyncProcesses()); 
			this.constructGraph(reader.getSyncConstraints());
			startStates = this.constructStartStates(states.values().iterator());
			endStates = this.constructEndStates(states.values().iterator());
		} catch (SyncException e) { 
			System.out.println("Error: "+e.getMessage()); 
			System.exit(0);
		} // end try..catch
	} // end 

	/** The method <i>constructStates</i> constructs a collection of states, 
		on the basis of the processes described in the synchronization 
		configuration. 
	*/ 
	public void constructStates (Iterator processIter) 
		throws SyncException
	{  
		log("Constructing states");
		int stateIdCounter = 0; 
		while (processIter.hasNext()) { 
			SyncProcess process = (SyncProcess) processIter.next();
			SyncState state = new SyncState(stateIdCounter);
			
			state.addProcess(process.getName(), process.getDataTypes());
			
			if (process.isIndicatedEndState())
				state.setAsIndicatedEndState() ; 

			if (process.isIndicatedStartState())
				state.setAsIndicatedStartState() ; 
			
			states.put(new Integer(state.getId()),state);
			processToStateMap.put(process.getName(),new Integer(state.getId())); 
			log("processToStateMap ["+process.getName()+","+new Integer(state.getId())+"]"); 
			stateIdCounter++;
		} // end while over SyncProcess objects
	} // end constructStates

	/** The method <i>constructGraph</i> constructs a graph from 
		the collection of synchronization constraints, given the 
		collection of states we have already built up. For each 
		constraint, we check the state corresponding to the process, 
		and generate input edges to the state. The method throws 
		an exception if a process is waiting for a data item which 
		is not generated by any process in the model. 
		
		@param Iterator an iterator over the synchronization constraints in the configuration
	*/ 
	public void constructGraph (Iterator constraintIter) 
		throws SyncException 
	{
		int edgeIdCounter = 0;
		while (constraintIter.hasNext()) { 
			SyncConstraint constraint  = (SyncConstraint) constraintIter.next();
			// get the process for which the constraint is defined
			String processName = constraint.getProcessName();
			// get the state defined for the process
			Integer stateIdInt = (Integer) processToStateMap.get(processName);
			SyncState processState = (SyncState) states.get(stateIdInt); 
			// now cycle over the data constraints
			Iterator dcIter = constraint.getDataConstraints();
			while (dcIter.hasNext()) { 
				SyncDataConstraint dc = (SyncDataConstraint) dcIter.next();
				String dataType = dc.getDataType();
				// find the state(s) which produce the required data-type
				Vector producingStates = this.producingStates(dataType); 
				// cycle over the producing states, construct edges
				for (Iterator pdsIter = producingStates.iterator(); pdsIter.hasNext(); ) {  
					Integer pdsId = (Integer) pdsIter.next();
					SyncState producingState = (SyncState) states.get(pdsId); 
					SyncEdge edge = new SyncEdge(edgeIdCounter); 
					edge.setDataConstraint(dc);
					edge.setTransitionMode(dc.getTransitionMode());
					edge.setTimeoutValue(dc.getTimeoutValue()); 
					edge.setFromState(producingState.getId()); 
					edge.setToState(processState.getId());
					producingState.addOutEdge(edge);
					processState.addInEdge(edge);
					edgeIdCounter++;
				} // end for over producing states
			} // end while over data constraints
		} // end while over configuration constraints
	} // end constructGraph

	/** Returns a Vector with identifiers of states that are considered start states. 
		A start state is defined as a state which does not have any in-edges. 
		
		@param Iterator an iterator over SyncState objects, from which start states should be collected
		@return Vector  a vector of identifiers (Integer)
	*/ 
	
	public Vector constructStartStates (Iterator stIter) 
		throws SyncException
	{ 
		Vector result = new Vector(); 
		while (stIter.hasNext()) { 
			SyncState state = (SyncState) stIter.next();
			if (!state.hasInEdges()) { result.addElement(new Integer(state.getId())); }
			else if (state.isIndicatedStartState()) { result.addElement(new Integer(state.getId())); }
		} // end while
		if (result.size() != 0) { 
			return result;
		} else { 
			throw new SyncException("Model has no identifiable start states!");
		} // end if..else.. check for start states
	} // end constructStartStates

	/** Returns a Vector with identifiers of states that are considered end states. 
		An end state is defined as a state which does not have any out-edges. 

		@param Iterator an iterator over SyncState objects, from which start states should be collected
		@return Vector  a vector of identifiers (Integer)
	*/ 
	public Vector constructEndStates (Iterator stIter) 
		throws SyncException
	{ 
		Vector result = new Vector(); 
		while (stIter.hasNext()) { 
			SyncState state = (SyncState) stIter.next();
			if (!state.hasOutEdges()) {result.addElement(new Integer(state.getId())); }
			else if (state.isIndicatedEndState()) { result.addElement(new Integer(state.getId())); }
		} // end while
		if (result.size() != 0) { 
			return result;
		} else { 
			throw new SyncException("Model has no identifiable end states!");
		} // end if..else.. check for end states
	} // end constructEndStates

	/** Returns a Vector with state identifiers (Integer), which include
		processes that produce the given data type. 
	*/
	private Vector producingStates (String dataType) { 
		Vector result = new Vector();
		for (Iterator idIter = states.keySet().iterator(); idIter.hasNext(); ) {  
			Integer stateId = (Integer) idIter.next();
			SyncState state = (SyncState) states.get(stateId);
			if (state.generatesDataType(dataType)) { 
				result.addElement(stateId);
			} // end if.. 
		} // end for
		return result;
	} // end producingStates

    //=================================================================
    // START, UPDATE METHODS
    //=================================================================
	
	/** Initializes the current state(s) to	point to the start state(s) of the model. 
	*/
	public void start() { 
		currentStates = (Vector) startStates.clone();
	} // end start

	/** Restarts the model; same function as start. 
		@see #start
	*/
	public void restart() { this.start(); } 

	// isInEndState
	
	/** The method <i>updateOnData</i> takes a data type as input, and 
		triggers a transition based on that data type. If a transition 
		is possible, the method updates the resulting current state(s). 
		The method returns a boolean indicating whether a transition 
		was made. 
		
		@param String data type name
		@return boolean whether a transition could be made
	*/ 

	public boolean updateOnData (CASTData dataType) { 
		log("Updating on data");
		boolean result = false; 
		Vector transitedToStates = new Vector ();
		for (Iterator<Integer> csIdIter=currentStates.iterator(); csIdIter.hasNext(); ) { 
			Integer csId = csIdIter.next();
			SyncState state = (SyncState) states.get(csId);
			try { 
				Vector edges = state.getSatisfyingEdges(dataType);
				for (Iterator edgeIter = edges.iterator(); edgeIter.hasNext(); ) { 
					// if we're still here then we know the edge is alright
					// else we're thrown out by the exception
					// so, add the identifier for the to-state
					SyncEdge edge = (SyncEdge) edgeIter.next();
					log("Adding edge");
					transitedToStates.addElement(new Integer(edge.getToState()));
					result = true;
				} // end for over edges 
			} catch (SyncNoSuchEdgeException e) {
				// do nothing, exception thrown if no satisfying edge is found
			} // end try..catch
		} // end for over current states
		if (result) { 
			currentStates = (Vector) transitedToStates.clone();
			
			// plison... include all the start states in the next possible states
			currentStates.addAll((Vector)startStates.clone()) ; 
			
			log("Updated current state(s): "+currentStates.toString());
		} // end if
		return result;
	} // end updateOnData
	
	// updateOnTimeOut


    //=================================================================
    // ACCESSOR METHODS
    //=================================================================
	
	/** Returns an iterator<Integer> over the current states */
	
	public Iterator getCurrentStates () { return currentStates.iterator(); }

	/** Returns the SyncState state object with the given id (or null, 
		if none exist with that id) */

	public SyncState getState (Integer id) {
		if (states.containsKey(id)) { 
			return (SyncState) states.get(id);
		} else { 
			return null; 
		} // end if..else check for state existence
	} // end getState

	/** Returns whether an information process with the given identifier 
		is supported on the current state(s) in which the model is (as
		automaton). */
	
	public boolean isCurrentlySupported (String ipid) { 
		boolean result = false;
		Iterator stateIter = getCurrentStates();
		while (stateIter.hasNext()) { 
			Integer currentStateId = (Integer) stateIter.next();
			SyncState currentState = (SyncState)states.get(currentStateId);
			if (currentState.supports(ipid)) {
				result = true;
				break;
			} // end if
		} // end while over current states
		return result;
	} // end supports

	
	public String getCurrentlySupported () { 
		String result = "[";
		Iterator stateIter = getCurrentStates();
		while (stateIter.hasNext()) { 
			Integer currentStateId = (Integer) stateIter.next();
			SyncState currentState = (SyncState)states.get(currentStateId);
			result += currentState.toString();
		} // end while over current states
		result = result.substring(0, result.length() -1) + "]";
		return result;
	} // end supports

    //=================================================================
    // MISC METHODS
    //=================================================================

	/** Returns a formatted representation of the model */

	public String toString () {
		String result = "";
		for (Iterator stIter = states.values().iterator(); stIter.hasNext(); ) { 
			result = result+( ((SyncState)stIter.next()).toString()+"\n");
		} // end for over states
		result = result+"\n"; 
		result = result+"Start states: "+startStates.toString()+"\n";
		result = result+"End states: "+endStates.toString();
		return result;
	} // end toString 

	public void log (String m) { 
		if (logOutput) { System.out.println(m); }

	} // end log

    //=================================================================
    // MAIN METHOD
    //=================================================================
	
	public static void main (String[] args) { 
		try { 
			SyncStateModel model = new SyncStateModel();
			model.construct(args[0]);
			System.out.println(model.toString());
		} catch (IOException e) { 
			System.out.println("Error: "+e.getMessage());
		} // end try..catch 
	} // end main


} // end class