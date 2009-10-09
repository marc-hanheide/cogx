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

package interconnectivity.io;

//=================================================================
// IMPORTS
//=================================================================

//-----------------------------------------------------------------
// INTERCONNECTIVITY IMPORTS
//-----------------------------------------------------------------

import java.io.*;
import java.util.*;

import interconnectivity.data.*;

//=================================================================
// CLASS DOCUMENTATION 
//=================================================================

/** 

The class <b>ConfigurationReader</b> implements a reader for synchronization 
specification files. A filename can be provided as argument to a the class, 
to a read-method, or as command-line argument. The reader creates a collection
of <tt>SyncProcess</tt> objects, and a collection of <tt>SyncConstraint</tt> 
objects, from which a state-based synchronization model can then be built. 
<p>
The syntax for process specifications: <p>
<tt>PROCESS &lt;type&gt; &lt;process-name&gt; &lt;dataItem1&gt; ... &lt;dataItemN&gt;</tt><p>
with <tt>type</tt> in {DD,GD}. Each item in the input is separated by white-spaces. 
<p>
The syntax for constraint specifications: <p>
<tt>SYNC &lt;process-name&gt; {&lt;type&gt;:&lt;dataItem1&gt; ; ... ; &lt;type&gt;:&lt;dataItemN&gt;</tt>}<p>
with <tt>type</tt> either WAIT or an integer specifying a time-out (in milliseconds). If there is no type specified, 
the parser assigns a default WAIT type. Note the curly brackets around the data constraints, and the semi-colon 
separator between the data constraints. 

@version 061019 (started 061018) 
@author  Geert-Jan M. Kruijff (gj@dfki.de) 

*/ 

public class ConfigurationReader { 

    //=================================================================
    // GLOBAL VARIABLES
    //=================================================================

	// Name of the file with the synchronization specification
	private String fileName; 
	
	// HashMap with the SyncProcess objects: processName (key), SyncProcess (value)
	private HashMap<String,SyncProcess> syncProcesses; 
	// Vector with the SyncConstraint objects
	private Vector<SyncConstraint> syncConstraints;

	// Counter for synchronization constraint identifiers
	private int syncConstraintCounter;

	// Boolean whether to print log output
	private boolean logOutput = false;

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public ConfigurationReader () { 
		init();
	} // end constructor

	public ConfigurationReader (String fn) { 
		init();
		fileName = fn;
	} // end constructor	


	private void init () { 
		fileName = "interconnectivity.sync"; 
		syncProcesses = new HashMap(); 
		syncConstraints = new Vector();
		syncConstraintCounter = 0; 
	} // end init

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

	/** Returns an iterator over the SyncProcess objects  */ 

	public Iterator getSyncProcesses () { 
		return syncProcesses.values().iterator();
	} // end getSyncProcesses

	/** Returns an iterator over the SyncConstraint objects */ 
	
	public Iterator getSyncConstraints () { 
		return syncConstraints.iterator();
	} // end getSyncConstraints

	//=================================================================
    // IO METHODS
    //=================================================================

	/** The method <i>read</i> reads the configuration from the specified
		filename, and parses the specification to SyncProcess and 
		SyncConstraint objects. 
	
		@throws IOException if the specification file is not found
	*/

	public void read () throws IOException { 
		read(fileName);
	} // end read

	/** The method <i>read</i> reads the configuration from the specified
		filename, and parses the specification to SyncProcess and 
		SyncConstraint objects. 
	
		@throws IOException if the specification file is not found
	*/

	public void read (String fn) 
		throws IOException 
	{ 
		fileName = fn; 
		BufferedReader reader=new BufferedReader (new FileReader(fileName)); 
		String line="";
		int lineCounter = 0;
		while ((line = reader.readLine()) != null) { 
			lineCounter++;
			if (line.startsWith("PROCESS")) {
				try { 
					SyncProcess process = parseToProcess(line);
					if (!syncProcesses.containsKey(process.getName())) { 
						syncProcesses.put(process.getName(), process);
					} else { 
						System.out.println("WARNING: Duplicate specification for process ["+process.getName()+"] in l."+lineCounter+" (previous specification will be overwritten."); 
 						syncProcesses.put(process.getName(), process);
					} // end if..else check for unique process names
				} catch (SyncException e) { 
					System.out.println("Syntax error in PROCESS specification l."+lineCounter+": "+e.getMessage()); 
				} // end try..catch 	
			} else if (line.startsWith("SYNC")) { 
				try { 
					SyncConstraint constraint = parseToConstraint(line);
					syncConstraints.addElement(constraint);
				} catch (SyncException e) { 
					System.out.println("Syntax error in SYNC specification l."+lineCounter+": "+e.getMessage());  
				} // end try..catch
			} 
			else if (line.startsWith("STARTSTATE")) {
				try {
					String[] lineTokens = line.split(" ") ;
					if (syncProcesses.containsKey(lineTokens[1])) {
						syncProcesses.get(lineTokens[1]).setAsIndicatedStartState() ;
					}
				}
				catch (Exception e) {
					System.out.println("Syntax error in SYNC specification l."+lineCounter+": "+e.getMessage());  
				}
			}
				
				else if (line.startsWith("ENDSTATE")) {
					try {
					String[] lineTokens = line.split(" ") ;
						if (syncProcesses.containsKey(lineTokens[1])) {
							syncProcesses.get(lineTokens[1]).setAsIndicatedEndState() ;
						}
					}
					catch (Exception e) {
						System.out.println("Syntax error in SYNC specification l."+lineCounter+": "+e.getMessage());  
					}
				}
			else { 
				// ignore input line
			} // end if..else check for type of input line
		} // end while reading the file
		reader.close();
	} // end read


	/** Returns a SyncProcess object built from the specification line 
	
		@throws SyncException if there is a syntax error in the specification line
	*/ 

	private SyncProcess parseToProcess (String line) 
		throws SyncException 
	{ 
		SyncProcess process = new SyncProcess();
		StringTokenizer st = new StringTokenizer(line);
		int tokenCounter = 0; 
		while (st.hasMoreTokens()) { 
			String token = st.nextToken();
			switch (tokenCounter) { 
				case 0 :	log(token); 
							break; 
				case 1 :	log(token); 
							if (token.equals("DD")) { process.setType(SyncProcess.PROCESSTYPE_DD); 
							} else if (token.equals("GD")) { process.setType(SyncProcess.PROCESSTYPE_GD); }
							break; 
				case 2 :	log(token);
							process.setName(token);
							break;
				default:	log(token);
							process.addDataType(token); 
							break; 
			} // end switch over token 
			tokenCounter++;
		} // end while over tokens
		return process; 
	} // end parseToProcess

	/** Returns a SyncConstraint object built from the specification line 
	
		@throws SyncException if there is a syntax error in the specification line	
	*/ 

	private SyncConstraint parseToConstraint (String line) 
		throws SyncException 
	{ 
		SyncConstraint constraint = new SyncConstraint();
		// First just get the process name
		StringTokenizer st = new StringTokenizer(line);
		int tokenCounter = 0;
		while (st.hasMoreTokens()) { 
			String token = st.nextToken();	
			switch (tokenCounter) {
				case 0 :	log(token); 
							break; 
				case 1 :	log(token);
							constraint.setProcessName(token);
							break;
				default :	break; 
			} // end switch over token
			tokenCounter++;
		} // end while
		// Next get the data items to be synch'd on
		int openBracketPos  = line.indexOf("{");
		int closeBracketPos = line.indexOf("}");
		if (closeBracketPos - openBracketPos > 2) { 
			String dcsString = line.substring(openBracketPos+1,closeBracketPos); 
			log("Parsing data constraints: ["+dcsString+"]");			
			StringTokenizer stdc = new StringTokenizer(dcsString,"; \t");
			while (stdc.hasMoreTokens()) { 
				String dcString = (String) stdc.nextToken();
				SyncDataConstraint dcConstraint = new SyncDataConstraint(syncConstraintCounter);
				syncConstraintCounter++;
				int colonSepPos = dcString.indexOf(":");
				if (colonSepPos == -1) { 
					// no colon, interpret as default WAIT
					dcConstraint.setDataType(dcString);
					dcConstraint.setTransitionMode(SyncDataConstraint.TRANSIT_WAIT);
				} else {  
					String mode = dcString.substring(0,colonSepPos);
					String dataType = dcString.substring(colonSepPos+1,dcString.length());
					dcConstraint.setDataType(dataType);
					if (mode.equals("WAIT")) {
						dcConstraint.setTransitionMode(SyncDataConstraint.TRANSIT_WAIT); 
					} else { 
						try { 
							Integer timeOutValue = new Integer(mode);
							dcConstraint.setTransitionMode(SyncDataConstraint.TRANSIT_TIMEOUT);
							dcConstraint.setTimeoutValue(timeOutValue.intValue()); 							
						} catch (NumberFormatException e) {
							throw new SyncException("Malformed SYNC data constraint transition mode ["+mode+"] for process ["+constraint.getProcessName()+"]\n"+e.getMessage());
						} // end try..catch for transition number value 
					} // end if..else check for mode
				} // end if..else check for colon 
				// Finally, add the data constraint
				log(dcConstraint.toString());
				constraint.addDataConstraint(dcConstraint);
			} // end while over tokens
		} else { 
			throw new SyncException("Malformed SYNC data constraint specification for process ["+constraint.getProcessName()+"]");
		} // end if..else check for data constraints
		return constraint; 
	} // end parseToProcess


    //=================================================================
    // MISC METHOD
    //=================================================================

	private void log (String msg) { 
		if (logOutput) {
			System.out.println(msg); 
		} 
	} // end log

    //=================================================================
    // MAIN METHOD
    //=================================================================

	public static void main (String[] args) { 
		try { 
			ConfigurationReader reader = new ConfigurationReader();  
			reader.read(args[0]);
		} catch (IOException e) { 
			System.out.println(e.getMessage()); 
		} // end try..catch
	} // end main


} // end class


