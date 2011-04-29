// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
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
package de.dfki.lt.tr.dialogue.ref;

//=================================================================
// IMPORTS

// Java
import java.util.HashMap;
import java.util.Iterator;
import java.util.Properties;
import java.util.Vector;
import javax.xml.datatype.DatatypeConfigurationException;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;

import de.dfki.lt.tr.dialogue.slice.ref.RefReading;
import de.dfki.lt.tr.dialogue.slice.ref.RefReadings;
import de.dfki.lt.tr.dialogue.util.DialogueInvalidOperationException;
import de.dfki.lt.tr.dialogue.util.DialogueMissingValueException;

// Dialogue API 
import de.dfki.lt.tr.dialogue.ref.ReadingFactory;

import de.dfki.lt.tr.dialogue.util.LFUtils;

// Meta API
import de.dfki.lt.tr.meta.TRModule;
import de.dfki.lt.tr.meta.TRResultListener;


/** 
 * The class <tt>ReferentialReadings</tt> determines, for a given logical form, the different referential 
 * readings it might have. To this end, the class uses a collection of registered factories. 
 * <p>
 * The class extends the <tt>TRModule</tt> meta specification of processes. Computation is triggered 
 * by invoking the <tt>run</tt> method on an input object. 
 * 
 * @author	Geert-Jan Kruijff
 * @email	gj@dfki.de
 * @started	090921
 * @version	100608
 */ 


public class ReferentialReadings 
extends TRModule
{

	private HashMap readingFactories = null; 

	/**
	 * The constructor initializes the internal variables. 
	 */
	
	public ReferentialReadings () 
	{
		init();
	} // end constructor

	/**
	 * Configuration
	 * 
	 * @param properties	The properties to use in configuration
	 */
	
	public void configure (Properties properties) 
	throws DatatypeConfigurationException
	{
	
	}
	
	/**
	 * Resets the internal variables, notably clearing the map with reading factories. 
	 */
	
	public void reset() 
	{
		readingFactories = null; 
	} // end reset
	
	private void init () { 
		readingFactories = new HashMap<String,ReadingFactory>();		
	} // end method init
	
	
	/**
	 * Main method to call on constructing referential readings (output) from a logical form (input). 
	 * The method overrides the run method provided by <tt>TRModule</tt>
	 * @param input		The input object, required to be a LogicalForm object
	 * @return Object	The output object, cast from a RefReadings object
	 * @throws UnsupportedOperationException	If the input object is not an instance of LogicalForm
	 */
	
	@Override
	public Object run (Object input) 
	throws UnsupportedOperationException
	{
		if (input instanceof LogicalForm)
		{
			try { 
				this.setInput(input);
				this.setOutput((Object) constructReadings ((LogicalForm)input)); 
				return this.getOutput();
			} 
			catch (DialogueMissingValueException dmve) {
				throw new UnsupportedOperationException (dmve.getMessage());
			}
			catch (DialogueInvalidOperationException dioe) {
				throw new UnsupportedOperationException (dioe.getMessage());
			}
		} else {
			throw new UnsupportedOperationException("Cannot run referential readings: "+
					"Object of class LogicalForm required");
		}
	} // end run
	
	
	/**
	 * Constructs the readings for a given logical form, using the reading factories that 
	 * have been registered. 
	 * 
	 * @param 	lf	The logical form for which the readings are to be determined
	 * @return	RefReadings	An object including a list of restricted and attributive readings
	 * @throws 	DialogueMissingValueException 		Thrown if the logical form is null
	 * @throws 	DialogueInvalidOperationException	Thrown if no factory is known for the logical form's 
	 * 				root sort, or if no factories have been registered at all
	 */
	
	public RefReadings constructReadings (LogicalForm lf) 
	throws DialogueMissingValueException, DialogueInvalidOperationException 
	{ 
		if (lf == null) 
		{
			throw new DialogueMissingValueException("Cannot construct readings: "+
					"Provided logical form is null");
		}
		if (readingFactories == null || readingFactories.size() == 0)
		{
			throw new DialogueInvalidOperationException("Cannot construct readings: "+
					"Reading factories list is null or empty");
		}
		RefReadings results = new RefReadings();
		Vector excludes = new Vector();
		// cycle over the nominals in the logical form, examine the non-yet-excluded ones
		Iterator<LFNominal> lfnomsIter = LFUtils.lfGetNominals(lf);
		while (lfnomsIter.hasNext()) {
			LFNominal nom = lfnomsIter.next();
			if (!excludes.contains(nom.nomVar)) { 
				// construct the readings
				if (!readingFactories.containsKey(nom.sort))
				{
					throw new DialogueInvalidOperationException("Cannot construct readings: "+
					"No factory known for nominal sort ["+nom.sort+"]");
				}
				ReadingFactory factory = (ReadingFactory) readingFactories.get(nom.sort);
				ReadingFactoryResults fresults = factory.constructReadings(lf);
				// get the results: update the readings, update the excludes
				excludes.addAll(fresults.getExcludes());
				RefReadings readings = fresults.getReadings();
				results.refRdngs = addRefReadingArrays(results.refRdngs,readings.refRdngs);
			} // end if.. 
		} // end while
		// return results
		return results;
	} // end method constructReadings
	
	
	
	/**
	 * Registers a factory for creating referential readings for a logical form with a specific 
	 * nominal root sort. The factory specifies the sort it operates on. 
	 * 
	 * @param factory	The factory to be registered
	 * @throws DialogueMissingValueException		Thrown if the provided factory is null, or missing fields
	 * @throws DialogueInvalidOperationException	Thrown if the map already contains a factory for the given nominal sort
	 */
	
	public void registerFactory (ReadingFactory factory)
	throws DialogueMissingValueException, DialogueInvalidOperationException
	{
		if (factory == null)
		{
			throw new DialogueMissingValueException("Cannot register factory: "+
					"Provided factory is null");
		}
		String factorySort = factory.getSort();
		if (factorySort == null || factorySort.equals(""))
		{
			throw new DialogueMissingValueException("Cannot register factory: "+
					"Factory has empty or null sort");
		}
		if (readingFactories.containsKey(factorySort))
		{
			throw new DialogueInvalidOperationException("Cannot register factory: "+
					"Factory acts on sort that is already registered");
		}
		readingFactories.put(factorySort, factory);
	} // end registerFactory
	
	
	/**
	 * Adds the given arrays of readings together, returns the combined array
	 * @param array1	The first array of readings
	 * @param array2	The second array of readings
	 * @return RefReading[] 	The combined array of RefReading objects
	 */
	
	private RefReading[] addRefReadingArrays (RefReading[] array1, RefReading[] array2) { 
		RefReading[] result = new RefReading[array1.length+array2.length];
		int i;
		for (i=0; i<array1.length; i++) { result[i] = array1[i]; } 
		i = i+1; // move one beyond the array1 length
		for (int j=0; j < array2.length; j++) { result[i+j] = array2[j]; }
		return result;
	} // end addReadingArrays
	
	
} // end class
