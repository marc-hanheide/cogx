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

package comsys.components.gui;

// =================================================================
// PACKAGE IMPORTS
// =================================================================

// -----------------------------------------------------------------
// CAST IMPORTS
// -----------------------------------------------------------------
import java.util.Hashtable;
import java.util.Properties;
import java.util.Vector;

import javax.swing.UIManager;

import java.util.Map;

import comsys.datastructs.comsysEssentials.*; 
import comsys.arch.ProcessingData;
import comsys.arch.ComsysException;
import comsys.utils.EventStructureUtils;
import comsys.utils.SDRSUtils;
import comsys.gui.PackedLFVisualizerGUI;
import comsys.lf.utils.LFUtils;

import comsys.components.PackedLFProcessorComponent;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.SubarchitectureComponentException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import cast.core.CASTData;



// =================================================================
// CLASS DOCUMENTATION
// =================================================================

/**
	The class <b>PackedLFVisualizer</b> monitors the comsys WM for 
	PackedLFs to appear there. Every time a PackedLF is added or updated, 
	a graph-visualization is made of its structure. 
	
	@started 070907 
	@version 070907
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
	@author	 Pierre Lison (pierrel@coli.uni-sb.de)
*/ 

public class PackedLFVisualizer 
	extends PackedLFProcessorComponent
{

    // =================================================================
    // GLOBAL DATA STRUCTURES
    // =================================================================

	// the subdirectory in which to store the DOT and PNG files
	String graphsDir = "./subarchitectures/comsys/graph/parser";


	// the subdirectory in which to store the DOT and PNG files for the SDRS
	String graphsDirSDRS = "./subarchitectures/comsys/graph/sdrs";
	
	// a hashtable with packedlf identifiers matched to counters, 
	// to be able to produce series of graphs for an incrementally formed packedlf
	Hashtable<String,Integer> plfIds; 

	// boolean indicating whether graphs should be made incrementally. 
	// if false, then the graph is always overwritten with the latest packed lf structure. 
	boolean incrementalSeries; 

	PackedLFVisualizerGUI gui; 
	
	SDRSFormula lastProcessedFormula ;
	public SDRS lastReceivedSDRS;
	public SpatioTemporalRepresentation lastReceivedSTR;
	
	boolean processSDRS = false;
	
	// increment on the SDRS structure
	int sdrsIncr = 0;
	
	String lastFormulaId = "";
	
	// whether we should generate the PNG files or not
	public boolean generatePNG = true;
	
    // =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================


	@Override
	public void start () { 

		super.start();
		
		log("Starting the Visualization module...");
		
	    //nah: try resetting to default incase matlab has
		//messed with it
		try {
		    UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		}
		catch(Exception e) {
		    e.printStackTrace();
		}


		plfIds = new Hashtable<String,Integer>();
		gui    = new PackedLFVisualizerGUI(this);
		gui.setGraphsDir(graphsDir);
		gui.setGraphsDirSDRS(graphsDirSDRS);
		try {
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(Nucleus.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					processNucleus(_wmc);
				}
			});
			
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(SpatioTemporalRepresentation.class,  WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					processSpatioTemporalRepresentation(_wmc);
				}
			});
			
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(SDRS.class,  WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				processSDRS(_wmc);
			}
			
			
		});
	
		lastProcessedFormula = new SDRSFormula();
		lastProcessedFormula.label = "start";
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	} // end init

	/**
	* The method <i>configure</i> processes the command-line options provided to the class. 
	* The following options are processed: 
	* <ul>
	* <li><tt>--dir [directory]</tt>: the directory to be used for storing the graphs, sets the <tt>graphsDir</tt> variable</li> 
	* </ul>
	* 
	* @param _config The properties table
	*/ 
	
    public void configure(Map<String, String> _config) {
        // _config.list(System.out);
        String parserArg = "";
        if (_config.containsKey("--series")) {
            String value = _config.get("--series");
			if (value.equals("true")) { 
				incrementalSeries = true;
			} else { 
				incrementalSeries = false;
			} // end if..else check for value
        } 
        else {
        	incrementalSeries = false;
        } 
        if (_config.containsKey("--sdrs")) {
            String value = _config.get("--sdrs");
			if (value.equals("true")) { 
				processSDRS = true;
			} else { 
				processSDRS = false;
			} // end if..else check for value
        }
        else {
        	processSDRS = false;
        } 		 
        if (_config.containsKey("--dir")) {
            graphsDir = _config.get("--dir");
        }
        else {
            graphsDir = "./graphs/parser/";
        }
        if (_config.containsKey("--dirSDRS")) {
            graphsDirSDRS = _config.get("--dirSDRS");
        }
        else {
            graphsDirSDRS = "./graphs/sdrs/";
        }
        if (_config.containsKey("--generatePNG")) {
        	String value = _config.get("--generatePNG");
			if (value.equals("true")) { 
				generatePNG = true;
				log("PNG files will be generated");
			} else { 
				generatePNG = false;
			} // end if..else check for value
        }
        else {
            graphsDirSDRS = "./graphs/sdrs/";
        }
        // end if..else check for command-line argument
	} // end configure


    synchronized private void processSpatioTemporalRepresentation (WorkingMemoryChange _wmc) {
    	try {
    		// get the id of the working memory entry
    		String id = _wmc.address.id;
    		// get the data from working memory and store it
    		// with its id
    		CASTData strWM = getWorkingMemoryEntry(id);
    		SpatioTemporalRepresentation str = (SpatioTemporalRepresentation) strWM.getData();
			lastReceivedSTR = str;
			log("received spatio-temporal representation!");
    	}
			catch (Exception e) {
				e.printStackTrace();
			}
    }
    synchronized private void processSDRS(WorkingMemoryChange _wmc) {

    	try {
    		// get the id of the working memory entry
    		String id = _wmc.address.id;
    		// get the data from working memory and store it
    		// with its id
    		CASTData plfWM = getWorkingMemoryEntry(id);
    		SDRS sdrs = (SDRS) plfWM.getData();
			lastReceivedSDRS = sdrs;

    		if (sdrs != null && processSDRS) {
    			
    			SDRSFormula formula = SDRSUtils.getLastFormula(sdrs);

    			
    			if (SDRSUtils.getFormulaType(formula).type.equals(SDRSUtils.RELATION_TYPE) &&
    					!formula.label.equals(lastProcessedFormula.label)) {
    				log("dialogue move formula found");
    				String filename = "sdrs-" + sdrsIncr;
    				SDRSFormula dm = formula;
    				formula = SDRSUtils.getFormula(sdrs, dm.tprec);
    				SDRSUtils.FormulaAndDMToGraph(formula, dm, graphsDirSDRS+filename, generatePNG);
					log("Generation of the SDRS formula + last dialogue move successful, file written in " + graphsDirSDRS+filename);
					sdrsIncr++;
					lastProcessedFormula = formula;
					log("finished processing DM formula");
					}
    			
    			else if (SDRSUtils.getFormulaType(formula).type.equals(SDRSUtils.PLF_TYPE) &&
    					!formula.label.equals(lastProcessedFormula.label)) {
    				if (formula.caches.length > 0) {
    					log("generating graphical representation of the SDRS formula...");
    					String filename = "sdrs-" + sdrsIncr;

    		/**			SDRSFormula dm = SDRSUtils.getFormula(sdrs, formula.tprec);
    					
    					if (dm != null &&
    					SDRSUtils.getFormulaType(dm) == SDRSUtils.SDRS_DISCRIM_RELATION) {							
    						SDRSUtils.FormulaAndDMToGraph(formula, dm, graphsDirSDRS+filename);
        					log("Generation of the SDRS formula + last dialogue move successful, file written in " + graphsDirSDRS+filename);
  					} */
    					
    			//		else {
    						SDRSUtils.FormulaToGraph(formula, graphsDirSDRS+filename, generatePNG);
        					log("Generation of the SDRS formula successful, file written in " + graphsDirSDRS+filename);
				//	}
    					sdrsIncr++;
    					lastProcessedFormula = formula;
    					log("finished processing formula");
    				}
    				else {
    					log("caches not yet integrated into the formula");
    				}
    			}
    		}
    		else {
    			log ("no SDRS structure to process");
    		}
    	}
    	catch (SubarchitectureComponentException e) {
    		e.printStackTrace();
    	} // end try..catch

    }
    


    private void processNucleus(WorkingMemoryChange _wmc) {

    	try {
    		// get the id of the working memory entry
    		String id = _wmc.address.id;
    		// get the data from working memory and store it
    		// with its id
    		CASTData plfWM = getWorkingMemoryEntry(id);
    		Nucleus nucleus = (Nucleus) plfWM.getData();
    		log("Nucleus retrieved from the working memory");
    		if (lastProcessedFormula !=null && !lastProcessedFormula.label.equals("start")) {
    			sdrsIncr--;
    			String filename = "sdrs-" + sdrsIncr;

    			SDRSFormula dm = SDRSUtils.getFormula(lastReceivedSDRS, lastProcessedFormula.tprec);

    			if (dm != null &&
    					SDRSUtils.getFormulaType(dm).type.equals(SDRSUtils.RELATION_TYPE)) {							
    				EventStructureUtils.nucleusAndFormulaAndDMToGraph
    				(nucleus, lastProcessedFormula, dm, graphsDirSDRS+filename, generatePNG);
    				log("Generation of the SDRS formula + last dialogue move + Nucleus successful, file written in " + graphsDirSDRS+filename);
    			}
    			else {
    				EventStructureUtils.nucleusAndFormulaToGraph(nucleus, lastProcessedFormula, graphsDirSDRS+filename, generatePNG);
    				log("Generation of the SDRS formula + Nucleus successful, file written in " + graphsDirSDRS+filename);
    			}
    			sdrsIncr++;
    		}
    	}
    	catch (SubarchitectureComponentException e) {
    		e.printStackTrace();
    	} // end try..catch

    }

    /**
		The method <i>executeProcessingTask</i> is the method called when a PackedLF processing task 
		has been accepted. For this class, it takes the PackedLF, and visualizes it. 
		
		@param  pd The PackedLF data to be processed
		@throws ComsysException Thrown if something goes wrong 
	*/ 
	@Override
	public void executeProcessingTask (ProcessingData pd) 
		throws ComsysException 
	{  
        Vector pdTypes = pd.getTypes();
		if (pdTypes.contains(CASTUtils.typeName(PackedLFs.class))) { 
			CASTData data = pd
                .getByType(CASTUtils.typeName(PackedLFs.class)); 
            String dataType = data.getType();
            log("Execute parse task on data item [" + dataType + "]"); 
            if (data != null) {
				// get the packed logical forms
				PackedLFs plfs = (PackedLFs) data.getData();
				// get the id, and check whether we've already generated graphs
				String id = plfs.id; 
				id = id.substring(0,id.indexOf(":"));
				String filename = "packedLF-"+id;
				log("going to generate DOT file for " + filename);
				log("incrementalSeries: " + incrementalSeries);
				if (incrementalSeries) {
					int step = 0;
					if (plfIds.containsKey(id)) { 
						step = ((Integer) plfIds.get(id)).intValue();
						step++;
					} // end if..check whether to update step
					plfIds.put(id,new Integer(step));
					filename = filename+"-"+step;	
					LFUtils.plfToGraph(plfs.packedLF,graphsDir+"/"+filename, generatePNG);
					gui.add(filename,plfs.phonStringLFPairs[0].phonStr.wordSequence);
				} // check whether incremental series should be produced

			} else { 
                throw new ComsysException("Error: processing task on empty data for type ["+dataType+"]");			
			} // end if..else check for non-empty data
      		
		} else { 
			throw new ComsysException("Error: processing PackedLF task on wrong data type");
		} // end if..else check for PackedLF
	} // end execute task

public int getSDRSIncr() {
	return sdrsIncr;
}
} // end class
