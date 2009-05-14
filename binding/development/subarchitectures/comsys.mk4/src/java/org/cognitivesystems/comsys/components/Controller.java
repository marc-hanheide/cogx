package org.cognitivesystems.comsys.components;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Properties;
import java.util.StringTokenizer;
import java.util.Vector;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.DialogueMove;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.SDRS;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.SDRSFormula;
import org.cognitivesystems.comsys.data.ProcessingData;
import org.cognitivesystems.comsys.data.SelectedLogicalForm;
import org.cognitivesystems.comsys.general.ComsysException;
import org.cognitivesystems.comsys.general.SDRSUtils;
import org.cognitivesystems.comsys.ontology.ComsysGoals;
import org.cognitivesystems.comsys.processing.ActiveIncrCCGParser;
import org.cognitivesystems.comsys.processing.dialoguemovefactories.Accept;
import org.cognitivesystems.comsys.processing.dialoguemovefactories.ActionDirective;
import org.cognitivesystems.comsys.processing.dialoguemovefactories.Assert;
import org.cognitivesystems.comsys.processing.dialoguemovefactories.Closing;
import org.cognitivesystems.comsys.processing.dialoguemovefactories.DialogueMoveFactory;
import org.cognitivesystems.comsys.processing.dialoguemovefactories.Opening;
import org.cognitivesystems.comsys.processing.dialoguemovefactories.QuestionW;
import org.cognitivesystems.comsys.processing.dialoguemovefactories.QuestionYN;
import org.cognitivesystems.comsys.processing.dialoguemovefactories.Reject;
import org.cognitivesystems.comsys.processing.examplegeneration.GenerationUtils;
import org.cognitivesystems.comsys.util.datastructs.AbstractFeatureValue;
import org.cognitivesystems.comsys.util.datastructs.DecisionTree;
import org.cognitivesystems.comsys.util.datastructs.FeatureValue;
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackedNominal;
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackingEdge;
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackingNode;
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackingNodeTarget;
import org.cognitivesystems.repr.lf.utils.LFUtils;

import balt.core.processes.FrameworkProcess.ProcessStatus;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.abstr.WorkingMemoryReaderWriterProcess;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TaskOutcome;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;
import cast.core.data.CASTData;

public class Controller extends WorkingMemoryReaderWriterProcess {


	double b = 0.95913751465998276;
	double w1 = 0.34377507134713742;
	double w2 = 0.20935157250443187;
	double w3 = 0.3250502512893351;
	double w4 = -0.33126812802371686;
	double w5 = -0.41867508312287838;
	double w6 = -0.33782108711440839;
	
	double[] w = {w1, w2, w3, w4, w5, w6};
	
	 int min1 = 345;
	 int max1 = 1933;
	 int min2 = 1;
	 int max2 = 16;
	 int min3 = 52;
	 int max3 = 1624;
	 int min4 = -942;
	 int max4 = 188;
	 int min5 = 1;
	 int max5 = 16;
	 int min6 = 39;
	 int max6 = 1624;
	
	 int[] mins = {min1,min2,min3,min4,min5,min6};
	 int[] maxs = {max1,max2,max3,max4,max5,max6};
	
	/**
	 * The unary constructor
	 * 
	 * @param _id
	 */
	public Controller(String _id) {
		super(_id);
		init();
	} // constructor/1

	/**
	 * The method <i>init</i> initializes the global data structures,
	 * and sets the ontology for this component.
	 * 
	 * @see #configure
	 */

	private void init() {
		log("Initializing dialogue move interpretation component");
		// nah: making all the comsys queue changes... don't want to
		// miss a thing
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;
		m_bLogOutput = true;
	} // init

	

	// =================================================================
	// ACCESSOR METHODS
	// =================================================================


	/**		
		The start method registers ADD- and OVERWRITE listeners for PACKEDLFs and CACHEs. 
	*/ 

	@Override
	public void start() {
		super.start();
		try {
			// Change filters for caches
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(SelectedLogicalForm.class,  WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					verifySelectedLF (_wmc);
				}
			});
			addChangeFilter(
					ChangeFilterFactory.createLocalTypeFilter(SelectedLogicalForm.class,  WorkingMemoryOperation.OVERWRITE),
					new WorkingMemoryChangeReceiver() {
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					verifySelectedLF (_wmc);
				}
			});			
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		} // end try..catch
	} // end start
	
	
	// =================================================================
	// EXECUTION METHODS
	// =================================================================


	public void verifySelectedLF (WorkingMemoryChange _wmc) {
		try {
			// get the id of the working memory entry
			String id = _wmc.m_address.m_id;
			// get the data from working memory
			CASTData slfWM = new CASTData(id, getWorkingMemoryEntry(id));
			SelectedLogicalForm slf = (SelectedLogicalForm) slfWM.getData();
	
			verifySelectedLF(slf);
			
				// Check what kind of type we are dealing with, only deal with disc refs
		} catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		} // end try..catch
	}
	
	
	
	public void verifySelectedLF(SelectedLogicalForm slf) {
		
		appendResultsToFile(slf);
		
		double[] values = extractValues(slf);
		
		double finalScore = b;
		
		for (int i = 0; i < values.length; i++) {		
			finalScore += w[i] * ((values[i] - mins[i])/(maxs[i] - mins[i]));
		}
	
		log("Final score for clarification: " + finalScore);
		if (finalScore < 0.75) {
			log ("==> ASK FOR FULL REPEAT!");
		}
		else if (finalScore >= 0.75 && finalScore < 0.82) {
			log("==> ASK FOR LIGHT REPEAT");
		}
		else {
			log("==> INTERPRETATION IS OK, NO NEED TO CLARIFY");
		}
		
	}
	
	
	static String ScoreResultsFileName = "data/scores.txt";
	
	
	public double[] extractValues (SelectedLogicalForm slf) {
		
		double v1 = slf.score;
		double v2 = countWords(slf.phon.wordSequence);
		double v3 = (slf.score / countWords(slf.phon.wordSequence));
		double v4 = (slf.score - slf.scoreOfSecondBest);
		double v5 = slf.lf.noms.length;
		double v6 = (slf.score / slf.lf.noms.length);
		
		double[] values = {v1,v2,v3,v4,v5,v6};	
		return values;
	}
	
	
	public void appendResultsToFile (SelectedLogicalForm slf) {
		
		String text = "";
		text += slf.score + ", ";

		text += countWords(slf.phon.wordSequence) + ", ";
		text += (slf.score / countWords(slf.phon.wordSequence)) + ", ";
			
		text += (slf.score - slf.scoreOfSecondBest) + ", ";
		
		text +=  slf.lf.noms.length + ", ";
		text +=  (slf.score / slf.lf.noms.length) + ", ";
		
		if (slf.exactMath)
			text += "ok" + ", ";
		else
			text += "ko" + ", ";
		
		text += slf.partialMatch + "\n";
		
		GenerationUtils.appendToFile(text, ScoreResultsFileName);
		
	}
	 
	
	public static double countWords(String str) {
		StringTokenizer tokenizer = new StringTokenizer(str);
		double nbWords = 0.0;
		while (tokenizer.hasMoreTokens()) {
			tokenizer.nextToken();
			nbWords++;
		}
		return nbWords;
	}

	// =================================================================
	// RUN METHODS
	// =================================================================

	// =================================================================
	// RUN METHODS
	// =================================================================

	/**
	 * The method <i>runComponent</i> cycles over the queue with
	 * processing data objects (for which tasks have been proposed),
	 * checking whether tasks can be executed. If there is a processing
	 * data object on the queue, the appropriate task is executed, and
	 * the task manager is informed of task completion (including
	 * success / failure).
	 * 
	 * @see cast.core.components.CASTComponent#runComponent()
	 */
	@Override
	public void runComponent() {
		try {
			log("Entering loop checking for data in dialogue move interpretation component");
			while (m_status == ProcessStatus.RUN) {
				// lock from external access
				lockProcess();
				// check (synchronised) processing data objects queue
				
				// Free the process
				unlockProcess();
				
                sleepProcess(20);

			} // end while running
		}
		catch (Exception e) {
			e.printStackTrace();
		} // end try..catch
	} // end runComponent

	
	
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
	@Override
	public void configure(Properties _config) {
//		_config.list(System.out);
		super.configure(_config);
		if (_config.containsKey("--log")) {
			String logFlag = _config.getProperty("--log");
			if (logFlag.equals("") | logFlag.equals("true")) {
				m_bLogOutput = true;
			}
			else {
				m_bLogOutput = false;
			} // end if..else
		} // end if..else check for command-line arguments
		
	} // end configure


} // end class

