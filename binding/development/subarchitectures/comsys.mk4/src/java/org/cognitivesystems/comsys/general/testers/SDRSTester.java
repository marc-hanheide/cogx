package org.cognitivesystems.comsys.general.testers;

import java.util.Enumeration;
import java.util.List;
import java.util.Properties;
import java.util.Vector;

import opennlp.ccg.hylo.HyloHelper;
import opennlp.ccg.hylo.Nominal;
import opennlp.ccg.synsem.Category;
import opennlp.ccg.synsem.LF;
import opennlp.ccg.synsem.Sign;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.SDRS;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.DialogueMove;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
import org.cognitivesystems.comsys.processing.ActiveIncrCCGParser;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalForm;
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackedLogicalForm;
import org.cognitivesystems.repr.lf.utils.LFPacking;
import org.cognitivesystems.repr.lf.utils.LFUtils;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTException;
import cast.core.data.CASTData;
import cast.testing.AbstractTester;

import org.cognitivesystems.comsys.util.XMLTestReader;
import org.cognitivesystems.comsys.data.testData.*;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.MoveType;


public class SDRSTester extends AbstractComsysTester {

	
	public class SDRSTest extends AbstractTest {

		SDRSTestData testData ;
		
		public SDRSTest(TestData testData) {
			this.testData =  (SDRSTestData) testData;
		}
		
		
		int nbrDialMoves = 0 ;
		
		protected void initSDRSTest(Vector<String >utterances) {
			
			log("Discourse size: " + utterances);
			
			try {
				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(SDRS.class,  WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								addedSDRS(_wmc);
							}
						}); 
				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(SDRS.class,  WorkingMemoryOperation.OVERWRITE),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								addedSDRS(_wmc);
							}
						}); 
				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(DialogueMove.class,  WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								nbrDialMoves++;
							}
						}); 

				log("Filter OK");

				for (Enumeration<String> e = utterances.elements() ; e.hasMoreElements() ; ) {

					PhonString phonString = new PhonString();
					phonString.wordSequence = e.nextElement();
					String id = newDataID();
					phonString.id = id;
					log("Inserting the phonstring 1 into the working memory");
					addToWorkingMemory(id, phonString);

					sleepProcess(800);
				}

			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			}
		}
		
		/**
		 * @param _wmc
		 * @param i
		 */
		private void addedSDRS(WorkingMemoryChange _wmc) {
			try {

				log("got updated SDRS");
				// get the id of the working memory entry
				String id = _wmc.m_address.m_id;
				// get the data from working memory and store it
				// with its id
				CASTData dmWM = new CASTData(id,
						getWorkingMemoryEntry(id));
				SDRS sdrs = (SDRS) dmWM.getData();
				if (checkSDRS(sdrs)) {
					testComplete(true);
				}
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			} // end try.. catch
		} 

		private boolean checkSDRS(SDRS sdrs) {
			log("SDRS size: " + sdrs.F.mapping.length);
			int total = (testData.getDiscourse().size() + nbrDialMoves);
			log("Total utterances + speech acts: " + total);
			return (total == sdrs.F.mapping.length);
		}
		
		@Override
		protected void startTest() {
			if (testData != null) {
				initSDRSTest(testData.getDiscourse());
			}
		}
	}
	
	
	String SDRSTestsFile = "";

	Vector<TestData> SDRSTests;
	
	/**
	 * @param _id
	 */
	public SDRSTester(String _id) {
		super(_id);
	}
	
	
	@Override
	public void configure(Properties _config) {
		
		if (_config.containsKey("--testfile")) {
			SDRSTestsFile = _config.getProperty("--testfile");
			XMLTestReader reader = new XMLTestReader();
			SDRSTests = reader.readTest(SDRSTestsFile);
			
		}
		try {
			for (int i = 0 ; SDRSTests != null && i < SDRSTests.size(); i++) {
				registerTest(""+(i+1), new SDRSTest(SDRSTests.elementAt(i)));
				log("Test DM"+(i+1)+" successfully registered");
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

	}

}
