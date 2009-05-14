package org.cognitivesystems.comsys.general.testers;

import java.util.List;
import java.util.Properties;
import java.util.Vector;

import opennlp.ccg.hylo.HyloHelper;
import opennlp.ccg.hylo.Nominal;
import opennlp.ccg.synsem.Category;
import opennlp.ccg.synsem.LF;
import opennlp.ccg.synsem.Sign;

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


public class DialMoveTester extends AbstractComsysTester {

	
	public class DialMoveTest extends AbstractTest {

		DMTestData testData ;
		
		public DialMoveTest(TestData testData) {
			this.testData = (DMTestData) testData;
		}

		MoveType moveType;
		
		protected void initDMTest(String utterance, MoveType moveType) {

			this.moveType = moveType;
			
			try {
				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(DialogueMove.class,  WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								addedDM(_wmc);
							}
						}); 

				log("Filter OK");

					PhonString phonString = new PhonString();
					phonString.wordSequence = utterance;
					String id = newDataID();
					phonString.id = id;
					log("Inserting the phonstring 1 into the working memory");
					addToWorkingMemory(id, phonString);

					sleepProcess(800);
					
					if (testData.getMoveType() == null) {
						sleepProcess(5000);
						testComplete(true);
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
		private void addedDM(WorkingMemoryChange _wmc) {
			try {

				log("got added dialogue move");
				// get the id of the working memory entry
				String id = _wmc.m_address.m_id;
				// get the data from working memory and store it
				// with its id
				CASTData dmWM = new CASTData(id,
						getWorkingMemoryEntry(id));
				DialogueMove dm = (DialogueMove) dmWM.getData();
				checkDM(dm);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			} // end try.. catch
		} 

		private void checkDM(DialogueMove dm) {
			boolean result = ((moveType.value()) == dm.mType.value());
			testComplete(result);
		}
		
		@Override
		protected void startTest() {
			if (testData != null && testData.getString() != "") {
				initDMTest(testData.getString(), testData.getMoveType());
			}
		}
	}
	
	
	String dmTestsFile = "";

	Vector<TestData> dmTests;
	
	/**
	 * @param _id
	 */
	public DialMoveTester(String _id) {
		super(_id);
	}
	
	
	@Override
	public void configure(Properties _config) {
		
		if (_config.containsKey("--testfile")) {
			dmTestsFile = _config.getProperty("--testfile");
			XMLTestReader reader = new XMLTestReader();
			dmTests = reader.readTest(dmTestsFile);
			
		}
		try {
			for (int i = 0 ; dmTests != null && i < dmTests.size(); i++) {
				registerTest(""+(i+1), new DialMoveTest(dmTests.elementAt(i)));
				log("Test DM"+(i+1)+" successfully registered");
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

	}

}
