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

import org.cognitivesystems.comsys.autogen.ComsysEssentials.*;

import org.cognitivesystems.comsys.util.XMLTestReader;
import org.cognitivesystems.comsys.data.testData.*;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.MoveType;
import org.cognitivesystems.comsys.general.EventStructureUtils;

public class EventStructTester extends AbstractComsysTester {

	
	public class EventStructTest extends AbstractTest {

		EventStructureData testData ;
		
		public EventStructTest(TestData testData) {
			this.testData = (EventStructureData) testData;
		}

		
		protected void initESTest(String utterance) {
			
			try {
				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(Nucleus.class,  WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								addedNucleus(_wmc);
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
					
					if (testData.getEventType().equals("") && 
							testData.getStateType().equals("")) {
						sleepProcess(10000);
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
		private void addedNucleus(WorkingMemoryChange _wmc) {
			try {

				log("got added nucleus");
				// get the id of the working memory entry
				String id = _wmc.m_address.m_id;
				// get the data from working memory and store it
				// with its id
				CASTData nucWM = new CASTData(id,
						getWorkingMemoryEntry(id));
				Nucleus nucleus = (Nucleus) nucWM.getData();
				checkNucleus(nucleus);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			} // end try.. catch
		} 

		private void checkNucleus(Nucleus nucleus) {
	
			boolean correctEventType = false;
			boolean correctStateType = false;
			for (int i=0; i < nucleus.events.length ; i++) {
				if (nucleus.events[i].type.equals(testData.getEventType())) {
					correctEventType = true;
					log("OK for the event type");
				}
			}
			for (int i=0; i < nucleus.states.length ; i++) {
				if (nucleus.states[i].type.equals(testData.getStateType())) {
					correctStateType = true;
					log("OK for the state type");
				}
			}
			if (correctEventType & correctStateType) {
				testComplete(true);
			}
			else {
				testComplete(false);
				EventStructureUtils.nucleusToGraph(nucleus, "nucleus", true);
			}
		}
		
		@Override
		protected void startTest() {
			if (testData != null && testData.getString() != "") {
				initESTest(testData.getString());
			}
		}
	}
	
	
	String esTestsFile = "";

	Vector<TestData> esTests;
	
	/**
	 * @param _id
	 */
	public EventStructTester(String _id) {
		super(_id);
	}
	
	
	@Override
	public void configure(Properties _config) {
		
		if (_config.containsKey("--testfile")) {
			esTestsFile = _config.getProperty("--testfile");
			XMLTestReader reader = new XMLTestReader();
			esTests = reader.readTest(esTestsFile);
			
		}
		try {
			for (int i = 0 ; esTests != null && i < esTests.size(); i++) {
				registerTest(""+(i+1), new EventStructTest(esTests.elementAt(i)));
				log("Test eventstruct"+(i+1)+" successfully registered");
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

	}

}
