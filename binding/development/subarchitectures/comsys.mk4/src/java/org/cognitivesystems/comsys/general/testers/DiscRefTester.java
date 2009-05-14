package org.cognitivesystems.comsys.general.testers;

import java.util.List;
import java.util.Properties;
import java.util.Vector;
import java.util.Enumeration;

import org.omg.CORBA.Any;

import org.cognitivesystems.comsys.data.testData.TestData;
import org.cognitivesystems.comsys.data.testData.DiscRefTestData;
import org.cognitivesystems.comsys.general.SDRSUtils;
import org.cognitivesystems.comsys.general.testers.PackingTester.PackingTest;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.SDRS;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.SDRSFormula;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Cache;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.DialogueMove;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.MoveType;
import org.cognitivesystems.comsys.processing.ActiveIncrCCGParser;
import org.cognitivesystems.comsys.util.XMLTestReader;
import org.cognitivesystems.repr.lf.utils.LFPacking;
import org.cognitivesystems.repr.lf.utils.LFUtils;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.*;
import org.cognitivesystems.repr.lf.autogen.LFPacking.*;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTException;
import cast.core.data.CASTData;
import cast.testing.AbstractTester;
//import cast.testing.AbstractTester.AbstractTest;

import opennlp.ccg.hylo.HyloHelper;
import opennlp.ccg.hylo.Nominal;
import opennlp.ccg.synsem.Category;
import opennlp.ccg.synsem.LF;
import opennlp.ccg.synsem.Sign;
import java.util.regex.Pattern;
import java.util.regex.Matcher;

public class DiscRefTester extends AbstractTester {

	public class DiscRefBindingTest extends AbstractTest {

		TestData testData ;
		
		Vector<String> utterances;
		
		String[] binding;
	
		public DiscRefBindingTest (TestData testData) {
			log("test DiscRefBindingTest");
			this.testData = testData;
		}
		
		protected void initDiscRefBindingTest(Vector<String >utterances, String[] binding) {

			this.utterances = utterances;
			this.binding = binding;
	
			try {
				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(SDRS.class,  WorkingMemoryOperation.OVERWRITE),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								addedDRB(_wmc);
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
		private void addedDRB(WorkingMemoryChange _wmc) {
			try {

				log("got added packedlf");
				// get the id of the working memory entry
				String id = _wmc.m_address.m_id;
				// get the data from working memory and store it
				// with its id
				CASTData sdrsWM = new CASTData(id,
						getWorkingMemoryEntry(id));
				SDRS sdrs = (SDRS) sdrsWM.getData();
				checkDRB(sdrs);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			} // end try.. catch
		} 

		private void checkDRB(SDRS sdrs) {

			SDRSFormula form = SDRSUtils.getLastFormula(sdrs);
			if (form.caches.length > 0 ) {
				Any[] Content1 = form.caches[0].content1;
				Any[] Content2 = form.caches[0].content2;
				for (int i=0; i < Content1.length; i++) {
					
					if (Content1[i].extract_string().length() >0 &&
							Content2[i].extract_string().length() > 0) {
					char firstLetter1 = Content1[i].extract_string().charAt(0);
					char firstLetter2 = Content2[i].extract_string().charAt(0);
					

						if (form.label.equals(binding[0]) && 
								firstLetter2 == new String(binding[2]).charAt(0)) { 

							if (firstLetter1 == new String(binding[1]).charAt(0)) {
								log ("binding in formula " + form.label + " for nominal " + 
										Content2[i].extract_string() + " correctled bound to nominal " + 
										Content1[i].extract_string());
								testComplete(true);
							}

							else {
								log ("WARNING: binding in formula " + form.label + " for nominal " + 
										Content2[i].extract_string() + " is bound to nominal " + 
										Content1[i].extract_string() + ", and is should be bound to " +
												"a nominal starting with the letter " + binding[1]) ;
								testComplete(false);
							}
						}
					}
					else {
						log("Problem: content of cache " +form.caches[0].CacheId + "appears to be empty");
					}
				}
				
			}
		}
		

		@Override
		protected void startTest() {
			if (testData != null ) {
				DiscRefTestData testData2 = (DiscRefTestData) testData;
				Vector<String> utterances = testData2.getDiscourse();
				
				DiscRefTestData.Reference reference = testData2.getReference();
				String refPosition = "pi"+reference.getRefPosition();
				String refExpression = new Character(reference.getRefExpression().charAt(0)).toString();
				String referent = new Character(reference.getReferent().charAt(0)).toString();
				String[] binding = new String[] {refPosition, referent, refExpression};
				log(utterances.toString());
				log(binding[0]);
				log(binding[1]);
				log(binding[2]);
				initDiscRefBindingTest(utterances, binding);
			}
		}
	}
	
	/**
	public class DiscRefBindingTest1 extends DiscRefBindingTest {
		@Override
		protected void startTest() {
			Vector<String> utterances = new Vector<String>();
			utterances.add("what is it");
			String[] b = new String[] {"pi0", "i", "i" } ;
			initDiscRefBindingTest(utterances, b);
		}
	}
	
	public class DiscRefBindingTest2 extends DiscRefBindingTest {
		@Override
		protected void startTest() {
			Vector<String> utterances = new Vector<String>();
			utterances.add("here is the mug");
			utterances.add("it is green");
			String[] b = new String[] {"pi1", "m", "i" } ;
			
			initDiscRefBindingTest(utterances, b);
		}
	}

	public class DiscRefBindingTest3 extends DiscRefBindingTest {
		@Override
		protected void startTest() {
			Vector<String> utterances = new Vector<String>();
			utterances.add("here is the mug");
			utterances.add("it is green");
			utterances.add("and there is a table");
			utterances.add("it is big and white");
			
			String[] b = new String[] {"pi3", "t", "i" } ;
			
			initDiscRefBindingTest(utterances, b);
		}
	}
	
	public class DiscRefBindingTest4 extends DiscRefBindingTest {
		@Override
		protected void startTest() {
			Vector<String> utterances = new Vector<String>();
			utterances.add("here is the mug");
			utterances.add("it is green");
			utterances.add("right it is also round");
			
			String[] b = new String[] {"pi2", "m", "i" } ;
			
			initDiscRefBindingTest(utterances, b);
		}
	}
	

	public class DiscRefBindingTest5 extends DiscRefBindingTest {
		@Override
		protected void startTest() {
			Vector<String> utterances = new Vector<String>();
			utterances.add("here is the mug");
			utterances.add("yes that is right");
			utterances.add("it is also round");
			
			String[] b = new String[] {"pi2", "m", "i" } ;
			
			initDiscRefBindingTest(utterances, b);
		}
	}

	*/

	String discRefTestsFile = "";

	Vector<TestData> drTests;
	
	/**
	 * @param _id
	 */
	public DiscRefTester(String _id) {
		super(_id);
	}
	
	
	@Override
	public void configure(Properties _config) {
		if (_config.containsKey("--testfile")) {
			discRefTestsFile = _config.getProperty("--testfile");
			XMLTestReader reader = new XMLTestReader();
			drTests = reader.readTest(discRefTestsFile);
			
		}
		try {
			for (int i = 0 ; drTests != null && i < drTests.size(); i++) {
				registerTest(""+(i+1), new DiscRefBindingTest(drTests.elementAt(i)));
				log("Test disrefbinding"+(i+1)+" successfully registered");
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

	}
}