package org.cognitivesystems.comsys.general.testers;

import java.util.Properties;
import java.util.Vector;


import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.core.CASTException;


import org.cognitivesystems.comsys.util.XMLTestReader;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.SpokenOutputItem;
import org.cognitivesystems.comsys.data.testData.*;
import org.cognitivesystems.comsys.general.ComsysUtils;


public class TTSTester extends AbstractComsysTester {

	
	public class TTSTest extends AbstractTest {

		TestData testData ;
		
		public TTSTest(TestData testData) {
			this.testData = (TestData) testData;
		}

		
		protected void initTTSTest(String utterance) {
	        try {
	            SpokenOutputItem spoi = ComsysUtils.newSpokenOutputItem();
	            spoi.phonString = utterance;
		    addToWorkingMemory(newDataID(), spoi); // refactored
	        }
	        catch (SubarchitectureProcessException e) {
	            e.printStackTrace();
	        }
		}

		
		@Override
		protected void startTest() {
			if (testData != null && testData.getString() != "") {

//q				log("sleeping for 10 seconds, waiting for the Mary server to start");
//				sleepProcess(10000);
				initTTSTest(testData.getString());
				sleepProcess(10000);
				log("Killing the mary server");
				try {
					String[] cmd = { "/bin/sh", "-c", "pkill -f Mary" };
					Process p = Runtime.getRuntime().exec(cmd);
					}
					catch(Exception e){
						//process exception
					}
				testComplete(true);
			}
		}
	}
	
	
	String ttsTestsFile = "";

	Vector<TestData> ttsTests;
	
	/**
	 * @param _id
	 */
	public TTSTester(String _id) {
		super(_id);
	}
	
	
	@Override
	public void configure(Properties _config) {
		
		if (_config.containsKey("--testfile")) {
			ttsTestsFile = _config.getProperty("--testfile");
			XMLTestReader reader = new XMLTestReader();
			ttsTests = reader.readTest(ttsTestsFile);
			
		}
		try {
			for (int i = 0 ; ttsTests != null && i < ttsTests.size(); i++) {
				registerTest(""+(i+1), new TTSTest(ttsTests.elementAt(i)));
				log("Test TTS"+(i+1)+" successfully registered");
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}
		
		super.configure(_config);

	}

}
