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

import BindingData.BindingUnion;
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


public class ProxyFactoriesTester extends AbstractComsysTester {


	public class ProxyFactoriesTest extends AbstractComsysTest {

		ProxyFactoriesTestData testData ;

		public ProxyFactoriesTest(TestData testData) {
			this.testData = (ProxyFactoriesTestData) testData;
			this.nbrUnions = this.testData.getNbrUnions();
		}

		int nbrUnions;


		@Override
		public void checkPLF(PackedLFs plf) {
			log("CHECK PLF!!!!!!");
			if (plf.finalized == ActiveIncrCCGParser.FINAL_PARSE) {
				// HENRIK + GJ -- HACK!!! 
				// need to check here whether binding is stable -- if to check for binding ... 
				sleepProcess(6000); 

				testComplete(verifyUnions());

			}
		}

		protected boolean verifyUnions() {
			try {
				CASTData<?>[] sourceIds2 = getWorkingMemoryEntries("binding.sa", BindingUnion.class, 0);

				log("UNION length: " + sourceIds2.length);
				
				return (sourceIds2.length == nbrUnions);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			return false;
		}



		@Override
		protected void startTest() {
			if (testData != null && testData.getString() != "") {
				initParseTest(testData.getString(), new Vector<String>());
			}
		}
	}


	String pTestsFile = "";

	Vector<TestData> pTests;

	/**
	 * @param _id
	 */
	public ProxyFactoriesTester(String _id) {
		super(_id);
	}


	@Override
	public void configure(Properties _config) {

		if (_config.containsKey("--testfile")) {
			pTestsFile = _config.getProperty("--testfile");
			XMLTestReader reader = new XMLTestReader();
			pTests = reader.readTest(pTestsFile);

		}
		try {
			for (int i = 0 ; pTests != null && i < pTests.size(); i++) {
				registerTest(""+(i+1), new ProxyFactoriesTest(pTests.elementAt(i)));
				log("Test DM"+(i+1)+" successfully registered");
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

	}

}
