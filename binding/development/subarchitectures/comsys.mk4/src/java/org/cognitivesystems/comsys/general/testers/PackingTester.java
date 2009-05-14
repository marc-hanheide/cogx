package org.cognitivesystems.comsys.general.testers;

import java.util.List;
import java.util.Properties;
import java.util.Vector;

import opennlp.ccg.hylo.HyloHelper;
import opennlp.ccg.hylo.Nominal;
import opennlp.ccg.synsem.Category;
import opennlp.ccg.synsem.LF;
import opennlp.ccg.synsem.Sign;

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


public class PackingTester extends AbstractComsysTester {

	
	public class PackingTest extends AbstractTest {

		TestData testData ;
		
		public PackingTest(TestData testData) {
			this.testData = testData;
		}
		
		private LogicalForm[] getParse(String input) {

			LogicalForm[] LFs = null;

			try {

				if (input.length() > 2 ) {
					opennlp.ccg.grammar.Grammar grammar = new opennlp.ccg.grammar.Grammar("subarchitectures//comsys.mk4//grammars//openccg//moloko.v5//grammar.xml");
					opennlp.ccg.parse.Parser parser = new opennlp.ccg.parse.Parser(grammar);
					parser.parse(input);
					List<Sign> parses = parser.getResult();
					Sign[] results = new Sign[parses.size()];
					parses.toArray(results);
					int numToShow = parses.size();

					LFs = new LogicalForm[parses.size()];

					for (int i=0; i < numToShow; i++) {
						Category cat = results[i].getCategory();
						LF convertedLF = null;
						if (cat.getLF() != null) {
							cat = cat.copy();
							Nominal index = cat.getIndexNominal(); 
							convertedLF = HyloHelper.compactAndConvertNominals(cat.getLF(), index);
							LFs[i] = LFUtils.convertFromLF(convertedLF); 
							cat.setLF(null);
						}
					}
				}
			}
			catch (opennlp.ccg.parse.ParseException f) {}
			catch (Exception e) {
				e.printStackTrace();
			}

			return LFs;
		}


		protected void initPackingTest(String utt) {

			log("init packing test");
			
			LFPacking packingTool = new LFPacking();

			boolean allOperationsOK = true;

				LogicalForm[] LFs1 = getParse(utt);
				
				log("parses extracted, number of logical forms: " + LFs1.length);
				
				PackedLogicalForm PLF = packingTool.packLogicalForms(LFs1);
				
				log("packing operation complete");
				
				LogicalForm[] LFs2 = packingTool.unpackPackedLogicalForm(PLF);

				log("unpacking operation complete");
				
				boolean LFsEquivalence = LFUtils.compareLFSets(LFs1, LFs2);

				if (!LFsEquivalence) {
					log ("WARNING, the packing operation changed the content of the logical forms for the following utterance \"" + utt + "\"");
					LFUtils.plfToGraph(PLF, "packing", true);
					for (int i=0; i< LFs1.length ; i++) {
						log("lfinit"+i+ ": " + LFUtils.lfToString(LFs1[i]));
						LFUtils.lfToGraph(LFs1[i], "lfinit"+i, true);
					}
					for (int i=0; i< LFs2.length ; i++) {
						log("lfafter"+i+ ": " + LFUtils.lfToString(LFs1[i]));
						LFUtils.lfToGraph(LFs2[i], "lfafter"+i, true);
					}
					allOperationsOK = false;
					testComplete(false);
				}	
				else {
					log("packing of  \"" + utt + "\" successful");
				}

			if (allOperationsOK) {
				testComplete(true);
			}
		}
		
		@Override
		protected void startTest() {
			if (testData != null && testData.getString() != "") {
			initPackingTest(testData.getString());
			}
		}
	}
	
	
	String packTestsFile = "";

	Vector<TestData> packTests;
	
	/**
	 * @param _id
	 */
	public PackingTester(String _id) {
		super(_id);
	}
	
	
	@Override
	public void configure(Properties _config) {
		
		if (_config.containsKey("--testfile")) {
			packTestsFile = _config.getProperty("--testfile");
			XMLTestReader reader = new XMLTestReader();
			packTests = reader.readTest(packTestsFile);
			
		}
		try {
			for (int i = 0 ; packTests != null && i < packTests.size(); i++) {
				registerTest(""+(i+1), new PackingTest(packTests.elementAt(i)));
				log("Test packing"+(i+1)+" successfully registered");
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

	}

}
