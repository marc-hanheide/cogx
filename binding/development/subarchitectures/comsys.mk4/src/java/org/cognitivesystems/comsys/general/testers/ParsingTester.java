package org.cognitivesystems.comsys.general.testers;

import java.util.Properties;
import java.util.Vector;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.processing.ActiveIncrCCGParser;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalForm;
import org.cognitivesystems.repr.lf.utils.LFPacking;
import org.cognitivesystems.repr.lf.utils.LFUtils;

import cast.core.CASTException;

import org.cognitivesystems.comsys.util.XMLTestReader;
import org.cognitivesystems.comsys.data.testData.* ;


public class ParsingTester extends AbstractComsysTester {


	String parseTestsFile = "";

	Vector<TestData> parseTests;
	
	/**
	 * @param _id
	 */
	public ParsingTester(String _id) {
		super(_id);
	}
	
	public class ParsingTest extends AbstractComsysTest  {

		TestData parseTest ;
		
		public ParsingTest(TestData parseTest) {
			this.parseTest = parseTest;
		}
		
		protected void startTest() {
			
			ParseTestData parseTest2 = (ParseTestData) parseTest;
			initParseTest(parseTest2.getString(), parseTest2.getParses());
		}
		
		@Override
		public void checkPLF(PackedLFs plf) {
			
			if (parses.size() == 0) {
				testComplete(false);
			}

			if (plf.finalized == ActiveIncrCCGParser.FINAL_PARSE) {
				LFPacking packingTool = new LFPacking();
				LogicalForm[] lfs = packingTool.unpackPackedLogicalForm(plf.packedLF);
			//	LFUtils.plfToGraph(plf.packedLF, "TEST1");

				LogicalForm[] lfs2 = new LogicalForm[parses.size()];
				
				for (int i= 0 ; i < parses.size() ; i++) {
				//	log(parses.elementAt(i));
					lfs2[i] = LFUtils.convertFromString(parses.elementAt(i));
				}
				
				boolean allParsesCorrect = LFUtils.compareLFSets (lfs, lfs2);

				if (!allParsesCorrect) {
					log("Test failed: the generated parses differs from the given ones");
					log("Given parses: ");
					for (int i= 0 ; i < lfs2.length ; i++) {
						log("Given parse " + (i+1) + ": ");
						log(LFUtils.lfToString(lfs2[i]));					
					}
					log("Generated parses: ");
					for (int i= 0 ; i < lfs.length ; i++) {
						log("Generated parse " + (i+1) + ": ");
						log(LFUtils.lfToString(lfs[i]));					
					}
				}
				testComplete(allParsesCorrect);

			}
		}

	}

	
	@Override
	public void configure(Properties _config) {
		
		if (_config.containsKey("--testfile")) {
			parseTestsFile = _config.getProperty("--testfile");
			XMLTestReader reader = new XMLTestReader();
			parseTests = reader.readTest(parseTestsFile);
			
		}
		try {
			for (int i = 0 ; parseTests != null && i < parseTests.size(); i++) {
				registerTest(""+(i+1), new ParsingTest(parseTests.elementAt(i)));
				log("Test parsing"+(i+1)+" successfully registered");
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

	}

}