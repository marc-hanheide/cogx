package org.cognitivesystems.comsys.general.testers;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Properties;
import java.util.Vector;
import java.util.Enumeration;

import org.cognitivesystems.comsys.components.UtteranceInterpretation;
import org.cognitivesystems.comsys.components.ParseSelection;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.DialogueMove;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
import org.cognitivesystems.comsys.processing.ActiveIncrCCGParser;
import org.cognitivesystems.comsys.processing.examplegeneration.GenerationUtils;
import org.cognitivesystems.comsys.processing.parseselection.Decoder;
import org.cognitivesystems.comsys.processing.parseselection.LearningUtils;
import org.cognitivesystems.comsys.processing.parseselection.ParameterVector;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.Feature;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LFNominal;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LFRelation;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalForm;
import org.cognitivesystems.repr.lf.utils.LFPacking;
import org.cognitivesystems.repr.lf.utils.LFUtils;

import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.COMPONENT_NUMBER_KEY;
import cast.cdl.OperationMode;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTException;
import cast.core.data.CASTData;

import org.cognitivesystems.comsys.util.XMLTestReader;
import org.cognitivesystems.comsys.data.testData.* ;
import org.cognitivesystems.comsys.data.ProcessingData;
import org.cognitivesystems.comsys.data.WordRecognitionLattice ;
import org.cognitivesystems.comsys.general.ComsysUtils;

public class ParseSelectionTester extends AbstractParseSelectionTester {

	/**
	 * @param _id
	 */
	public ParseSelectionTester(String _id) {
		super(_id);
	}

	public class ParseSelectionTest extends AbstractParseSelectionTest  {


		public ParseSelectionTest(TestData test) {
			super(test);
		}

		protected void startTest() {
			initParseSelectionTest(test.getASRInputs());
		}
		

		public void initParseSelectionTest(Vector<PhonString> inputs) {

			String id = newDataID();

			try {
				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(LogicalForm.class,  WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								log("Selected LF received");
								appendWERResultsToFile(ParseSelection.recogString);
								String correctparse = test.getCorrectParse();
								if (!correctparse.equals("")) {
									boolean comparison = compareResults(_wmc, correctparse);
									LogicalForm lf2 = LFUtils.convertFromString(correctparse);
									lf2 = GenerationUtils.removeDuplicateNominalsAndFeatures(lf2);
									String id = _wmc.m_address.m_id;
									try {
										CASTData lfWM = new CASTData(id, getWorkingMemoryEntry(id));
										LogicalForm lf1 = (LogicalForm) lfWM.getData();
										int partialmatch = partialMatch(lf2, lf1);
										int totalCount = countSubstructures(lf2);
										appendAccuracyResultsToFile(!correctparse.equals(""), true, comparison, ParseSelection.lastscore, 
												partialmatch, totalCount);
									}
									catch (Exception e) {e.printStackTrace();}

									testComplete(comparison);
								}
								else {
									testComplete(false);
								}
							}
						}); 
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			WordRecognitionLattice lattice = new WordRecognitionLattice(inputs, id);
			if (lattice.getMaximumLength() > 0) {
				log("Inserting the word lattice into the working memory");
				try {
					addToWorkingMemory(id, lattice);
				}
				catch(Exception e) {
					e.printStackTrace();
				}
			}
			else {
				appendWERResultsToFile("");
				testComplete(true);
			}

			sleepProcess(800);
		}
	}


	@Override
	public void configure(Properties _config) {

		if (_config.containsKey("--testfile")) {
			testFile = _config.getProperty("--testfile");
			XMLTestReader reader = new XMLTestReader();
			parseSelectionTests = reader.readTest(testFile);

		}
		try {
			for (int i = 0 ; parseSelectionTests != null && i < parseSelectionTests.size(); i++) {
				registerTest(""+(i+1), new ParseSelectionTest(parseSelectionTests.elementAt(i)));
				log("Test parsing"+(i+1)+" successfully registered");
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

	}


}

