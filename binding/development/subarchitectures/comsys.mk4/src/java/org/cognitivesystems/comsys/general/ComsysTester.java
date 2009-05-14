package org.cognitivesystems.comsys.general;

import java.util.List;
import java.util.Properties;
import java.util.Vector;
import java.util.Enumeration;

import org.omg.CORBA.Any;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.SDRS;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.SDRSFormula;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Cache;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.PackedLFs;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.DialogueMove;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.MoveType;
import org.cognitivesystems.comsys.processing.ActiveIncrCCGParser;
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

public class ComsysTester extends AbstractTester {


	abstract class ParsingTest extends AbstractTest {
		
		private String[] parses;
		
		protected void initParseTest (String utterance, String[] parses) {
		
			this.parses = parses;
			
			try {
				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.ADD),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								addedPackedLF(_wmc);
							}
						}); 

				addChangeFilter(
						ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.OVERWRITE),
						new WorkingMemoryChangeReceiver() {
							public void workingMemoryChanged(WorkingMemoryChange _wmc) {
								addedPackedLF(_wmc);
							}
						});
				log("Filters OK");

					PhonString phonString = new PhonString();
					phonString.wordSequence = utterance;
					String id = newDataID();
					phonString.id = id;
					log("Inserting the phonstring 1 into the working memory");
					addToWorkingMemory(id, phonString);

					sleepProcess(800);

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
		private void addedPackedLF(WorkingMemoryChange _wmc) {
			try {
				
				log("got added packedlf");
				// get the id of the working memory entry
				String id = _wmc.m_address.m_id;
				// get the data from working memory and store it
				// with its id
				CASTData plfWM = new CASTData(id,
						getWorkingMemoryEntry(id));
				PackedLFs plf = (PackedLFs) plfWM.getData();
				checkPLF(plf);
			}
			catch (SubarchitectureProcessException e) {
				e.printStackTrace();
				testComplete(false);
			} // end try.. catch
		} 
	
		public void checkPLF(PackedLFs plf) {

			
			if (plf.finalized == ActiveIncrCCGParser.FINAL_PARSE) {
				LFPacking packingTool = new LFPacking();
				LogicalForm[] lfs = packingTool.unpackPackedLogicalForm(plf.packedLF);
				//	 LFUtils.plfToGraph(plf.packedLF, "TEST1");
				
				LogicalForm[] lfs2 = new LogicalForm[parses.length];
				for (int i= 0 ; i < parses.length ; i++) {
					lfs2[i] = LFUtils.convertFromString(parses[i]);
				}

				boolean allParsesCorrect = LFUtils.compareLFSets (lfs, lfs2);
				
				
				// HENRIK + GJ -- HACK!!! 
				// need to check here whether binding is stable -- if to check for binding ... 
				sleepProcess(5000); 
				
				
				testComplete(allParsesCorrect);
				
				
				
			}
		}
		
	}
	

	class ParsingTest1 extends ParsingTest {
	
		protected void startTest() {
			String utterance ="now take the box";
			String[] parses = new String[2];
			parses[0] = "(@t1_1:action-motion(take ^ <Mood>imp ^ <Tense>pres ^  <Actor>(a1_1:animate ^ addressee) ^  <Patient>(b3_1:thing ^ box ^ <Delimitation>unique ^ <Number>sg ^ <Quantification>specific_singular) ^  <Time>(n0_1:m-time ^ now)))";		
			parses[1] = "(@n0_1:dis-connective(now ^  <Body>(t1_1:action-motion ^ take ^ <Mood>imp ^ <Tense>pres ^  <Actor>(a1_1:animate ^ addressee) ^  <Patient>(b3_1:thing ^ box ^ <Delimitation>unique ^ <Number>sg ^ <Quantification>specific_singular))))";
		
			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest2 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="here is the ball";
				String[] parses = new String[2];
				parses[0] = "(@b1_4:ascription(be ^ <Mood>ind ^ <Tense>pres ^  <Restr>(h0_2:deictic-pronoun ^ here ^ <Proximity>proximal) ^  <Scope>(b3_8:thing ^ ball ^ <Delimitation>unique ^ <Number>sg ^ <Quantification>specific_singular)))";
				parses[1] = "(@b1_2:ascription(be ^ <Mood>ind ^ <Tense>pres ^  <Restr>(b3_2:thing ^ ball ^ <Delimitation>unique ^ <Number>sg ^ <Quantification>specific_singular) ^  <Scope>(r1_2:entity ^ restricted-entity ^  <Location>(h0_2:m-location ^ here))))";
		
			initParseTest(utterance, parses);
		}
	}
	
	
	class ParsingTest3 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="ok that is right";
			String[] parses = new String[4];
			parses[0] = "(@o0_3:dis-connective(ok ^  <Body>(b2_5:ascription ^ be ^ <Mood>ind ^ <Tense>pres ^  <Restr>(t1_5:deictic-pronoun ^ that ^  <VisCtxt>(e1_5:entity ^ entity ^ <Delimitation>unique ^ <Number>sg ^ <Proximity>distal ^ <Quantification>specific_singular)) ^  <Scope>(r1_5:entity ^ restricted-entity ^  <Property>(r3_5:q-attitude ^ right)))))";
			parses[1] = "(@o0_3:dis-connective(ok ^  <Body>(b2_5:ascription ^ be ^ <Mood>ind ^ <Tense>pres ^  <Restr>(t1_5:deictic-pronoun ^ that ^  <VisCtxt>(e1_5:entity ^ entity ^ <Delimitation>unique ^ <Number>sg ^ <Proximity>distal ^ <Quantification>specific_singular)) ^  <Scope>(r1_5:entity ^ restricted-entity ^  <Property>(r3_1:q-location ^ right)))))";
			parses[2] = "(@y0_3:cue(yes ^ <Polarity>+ ^  <Scope>(b2_7:ascription ^ be ^ <Mood>ind ^ <Tense>pres ^  <Restr>(t1_5:deictic-pronoun ^ that ^  <VisCtxt>(e1_7:entity ^ entity ^ <Delimitation>unique ^ <Number>sg ^ <Proximity>distal ^ <Quantification>specific_singular)) ^  <Scope>(r1_7:entity ^ restricted-entity ^  <Property>(r3_7:q-location ^ right)))))";
			parses[3] = "(@y0_3:cue(yes ^ <Polarity>+ ^  <Scope>(b2_7:ascription ^ be ^ <Mood>ind ^ <Tense>pres ^  <Restr>(t1_5:deictic-pronoun ^ that ^  <VisCtxt>(e1_7:entity ^ entity ^ <Delimitation>unique ^ <Number>sg ^ <Proximity>distal ^ <Quantification>specific_singular)) ^  <Scope>(r1_7:entity ^ restricted-entity ^  <Property>(r3_1:q-attitude ^ right)))))";
		
			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest4 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="go to the hallway";
			String[] parses = new String[1];
			parses[0] = "@g1_0:action-motion(go ^ <Mood>imp ^  <Tense>pres ^  <Actor>(a1_0:animate ^ addressee) ^  <Dynamic>(t1_0:m-whereto ^ to ^  <Arg>(h1_0:e-location ^ hall ^  <Delimitation>unique ^  <Number>sg ^  <Quantification>specific_singular)))";
		
			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest5 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="robot please stop";
			String[] parses = new String[2];
			parses[0] = "@s1_0:action-non-motion(stop ^  <Actor>(r1_0:animate ^ Robot) ^  <Modifier>(p1_0:m-comment ^ please))";
			parses[1] = "@s1_0:action-non-motion(stop ^ <Mood>imp ^ <Tense>pres ^ <Actor>(r1_0:animate ^ Robot) ^ <Modifier>(p1_0:m-comment ^ please))";
		
			initParseTest(utterance, parses);
		}
	}
	
	
	
	class ParsingTest6 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the red triangle";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ triangle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^  <Modifier>(r1_0:q-color ^ red)))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ triangle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ red)) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(triangle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ red))";

			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest7 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the blue circle";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ circle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^  <Modifier>(r1_0:q-color ^ blue)))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ circle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ blue)) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(circle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ blue))";

			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest8 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the green square";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ square ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^  <Modifier>(r1_0:q-color ^ green)))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ square ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ green)) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(square ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ green))";

			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest9 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the yellow star";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ star ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^  <Modifier>(r1_0:q-color ^ yellow)))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ star ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ yellow)) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(star ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ yellow))";

			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest10 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the red thing";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^  <Modifier>(r1_0:q-color ^ red)))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ red)) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ red))";

			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest11 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the blue thing";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^  <Modifier>(r1_0:q-color ^ blue)))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ blue)) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ blue))";

			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest12 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the green thing";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^  <Modifier>(r1_0:q-color ^ green)))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ green)) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ green))";

			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest13 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the yellow thing";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^  <Modifier>(r1_0:q-color ^ yellow)))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ yellow)) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(thing ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(r1_0:q-color ^ yellow))";

			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest14 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the triangle";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ triangle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ triangle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(triangle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific)";

			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest15 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the circle";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ circle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ circle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(circle ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific)";

			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest16 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the square";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ square ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ square ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(square ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific)";

			initParseTest(utterance, parses);
		}
	}
	
	class ParsingTest17 extends ParsingTest {
		
		protected void startTest() {
			String utterance ="the star";
			String[] parses = new String[3];
			parses[0] = "@x1_0:event(<Mood>ind ^ <Subject>(t1_0:thing ^ star ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific))";
			parses[1] =  "@x1_0:event(<Mood>ind ^ <Fronted>(t1_0:thing ^ star ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific) ^ <Subject>x22_0)";
			parses[2] =  "@t1_0:thing(star ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific)";

			initParseTest(utterance, parses);
		}
	}
	
	
	

	abstract public class PackingTest extends AbstractTest {


		private LogicalForm[] getParse(String input) {

			LogicalForm[] LFs = null;

			try {

				if (input.length() > 2 ) {
					opennlp.ccg.grammar.Grammar grammar = new opennlp.ccg.grammar.Grammar("subarchitectures//comsys.mk4//grammars//openccg//moloko.v4.1//grammar.xml");
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


			LFPacking packingTool = new LFPacking();

			boolean allOperationsOK = true;

				LogicalForm[] LFs1 = getParse(utt);
				PackedLogicalForm PLF = packingTool.packLogicalForms(LFs1);
				LogicalForm[] LFs2 = packingTool.unpackPackedLogicalForm(PLF);

				boolean LFsEquivalence = true;

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
	}
	
	
	public class PackingTest1 extends PackingTest {
		
		@Override
		protected void startTest() {
			String utterance = "put the ball to the left of the box";
			initPackingTest(utterance);
		}
	}
	
	public class PackingTest2 extends PackingTest {
		
		@Override
		protected void startTest() {
			String utterance = "put the ball to the left of the mug onto the box";
			initPackingTest(utterance);
		}
	}
	
	public class PackingTest3 extends PackingTest {
		
		@Override
		protected void startTest() {
			String utterance = "there is a mug on the table";
			initPackingTest(utterance);
		}
	}
	
	public class PackingTest4 extends PackingTest {
		
		@Override
		protected void startTest() {
			String utterance = "take the box and put it here";
			initPackingTest(utterance);
		}
	}
	
	public class PackingTest5 extends PackingTest {
		
		@Override
		protected void startTest() {
			String utterance = "look at the table then take the green mug on it and give it to me";
			initPackingTest(utterance);
		}
	}



	abstract class DialogueMovesTest extends AbstractTest  {

		Integer moveType;
		
		protected void initDMTest(String utterance, Integer moveType) {

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
			boolean result = ((moveType) == dm.mType.value());
			testComplete(result);
		}
	}
	
	
	
public class DMTest1 extends DialogueMovesTest {
		
		@Override
		protected void startTest() {
			String utterance = "hi robot";
			Integer moveType = new Integer(MoveType._OPENING);
			initDMTest(utterance, moveType);
		}
	}

public class DMTest2 extends DialogueMovesTest {
	
	@Override
	protected void startTest() {
		String utterance = "now take the box";
		Integer moveType = new Integer(MoveType._ACTION_DIRECTIVE);
		initDMTest(utterance, moveType);
	}
}

public class DMTest3 extends DialogueMovesTest {
	
	@Override
	protected void startTest() {
		String utterance = "no this is a mug";
		Integer moveType = new Integer(MoveType._REJECT);
		initDMTest(utterance, moveType);
	}
}

public class DMTest4 extends DialogueMovesTest {
	
	@Override
	protected void startTest() {
		String utterance = "where is the box now";
		Integer moveType = new Integer(MoveType._QUESTION_W);
		initDMTest(utterance, moveType);
	}
}

public class DMTest5 extends DialogueMovesTest {
	
	@Override
	protected void startTest() {
		String utterance = "are you there";
		Integer moveType = new Integer(MoveType._QUESTION_YN);
		initDMTest(utterance, moveType);
	}
}


	abstract public class DiscRefBindingTest extends AbstractTest {


		Vector<String> utterances;
		
		String[] binding;
	
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
				
			}
		}
	}
	
	
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

	
	

	/**
	 * @param _id
	 */
	public ComsysTester(String _id) {
		super(_id);
	}

	@Override
	public void configure(Properties _config) {
		try {
			registerTest("parsing1", new ParsingTest1());
			registerTest("parsing2", new ParsingTest2());
			registerTest("parsing3", new ParsingTest3());
			registerTest("parsing4", new ParsingTest4());
			registerTest("parsing5", new ParsingTest5());
			registerTest("parsing6", new ParsingTest6());
			registerTest("parsing7", new ParsingTest7());
			registerTest("parsing8", new ParsingTest8());
			registerTest("parsing9", new ParsingTest9());
			registerTest("parsing10", new ParsingTest10());
			registerTest("parsing11", new ParsingTest11());
			registerTest("parsing12", new ParsingTest12());
			registerTest("parsing13", new ParsingTest13());
			registerTest("parsing14", new ParsingTest14());
			registerTest("parsing15", new ParsingTest15());
			registerTest("parsing16", new ParsingTest16());
			registerTest("parsing17", new ParsingTest17());
			
			registerTest("packing1", new PackingTest1());
			registerTest("packing2", new PackingTest2());
			registerTest("packing3", new PackingTest3());
			registerTest("packing4", new PackingTest4());
			registerTest("packing5", new PackingTest5());
			
			registerTest("dialoguemoves1", new DMTest1());
			registerTest("dialoguemoves2", new DMTest2());
			registerTest("dialoguemoves3", new DMTest3());
			registerTest("dialoguemoves4", new DMTest4());
			registerTest("dialoguemoves5", new DMTest5());
			
			registerTest("discrefbinding1", new DiscRefBindingTest1());
			registerTest("discrefbinding2", new DiscRefBindingTest2());
			registerTest("discrefbinding3", new DiscRefBindingTest3());
			registerTest("discrefbinding4", new DiscRefBindingTest4());
			registerTest("discrefbinding5", new DiscRefBindingTest5());
			
		} catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

	}
}
