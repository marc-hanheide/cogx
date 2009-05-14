package org.cognitivesystems.comsys.general.testers;

import java.io.File;
import java.net.URL;
import java.util.Iterator;
import java.util.List;
import java.util.Properties;
import java.util.Vector;
import java.util.Enumeration;

import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.core.CASTException;


import org.cognitivesystems.comsys.util.XMLTestReader;
import org.cognitivesystems.comsys.util.uttplan.UtterancePlanner;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.SpokenOutputItem;
import org.cognitivesystems.comsys.data.testData.*;
import org.cognitivesystems.comsys.general.ComsysUtils;
import org.cognitivesystems.repr.lf.autogen.LFEssentials.LogicalForm;
import org.cognitivesystems.repr.lf.utils.LFUtils;
import org.cognitivesystems.comsys.util.uttplan.UtterancePlanner;
import org.cognitivesystems.comsys.util.uttplan.UPReqHandler;
import org.cognitivesystems.comsys.util.UPDebugger;

import opennlp.ccg.realize.Edge;
import opennlp.ccg.realize.Realizer;
import opennlp.ccg.synsem.LF;
import opennlp.ccg.grammar.Grammar;


public class UttPlanningTester extends AbstractComsysTester {


	public class UttPlanningTest extends AbstractTest {

		PlanningTestData testData ;
		UtterancePlanner planner;
		UPDebugger up;
		String grammarfile = "subarchitectures/comsys.mk4/grammars/contentPlanning/grammar.xml";
		String ccgfile = "subarchitectures/comsys.mk4/grammars/openccg/moloko.v5/grammar.xml";
		Realizer realizer;

		public UttPlanningTest(TestData testData) {
			try {
				this.testData = (PlanningTestData) testData;
				up = new UPDebugger();
				URL grammarURL = new File(ccgfile).toURL();
			//	System.out.println("Loading a CCG grammar from URL: " + grammarURL);
				Grammar grammar = new Grammar(grammarURL);
				realizer = new Realizer(grammar);
				planner = new UtterancePlanner(grammarfile, up);
			//	planner.setLogging(false);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
		}

		protected void initUttPlanningTest(String input, Vector<String> realization) {
			int atop = input.indexOf("@");
			if (atop > -1) { 
				String lfstr = input.substring(atop,input.length());
				LogicalForm lf = LFUtils.convertFromString(lfstr);
				LogicalForm planlf = planner.plan(lf);
				log("Resulting logical form: " + LFUtils.lfToString(planlf));
				LogicalForm reduxLF = up.applyModelReduction(planlf);	
				LF lf2 = LFUtils.convertToLF(reduxLF);		
				realizer.realize(lf2);
				opennlp.ccg.realize.Chart chart = realizer.getChart();
				List bestEdges = chart.bestEdges();
				
				Vector<String> result = new Vector<String>();
				for (Iterator beIter = bestEdges.iterator(); beIter.hasNext(); ) {
					Edge edge = (Edge) beIter.next(); 
				//	log("Possible realization: " + edge.toString());
					String str = edge.toString();
					String[] split1 = str.split("] ");
					String[] split2 = split1[1].split(" :-");
					result.add(split2[0]);
				} 
				
				boolean isOK = true;
				for (Enumeration<String> e = realization.elements(); e.hasMoreElements();) {
					String real = e.nextElement();
					if (!result.contains(real)) {
						log("PROBLEM: \"" + real + "\" not realized");
						isOK = false;
						testComplete(false);
					}
					else {
						log("OK: \"" + real + "\" successfully realized");
					}
				}
				
				if (isOK)
					testComplete(true);
			}
		}

		@Override
		protected void startTest() {
			if (testData != null && testData.getInput() != "") {

//				q				log("sleeping for 10 seconds, waiting for the Mary server to start");
//				sleepProcess(10000);
				initUttPlanningTest(testData.getInput(), testData.getOutput());
			}
		}
	}


	String UttPlanningTestFile = "";

	Vector<TestData> UttPlanningTests;

	/**
	 * @param _id
	 */
	public UttPlanningTester(String _id) {
		super(_id);
	}


	@Override
	public void configure(Properties _config) {

		if (_config.containsKey("--testfile")) {
			UttPlanningTestFile = _config.getProperty("--testfile");
			XMLTestReader reader = new XMLTestReader();
			UttPlanningTests = reader.readTest(UttPlanningTestFile);

		}
		try {
			for (int i = 0 ; UttPlanningTests != null && i < UttPlanningTests.size(); i++) {
				registerTest(""+(i+1), new UttPlanningTest(UttPlanningTests.elementAt(i)));
				log("Test utterance planning"+(i+1)+" successfully registered");
			}
		} catch (CASTException e) {
			e.printStackTrace();
		}

		super.configure(_config);

	}

}
