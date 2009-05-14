package org.cognitivesystems.comsys.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;


import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
import org.cognitivesystems.comsys.data.testData.* ;
import org.cognitivesystems.comsys.processing.examplegeneration.GenerationUtils;

import java.util.Vector;
import java.util.StringTokenizer;

import com.sun.org.apache.xerces.internal.parsers.DOMParser;

public class XMLTestReader {


	Vector<TestData> tests;


	public XMLTestReader() {
		tests = new Vector<TestData>();
	}



	public Vector<TestData> readTest(String file) {

		DOMParser parser = new DOMParser();
		File f = new File(file);

		try {
			FileReader reader = new FileReader(f);

			InputSource source = new InputSource(reader);

			parser.parse(source);
			Document testDoc = parser.getDocument();

			NodeList childNodes = testDoc.getChildNodes().item(0).getChildNodes();

			String topCategory = testDoc.getChildNodes().item(0).getNodeName();


			if(topCategory.equals("utterancesToParse")) {

				for (int i = 0; i < childNodes.getLength(); i++) {
					Node uttNode = childNodes.item(i);

					if (uttNode.getNodeName().equals("utterance")) {
						ParseTestData parseTest= createParseTest(uttNode);
						tests.add(parseTest);
					}
				}
			}

			else if (topCategory.equals("utterancesToPack") || 
					topCategory.equals("utterancesTTS")) {
				for (int i = 0; i < childNodes.getLength(); i++) {
					Node uttNode = childNodes.item(i);

					if (uttNode.getNodeName().equals("utterance")) {
						TestData test= createTest(uttNode);
						tests.add(test);
					}
				}
			}

			else if (topCategory.equals("utterancesDialMoves")) {
				for (int i = 0; i < childNodes.getLength(); i++) {
					Node uttNode = childNodes.item(i);

					if (uttNode.getNodeName().equals("utterance")) {
						DMTestData test= createDMTest(uttNode);
						tests.add(test);
					}
				}
			}

			else if (topCategory.equals("utterancesDiscRef")) {
				for (int i = 0; i < childNodes.getLength(); i++) {
					Node uttNode = childNodes.item(i);

					if (uttNode.getNodeName().equals("utterance")) {
						DiscRefTestData test= createDiscRefTest(uttNode);
						tests.add(test);
					}
				}
			}

			else if (topCategory.equals("utterancesVisualGrounding")) {
				for (int i = 0; i < childNodes.getLength(); i++) {
					Node uttNode = childNodes.item(i);

					if (uttNode.getNodeName().equals("utterance")) {
						VisualGroundingTestData test= createVGTest(uttNode);
						tests.add(test);
					}
				}
			}
			
			else if (topCategory.equals("utterancesEventStruct")) {
				for (int i = 0; i < childNodes.getLength(); i++) {
					Node uttNode = childNodes.item(i);

					if (uttNode.getNodeName().equals("utterance")) {
						TestData test= createEventStructTest(uttNode);
						tests.add(test);
					}
				}
			}
			else if (topCategory.equals("utterancesProxyFactories")) {
				for (int i = 0; i < childNodes.getLength(); i++) {
					Node uttNode = childNodes.item(i);

					if (uttNode.getNodeName().equals("utterance")) {
						TestData test= createProxyFactoriesTest(uttNode);
						tests.add(test);
					}
				}
			}
			

			else if (topCategory.equals("utterancesSDRS")) {
				for (int i = 0; i < childNodes.getLength(); i++) {
					Node uttNode = childNodes.item(i);

					if (uttNode.getNodeName().equals("discourse")) {
						TestData test= createSDRSTest(uttNode);
						tests.add(test);
					}
				}
			}
			
			else if (topCategory.equals("utterancesToPlan")) {
				for (int i = 0; i < childNodes.getLength(); i++) {
					Node uttNode = childNodes.item(i);

					if (uttNode.getNodeName().equals("utterance")) {
						TestData test= createPlanningTest(uttNode);
						tests.add(test);
					}
				}
			}
			
			else if (topCategory.equals("utterancesToSelect")) {
				for (int i = 0; i < childNodes.getLength(); i++) {
					Node uttNode = childNodes.item(i);

					if (uttNode.getNodeName().equals("utterance")) {
						TestData test= createParseSelectionTest(uttNode);
						tests.add(test);
					}
				}
			}

			

		} catch (SAXException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		return tests;
	}

	private ParseTestData createParseTest(Node uttNode)
	throws FileNotFoundException {

		ParseTestData parseTest = new ParseTestData();
		String string = "";
		Vector<String> parses = new Vector<String>();

		NodeList parameterNodes = uttNode.getChildNodes();
		for (int i = 0; i < parameterNodes.getLength(); i++) {
			Node parameter = parameterNodes.item(i);
			// ugly but fast
			if (parameter.getNodeName().equals("string")) {	
				string = parameter.getFirstChild().getNodeValue();
				//	log("added string: " + string);
				parseTest.setString(string);
			}
			if (parameter.getNodeName().equals("parses")) {
				NodeList parseNodes = parameter.getChildNodes();		
				for (int j = 0; j < parseNodes.getLength(); j++) {
					Node parseNode = parseNodes.item(j);
					//	log("parse name: " + parseNode.getNodeName());

					if (parseNode.getNodeName().equals("parse")) {
						String parse = parseNode.getFirstChild().getNodeValue();
						parse = parse.replace("/", "<").replace("\\", ">");
						log("added parse: " + parse);
						parses.add(parse);
					}
				}
				parseTest.setParses(parses);
			}
		}
		return parseTest;

	}



	private TestData createTest(Node uttNode)
	throws FileNotFoundException {

		TestData parseTest = new TestData();
		String string = "";

		NodeList parameterNodes = uttNode.getChildNodes();
		for (int i = 0; i < parameterNodes.getLength(); i++) {
			Node parameter = parameterNodes.item(i);
			// ugly but fast
			if (parameter.getNodeName().equals("string")) {	
				string = parameter.getFirstChild().getNodeValue();
				//	log("added string: " + string);
				parseTest.setString(string);
			}
		}
		return parseTest;

	}

	private DMTestData createDMTest(Node uttNode)
	throws FileNotFoundException {

		DMTestData dmTest = new DMTestData();
		String string = "";
		String moveType = "";

		NodeList parameterNodes = uttNode.getChildNodes();
		for (int i = 0; i < parameterNodes.getLength(); i++) {
			Node parameter = parameterNodes.item(i);
			// ugly but fast
			if (parameter.getNodeName().equals("string")) {	
				string = parameter.getFirstChild().getNodeValue();
				//	log("added string: " + string);
				dmTest.setString(string);
			}
			if (parameter.getNodeName().equals("movetype")) {	
				moveType = parameter.getFirstChild().getNodeValue();
				log("added moveType: " + moveType);

				dmTest.setMoveType(moveType);
			}
		}
		return dmTest;

	}


	private DiscRefTestData createDiscRefTest(Node uttNode)
	throws FileNotFoundException {

		DiscRefTestData drData = new DiscRefTestData();
		Vector<String> discourse = new Vector<String>();

		NodeList parameterNodes = uttNode.getChildNodes();
		for (int i = 0; i < parameterNodes.getLength(); i++) {
			Node parameter = parameterNodes.item(i);
			// ugly but fast
			if (parameter.getNodeName().equals("discourse")) {	
				NodeList parseNodes = parameter.getChildNodes();		
				for (int j = 0; j < parseNodes.getLength(); j++) {
					Node parseNode = parseNodes.item(j);
					//	log("parse name: " + parseNode.getNodeName());

					if (parseNode.getNodeName().equals("string")) {
						String parse = parseNode.getFirstChild().getNodeValue();
						log("added string: " + parse);
						discourse.add(parse);
					}
				}
				drData.setDiscourse(discourse);
			}
			if (parameter.getNodeName().equals("reference")) {	

				NodeList parseNodes = parameter.getChildNodes();
				String refexpr = "";
				String refpos = "";
				String referent = "";

				for (int j = 0; j < parseNodes.getLength(); j++) {
					Node parseNode = parseNodes.item(j);
					//	log("parse name: " + parseNode.getNodeName());

					if (parseNode.getNodeName().equals("refexpr")) {
						refexpr = parseNode.getFirstChild().getNodeValue();
					}
					if (parseNode.getNodeName().equals("refpos")) {
						refpos = parseNode.getFirstChild().getNodeValue();
					}
					if (parseNode.getNodeName().equals("referent")) {
						referent = parseNode.getFirstChild().getNodeValue();
					}
				}

				drData.setReference(refexpr, refpos, referent);
			}
		}
		return drData;

	}


	private VisualGroundingTestData createVGTest(Node uttNode)
	throws FileNotFoundException {

		VisualGroundingTestData drData = new VisualGroundingTestData();
		String string = "";

		NodeList parameterNodes = uttNode.getChildNodes();
		for (int i = 0; i < parameterNodes.getLength(); i++) {
			Node parameter = parameterNodes.item(i);
			// ugly but fast
			if (parameter.getNodeName().equals("string")) {	
				string = parameter.getFirstChild().getNodeValue();
				//	log("added string: " + string);
				drData.setString(string);
			}

			if (parameter.getNodeName().equals("visualscene")) {	

				NodeList parseNodes = parameter.getChildNodes();

				for (int j = 0; j < parseNodes.getLength(); j++) {
					Node node = parseNodes.item(j);

					NodeList childNodes = node.getChildNodes();
					for (int k = 0; k < childNodes.getLength(); k++) {
						Node attrNode = childNodes.item(k);
						
						String label = "";
						String color = "";
						String size = "";

						//	log("parse name: " + parseNode.getNodeName());

						if (attrNode.getNodeName().equals("label")) {
							label = attrNode.getFirstChild().getNodeValue();
						}
						if (attrNode.getNodeName().equals("color")) {
							color = attrNode.getFirstChild().getNodeValue();
						}
						if (attrNode.getNodeName().equals("size")) {
							size = attrNode.getFirstChild().getNodeValue();
						}
						
						VisualGroundingTestData.VisualObject object = 
							drData.addVisualObject(label);
						if (!color.equals("")) {
							object.setColor(color);
						}
						if (!size.equals("")) {
							object.setSize(size);
						}
					}

				}
				}
			
			if (parameter.getNodeName().equals("grounding")) {	

				NodeList parseNodes = parameter.getChildNodes();
				String shouldbegrounded = "";
				String referent = "";

				for (int j = 0; j < parseNodes.getLength(); j++) {
					Node parseNode = parseNodes.item(j);
					//	log("parse name: " + parseNode.getNodeName());

					if (parseNode.getNodeName().equals("shouldbegrounded")) {
						shouldbegrounded = parseNode.getFirstChild().getNodeValue();
					}
					if (parseNode.getNodeName().equals("referentobject")) {
						referent = parseNode.getFirstChild().getNodeValue();
					}
				}

				if (shouldbegrounded.equals("yes")) {
					drData.setShouldBeGrounded(true);
					drData.setReferent(referent);
				}
				else {
					drData.setShouldBeGrounded(false);
				}
			}
			}
		
			return drData;

		}


	private EventStructureData createEventStructTest(Node uttNode)
	throws FileNotFoundException {

		EventStructureData esTest = new EventStructureData();
		String string = "";

		NodeList parameterNodes = uttNode.getChildNodes();
		for (int i = 0; i < parameterNodes.getLength(); i++) {
			Node parameter = parameterNodes.item(i);
			// ugly but fast
			if (parameter.getNodeName().equals("string")) {	
				string = parameter.getFirstChild().getNodeValue();
				//	log("added string: " + string);
				esTest.setString(string);
			}
			
			if (parameter.getNodeName().equals("nucleus")) {	

				NodeList parseNodes = parameter.getChildNodes();
				String eventtype = "";
				String statetype = "";

				for (int j = 0; j < parseNodes.getLength(); j++) {
					Node parseNode = parseNodes.item(j);
					//	log("parse name: " + parseNode.getNodeName());

					if (parseNode.getNodeName().equals("eventtype")) {
						eventtype = parseNode.getFirstChild().getNodeValue();
						esTest.setEventType(eventtype);
					}
					if (parseNode.getNodeName().equals("statetype")) {
						statetype = parseNode.getFirstChild().getNodeValue();
						esTest.setStateType(statetype);
					}
				}
			}

		}
		return esTest;

	}


	private ProxyFactoriesTestData createProxyFactoriesTest(Node uttNode)
	throws FileNotFoundException {

		ProxyFactoriesTestData pTest = new ProxyFactoriesTestData();
		String string = "";
		String nbrUnions = "";

		NodeList parameterNodes = uttNode.getChildNodes();
		for (int i = 0; i < parameterNodes.getLength(); i++) {
			Node parameter = parameterNodes.item(i);
			// ugly but fast
			if (parameter.getNodeName().equals("string")) {	
				string = parameter.getFirstChild().getNodeValue();
				//	log("added string: " + string);
				pTest.setString(string);
			}
			if (parameter.getNodeName().equals("unions")) {	
				nbrUnions = parameter.getFirstChild().getNodeValue();
		//		log("added nbr of unions: " + nbrUnions);

				pTest.setNbrUnions(nbrUnions);
			}
		}
		return pTest;

	}
	
	private TestData createSDRSTest(Node uttNode)
	throws FileNotFoundException {

		SDRSTestData sdrsTest = new SDRSTestData();
		Vector<String> discourse = new Vector<String>();

		NodeList parameterNodes = uttNode.getChildNodes();
		for (int i = 0; i < parameterNodes.getLength(); i++) {
			Node parameter = parameterNodes.item(i);
			// ugly but fast
			if (parameter.getNodeName().equals("string")) {	
				discourse.add(parameter.getFirstChild().getNodeValue());
				//	log("added string: " + string);			
			}
		}
		sdrsTest.setDiscourse(discourse);
		return sdrsTest;
	}


	private TestData createPlanningTest(Node uttNode)
	throws FileNotFoundException {

		PlanningTestData test = new PlanningTestData();
		String string = "";
		Vector<String> realization = new Vector<String>();
		
		NodeList parameterNodes = uttNode.getChildNodes();
		for (int i = 0; i < parameterNodes.getLength(); i++) {
			Node parameter = parameterNodes.item(i);
			// ugly but fast
			if (parameter.getNodeName().equals("input")) {	
				string = parameter.getFirstChild().getNodeValue();
				//	log("added string: " + string);
				test.setInput(string);
			}
			
			if (parameter.getNodeName().equals("realizations")) {		
				NodeList parseNodes = parameter.getChildNodes();				
				for (int j = 0; j < parseNodes.getLength(); j++) {
					Node parseNode = parseNodes.item(j);
					//	log("parse name: " + parseNode.getNodeName());
					if (parseNode.getNodeName().equals("string")) {
						String parse = parseNode.getFirstChild().getNodeValue();
						realization.add(parse);
					}
				}
			}
		}
		test.setOutput(realization);
		return test;

	}
	
	
	
	int idCount = 0;
	public int NbNBests = 5;
	
	private TestData createParseSelectionTest(Node uttNode)
	throws FileNotFoundException {

		ParseSelectionTestData test = new ParseSelectionTestData();

		Vector<PhonString> inputs = 
			new Vector<PhonString>();
	
		NodeList parameterNodes = uttNode.getChildNodes();
		for (int i = 0; i < parameterNodes.getLength(); i++) {
			Node parameter = parameterNodes.item(i);
	
			if (parameter.getNodeName().equals("transcription")) {
				String str = parameter.getFirstChild().getNodeValue();
				test.setTranscription(str);
			}
			
			if (parameter.getNodeName().equals("parse")) {
				if(parameter.getFirstChild()!=null) {
					String parse = parameter.getFirstChild().getNodeValue();
					parse = parse.replace("/", "<").replace("\\", ">");
					test.setCorrectParse(parse);
				}
			}
			
			else if (parameter.getNodeName().equals("ASRInputs")) {	

				NodeList parseNodes = parameter.getChildNodes();

				for (int j = 0; j < parseNodes.getLength(); j++) {
					Node node = parseNodes.item(j);
					
					if (node.getNodeName().equals("input")) {	

						NodeList childNodes = node.getChildNodes();
						
						PhonString asrInput = new PhonString();
						
						for (int k = 0; k < childNodes.getLength(); k++) {
							Node subnode = childNodes.item(k);
							
							if (subnode.getNodeName().equals("string")) {
								String str = subnode.getFirstChild().getNodeValue();
								str = GenerationUtils.modifyCase(str);
								str = GenerationUtils.removeDisfluencies(str);
								asrInput.wordSequence = str;
								
								StringTokenizer tokenizer = new StringTokenizer(str);
								asrInput.length = tokenizer.countTokens();
								
							}
							if (subnode.getNodeName().equals("conf")) {
								String conf = subnode.getFirstChild().getNodeValue();
								asrInput.confidenceValue = new Float(conf).floatValue();
							}
							if (subnode.getNodeName().equals("NLconf")) {
								String conf = subnode.getFirstChild().getNodeValue();
								asrInput.NLconfidenceValue = new Float(conf).floatValue();
							}
							if (subnode.getNodeName().equals("rank")) {
								String rank = subnode.getFirstChild().getNodeValue();
								asrInput.rank = new Integer(rank).intValue();
							}
						}		
						asrInput.id = "input"+idCount;
						idCount++;
						if (inputs.size() < NbNBests) {
						inputs.add(asrInput);
						}
					}				
				}
				}
		}
		
		test.setASRInputs(inputs);
		return test;

	}
	
		private void log (String str) {
			System.out.println("[PARSING TEST READER] " +  str);
		}

	}
