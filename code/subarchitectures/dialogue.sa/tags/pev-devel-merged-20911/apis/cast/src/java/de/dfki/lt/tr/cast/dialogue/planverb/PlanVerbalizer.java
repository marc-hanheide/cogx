package de.dfki.lt.tr.cast.dialogue.planverb;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.net.UnknownHostException;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Scanner;
import java.util.Map.Entry;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import coma.aux.ComaGBeliefHelper;
import comadata.ComaRoom;

import SpatialData.Place;
import SpatialData.PlaceStatus;
import VisionData.VisualObject;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryReaderComponent;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.IceXMLSerializer;
import castutils.castextensions.WMView;

import scala.collection.immutable.Set;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidQueryException;
import de.dfki.lt.tr.cast.dialogue.ComaReferringExpressionGeneration;
import de.dfki.lt.tr.cast.dialogue.GBeliefMemory;
import de.dfki.lt.tr.cast.dialogue.POPlanMonitor;
import de.dfki.lt.tr.cast.dialogue.realisation.LFRealiser;
import de.dfki.lt.tr.cast.dialogue.realisation.RealisationClient;
import de.dfki.lt.tr.cast.dialogue.realisation.TarotCCGRealiser;
import de.dfki.lt.tr.planverb.generation.Message;
import de.dfki.lt.tr.planverb.generation.ProtoLFMessage;
import de.dfki.lt.tr.planverb.generation.RhetoricalMarkerMessage;
import de.dfki.lt.tr.planverb.generation.StringMessage;
import de.dfki.lt.tr.planverb.planning.pddl.PDDLContentDeterminator;
import de.dfki.lt.tr.planverb.planning.pddl.PDDLDomainModel;
import de.dfki.lt.tr.planverb.planning.pddl.POPlan;
import de.dfki.lt.tr.planverb.planning.pddl.PDDLHistory;
import de.dfki.lt.tr.planverb.history.History;
import de.dfki.tarot.cogx.CASTLogicalForms;
import de.dfki.tarot.cogx.CogXJavaHelpers;
import de.dfki.tarot.cogx.FeatureReplacer;
import de.dfki.tarot.cogx.ReferentReplacer;
import de.dfki.tarot.cogx.WMAddress;
import de.dfki.tarot.nlp.lf.BasicLogicalForm;
import de.dfki.tarot.nlp.lf.BasicState;
import de.dfki.tarot.nlp.lf.pattern.BasicPatterns;
import de.dfki.tarot.util.BuildException;
import de.dfki.tarot.util.ParseException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class PlanVerbalizer {
	
	private final PDDLContentDeterminator m_contentDeterminator;
	private LFRealiser m_realiser;
	private ManagedComponent m_castComponent;
	
	private HashMap<String, String> m_preLexicalSub  = new HashMap<String, String>();
	private HashMap<String, String> m_postLexicalSub = new HashMap<String, String>();
	
	private final static String DEFAULTNGRAMFILE   = "subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/ngram-corpus.txt";
	private final static String DEFAULTGRAMMARFILE = "subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/grammar.xml";
	
	public boolean debug_lf_out = false;
	
	public GBeliefMemory m_gbmemory = new GBeliefMemory();
	
	final protected WMView<GroundedBelief> view = WMView.create(this.m_castComponent, GroundedBelief.class);
	

	/**
	 * Creates and initializes a new PlanVerbalizer object.
	 * After initialization, it can be queried subsequently for verbalizations of event structures.
	 * The domains and grammar etc. it operates on remain static.
	 * @param annotatedDomainFile
	 * @param pddlDomainFile
	 * @param grammarFile -- or "" -> not needed for the realiserver // or use default file 
	 * @param ngramFile  -- or "" -> not needed for the realiserver // or use default file
	 * @param hostname -- or "" -> try default hostname (localhost) or fallback to internal tarot realiser
	 * @param port -- or null -> try default port (4444) or fallback to internal tarot realiser
	 * @throws IOException
	 */
	public PlanVerbalizer(String annotatedDomainFile, String pddlDomainFile,  String grammarFile, String ngramFile, String hostname, Integer port, WorkingMemoryReaderComponent component) throws IOException  {
		// init CAST component for WM access and logging
		m_castComponent = (ManagedComponent)component;
		
		log("PlanVerbalizer constructor called with annotatedDomainFile = " + annotatedDomainFile +
				" pddlDomainFile = " + pddlDomainFile + " grammarFile = " + grammarFile);
		
		// initialize planner-related stuff
		File adf = new File(annotatedDomainFile);
		File pddf = new File(pddlDomainFile);
		PDDLDomainModel domainModel = new PDDLDomainModel(adf, pddf);
		m_contentDeterminator = new PDDLContentDeterminator(domainModel);

	
		try {
			m_realiser = new RealisationClient(hostname, port);
		} catch (UnknownHostException e) {
			logException(e);
			log("trying to use object-based tarot realiser instead of the realiserver.");
			if (grammarFile.equals("")) grammarFile = DEFAULTGRAMMARFILE;
			if (ngramFile.equals("")) ngramFile = DEFAULTNGRAMFILE;
			m_realiser = new TarotCCGRealiser(grammarFile, ngramFile);
		} catch (IOException e) {
			logException(e);
			log("trying to use object-based tarot realiser instead of the realiserver.");
			if (grammarFile.equals("")) grammarFile = DEFAULTGRAMMARFILE;
			if (ngramFile.equals("")) ngramFile = DEFAULTNGRAMFILE;
			m_realiser = new TarotCCGRealiser(grammarFile, ngramFile);
		}
		
		// initialize lexicon substitutions for the time being...
		initLexicalSubstitutions();
		log("finished PlanVerbalizer constructor");
	}
	
	public PlanVerbalizer(String annotatedDomainFile, String pddlDomainFile,  String grammarFile, String ngramFile, String hostname, Integer port, String gbmemoryFile) throws IOException  {
		// init CAST component for WM access and logging
		m_castComponent = null;

		// Load gbmemory from file
		File f = new File(gbmemoryFile);
		readFromFile(f);
		
		log(m_gbmemory.getTimeStampMap());
		
		log("PlanVerbalizer constructor called with annotatedDomainFile = " + annotatedDomainFile +
				"\npddlDomainFile = " + pddlDomainFile + "\ngrammarFile = " + grammarFile + 
				"\nngamFile = " + ngramFile + "\nhostname = " + hostname + 
				"\nport = " + port + "\ngbmemoryFile = " + gbmemoryFile);
		
		// initialize planner-related stuff
		File adf = new File(annotatedDomainFile);
		File pddf = new File(pddlDomainFile);
		PDDLDomainModel domainModel = new PDDLDomainModel(adf, pddf);
		m_contentDeterminator = new PDDLContentDeterminator(domainModel);

	
		try {
			m_realiser = new RealisationClient(hostname, port);
		} catch (UnknownHostException e) {
			logException(e);
			log("trying to use object-based tarot realiser instead of the realiserver.");
			if (grammarFile.equals("")) grammarFile = DEFAULTGRAMMARFILE;
			if (ngramFile.equals("")) ngramFile = DEFAULTNGRAMFILE;
			m_realiser = new TarotCCGRealiser(grammarFile, ngramFile);
		} catch (IOException e) {
			logException(e);
			log("trying to use object-based tarot realiser instead of the realiserver.");
			if (grammarFile.equals("")) grammarFile = DEFAULTGRAMMARFILE;
			if (ngramFile.equals("")) ngramFile = DEFAULTNGRAMFILE;
			m_realiser = new TarotCCGRealiser(grammarFile, ngramFile);
		}
		
		// initialize lexicon substitutions for the time being...
		initLexicalSubstitutions();
		log("finished PlanVerbalizer constructor");
	}
	
	private GroundedBelief getGBelief(WMAddress referentWMA) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		//Uncommented but not the deleted, in case future generations don't want to use GBeliefMemory

		/*if (m_castComponent != null) {
			GroundedBelief gbWME = m_castComponent.getMemoryEntry(new WorkingMemoryAddress(referentWMA.id(), referentWMA.subarchitecture()), GroundedBelief.class);
			return gbWME;
		} else {*/
		//GroundedBelief gbWME = m_gbmemory.getGBelief(new WorkingMemoryAddress(referentWMA.id(), referentWMA.subarchitecture()), taskID, poplanID);

		log("called getGBelief("+referentWMA.toString()+")");
		GroundedBelief gbWME;
		String[] beliefTempIndexWMA = referentWMA.id().split(",");
		if (beliefTempIndexWMA.length>1) {

			WorkingMemoryAddress beliefWMA = new WorkingMemoryAddress(beliefTempIndexWMA[1], referentWMA.subarchitecture());
			String[] beliefTempIndices = beliefTempIndexWMA[0].split("_");
			int beliefTaskID = Integer.parseInt(beliefTempIndices[0]);
			int beliefPOPlanID = Integer.parseInt(beliefTempIndices[1]);

//			log("beliefWMA = " + beliefWMA.id + "@" + beliefWMA.subarchitecture + " - beliefTaskID = " + beliefTaskID + " - beliefPOPlanID = " + beliefPOPlanID);

			gbWME = m_gbmemory.getValidGBelief(beliefWMA, beliefTaskID, beliefPOPlanID);
		} else {
			gbWME = m_gbmemory.getLastValidGBelief(new WorkingMemoryAddress(referentWMA.id(), referentWMA.subarchitecture()));
		}

		
		return gbWME;
	}

	/**
	 * Read the GBeliefMemory object from the given file
	 * 
	 * @param gbmemoryFile
	 * 				the file to read from
	 */
	private void readFromFile(File gbmemoryFile) {
		
		try {
//			FileInputStream f = new FileInputStream(gbmemoryFile);
//			ObjectInputStream s = new ObjectInputStream(f);
//			m_gbmemory = (GBeliefMemory) s.readObject();
			StringBuilder text = new StringBuilder();
		    String NL = System.getProperty("line.separator");
		    Scanner scanner = new Scanner(new FileInputStream(gbmemoryFile));
		    try {
		      while (scanner.hasNextLine()){
		        text.append(scanner.nextLine() + NL);
		      }
		    }
		    finally{
		      scanner.close();
		    }
//		    log("Text read in: " + text);
		    
		    m_gbmemory = IceXMLSerializer.fromXMLString(text.toString(), GBeliefMemory.class);
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		 catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * add missing lexical items to this method!
	 * caution: please add blanks before and after each word!
	 */
	private void initLexicalSubstitutions() {
		//m_preLexicalSub. put(" meetingroom ", " lab ");
		//m_postLexicalSub.put("lab", "meetingroom");
		
		//m_preLexicalSub. put(" corridor ", " hall ");
		//m_postLexicalSub.put("hall", "corridor");
		
		//m_preLexicalSub. put(" cerealbox ", " box ");
		//m_postLexicalSub.put("box", "cerealbox");
		
		//com_preLexicalSub. put("create ", "make ");

		//m_preLexicalSub. put(" magazine ", " book ");
		//m_postLexicalSub.put("book", "magazine");

		//m_preLexicalSub. put("m-location ^ via ", "m-through ^ through");
        // TODO find a solution for indirect speech
		m_preLexicalSub. put("ascription ^ be", "ascription ^ be ^ <Mood>ind ^ <Tense>past");
		//m_preLexicalSub. put("<ExecutionStatus>PENDING", "<Mood>ind ^ <Tense>fut ^ <Modifier>(will1_0:modal ^ will)");
		
		m_preLexicalSub.put("<ExecutionStatus>FAILED", "<Mood>ind ^ <Polarity>neg ^ <Modifier>(could1_0:modal ^ could)");
		m_preLexicalSub.put("m-cause", "m-condition");
		
		m_postLexicalSub.put("cones", "viewcones");
		m_postLexicalSub.put("did not search for", "didn't find");
		m_postLexicalSub.put("did not", "could not successfully");
		
		//m_preLexicalSub.put("placeholder", "recreationroom");
		//m_postLexicalSub.put("recreationroom", "placeholder");
	}
	
	public String verbalizeHistory(final List<POPlan> hlist, int planningTaskID, String task) {
		log("entering verbalizeHistory()");
		History h = new PDDLHistory(hlist, planningTaskID, task);
		return verbalizeHistory(h);
	}
	
	public String verbalizeHistory(History h) {
	  log("entering verbalizeHistory()");
		
		List<Message> messages = m_contentDeterminator.determineMessages(h);	
		log("contentDeterminator returned " + messages.size() + " messages for the full History.");

		return aggregateStrings(realizeMessages(messages));
	}


	
	private String aggregateStrings(String text) {
//		System.out.println("entered");
		String[] sentences = text.split("\n");
		for (int i=0; i < sentences.length -1 ; i++) {
//			System.out.println(sentences[i]);
			Pattern pattern = Pattern.compile("(in the room )([a-z]+)( at the place )([a-z]+)");
			// In case you would like to ignore case sensitivity you could use this
			// statement
			// Pattern pattern = Pattern.compile("\\s+", Pattern.CASE_INSENSITIVE);
			Matcher matcher = pattern.matcher(sentences[i]);
			// Check all occurrences
			while (matcher.find()) {
//				System.out.println("Current sentence = " + sentences[i]);
//				System.out.print("Start index: " + matcher.start());
//				System.out.print(" End index: " + matcher.end() + " ");
//				System.out.println(matcher.group());
				sentences[i+1] = sentences[i+1].replaceAll(matcher.group(), "there");
				sentences[i+1] = "and " + sentences[i+1];
			}
		}
		
		String output = "";
		for (int i=0; i < sentences.length -1; i++) {
			output+=sentences[i];
			if (!sentences[i+1].startsWith("because") && !sentences[i+1].startsWith("and") && !sentences[i+1].startsWith("but")) output += ".\n";
//			else if (sentences[i+1].startsWith("and then")) output += ".\n";
			else output += " ";
		}
		
		return output + sentences[sentences.length-1] + ".";
	}

	public String realizeMessages(List<Message> messages) {
		StringBuilder log_sb = new StringBuilder();
		StringBuilder output_sb = new StringBuilder();

        // TODO missing: aggregation! 
        
		// realize each message
		for (Message msg : messages) {
			if (msg instanceof ProtoLFMessage) {
				log_sb.append("Current Message is a ProtoLFMessage:\n" + ((ProtoLFMessage) msg).getProtoLF());
				BasicLogicalForm protoLF = ((ProtoLFMessage) msg).getProtoLF();
				if (protoLF.toString().contains("assume") && protoLF.toString().contains("that")) {
					log_sb.append("\n Error: I got as 'assume that' protoLF -- ignoring it!");
					log(log_sb.toString());
					return "";
				}
				log("Current Message is a ProtoLFMessage:\n" + ((ProtoLFMessage) msg).getProtoLF());

				// fixing malformed WMAs before GRE
				try {
					protoLF = preProcessWMAs(protoLF);
					log_sb.append("\n WMA substitution before realization yielded: \n" + protoLF.toString());
				} catch (BuildException e) {
					logException(e);
				} catch (ParseException e) {
					logException(e);
				}
				
				
				// do GRE 
				protoLF = doGRE(protoLF);
				log_sb.append("\n doGRE() yielded: \n" + protoLF.toString());

				// perform lexical substitution before realization
				try {
					protoLF = preProcessLexiconSubstitution(protoLF);
					log_sb.append("\n lexical substitution before realization yielded: \n" + protoLF.toString());
				} catch (BuildException e) {
					logException(e);
				} catch (ParseException e) {
					logException(e);
				}
 
				// make missing parts consistent (e.g. subj agreement)
				BasicLogicalForm finalLF = finalizeProtoLF(protoLF);
				log_sb.append("\n finalizeProtoLF() yielded: \n" + finalLF.toString());

				// surface realization
				// perform lexical re-substitution after realization, before appending to the verbal report
				String realization = m_realiser.realiseLF(finalLF);
				log_sb.append("\n realizeLF() yielded: \n" + realization);

				if (!realization.equals("")) {
					String outputText = postProcessLexiconSubstitution(realization);
					log_sb.append("\n appending postProcessLexiconSubstitution() final output text: \n" + outputText);
					output_sb.append(outputText + "\n");
				} else {
					if (debug_lf_out) {
						log_sb.append("\n appending original LF to output text.");
						output_sb.append(finalLF.toString() + "\n");
					} else {
						log_sb.append("\n no realisation found for LF " + finalLF.toString());
						output_sb.append("\n");
					}
				}
			} else if (msg instanceof StringMessage) {
				String outputText = ((StringMessage) msg).getText();
				log_sb.append("\n appending current Message, which is a StringMessage: \n " + outputText);
				// rhetorical markers are prepended to the subsequent sentence:
				if (msg instanceof RhetoricalMarkerMessage) {
					output_sb.append(outputText + " "); // no full stop + line break
				} else output_sb.append(outputText + "\n");
			}
        }
    	log(log_sb.toString());
    	return output_sb.toString();
	}
	


    
    /**
     * This method performs GRE. It replaces castreferents with their appropriate natural language semantics.
     * - for now, the robot is statically realized as "I"
     * 
     * @param protoLF
     * @return a logical form that contains proper nominals for former free castreferent nominals
     */
    private BasicLogicalForm doGRE(BasicLogicalForm protoLF) {
    	// protoLF = CogXJavaHelpers.replaceSelfReferenceByI(protoLF, new WMAddress("0:D","spatial.sa"));
    	StringBuilder log_sb = new StringBuilder("doGRE() for protoLF: \n" + protoLF.toString());
    	
    	Set<WMAddress> swma = CASTLogicalForms.referentsInLF(protoLF);
    	Collection<WMAddress> jswma = scala.collection.JavaConversions.asJavaCollection(swma);
    	for (WMAddress referentWMA : jswma) {
    		log_sb.append("\n current referentWMA = " + referentWMA);
    		WMAddress lookupWMA;
    		if (referentWMA.subarchitecture().equals("PLANNERPLACE") || referentWMA.subarchitecture().equals("COMAROOM")) {
    			String[] parts = referentWMA.id().split(":");
    			
    			if (parts[0].charAt(0) == '_') {
    				parts[0] = parts[0].substring(1,parts[0].length());
    				parts[0] = parts[0].toUpperCase();
    			}
    			
    			if (parts[1].charAt(0) == '_') {
    				parts[1] = parts[1].substring(1,parts[1].length());
    				parts[1] = parts[1].toUpperCase();
    			}
    			
    			if (referentWMA.subarchitecture().equals("PLANNERPLACE")) lookupWMA = new WMAddress(parts[0] + ":" + parts[1], "spatial.sa");
    			else lookupWMA = new WMAddress(parts[0] + ":" + parts[1], "coma"); //if (referentWMA.subarchitecture().equals("COMAROOM"))
    		} else lookupWMA = referentWMA;
    		try {
    			//GroundedBelief gbWME = m_castComponent.getMemoryEntry(new WorkingMemoryAddress(referentWMA.id(), referentWMA.subarchitecture()), GroundedBelief.class);
    			GroundedBelief gbWME = getGBelief(lookupWMA);
    			
    			log("getDeterminer with address " + lookupWMA + " returned: " + getDeterminer(gbWME));
    			
    			// check if it is the robot itself
    			if (isRobot(gbWME)) {
    				log_sb.append("\n WMA is the robot itself.");
        			protoLF = CogXJavaHelpers.replaceSelfReferenceByI(protoLF, referentWMA);
        		} else {
        			log_sb.append("\n attempting GRE via the GBelief.");
        			ReferentReplacer myRefRep = getGBeliefCategoryReplacer(gbWME);
        			if (myRefRep!=null) {    	        
        				log_sb.append("\n got a GBeliefCategoryReplacer for " + referentWMA);
        				protoLF = CogXJavaHelpers.replaceReferent(protoLF, referentWMA, myRefRep);
        				log_sb.append("\n GBeliefCategoryReplacer yielded protoLF: \n " + protoLF);
        			} else {
        				log_sb.append("\n got a null reference replacer via GBelief! WMA type unknown... ignoring...");
        			}
        		}
			} catch (DoesNotExistOnWMException e) {
				log_sb.append("\n " + e + ": " + referentWMA + " attempting GRE via alternative method.");
				ReferentReplacer myRefRep = getDoesNotExistReplacer(referentWMA);
				if (myRefRep!=null) {    	        
					log_sb.append("\n got a DoesNotExistReplacer for " + referentWMA);
    				protoLF = CogXJavaHelpers.replaceReferent(protoLF, referentWMA, myRefRep);
    				log_sb.append("\n DoesNotExistReplacer yielded protoLF: \n " + protoLF);
    			} else {
    				log_sb.append("\n got a null reference replacer via alternative method! ignoring...");
    			}
			} catch (UnknownSubarchitectureException e) {
				log_sb.append(e);
				logException(e);
			}
    	}
    	log_sb.append("\n *** doGRE returns protoLF: *** \n " + (protoLF==null ? "null" : protoLF.toString()));
    	log(log_sb.toString());
    	return protoLF;
    }
    
    private String getDeterminer(GroundedBelief referentGB) {
		
    	if (this.m_castComponent != null) {
    		
    		if (((POPlanMonitor) this.m_castComponent).m_comareasoner != null) {
    	
		    	this.m_castComponent.log("GETDET: getDeterminer(" + referentGB.id + ")called");
				// currently only handles "the" vs. "this" -- could also be extended to "a"
				// currently only handles ComaRooms
				CASTIndependentFormulaDistributionsBelief<GroundedBelief> referentGBProxy = 
					CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, referentGB);
			
				if (referentGBProxy.getType().equals(
						SimpleDiscreteTransferFunction
						.getBeliefTypeFromCastType(ComaRoom.class))) {
					this.m_castComponent.log("GETDET: referent is a ComaRoom");
					try {
		
						for (Entry<WorkingMemoryAddress, GroundedBelief> g : this.view.entrySet()) {
							GroundedBelief currGBelief = g.getValue();
							if (currGBelief.type.equals("Robot")) {
								this.m_castComponent.log("GETDET: found the GroundedBelief of the Robot!");
								CASTIndependentFormulaDistributionsBelief<GroundedBelief> robotGBProxy =
									CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, 	currGBelief);
								robotGBProxy.getContent();
								WMPointer isInRelateePtr = WMPointer.create(robotGBProxy.getContent().get(
								"is-in").getDistribution().getMostLikely().get());
		
								if (isInRelateePtr!=null) {
									this.m_castComponent.log("GETDET: found the is-in relatee, going to read its WME");
									GroundedBelief gbOfRobotsPlace = this.m_castComponent.getMemoryEntry(isInRelateePtr.get().pointer, 
											GroundedBelief.class);
									this.m_castComponent.debug("GETDET: loaded WME of Robot's Place GBelief with ID " + gbOfRobotsPlace.id);
		
									CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbPlaceProxy = 
										CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, gbOfRobotsPlace);
									this.m_castComponent.debug("GETDET: created a CASTIndependentFormulaDistributionsBelief proxy for the Robot's Place");
		
									int placeID = -1;
									try {
										placeID  = gbPlaceProxy.getContent().get(
												PlaceTransferFunction.PLACE_ID_ID)
												.getDistribution().getMostLikely().getInteger();
									} catch (BeliefInvalidQueryException e) {
										this.m_castComponent.log("GETDET: Caught runtime exception: Belief formula is not of type proposition");
										this.m_castComponent.logException(e);
									}
									this.m_castComponent.log("GETDET: robot is at place with ID " + placeID);
		
									String[] roomInstances = ((POPlanMonitor) this.m_castComponent).m_comareasoner.getRelatedInstancesByRelation(
											"dora:place"+placeID, "dora:constituentOfRoom");
									this.m_castComponent.log("GETDET: place" + placeID + " is in " + roomInstances.length + " room(s)");
									
									String roomReferent = "dora:" + ComaGBeliefHelper.getGBeliefComaIndividualName(referentGB);
									this.m_castComponent.log("GETDET: let's check whether one of them is the referent " + referentGB.id);
		
									for (String roomIns : roomInstances) {
										if (roomIns.startsWith(":")) roomIns = "dora" + roomIns;
										this.m_castComponent.log("GETDET: current room instance is: " + roomIns + " and room refertent is " + roomReferent);
										if (roomIns.equals(roomReferent)) {
											this.m_castComponent.log("GETDET: the referent is the room in which the robot currently is -> THIS");
											return "this";
										}
									}
								}
		
							} else {
								this.m_castComponent.debug("GETDET: current looked at GBelief is not about the Robot -- continuing...");
								continue;
							}
						}
					} catch (UnknownSubarchitectureException e) {
						this.m_castComponent.log("GETDET: caught an UnknownSubarchitectureException");
						this.m_castComponent.logException(e);
					} catch (DoesNotExistOnWMException e) {
						this.m_castComponent.log("GETDET: caught a DoesNotExistOnWMException");
						this.m_castComponent.logException(e);
					}
				} else {
					this.m_castComponent.log("GETDET: referent is not a ComaRoom");
				}
				this.m_castComponent.log("GETDET: reached end of getDeterminer() method -> THE");
				return "the";
    		} else {
    			this.m_castComponent.log("GETDET: no coma reasoner -> THE");
    			return "the";
    		}
    	} else {
    		log("GETDET: running in standalone mode -> THE");
    		return "the";
    	}
	}
    

	/**
	 * This method expands several proto features to correct grammar features:
	 * - <ExecutionStatus>SUCCEEDED => <Mood>ind ^ <Tense>past
	 * // - <ExecutionStatus>PENDING => <Mood>ind ^ <Tense>fut
	 * - Subject-Actor agreement
	 * 
	 * @param protoLF
	 * @return a finalized logical form
	 */
    private static BasicLogicalForm finalizeProtoLF(BasicLogicalForm protoLF) {

    	// execution status = success yields past tense report
        FeatureReplacer pastTenseReplacer = new FeatureReplacer() {
            @Override
            public BasicState.Builder doWork(BasicState.Builder builder) {
                return builder.addFeature("Mood", "ind").addFeature("Tense", "past");
            }
        };
        protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "SUCCEEDED", pastTenseReplacer);
        protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "PENDING", pastTenseReplacer);

        FeatureReplacer failedPastTenseReplacer = new FeatureReplacer() {
        	// TODO add <Modifier>(w1:modal ^ could)
        	@Override
            public BasicState.Builder doWork(BasicState.Builder builder) {
                return builder.addFeature("Mood", "ind").addFeature("Polarity", "neg").addFeature("Tense", "past"); 
            }
        };        
        protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "FAILED", failedPastTenseReplacer);
        protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "UNSUCCESSFUL", failedPastTenseReplacer);

        
        // execution status = pending will be removed and the overall LF will be put into a control verb
//        if (protoLF.toString().contains("<ExecutionStatus>PENDING")) {
//            FeatureReplacer pendingReplacer = new FeatureReplacerPlanVerbalizer constructor cal() {
//                @Override
//                public BasicState.Builder doWork(BasicState.Builder builder) {
//
//                	return builder;
//                }
//            };
//            protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "PENDING", pendingReplacer);
//
//            String wantLFString = "@{want1_0:cognition}(want ^ <Mood>ind ^ <Tense>past ^ <Actor>(\"0:C@spatial.sa\":castreferent ^ \"0:C@spatial.sa\"))";
//            BasicLogicalForm wantLF;
//            try {
//            	wantLF = BasicLogicalForm.checkedFromString(wantLFString);
//            	BasicLogicalForm outputLF = BasicPatterns.attachLogicalFormAsSubtree(wantLF, protoLF, wantLF.root(), "Event", BasicPatterns.ignoreAdded());
//            	protoLF = outputLF;
//            } catch (BuildException e) {
//            	logException(e);
//            } catch (ParseException e) {
//            	logException(e);
//            }
//        }
        
        
        // make sure there is an appropriate subject
        // TODO cop-restr as subject!
        // protoLF = CogXJavaHelpers.ensureSubjectFilled(protoLF);

        return protoLF;
    }

    
	/**
	 * This method performs the WMA substitution pre-processing step.
	 * 
	 * @param blf
	 * @return a BasicLogicalForm that doesn't contain a malformed WMA
	 * @throws BuildException
	 * @throws ParseException
	 */
	private BasicLogicalForm preProcessWMAs(BasicLogicalForm blf) throws BuildException, ParseException {
		boolean changed = false;
		String lfString = blf.toString();
		
//		System.out.println("++++++++++++++ WMA" + lfString);
		
		if (lfString.contains("place_")) {
			String matchPattern = "(place_)(_?[0-9a-zA-Z]+)(_)(_?[0-9a-zA-Z]+)(\")";
			String replacePattern = "$2:$4@PLANNERPLACE$5";
			lfString = lfString.replaceAll(matchPattern, replacePattern);
			changed = true;
//			System.out.println("++++++++++++++ place_!" + lfString);
		}
		
		// 1_2_1,room_0__a1
		if (lfString.contains("room_")) {
//			String matchPattern = "(room_)([0-9a-zA-Z]+)(\":castreferent)";
//			String replacePattern = "room:$2@HYPOTHETICAL$3";
			String matchPattern = "(room_)(_?[0-9a-zA-Z]+)(_)(_?[0-9a-zA-Z]+)(\")";
			String replacePattern = "$2:$4@COMAROOM$5";
			lfString = lfString.replaceAll(matchPattern, replacePattern);
			changed = true;
//			System.out.println("++++++++++++++ roomX!" + lfString);
		}
		
		if (lfString.contains("room")) {
//			String matchPattern = "(room_)([0-9a-zA-Z]+)(\":castreferent)";
//			String replacePattern = "room:$2@HYPOTHETICAL$3";
			String matchPattern = "(room)([0-9a-zA-Z]+)(\")";
			String replacePattern = "$1$2@HYPOROOM$3";
			lfString = lfString.replaceAll(matchPattern, replacePattern);
			changed = true;
//			System.out.println("++++++++++++++ roomX!" + lfString);
		}

		//		System.out.println("++++++++++++++ RESULT" + lfString);
		
		if (!changed) return blf;
		else return BasicLogicalForm.checkedFromString(lfString);
	}   
    
	/**
	 * This method performs the lexical substitution pre-processing step.
	 * 
	 * @param blf
	 * @return a BasicLogicalForm that doesn't contain any of the specified out-of-vocabulary words, but temporary replacement words
	 * @throws BuildException
	 * @throws ParseException
	 */
	private BasicLogicalForm preProcessLexiconSubstitution(BasicLogicalForm blf) throws BuildException, ParseException {
		boolean changed = false;
		String lfString = blf.toString();
		
		for (String badWord : m_preLexicalSub.keySet()) {
			if (lfString.contains(badWord)) {
				lfString = lfString.replace(badWord, m_preLexicalSub.get(badWord));
				changed = true;
			}
		}

		if (!changed) return blf;
		else return BasicLogicalForm.checkedFromString(lfString);
	}

	/**
	 * This method performs the lexical substitution post-processing step.
	 * 
	 * @param lfString
	 * @return a String in which temporary replacement words are again substituted with the original out-of-vocabulary words
	 */
	private String postProcessLexiconSubstitution(String lfString) {

		for (String tmpWord : m_postLexicalSub.keySet()) {
			if (lfString.contains(tmpWord)) {
				lfString = lfString.replace(tmpWord, m_postLexicalSub.get(tmpWord));
			}
		}
		
		return lfString;
	}

	
	/**
	 * Returns the most likely (according to its probability distribution) 
	 * category of the entity the grounded belief is about. 
	 * Currently only handles VisualObject and ComaRoom GBeliefs!
	 * 
	 * @param gb - the GroundedBelief
	 * @return most likely category_id (for rooms) or label_id (for visual objects) or empty String if n/a
	 */
	public ReferentReplacer getGBeliefCategoryReplacer(GroundedBelief gb) {
//		log("getGBeliefCategoryReplacer called");
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbProxy = CASTIndependentFormulaDistributionsBelief
		.create(GroundedBelief.class, gb);
		
		ReferentReplacer myRefRep = null;		

		if (gbProxy.getType().equals(
				SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(ComaRoom.class))) {
			String cat = "";
			try {
				cat = gbProxy.getContent().get(
					ComaRoomTransferFunction.CATEGORY_ID)
					.getDistribution().getMostLikely().getProposition();
			} catch (AssertionError ae) {
				logException(ae);
			}
			if (cat==null || cat.equals("")) cat = "room";
			// for PEV!!! 2012-06-25 (hz)
			cat = "room";
			int roomID = gbProxy.getContent().get(
					ComaRoomTransferFunction.ROOM_ID).
					getDistribution().getMostLikely().getInteger();
			// if (placeID==0) placeID = 10; // TODO temporary fix for out of vocab word 'zeroth'!
			final String placeIDF = new Integer(roomID).toString();
			final String catF = cat;
			
//			log("Gbelief is a room with category: " + catF);
			
			myRefRep = new ReferentReplacer() {
	            @Override
	            public BasicLogicalForm.Builder doWork(String nom, BasicState s, BasicLogicalForm.Builder lfBuilder) {

	                String depStateNom = BasicPatterns.uniqueNominal(lfBuilder);

	                BasicState depState = BasicState.newBuilder("number-id") //"number-ordinal")
	                        .setProposition(placeIDF)
	                        .build();
	                
	                BasicState headState = BasicState.newBuilder("e-place")
	                        .setProposition(catF)
	                        .addFeature("Delimitation", "unique")
	                        .addFeature("Quantification", "specific")
	                        .addFeature("Num", "sg")
	                        .addRelation("Modifier", depStateNom)
	                        .build();

	                return lfBuilder.addState(depStateNom, depState).updateState(nom, headState);
	            }
	        };
	        return myRefRep;
		} else if (gbProxy.getType().equals(
				SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(VisualObject.class))) {
			String cat = gbProxy.getContent().get("label")
			//VisualObjectTransferFunction.LABEL_ID)
			.getDistribution().getMostLikely().getProposition();
			if (cat==null || cat.equals("")) cat = "object";
			final String catF = cat;
			
//			log("Gbelief is a visual object with category: " + catF);
			
			myRefRep = new ReferentReplacer() {
	            @Override
	            public BasicLogicalForm.Builder doWork(String nom, BasicState s, BasicLogicalForm.Builder lfBuilder) {

	                BasicState headState = BasicState.newBuilder("e-place")
	                        .setProposition(catF)
	                        .addFeature("Delimitation", "unique")
	                        .addFeature("Quantification", "specific")
	                        .addFeature("Num", "sg")
	                        .build();

	                return lfBuilder.updateState(nom, headState);
	            }
	        };
	        return myRefRep;
		} else if (gbProxy.getType().equals(
				SimpleDiscreteTransferFunction
				.getBeliefTypeFromCastType(Place.class))) {
			int placeID = gbProxy.getContent().get(
					PlaceTransferFunction.PLACE_ID_ID).
					getDistribution().getMostLikely().getInteger();
			// if (placeID==0) placeID = 10; // TODO temporary fix for out of vocab word 'zeroth'!
			final String placeIDF = new Integer(placeID).toString();
			String placestatus = gbProxy.getContent().get(PlaceTransferFunction.PLACE_STATUS_ID)
					.getDistribution().getMostLikely().getProposition();
			if (placestatus.equals(PlaceStatus.PLACEHOLDER.toString())) placestatus = "placeholder";
			else placestatus = "place";
			final String placeStatusF = placestatus;
			
//			log("Gbelief is a place with place ID: " + placeIDF);
			
			myRefRep = new ReferentReplacer() {
	            @Override
	            public BasicLogicalForm.Builder doWork(String nom, BasicState s, BasicLogicalForm.Builder lfBuilder) {

	                String depStateNom = BasicPatterns.uniqueNominal(lfBuilder);

	                BasicState depState = BasicState.newBuilder("number-id") //"number-ordinal")
	                        .setProposition(placeIDF)
	                        .build();

	                BasicState headState = BasicState.newBuilder("e-place")
	                		.setProposition(placeStatusF) 
	                		.addFeature("Delimitation", "unique")
	                		.addFeature("Quantification", "specific")
	                		.addFeature("Num", "sg")
	                		.addRelation("Modifier", depStateNom)
	                			.build();

	                return lfBuilder.addState(depStateNom, depState).updateState(nom, headState);
	            }
	        };
	        
	        return myRefRep;
			
		} else {
//			log("GBelief is of type: " + gbProxy.getType());
		}
		return null;
	}
	
	/**
//	 * Returns the most likely (according to its probability distribution) 
//	 * category of the entity the grounded belief is about. 
//	 * Currently only handles VisualObject and ComaRoom GBeliefs!
//	 * 
//	 * @param gb - the GroundedBelief
//	 * @return most likely category_id (for rooms) or label_id (for visual objects) or empty String if n/a
	 */
	public ReferentReplacer getDoesNotExistReplacer(WMAddress _lfWMA) {
		log("getDoesNotExistReplacer called for " + _lfWMA);
				
		ReferentReplacer myRefRep = null;	

		if (_lfWMA.subarchitecture().equals("spatial.sa")) {
			if (_lfWMA.id().split(":").length==2) {
				log(_lfWMA + " looks like a well-formed place WMA, which just happens to have been deleted earlier...");
				myRefRep = new ReferentReplacer() {
					@Override
					public BasicLogicalForm.Builder doWork(String nom, BasicState s, BasicLogicalForm.Builder lfBuilder) {

						BasicState headState = BasicState.newBuilder("e-place")
								.setProposition("place")
								.addFeature("Delimitation", "existential")
								.addFeature("Quantification", "specific")
								.addFeature("Num", "sg")
								.build();

						return lfBuilder.updateState(nom, headState);
					}
				};
				return myRefRep;
			} // end if well-formed WMA
			else {
				// assuming it is a ROOM0@spatial.sa type
				log(_lfWMA + " looks like a dummy-object or object category (buggy) WMA...");
//				@book1_0:thing(book ^ 
//		                 <Delimitation>existential ^ 
//		                 <Num>sg ^ 
//		                 <Quantification>specific)
				final String catF;
				final String builderTypeF;
				if (_lfWMA.id().toLowerCase().startsWith("room") || _lfWMA.subarchitecture().contains("coma")) {
					catF = "room";
					builderTypeF = "e-place";
				} else {
					catF = _lfWMA.id().toLowerCase();
					builderTypeF = "thing";
				}
				myRefRep = new ReferentReplacer() {
					@Override
					public BasicLogicalForm.Builder doWork(String nom, BasicState s, BasicLogicalForm.Builder lfBuilder) {

						BasicState headState = BasicState.newBuilder(builderTypeF)
								.setProposition(catF)
								.addFeature("Delimitation", "unique")
								.addFeature("Quantification", "specific")
								.addFeature("Num", "sg")
								.build();

						return lfBuilder.updateState(nom, headState);
					}
				};
				return myRefRep;
			} // end else (i.e. spatial.sa but not well-formed WMA id
		} // end if spatial.sa WMA
		else {
			// "1_1_6,room3":castreferent ^ "1_1_6,room3"
			log(_lfWMA + " looks like a hypothetical room");
			final String catF;
			final String builderTypeF;
			if (_lfWMA.toString().split(",")[1].toLowerCase().startsWith("room")) {
				catF = "room";
				builderTypeF = "e-place";
			} else {
				catF = "thing";
				builderTypeF = "thing";
			}
			myRefRep = new ReferentReplacer() {
				@Override
				public BasicLogicalForm.Builder doWork(String nom, BasicState s, BasicLogicalForm.Builder lfBuilder) {

					BasicState headState = BasicState.newBuilder(builderTypeF)
							.setProposition(catF)
							.addFeature("Delimitation", "unique")
							.addFeature("Quantification", "specific")
							.addFeature("Num", "sg")
							.build();

					return lfBuilder.updateState(nom, headState);
				}
			};
			return myRefRep;
		}
		//log("Don't know what to do with WMA: " + _lfWMA);
		//return null;
	}
	
	/** 
	 * determines if the CAST referent in the GroundedBelief gb is the robot itself.
	 * @param gb
	 * @return true if the type of the proxy of gb is "Robot"
	 */
	private boolean isRobot(GroundedBelief gb) {
		CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbProxy = CASTIndependentFormulaDistributionsBelief
				.create(GroundedBelief.class, gb);
		return (gbProxy.getType().equals("Robot"));
			
	}
	
	
	private void log(String s) {
		if (this.m_castComponent!=null) m_castComponent.log(s);
		else System.out.println(s);
	}

	private void logException(Throwable e) {
		if (m_castComponent!=null) m_castComponent.logException(e);
		else System.out.println(e.getLocalizedMessage());
	}
	
	public static void main(String[] args) {
		File f = new File(args[7]);
		
		
		PlanVerbalizer test;
		
	
		try {
			test = new PlanVerbalizer(args[0], args[1], args[2], args[3], args[4], Integer.parseInt(args[5]), args[6]);
			test.debug_lf_out = true;
			//System.out.println(test.aggregateStrings("my task was to search for a magazine and to find it.\nI assumed the room zero was a meetingroom.\nI moved to the placeholder one from the place zero.\nbecause I wanted to go to the placeholder two via the place zero from the placeholder one.\nI wanted to go to the placeholder three from the placeholder two via the place zero.\nI wanted to move from the placeholder three to the place zero.\nI wanted to create viewcones in the room zero at the place zero.\nI wanted to search for a magazine in the room zero at the place zero.\nthen I assumed the placeholder four was in a meetingroom.\nI moved from the place one to the placeholder four.\nbecause I wanted to create viewcones in a room at the placeholder four.\nI wanted to look for a magazine in some room at the placeholder four.\nafter that I assumed the room zero was a meetingroom.\nI went to the placeholder two via the place one from the place four.\nbecause I wanted to move from the placeholder two to the place one.\nI wanted to go to the placeholder three from the place one via the place zero.\nI wanted to move from the placeholder three to the place zero.\nI wanted to create viewcones in the room zero at the place zero.\nI wanted to search for a magazine in the room zero at the place zero.\nafterwards I assumed the room zero was a meetingroom.\nI moved from the place two to the placeholder three.\nbecause I wanted to go to the placeholder six from the placeholder three via the place two.\nI wanted to go to the placeholder seven from the placeholder six via the place two.\nI wanted to move from the placeholder seven to the place two.\nI wanted to create viewcones in the room zero at the place two.\nI wanted to search for a magazine in the room zero at the place two.\nafter that I assumed the room zero was a meetingroom.\nI moved to the placeholder seven from the place three.\nbecause I wanted to go to the placeholder eight from the placeholder seven via the place three.\nI wanted to move from the placeholder eight to the place three.\nI wanted to go to the placeholder six via the place two from the place three.\nI wanted to move from the placeholder six to the place two.\nI wanted to create viewcones in the room zero at the place two.\nI wanted to search for a magazine in the room zero at the place two.\nthen I assumed the room zero was a meetingroom.\nI moved from the place seven to the placeholder nine.\nI moved to the place seven from the placeholder nine.\nI went to the placeholder eight from the place seven via the place three.\nI moved from the placeholder eight to the place three.\nbecause I wanted to create viewcones in the room zero at the place three.\nI wanted to search for a magazine in the room zero at the place three.\nafterwards I assumed a magazine was in the room zero.\nbecause I wanted to search for a magazine in the room zero at the place three.\nthen I assumed a magazine was in the room zero.\nI didn't find a magazine in the room zero at the place three.\n"));
			
			//if ("".toLowerCase().equals("")) return;
			
			
			System.out.println(test.verbalizeHistory(new PDDLHistory(f, Integer.parseInt(args[8]))));
		} catch (NumberFormatException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

//	public static void main(String[] args) throws FileNotFoundException, IOException, BuildException, ParseException, NoAnnotationFoundException, UnknownOperatorException {
//
//        // for testing:
//        // this LF should always be realisable!
//        String s = "@{event_1:action-non-motion}(take ^ <Mood>imp ^ <Actor>(agent_1:entity ^ addressee) ^ <Patient>(thing_1:thing ^ mug ^ <Num>sg ^ <Delimitation>unique ^ <Quantification>specific) ^ <Subject>(agent_1:entity))";
//        String s2 = "@{e_0:action-motion}(move ^ <Mood>ind ^ <Tense>past ^ <Actor>(i_0:person ^ I ^ <Num>sg) ^ <Modifier>(from_0:m-wherefrom ^ from ^ <Anchor>(kitchen_0:e-place ^ kitchen ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific)) ^ <Modifier>(to_0:m-whereto ^ to ^ <Anchor>(livingroom_0:e-place ^ livingroom ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific)) ^ <Subject>(i_0:person))";
//        String s3 = "@{e_0:action-non-motion}(make ^ <Mood>ind ^ <Tense>pres ^ <Actor>(i_0:person ^ I ^ <Num>sg) ^ <Patient>(place_0:e-place ^ place ^ <Delimitation>variable ^ <Quantification>unspecific ^ <Modifier>(in_0:whereto ^ in ^ <Anchor>(room_0:e-place ^ room ^ <Delimination>unique ^ <Num>sg ^ <Quantification>specific)) ^ <Modifier>(at_0:m-location ^ at ^ <Anchor>(place2_0:e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific))) ^ <Subject>(i_0:person))";
//        String s4 = "@{e:action-motion}(move ^ <ExecutionStatus>EXECUTED ^ <Actor>(robot_0__f:castreferent ^ robot_0__f) ^ <Modifier>(from_1:m-wherefrom ^ from ^ <Anchor>(place_0__e:castreferent ^ place_0__e)) ^ <Modifier>(to_1:m-whereto ^ to ^ <Anchor>(place_1__e:castreferent ^ place_1__e)))";
//
//        String exampleMove = "@{step_0_0:action-motion}(move ^ <ExecutionStatus>SUCCEEDED ^ <Actor>(\"0:D@spatial.sa\":castreferent ^ \"0:D@spatial.sa\") ^ <Modifier>(from_1:m-wherefrom ^ from ^ <Anchor>(\"0:C@spatial.sa\":castreferent ^ \"0:C@spatial.sa\")) ^ <Modifier>(to_1:m-whereto ^ to ^ <Anchor>(place_1__c:castreferent ^ place_1__c)))";
//        exampleMove = exampleMove.replace("<ExecutionStatus>SUCCEEDED", "<Mood>ind ^ <Tense>past");
//        // dangerous: adding a new state!
//        exampleMove = exampleMove.replace("<Actor>(\"0:D@spatial.sa\":castreferent ^ \"0:D@spatial.sa\")", "<Actor>(\"0:D@spatial.sa\":person ^ I ^ <Num>sg) ^ <Subject>(\"0:D@spatial.sa\":person) ");
//        exampleMove = exampleMove.replace("\"0:C@spatial.sa\":castreferent ^ \"0:C@spatial.sa\"", "\"0:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n2_0:number-ordinal ^ 2)");
//        exampleMove = exampleMove.replace("<Anchor>(place_1__c:castreferent ^ place_1__c)", "<Anchor>(place_1__c:e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n3_0:number-ordinal ^ 3))");
//
//        String targetMove = "@{step_0_0:action-motion}(move ^ <Mood>ind ^ <Tense>past ^ <Actor>(\"0:D@spatial.sa\":person ^ I ^ <Num>sg) ^ <Modifier>(from_1:m-wherefrom ^ from ^ <Anchor>(\"0:C@spatial.sa\":e-place ^ place ^ <Num>sg ^ <Delimitation>unique ^ <Quantification>specific ^ <Modifier>(n2_0:number-ordinal ^ 2))) ^ <Modifier>(to_1:m-whereto ^ to ^ <Anchor>(place_1__c:e-place ^ place ^ <Num>sg ^ <Delimitation>unique ^ <Quantification>specific ^ <Modifier>(n3_0:number-ordinal ^ 3))) ^ <Subject>(\"0:D@spatial.sa\":person))";
//
//        String targetConnected = "@{state_0_4:event}(context ^ <Mood>ind ^ <Tense>past ^ <Subject>(\"2:C@spatial.sa\":e-place ^ place ^ <Num>sg ^ <Delimitation>unique ^ <Quantification>specific ^ <Modifier>(n2_0:number-ordinal ^ 2)) ^ <Modifier>(next1_0:m-whereto ^ next ^ <Anchor>(\"3:C@spatial.sa\":e-place ^ place ^ <Num>sg ^ <Delimitation>unique ^ <Quantification>specific ^ <Modifier>(n3_0:number-ordinal ^ 3))))";
//        // event, context, <Modifier>(next1_0:m-whereto ^ next ^, <Subject>
//
//        // realizeAndSysout(fillProtoSlots(targetConnected), realiser);
//
//        // "@{state_0_4:event}(context ^ <Subject>(\"5:C@spatial.sa\":e-place ^ place ^ <Num>sg ^ <Delimitation>unique ^ <Quantification>specific ^ <Modifier>(n5_0:number-ordinal ^ 5)) ^ <Modifier>(next1:m-whereto ^ next ^ <Anchor>(\"6:C@spatial.sa\":e-place ^ place ^ <Num>sg ^ <Delimitation>unique ^ <Quantification>specific ^ <Modifier>(n6_0:number-ordinal ^ 6))))"
//
//        // if (1==1) return;
//
//		List<POPlan> historyBlocks = history.getEpisodes();
//        int blockNumber = 0;
//		for (POPlan poplan : historyBlocks) {
//            blockNumber++;
//            List<Message> messages = contentDeterminator.determineMessages(poplan);
//            for (Message msg : messages) {
//                if (msg instanceof ProtoLFMessage) {
//                    realizeAndSysout(fillProtoSlots(finalizeProtoLF(((ProtoLFMessage) msg).getProtoLF()).toString()), realiser);
//                } else {
//                    if (msg instanceof StringMessage) {
//                        System.out.println(((StringMessage) msg).getText());
//                    }
//                }
//            }
//        }
//	}


}
