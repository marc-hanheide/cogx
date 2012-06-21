package de.dfki.lt.tr.cast.dialogue.planverb;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

import comadata.ComaRoom;

import SpatialData.Place;
import VisionData.VisualObject;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.WorkingMemoryReaderComponent;
import cast.cdl.WorkingMemoryAddress;

import scala.collection.immutable.Set;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.cast.dialogue.GBeliefMemory;
import de.dfki.lt.tr.cast.dialogue.realisation.LFRealiser;
import de.dfki.lt.tr.cast.dialogue.realisation.RealisationClient;
import de.dfki.lt.tr.cast.dialogue.realisation.TarotCCGRealiser;
import de.dfki.lt.tr.planverb.domain.NoAnnotationFoundException;
import de.dfki.lt.tr.planverb.domain.UnknownOperatorException;
import de.dfki.lt.tr.planverb.generation.Message;
import de.dfki.lt.tr.planverb.generation.ProtoLFMessage;
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
	private WorkingMemoryReaderComponent m_castComponent;
	
	private HashMap<String, String> m_preLexicalSub  = new HashMap<String, String>();
	private HashMap<String, String> m_postLexicalSub = new HashMap<String, String>();
	
	private final static String DEFAULTNGRAMFILE   = "subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/ngram-corpus.txt";
	private final static String DEFAULTGRAMMARFILE = "subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/grammar.xml";
	
	public GBeliefMemory m_gbmemory = new GBeliefMemory();
	

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
		m_castComponent = component;

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
	
	private GroundedBelief getGBelief(WMAddress referentWMA) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		if (m_castComponent != null) {
			GroundedBelief gbWME = m_castComponent.getMemoryEntry(new WorkingMemoryAddress(referentWMA.id(), referentWMA.subarchitecture()), GroundedBelief.class);
			return gbWME;
		} else {
			GroundedBelief gbWME = m_gbmemory.getLastValidGBelief(new WorkingMemoryAddress(referentWMA.id(), referentWMA.subarchitecture()));
			return gbWME;
		}
		
	}
	
    private void readFromFile(File gbmemoryFile) {
		
		try {
			FileInputStream f = new FileInputStream(gbmemoryFile);
			ObjectInputStream s = new ObjectInputStream(f);
			m_gbmemory = (GBeliefMemory) s.readObject();
			s.close();
			
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	     catch (ClassNotFoundException e) {
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
		m_preLexicalSub. put("ascription ^ be", "ascription ^ be ^ <Mood>ind ^ <Tense>pres");
		//m_preLexicalSub. put("<ExecutionStatus>PENDING", "<Mood>ind ^ <Tense>fut ^ <Modifier>(will1_0:modal ^ will)");
		
		m_preLexicalSub.put("<ExecutionStatus>FAILED", "<Mood>ind ^ <Polarity>neg ^ <Modifier>(could1_0:modal ^ could)");
	}
	
	public String verbalizeHistory(final List<POPlan> hlist) {
		log("entering verbalizeHistory()");
		History h = new PDDLHistory(hlist);
		return verbalizeHistory(h);
	}
	
	public String verbalizeHistory(History h) {
	  log("entering verbalizeHistory()");
		
		// this does not work yet!
		List<Message> messages = m_contentDeterminator.determineMessages(h);	
		if (messages!=null) {
			log("contentDeterminator returned " + messages.size() + " messages for the full History.");
		} else {
			// therefore process each POPlan individually:
			log("determineMessages(History) doesn't work yet. Verbalizing each POPlan individually instead.");

			messages = new ArrayList<Message>();
			int _currPOPlanBlock = 0;
			for (POPlan poPlan : (List<POPlan>) h.getEpisodes()) {
				try {
					messages.addAll(m_contentDeterminator.determineMessages(poPlan, _currPOPlanBlock));
				} catch (BuildException e) {
					logException(e);
				} catch (ParseException e) {
					logException(e);
				} catch (NoAnnotationFoundException e) {
					logException(e);
				} catch (UnknownOperatorException e) {
					logException(e);
				}
				_currPOPlanBlock+=1;
			}

			log("contentDeterminator returned " + messages.size() + " messages.");
		}
		return realizeMessages(messages);
	}
	
	
	/**
	 * The main interface method. Generates a verbal report for a given partially ordered plan (POPlan).
	 * 
	 * @param poplan
	 * @return a string with the full verbal report of the POPlan
	 */
	public String verbalizePOPlan(POPlan poplan) {
        // call the content determinator
		List<Message> messages = null;
		try {
			messages = m_contentDeterminator.determineMessages(poplan, 1);
		} catch (BuildException e) {
			logException(e);
		} catch (ParseException e) {
			logException(e);
		} catch (NoAnnotationFoundException e) {
			logException(e);
		} catch (UnknownOperatorException e) {
			logException(e);
		}
		// if errors prevented the creation of a message list, report it...
		if (messages==null) return "I am sorry. I don't know what to say about this Pee Oh Plan.";
		else log("verbalizePOPlan() determined " + messages.size() + " messages for POPlan.");

		return realizeMessages(messages);
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
				// perform lexical substitution before realization
				try {
					protoLF = preProcessLexiconSubstitution(protoLF);
					log_sb.append("\n lexical substitution before realization yielded: \n" + protoLF.toString());
				} catch (BuildException e) {
					logException(e);
				} catch (ParseException e) {
					logException(e);
				}

				// do GRE 
				protoLF = doGRE(protoLF);
				log_sb.append("\n doGRE() yielded: \n" + protoLF.toString());

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
					output_sb.append(outputText + ". \n");
				} else {
					log_sb.append("\n not appending any output text.");        			
				}
			} else if (msg instanceof StringMessage) {
				String outputText = ((StringMessage) msg).getText();
				log_sb.append("\n appending current Message, which is a StringMessage: \n " + outputText);

				output_sb.append(outputText + ". \n");
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
    		try {
    			//GroundedBelief gbWME = m_castComponent.getMemoryEntry(new WorkingMemoryAddress(referentWMA.id(), referentWMA.subarchitecture()), GroundedBelief.class);
    			GroundedBelief gbWME = getGBelief(referentWMA);
    			
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
                return builder.addFeature("Mood", "ind").addFeature("Polarity", "neg"); 
            }
        };        
        protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "FAILED", failedPastTenseReplacer);

        
        // execution status = pending will be removed and the overall LF will be put into a control verb
//        if (protoLF.toString().contains("<ExecutionStatus>PENDING")) {
//            FeatureReplacer pendingReplacer = new FeatureReplacer() {
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
			String cat = gbProxy.getContent().get(
					ComaRoomTransferFunction.CATEGORY_ID)
					.getDistribution().getMostLikely().getProposition();
			if (cat==null || cat.equals("")) cat = "room";
			final String catF = cat;
			
//			log("Gbelief is a room with category: " + catF);
			
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
			
//			log("Gbelief is a place with place ID: " + placeIDF);
			
			myRefRep = new ReferentReplacer() {
	            @Override
	            public BasicLogicalForm.Builder doWork(String nom, BasicState s, BasicLogicalForm.Builder lfBuilder) {

	                String depStateNom = BasicPatterns.uniqueNominal(lfBuilder);

	                BasicState depState = BasicState.newBuilder("number-id") //"number-ordinal")
	                        .setProposition(placeIDF)
	                        .build();
	                
	                BasicState headState = BasicState.newBuilder("e-place")
	                        .setProposition("place")
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
				if (_lfWMA.id().toLowerCase().startsWith("room")) {
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
								.addFeature("Delimitation", "existential")
								.addFeature("Quantification", "specific")
								.addFeature("Num", "sg")
								.build();

						return lfBuilder.updateState(nom, headState);
					}
				};
				return myRefRep;
			} // end else (i.e. spatial.sa but not well-formed WMA id
		} // end if spatial.sa WMA
		log("Don't know what to do with WMA: " + _lfWMA);
		return null;
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
			System.out.println(test.verbalizeHistory(new PDDLHistory(f)));
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
