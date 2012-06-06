package de.dfki.lt.tr.cast.dialogue.planverb;

import java.io.File;
import java.io.IOException;
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

import opennlp.ccg.grammar.Grammar;
import opennlp.ccg.synsem.SignScorer;
import scala.Option;
import scala.collection.immutable.Set;

import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.planverb.domain.NoAnnotationFoundException;
import de.dfki.lt.tr.planverb.domain.UnknownOperatorException;
import de.dfki.lt.tr.planverb.generation.Message;
import de.dfki.lt.tr.planverb.generation.ProtoLFMessage;
import de.dfki.lt.tr.planverb.generation.StringMessage;
import de.dfki.lt.tr.planverb.planning.pddl.PDDLContentDeterminator;
import de.dfki.lt.tr.planverb.planning.pddl.PDDLDomainModel;
import de.dfki.lt.tr.planverb.planning.pddl.POPlan;
import de.dfki.lt.tr.planverb.history.Episode;
import de.dfki.lt.tr.planverb.history.History;
import de.dfki.tarot.cogx.CASTLogicalForms;
import de.dfki.tarot.cogx.CogXJavaHelpers;
import de.dfki.tarot.cogx.FeatureReplacer;
import de.dfki.tarot.cogx.ReferentReplacer;
import de.dfki.tarot.cogx.WMAddress;
import de.dfki.tarot.nlp.lf.BasicLogicalForm;
import de.dfki.tarot.nlp.lf.BasicState;
import de.dfki.tarot.nlp.lf.BasicUtils;
import de.dfki.tarot.nlp.realisation.ccg.CCGRealiser;
import de.dfki.tarot.nlp.realisation.openccg.NgramPrecisionModelFactory;
import de.dfki.tarot.util.BuildException;
import de.dfki.tarot.util.ParseException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

import static scala.collection.JavaConversions.*;

public class PlanVerbalizer {
	
	private final PDDLContentDeterminator m_contentDeterminator;
	private final CCGRealiser m_realiser;
	private final WorkingMemoryReaderComponent m_castComponent;
	
	private HashMap<String, String> m_preLexicalSub  = new HashMap<String, String>();
	private HashMap<String, String> m_postLexicalSub = new HashMap<String, String>();
	
	/**
	 * Creates and initializes a new PlanVerbalizer object.
	 * After initialization, it can be queried subsequently for verbalizations of event structures.
	 * The domains and grammar etc. it operates on remain static.
	 * @param annotatedDomainFile
	 * @param pddlDomainFile
	 * @param grammarFile
	 * @throws IOException
	 */
	public PlanVerbalizer(String annotatedDomainFile, String pddlDomainFile,  String grammarFile, WorkingMemoryReaderComponent component) throws IOException {
		// init CAST component for WM access and logging
		m_castComponent = component;

		m_castComponent.log("PlanVerbalizer constructor called with annotatedDomainFile = " + annotatedDomainFile +
				" pddlDomainFile = " + pddlDomainFile + " grammarFile = " + grammarFile);
		
//		static String defaultDomainFile = "./resources/domain2test.pddl";
//		static String defaultDomainDictionaryFile = "./resources/dora-interactive_annotated.txt";
//	    static String defaultGrammarPath = "/de/dfki/tarot/nlp/resources/moloko/grammar.xml";
		
		// initialize planner-related stuff
		File adf = new File(annotatedDomainFile);
		File pddf = new File(pddlDomainFile);
		PDDLDomainModel domainModel = new PDDLDomainModel(adf, pddf);
		m_contentDeterminator = new PDDLContentDeterminator(domainModel);

		// initialize grammar-related stuff
		Grammar grammar = new Grammar(grammarFile);
				//new Object().getClass().getResource(grammarFile));
		
		SignScorer scorer = NgramPrecisionModelFactory.fromURL(new File("./subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/ngram-corpus.txt").toURL(), "UTF-8");
				
			//	fromURL(getClass.getResource("/de/dfki/tarot/nlp/realisation/openccg/test-corpus.txt"), "UTF-8"));
		m_realiser = new CCGRealiser(grammar, scorer);
				
		
		// initialize lexicon substitutions for the time being...
		initLexicalSubstitutions();
		m_castComponent.log("finished PlanVerbalizer constructor");
	}
	
	/**
	 * add missing lexical items to this method!
	 * caution: please add blanks before and after each word!
	 */
	private void initLexicalSubstitutions() {
		m_preLexicalSub. put(" meetingroom ", " lab ");
		m_postLexicalSub.put("lab", "meetingroom");
		
		m_preLexicalSub. put(" corridor ", " hall ");
		m_postLexicalSub.put("hall", "corridor");
		
		m_preLexicalSub. put(" cerealbox ", " box ");
		m_postLexicalSub.put("box", "cerealbox");
		
		m_preLexicalSub. put("create ", "make ");

		m_preLexicalSub. put(" magazine ", " book ");
		m_postLexicalSub.put("book", "magazine");

		m_preLexicalSub. put("m-location ^ via ", "m-through ^ through");
        // TODO find a solution for indirect speech
		m_preLexicalSub. put("ascription ^ be", "ascription ^ be ^ <Mood>ind ^ <Tense>pres");
		m_preLexicalSub. put("<ExecutionStatus>PENDING", "<Mood>ind ^ <Tense>fut ^ <Modifier>(will1_0:modal ^ will)");
		
		m_preLexicalSub. put("<ExecutionStatus>FAILED", "<Mood>ind ^ <Tense>past ^ <Polarity>neg ^ <Modifier>(could1_0:modal ^ could)");
	}
	
	public String verbalizeHistory(final List<de.dfki.lt.tr.planverb.planning.pddl.POPlan> hlist) {
		
		History h = new History() {
			
			// ??? String goal = "";
			@Override
			public boolean hasSharedGoal() {
				return true;
			}
			
			@Override
			public List<de.dfki.lt.tr.planverb.planning.pddl.POPlan> getEpisodes() {
				return hlist;
			}
		};
		
		// this does not work yet!
		// List<Message> messages = m_contentDeterminator.determineMessages(h);
		// therefore process each POPlan individually:
		
		List<Message> messages = new ArrayList<Message>();
		for (POPlan poPlan : hlist) {
			try {
				messages.addAll(m_contentDeterminator.determineMessages(poPlan));
			} catch (BuildException e) {
				m_castComponent.logException(e);
			} catch (ParseException e) {
				m_castComponent.logException(e);
			} catch (NoAnnotationFoundException e) {
				m_castComponent.logException(e);
			} catch (UnknownOperatorException e) {
				m_castComponent.logException(e);
			}
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
			messages = m_contentDeterminator.determineMessages(poplan);
		} catch (BuildException e) {
			m_castComponent.logException(e);
		} catch (ParseException e) {
			m_castComponent.logException(e);
		} catch (NoAnnotationFoundException e) {
			m_castComponent.logException(e);
		} catch (UnknownOperatorException e) {
			m_castComponent.logException(e);
		}
		// if errors prevented the creation of a message list, report it...
		if (messages==null) return "I am sorry. I don't know what to say about this Pee Oh Plan.";
		else m_castComponent.log("verbalizePOPlan() determined " + messages.size() + " messages for POPlan.");

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
					m_castComponent.log(log_sb);
					return "";
				}
				// perform lexical substitution before realization
				try {
					protoLF = preProcessLexiconSubstitution(protoLF);
					log_sb.append("\n lexical substitution before realization yielded: \n" + protoLF.toString());
				} catch (BuildException e) {
					m_castComponent.logException(e);
				} catch (ParseException e) {
					m_castComponent.logException(e);
				}

				// do GRE 
				protoLF = doGRE(protoLF);
				log_sb.append("\n doGRE() yielded: \n" + protoLF.toString());

				// make missing parts consistent (e.g. subj agreement)
				BasicLogicalForm finalLF = finalizeProtoLF(protoLF);
				log_sb.append("\n finalizeProtoLF() yielded: \n" + finalLF.toString());

				// surface realization
				// perform lexical re-substitution after realization, before appending to the verbal report
				String realization = realizeLF(finalLF);
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
    	m_castComponent.log(log_sb);
    	return output_sb.toString();
	}
	

	/**
	 * This method realizes a given BasicLogicalForm into a surface String 
	 * as defined by the grammar realizer. 
	 * 
	 * @param blf
	 * @return a natural language surface realization of the blf
	 */
    private String realizeLF(BasicLogicalForm blf) {
    	//if (blf.toString().contains("PENDING")) {
    	//	return "not verbalizing pending steps";
    	//}
    	
    	StringBuilder outputBldr = new StringBuilder(); 

    	// realize thru the CCG Realizer
    	Option<scala.collection.immutable.List<String>> result = m_realiser.bestRealisationFor(blf);
    	if (result.isDefined()) {
    		List<String> words =  seqAsJavaList(result.get());
    		for (String w: words) {
    			outputBldr.append(w + " ");
    		}
    	} else {
    		if (blf.toString().contains("assume")) {
    			m_castComponent.log("not verbalising un-realizable assumption");
    			// outputBldr.append("I made another assumption");
    		} else {
    			m_castComponent.log("NO REALISATION FOUND for LF: " + blf.toString());
    		}
    	}
    	
    	return outputBldr.toString();
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
    			GroundedBelief gbWME = m_castComponent.getMemoryEntry(new WorkingMemoryAddress(referentWMA.id(), referentWMA.subarchitecture()), GroundedBelief.class);
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
				m_castComponent.logException(e);
			}
    	}
    	log_sb.append("\n *** doGRE returns protoLF: *** \n " + (protoLF==null ? "null" : protoLF.toString()));
    	m_castComponent.log(log_sb);
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
        FeatureReplacer successReplacer = new FeatureReplacer() {
            @Override
            public BasicState.Builder doWork(BasicState.Builder builder) {
                return builder.addFeature("Mood", "ind").addFeature("Tense", "past");
            }
        };
        protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "SUCCEEDED", successReplacer);

        // execution status = pending yields future tense report
        FeatureReplacer pendingReplacer = new FeatureReplacer() {
            @Override
            public BasicState.Builder doWork(BasicState.Builder builder) {
            	
                return builder.addFeature("Mood", "ind").addFeature("Tense", "fut");
            }
        };
        protoLF = CogXJavaHelpers.replaceFeature(protoLF, "ExecutionStatus", "PENDING", pendingReplacer);
        
        // make sure there is an appropriate subject
        // TODO cop-restr as subject!
        protoLF = CogXJavaHelpers.ensureSubjectFilled(protoLF);

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
//		m_castComponent.log("getGBeliefCategoryReplacer called");
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
			
//			m_castComponent.log("Gbelief is a room with category: " + catF);
			
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
			
//			m_castComponent.log("Gbelief is a visual object with category: " + catF);
			
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
			
//			m_castComponent.log("Gbelief is a place with place ID: " + placeIDF);
			
			myRefRep = new ReferentReplacer() {
	            @Override
	            public BasicLogicalForm.Builder doWork(String nom, BasicState s, BasicLogicalForm.Builder lfBuilder) {

	                String depStateNom = BasicUtils.uniqueNominal(lfBuilder);

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
//			m_castComponent.log("GBelief is of type: " + gbProxy.getType());
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
		m_castComponent.log("getDoesNotExistReplacer called for " + _lfWMA);
				
		ReferentReplacer myRefRep = null;		

		if (_lfWMA.subarchitecture().equals("spatial.sa")) {
			if (_lfWMA.id().split(":").length==2) {
				m_castComponent.log(_lfWMA + " looks like a well-formed place WMA, which just happens to have been deleted earlier...");
				//				@place1_0:e-place(place ^ 
				//	                    <Delimitation> ^ 
				//	                    <Num>sg ^ 
				//	                    <Quantification>specific ^ 
				//	                    <Modifier>(away1_0:m-direction ^ away) ^ 
				//	                    <Scope-in>(go1_0:action-motion ^ go ^ 
				//	                               <Tense>past ^ 
				//	                               <Actor>place1_0:e-place ^ 
				//	                               <Subject>place1_0:e-place))
				myRefRep = new ReferentReplacer() {
					@Override
					public BasicLogicalForm.Builder doWork(String nom, BasicState s, BasicLogicalForm.Builder lfBuilder) {

//						String depModStateNom = BasicUtils.uniqueNominal(lfBuilder);
//						BasicState depModState = BasicState.newBuilder("m-direction")
//								.setProposition("away")
//								.build();
//
//						String depScopStateNom = BasicUtils.uniqueNominal(lfBuilder);
//						BasicState depScopState = BasicState.newBuilder("action-motion")
//								.setProposition("go")
//								.addFeature("Tense", "past")
//								.addRelation("Actor", nom)
//								.addRelation("Subject", nom)
//								.build();

						BasicState headState = BasicState.newBuilder("e-place")
								.setProposition("place")
								.addFeature("Delimitation", "existential")
								.addFeature("Quantification", "specific")
								.addFeature("Num", "sg")
//								.addRelation("Modifier", depModStateNom)
//								.addRelation("Scope-in", depScopStateNom)
								.build();

//						return lfBuilder.addState(depModStateNom, depModState).addState(depScopStateNom, depScopState).updateState(nom, headState);
						return lfBuilder.updateState(nom, headState);
					}
				};
				return myRefRep;
			} // end if well-formed WMA
			else {
				// assuming it is a ROOM0@spatial.sa type
				m_castComponent.log(_lfWMA + " looks like a dummy-object or object category (buggy) WMA...");
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
		m_castComponent.log("Don't know what to do with WMA: " + _lfWMA);
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
	
	
//	private static String fillProtoSlots(String logicalform) {
//        // room 0:71 = this room
//        // GRE
//        logicalform = logicalform.replace("\"0:71@coma\":castreferent ^ \"0:71@coma\"", "room_0_71:e-place ^ room ^ <Delimitation>unique ^ <Num>sg ^ <Proximity>proximal ^ <Quantification>specific");
//        logicalform = logicalform.replace("room_0_71:castreferent ^ room_0_71", "room_0_71:e-place ^ room ^ <Delimitation>unique ^ <Num>sg ^ <Proximity>proximal ^ <Quantification>specific");
//        // cop-restr as subject
//        logicalform = logicalform.replace("<Cop-Restr>(room_0_71", "<Subject>(room_0_71:e-place) ^ <Cop-Restr>(room_0_71");
//
//        // places
//        // GRE
//        logicalform = logicalform.replace("\"0:C@spatial.sa\":castreferent ^ \"0:C@spatial.sa\"", "\"0:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(empty1_0:q-state ^ empty)");
//        logicalform = logicalform.replace("\"1:C@spatial.sa\":castreferent ^ \"1:C@spatial.sa\"", "\"1:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n1_0:number-ordinal ^ 1)");
//        logicalform = logicalform.replace("\"2:C@spatial.sa\":castreferent ^ \"2:C@spatial.sa\"", "\"2:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n2_0:number-ordinal ^ 2)");
//        logicalform = logicalform.replace("\"3:C@spatial.sa\":castreferent ^ \"3:C@spatial.sa\"", "\"3:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n3_0:number-ordinal ^ 3)");
//        logicalform = logicalform.replace("\"4:C@spatial.sa\":castreferent ^ \"4:C@spatial.sa\"", "\"4:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n4_0:number-ordinal ^ 4)");
//        logicalform = logicalform.replace("\"5:C@spatial.sa\":castreferent ^ \"5:C@spatial.sa\"", "\"5:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n5_0:number-ordinal ^ 5)");
//        logicalform = logicalform.replace("\"6:C@spatial.sa\":castreferent ^ \"6:C@spatial.sa\"", "\"6:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n6_0:number-ordinal ^ 6)");
//        logicalform = logicalform.replace("\"7:C@spatial.sa\":castreferent ^ \"7:C@spatial.sa\"", "\"7:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n7_0:number-ordinal ^ 7)");
//        logicalform = logicalform.replace("\"8:C@spatial.sa\":castreferent ^ \"8:C@spatial.sa\"", "\"8:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n8_0:number-ordinal ^ 8)");
//        logicalform = logicalform.replace("\"9:C@spatial.sa\":castreferent ^ \"9:C@spatial.sa\"", "\"9:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n9_0:number-ordinal ^ 9)");
//        logicalform = logicalform.replace("\"10:C@spatial.sa\":castreferent ^ \"10:C@spatial.sa\"", "\"10:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n10_0:number-ordinal ^ 10)");
//        // bug?
//        logicalform = logicalform.replace("place_1__c:castreferent ^ place_1__c", "\"1:C@spatial.sa\":e-place ^ place ^ <Delimitation>unique ^ <Num>sg ^ <Quantification>specific ^ <Modifier>(n1_0:number-ordinal ^ 1)");
//
//        // state ascriptions
//        // obsolete
//        // logicalform = logicalform.replace("ascription}(be", "ascription}(be ^ <Mood>ind ^ <Tense>past");
//        // cop-rest as subj
//        logicalform = logicalform.replace("<Cop-Restr>(\"0:D@spatial.sa\"", "<Subject>(\"0:D@spatial.sa\":person) ^ <Cop-Restr>(\"0:D@spatial.sa\"");
//
//        return logicalform;
//    }

    
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
