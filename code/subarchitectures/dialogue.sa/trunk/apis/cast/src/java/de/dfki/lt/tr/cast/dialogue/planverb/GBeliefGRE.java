package de.dfki.lt.tr.cast.dialogue.planverb;

import java.util.Collection;
import java.util.Map.Entry;

import SpatialData.Place;
import SpatialData.PlaceStatus;
import VisionData.VisualObject;

import coma.aux.ComaGBeliefHelper;
import comadata.ComaRoom;

import scala.collection.immutable.Set;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidQueryException;
import de.dfki.lt.tr.cast.dialogue.GBeliefMemory;
import de.dfki.lt.tr.cast.dialogue.POPlanMonitor;
import de.dfki.tarot.cogx.CASTLogicalForms;
import de.dfki.tarot.cogx.CogXJavaHelpers;
import de.dfki.tarot.cogx.ReferentReplacer;
import de.dfki.tarot.cogx.WMAddress;
import de.dfki.tarot.nlp.lf.BasicLogicalForm;
import de.dfki.tarot.nlp.lf.BasicState;
import de.dfki.tarot.nlp.lf.pattern.BasicPatterns;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.ComaRoomTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;

public class GBeliefGRE {
	
	private PlanVerbalizer m_pevModule; 

    public GBeliefGRE(PlanVerbalizer pevModule) {
    		m_pevModule = pevModule;
    }
	

	
    /**
     * This method performs GRE. It replaces castreferents with their appropriate natural language semantics.
     * - for now, the robot is statically realized as "I"
     * 
     * @param protoLF
     * @return a logical form that contains proper nominals for former free castreferent nominals
     */
    public BasicLogicalForm doGRE(BasicLogicalForm protoLF) {
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
    			GroundedBelief gbWME = m_pevModule.getGBelief(lookupWMA);
    			
    			// m_pevModule.log("getDeterminer with address " + lookupWMA + " returned: " + getDeterminer(gbWME));
    			
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
				System.err.println(e.getStackTrace());
//				logException(e);
			}
    	}
    	log_sb.append("\n *** doGRE returns protoLF: *** \n " + (protoLF==null ? "null" : protoLF.toString()));
//    	log(log_sb.toString());
    	return protoLF;
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
	
	/**
	 * Returns the most likely (according to its probability distribution) 
	 * category of the entity the grounded belief is about. 
	 * Currently only handles VisualObject and ComaRoom GBeliefs!
	 * 
	 * @param gb - the GroundedBelief
	 * @return most likely category_id (for rooms) or label_id (for visual objects) or empty String if n/a
	 */
	private ReferentReplacer getGBeliefCategoryReplacer(GroundedBelief gb) {
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
				m_pevModule.logException(ae);
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
	 * Returns the most likely (according to its probability distribution) 
	 * category of the entity the grounded belief is about. 
	 * Currently only handles VisualObject and ComaRoom GBeliefs!
	 * 
	 * @param gb - the GroundedBelief
	 * @return most likely category_id (for rooms) or label_id (for visual objects) or empty String if n/a
	 */
	private ReferentReplacer getDoesNotExistReplacer(WMAddress _lfWMA) {
			
		ReferentReplacer myRefRep = null;	

		if (_lfWMA.subarchitecture().equals("spatial.sa")) {
			if (_lfWMA.id().split(":").length==2) {
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
				final String catF;
				final String builderTypeF;
				if (_lfWMA.id().toLowerCase().contains("room") || _lfWMA.subarchitecture().contains("coma")) {
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
			final String catF;
			final String builderTypeF;
			if (_lfWMA.toString().toLowerCase().contains("room")) {
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
	}
	

	private String getDeterminer(GroundedBelief referentGB) {

		if (this.m_pevModule.m_castComponent != null) {

			if (((POPlanMonitor) this.m_pevModule.m_castComponent).m_comareasoner != null) {

				this.m_pevModule.m_castComponent.log("GETDET: getDeterminer(" + referentGB.id + ")called");
				// currently only handles "the" vs. "this" -- could also be extended to "a"
				// currently only handles ComaRooms
				CASTIndependentFormulaDistributionsBelief<GroundedBelief> referentGBProxy = 
						CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, referentGB);

				if (referentGBProxy.getType().equals(
						SimpleDiscreteTransferFunction
						.getBeliefTypeFromCastType(ComaRoom.class))) {
					this.m_pevModule.m_castComponent.log("GETDET: referent is a ComaRoom");
					try {

						for (Entry<WorkingMemoryAddress, GroundedBelief> g : this.m_pevModule.view.entrySet()) {
							GroundedBelief currGBelief = g.getValue();
							if (currGBelief.type.equals("Robot")) {
								this.m_pevModule.m_castComponent.log("GETDET: found the GroundedBelief of the Robot!");
								CASTIndependentFormulaDistributionsBelief<GroundedBelief> robotGBProxy =
										CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, 	currGBelief);
								robotGBProxy.getContent();
								WMPointer isInRelateePtr = WMPointer.create(robotGBProxy.getContent().get(
										"is-in").getDistribution().getMostLikely().get());

								if (isInRelateePtr!=null) {
									this.m_pevModule.m_castComponent.log("GETDET: found the is-in relatee, going to read its WME");
									GroundedBelief gbOfRobotsPlace = this.m_pevModule.m_castComponent.getMemoryEntry(isInRelateePtr.get().pointer, 
											GroundedBelief.class);
									this.m_pevModule.m_castComponent.debug("GETDET: loaded WME of Robot's Place GBelief with ID " + gbOfRobotsPlace.id);

									CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbPlaceProxy = 
											CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, gbOfRobotsPlace);
									this.m_pevModule.m_castComponent.debug("GETDET: created a CASTIndependentFormulaDistributionsBelief proxy for the Robot's Place");

									int placeID = -1;
									try {
										placeID  = gbPlaceProxy.getContent().get(
												PlaceTransferFunction.PLACE_ID_ID)
												.getDistribution().getMostLikely().getInteger();
									} catch (BeliefInvalidQueryException e) {
										this.m_pevModule.m_castComponent.log("GETDET: Caught runtime exception: Belief formula is not of type proposition");
										this.m_pevModule.m_castComponent.logException(e);
									}
									this.m_pevModule.m_castComponent.log("GETDET: robot is at place with ID " + placeID);

									String[] roomInstances = ((POPlanMonitor) this.m_pevModule.m_castComponent).m_comareasoner.getRelatedInstancesByRelation(
											"dora:place"+placeID, "dora:constituentOfRoom");
									this.m_pevModule.m_castComponent.log("GETDET: place" + placeID + " is in " + roomInstances.length + " room(s)");

									String roomReferent = "dora:" + ComaGBeliefHelper.getGBeliefComaIndividualName(referentGB);
									this.m_pevModule.m_castComponent.log("GETDET: let's check whether one of them is the referent " + referentGB.id);

									for (String roomIns : roomInstances) {
										if (roomIns.startsWith(":")) roomIns = "dora" + roomIns;
										this.m_pevModule.m_castComponent.log("GETDET: current room instance is: " + roomIns + " and room refertent is " + roomReferent);
										if (roomIns.equals(roomReferent)) {
											this.m_pevModule.m_castComponent.log("GETDET: the referent is the room in which the robot currently is -> THIS");
											return "this";
										}
									}
								}

							} else {
								this.m_pevModule.m_castComponent.debug("GETDET: current looked at GBelief is not about the Robot -- continuing...");
								continue;
							}
						}
					} catch (UnknownSubarchitectureException e) {
						this.m_pevModule.m_castComponent.log("GETDET: caught an UnknownSubarchitectureException");
						this.m_pevModule.m_castComponent.logException(e);
					} catch (DoesNotExistOnWMException e) {
						this.m_pevModule.m_castComponent.log("GETDET: caught a DoesNotExistOnWMException");
						this.m_pevModule.m_castComponent.logException(e);
					}
				} else {
					this.m_pevModule.m_castComponent.log("GETDET: referent is not a ComaRoom");
				}
				this.m_pevModule.m_castComponent.log("GETDET: reached end of getDeterminer() method -> THE");
				return "the";
			} else {
				this.m_pevModule.m_castComponent.log("GETDET: no coma reasoner -> THE");
				return "the";
			}
		} else {
			m_pevModule.log("GETDET: running in standalone mode -> THE");
			return "the";
		}
	}

}
