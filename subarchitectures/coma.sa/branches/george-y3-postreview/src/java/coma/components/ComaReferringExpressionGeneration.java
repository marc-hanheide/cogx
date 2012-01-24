package coma.components;

import java.util.Map;
import java.util.Map.Entry;

import VisionData.VisualObject;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.CASTIndependentFormulaDistributionsBelief;
import de.dfki.lt.tr.beliefs.data.formulas.WMPointer;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidQueryException;
import de.dfki.lt.tr.cast.dialogue.AbstractReferringExpressionGenerationComponent;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationRequest;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationResult;
import de.dfki.lt.tr.dialogue.production.ReferringExpressionGenerator;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.perceptmediator.transferfunctions.PlaceTransferFunction;
import eu.cogx.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import coma.aux.ComaGBeliefHelper;
import coma.components.ComaReferringExpressionGeneration.ComaREGenerator;
import coma.refex.ComaGREAlgorithm;
import comadata.ComaReasonerInterfacePrx;
import comadata.ComaRoom;

public class ComaReferringExpressionGeneration extends
		AbstractReferringExpressionGenerationComponent<ComaREGenerator> {

	private String m_comareasoner_component_name;
	private ComaReasonerInterfacePrx m_comareasoner;
	final protected WMView<GroundedBelief> view = WMView.create(this, GroundedBelief.class);

	@Override
	public void onConfigure(Map<String, String> args) {
		log("onConfigure() called");
		super.onConfigure(args);

		if (args.containsKey("--reasoner-name")) {
			m_comareasoner_component_name=args.get("--reasoner-name");
		} else {
			log("no --reasoner-name given! You must specify the name of the ComaReasoner CAST component. Exiting now!");
			System.exit(-1);
		}
	}
	
	@Override
	public void onStart() {
		try {
			view.start();
		} catch (UnknownSubarchitectureException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		// connection to the coma reasoner
		try {
			if (m_comareasoner_component_name!=null) log("initiating connection to Ice server " + m_comareasoner_component_name);
			if (m_comareasoner_component_name!=null) m_comareasoner = getIceServer(m_comareasoner_component_name, comadata.ComaReasonerInterface.class , comadata.ComaReasonerInterfacePrx.class);
			if (m_comareasoner!=null) log("initiated comareasoner connection");
			else throw new CASTException();
		} catch (CASTException e) {
			e.printStackTrace();
			log("Connection to the coma reasoner Ice server at "+ m_comareasoner_component_name + " failed! Exiting. (Specify the coma reasoner component name using --reasoner-name)");
			System.exit(-1);
		}	
		super.onStart();
	}
	
	@Override
	protected ComaREGenerator initGenerator() {
		return new ComaREGenerator(m_comareasoner, this);
	}

	protected static class ComaREGenerator implements ReferringExpressionGenerator {

		private ComaGREAlgorithm my_GREAlgo;
		private ManagedComponent component;
		
		public ComaREGenerator(ComaReasonerInterfacePrx reasonerPrx, ManagedComponent component) {
			this.my_GREAlgo = new ComaGREAlgorithm(reasonerPrx);
			this.my_GREAlgo.setGlobalAnchor("dora:defaultScene");
			this.my_GREAlgo.setInitialAnchor("dora:defaultScene");
			this.component = component;
		}
		
		@Override
		public ReferenceGenerationResult generate(ReferenceGenerationRequest request, WorkingMemoryAddress requestAddr) {

			// shortNP + omit property "identity"=="roomtype"
			// the room or sth like that
			// currently only implemented for shortNP + no spatial relation
			
			// additional: look up current robot pos, if that's
			// the referent, then we can say "this" instead of "the"
		
			
			// should we produce a short NP or a full refex?
			if (request.shortNP) {
				try {
					GroundedBelief referentGBelief = component.getMemoryEntry(request.obj, GroundedBelief.class);
					if (!request.spatialRelation) {
						String headNoun = "thingy";
						if (request.disabledProperties.contains("identity") 
								|| request.disabledProperties.contains("roomtype")) {
							headNoun = "room";
						} else {
							headNoun = ComaGBeliefHelper.getGBeliefCategory(referentGBelief);
						}
						return new ReferenceGenerationResult(requestAddr, 
								getDeterminer(referentGBelief) + " "
								+ headNoun);
					} else {
						GroundedBelief gbOfRelatedObjectInWM = component.getMemoryEntry(ComaGBeliefHelper.
								getGBeliefRelatee(referentGBelief).get().pointer, GroundedBelief.class);
						
						return new ReferenceGenerationResult(requestAddr, 
								ComaGBeliefHelper.getGBeliefRelation(referentGBelief) + " " +
								getDeterminer(referentGBelief) + " " + 
								ComaGBeliefHelper.getGBeliefCategory(gbOfRelatedObjectInWM));
					}
				} catch (DoesNotExistOnWMException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			} else {
				try {
					GroundedBelief referentGBelief = component.getMemoryEntry(request.obj, GroundedBelief.class);
					if (!request.spatialRelation) {
						return new ReferenceGenerationResult(requestAddr, 
								generateRefExFromGBeliefs(referentGBelief));
					} else {
						GroundedBelief gbOfRelatedObjectInWM = component.getMemoryEntry(ComaGBeliefHelper.
								getGBeliefRelatee(referentGBelief).get().pointer, GroundedBelief.class);
						
						return new ReferenceGenerationResult(requestAddr, 
								ComaGBeliefHelper.getGBeliefRelation(referentGBelief)  + " " 
								+ generateRefExFromGBeliefs(gbOfRelatedObjectInWM));
					}
				} catch (DoesNotExistOnWMException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}				
			}
			
			// we should only end here if sth was wrong...
			return new ReferenceGenerationResult(requestAddr, "the thingy"); 
		}
		
		private String generateRefExFromGBeliefs(GroundedBelief referentGB) {
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbProxy = 
				CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, referentGB);
		
			if (gbProxy.getType().equals(
					SimpleDiscreteTransferFunction
					.getBeliefTypeFromCastType(ComaRoom.class))) {
				return getDeterminer(referentGB) + " " + ComaGBeliefHelper.getGBeliefCategory(referentGB); 
			} else if (gbProxy.getType().equals(
					SimpleDiscreteTransferFunction
					.getBeliefTypeFromCastType(VisualObject.class))) {

				GroundedBelief relateeGB = null;
				try {
					relateeGB = component.getMemoryEntry(ComaGBeliefHelper.
							getGBeliefRelatee(referentGB).get().pointer, GroundedBelief.class);
				} catch (DoesNotExistOnWMException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				if (relateeGB!=null) {
					return getDeterminer(referentGB) + " " +  
					ComaGBeliefHelper.getGBeliefCategory(referentGB) + " " +
					ComaGBeliefHelper.getGBeliefRelation(referentGB) + " " +
					generateRefExFromGBeliefs(relateeGB);
				} else {
					return getDeterminer(referentGB) + " " + ComaGBeliefHelper.getGBeliefCategory(referentGB);
				}
			}
			return "the thingy";
		}
		
		private String getDeterminer(GroundedBelief referentGB) {
			component.log("getDeterminer(" + referentGB.id + ")called");
			// currently only handles "the" vs. "this" -- could also be extended to "a"
			// currently only handles ComaRooms
			CASTIndependentFormulaDistributionsBelief<GroundedBelief> referentGBProxy = 
				CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, referentGB);
		
			if (referentGBProxy.getType().equals(
					SimpleDiscreteTransferFunction
					.getBeliefTypeFromCastType(ComaRoom.class))) {
				component.log("referent is a ComaRoom");
				try {

					for (Entry<WorkingMemoryAddress, GroundedBelief> g : ((ComaReferringExpressionGeneration) component).view.entrySet()) {
						GroundedBelief currGBelief = g.getValue();
						if (currGBelief.type.equals("Robot")) {
							component.log("found the GroundedBelief of the Robot!");
							CASTIndependentFormulaDistributionsBelief<GroundedBelief> robotGBProxy =
								CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, 	currGBelief);
							robotGBProxy.getContent();
							WMPointer isInRelateePtr = WMPointer.create(robotGBProxy.getContent().get(
							"is-in").getDistribution().getMostLikely().get());

							if (isInRelateePtr!=null) {
								component.log("found the is-in relatee, going to read its WME");
								GroundedBelief gbOfRobotsPlace = component.getMemoryEntry(isInRelateePtr.get().pointer, 
										GroundedBelief.class);
								component.debug("loaded WME of Robot's Place GBelief with ID " + gbOfRobotsPlace.id);

								CASTIndependentFormulaDistributionsBelief<GroundedBelief> gbPlaceProxy = 
									CASTIndependentFormulaDistributionsBelief.create(GroundedBelief.class, gbOfRobotsPlace);
								component.debug("created a CASTIndependentFormulaDistributionsBelief proxy for the Robot's Place");

								int placeID = -1;
								try {
									placeID  = gbPlaceProxy.getContent().get(
											PlaceTransferFunction.PLACE_ID_ID)
											.getDistribution().getMostLikely().getInteger();
								} catch (BeliefInvalidQueryException e) {
									component.log("Caught runtime exception: Belief formula is not of type proposition");
									component.logException(e);
								}
								component.log("robot is at place with ID " + placeID);

								String[] roomInstances = ((ComaReferringExpressionGeneration) component).m_comareasoner.getRelatedInstancesByRelation(
										"dora:place"+placeID, "dora:constituentOfRoom");
								component.log("place" + placeID + " is in " + roomInstances.length + " room(s)");
								
								String roomReferent = "dora:" + ComaGBeliefHelper.getGBeliefComaIndividualName(referentGB);
								component.log("let's check whether one of them is the referent " + referentGB.id);

								for (String roomIns : roomInstances) {
									if (roomIns.startsWith(":")) roomIns = "dora" + roomIns;
									component.log("current room instance is: " + roomIns + " and room refertent is " + roomReferent);
									if (roomIns.equals(roomReferent)) {
										component.log("the referent is the room in which the robot currently is -> THIS");
										return "this";
									}
								}
							}

						} else {
							component.debug("current looked at GBelief is not about the Robot -- continuing...");
							continue;
						}
					}
				} catch (UnknownSubarchitectureException e) {
					component.log("caught an UnknownSubarchitectureException");
					component.logException(e);
				} catch (DoesNotExistOnWMException e) {
					component.log("caught a DoesNotExistOnWMException");
					component.logException(e);
				}
			} else {
				component.log("referent is not a ComaRoom");
			}
			component.log("reached end of getDeterminer() method -> THE");
			return "the"; 
		}

		public ReferenceGenerationResult generateWithComaGRE(ReferenceGenerationRequest request, WorkingMemoryAddress requestAddr) {
			// should we produce a short NP or a full refex?
			if (request.shortNP) {
				try {
					GroundedBelief referentGBelief = component.getMemoryEntry(requestAddr, GroundedBelief.class);
					if (!request.spatialRelation) {
						return new ReferenceGenerationResult(requestAddr, 
								"the " + ComaGBeliefHelper.getGBeliefCategory(referentGBelief));
					} else {
						GroundedBelief gbOfRelatedObjectInWM = component.getMemoryEntry(ComaGBeliefHelper.
								getGBeliefRelatee(referentGBelief).get().pointer, GroundedBelief.class);
						
						return new ReferenceGenerationResult(requestAddr, 
								ComaGBeliefHelper.getGBeliefRelation(referentGBelief) + " the " + 
								ComaGBeliefHelper.getGBeliefCategory(gbOfRelatedObjectInWM));
					}
				} catch (DoesNotExistOnWMException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			} else {
				try {
					GroundedBelief referentGBelief = component.getMemoryEntry(requestAddr, GroundedBelief.class);
					if (!request.spatialRelation) {
						return new ReferenceGenerationResult(requestAddr, 
								my_GREAlgo.generateRefExG(
										"dora:" + ComaGBeliefHelper.getGBeliefComaIndividualName(referentGBelief)));
					} else {
						GroundedBelief gbOfRelatedObjectInWM = component.getMemoryEntry(ComaGBeliefHelper.
								getGBeliefRelatee(referentGBelief).get().pointer, GroundedBelief.class);
						
						return new ReferenceGenerationResult(requestAddr, 
								ComaGBeliefHelper.getGBeliefRelation(referentGBelief) + " the " +
								my_GREAlgo.generateRefExG(
										"dora:" + ComaGBeliefHelper.getGBeliefComaIndividualName(gbOfRelatedObjectInWM)));
					}
				} catch (DoesNotExistOnWMException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (UnknownSubarchitectureException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (Exception e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}				
			}
			
			// we should only end here if sth was wrong...
			return new ReferenceGenerationResult(requestAddr, "the thingy"); 
		
		}
		
	}

}
