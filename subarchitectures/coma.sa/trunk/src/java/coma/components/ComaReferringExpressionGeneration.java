package coma.components;

import java.awt.Component;
import java.util.Map;

import sun.applet.GetMemberPluginCallRequest;

import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryAttachedComponent;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.cast.dialogue.AbstractReferringExpressionGenerationComponent;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationRequest;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationResult;
import de.dfki.lt.tr.dialogue.production.ReferringExpressionGenerator;
import eu.cogx.beliefs.slice.GroundedBelief;
import coma.aux.ComaGBeliefHelper;
import coma.components.ComaReferringExpressionGeneration.ComaREGenerator;
import coma.refex.ComaGREAlgorithm;
import comadata.ComaReasonerInterfacePrx;

public class ComaReferringExpressionGeneration extends
		AbstractReferringExpressionGenerationComponent<ComaREGenerator> {

	private String m_comareasoner_component_name;
	private ComaReasonerInterfacePrx m_comareasoner;

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

	public static class ComaREGenerator implements ReferringExpressionGenerator {

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
