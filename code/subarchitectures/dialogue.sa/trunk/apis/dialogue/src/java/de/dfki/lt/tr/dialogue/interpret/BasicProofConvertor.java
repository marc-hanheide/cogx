package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.util.BeliefFormulaFactory;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequestExtractor;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.ref.NominalReference;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import de.dfki.lt.tr.infer.abducer.util.ProofUtils;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import org.apache.log4j.Logger;

public class BasicProofConvertor
implements ProofConvertor {

	private final Logger logger;

	private String thisAgent;

	private String dialogueSA;
	private String commitSA;

	private final IdentifierGenerator<String> idGen;
	
	private final ReferenceResolutionRequestExtractor extractor;

	public BasicProofConvertor(Logger logger, IdentifierGenerator<String> idGen, ReferenceResolutionRequestExtractor extractor, String thisAgent, String dialogueSA, String commitSA) {
		this.logger = logger;
		this.idGen = idGen;
		this.dialogueSA = dialogueSA;
		this.commitSA = commitSA;
		this.thisAgent = thisAgent;
		this.extractor = extractor;
	}

	@Override
	public ReferenceResolutionRequestExtractor getReferenceResolutionRequestExtractor() {
		return extractor;
	}

	@Override
	public IntentionRecognitionResult proofToIntentionRecognitionResult(LogicalForm lf, ProofWithCost pwc, float probBound, TimeInterval ival, List<ReferenceResolutionRequest> rrqs) {

		LinkedList<EpistemicObject> results = new LinkedList<EpistemicObject>();
		List<dBelief> bels_pre = new LinkedList<dBelief>();
		List<dBelief> bels_post = new LinkedList<dBelief>();

//		Map<String, dBelief> marks = new HashMap<String, dBelief>();
//		List<InterBeliefPointer> pointers = new LinkedList<InterBeliefPointer>();

		List<ProofWithCost> pwcs = new ArrayList<ProofWithCost>();
		pwcs.add(pwc);
		IntentionRecognitionResult ri = new IntentionRecognitionResult(lf, ival, pwcs);
		if (!rrqs.isEmpty()) {
			ri.getResolutionRequests().addAll(rrqs);
		}

		// determine the speaker
		String attribAgent = null;
		for (ModalisedAtom ma : ProofUtils.stripMarking(pwc.proof)) {
			if (ma.m.size() == 2 && ma.m.get(0) == Modality.Understanding && ma.m.get(1) == Modality.Event && ma.a.predSym.equals(ConversionUtils.predsym_UTTER) && ma.a.args.size() == 3) {
				Term speakerTerm = ma.a.args.get(0);
				if (speakerTerm instanceof FunctionTerm) {
					attribAgent = ((FunctionTerm) speakerTerm).functor;
				}
			}
		}

		assert (attribAgent != null);

		Map<String, WorkingMemoryAddress> usedRefs = new HashMap<String, WorkingMemoryAddress>();
		List<ModalisedAtom> rrs = ProofUtils.stripMarking(ProofUtils.filterAssumedAndForget(pwc.proof));

		for (ModalisedAtom ma : rrs) {
			if (ma.m.size() == 1 && ma.m.get(0) == Modality.Understanding && ma.a.predSym.equals(ConversionUtils.predsym_RESOLVES_TO_BELIEF) && ma.a.args.size() == 3) {
				String n = ((FunctionTerm)ma.a.args.get(0)).functor;
				WorkingMemoryAddress wma = ConversionUtils.termToWorkingMemoryAddress(ma.a.args.get(1));
				if (wma != null) {
					usedRefs.put(n, wma);
				}
			}
		}

		if (usedRefs.size() == 1) {
			for (String refnom : usedRefs.keySet()) {
				ri.setNominalReference(new NominalReference(refnom, BeliefFormulaFactory.newPointerFormula(usedRefs.get(refnom))));
				break;
			}
		}

		List<ModalisedAtom> imfs = ProofUtils.filterStripByModalityPrefix(
				ProofUtils.stripMarking(ProofUtils.filterAssumedAndForget(pwc.proof)),
				Arrays.asList(new Modality[] {Modality.Intention}));

		HashMap<String, IntentionalContent> rIts = new HashMap<String, IntentionalContent>();

		// TODO: make a dedicated class for this
		for (ModalisedAtom ma : imfs) {
			if (ma.a.predSym.equals(ConversionUtils.predsym_AGENT)) {
				FunctionTerm idTerm = (FunctionTerm) ma.a.args.get(0);
				FunctionTerm agentTerm = (FunctionTerm) ma.a.args.get(1);

				if (!rIts.containsKey(idTerm.functor)) {
					rIts.put(idTerm.functor, ConversionUtils.newIntentionalContent(probBound));
				}
				IntentionalContent itc = rIts.get(idTerm.functor);

				itc.agents = new LinkedList<String>();
				itc.agents.add(agentTerm.functor);
			}
			else if (ma.a.predSym.equals(ConversionUtils.predsym_PRECONDITION)) {
				FunctionTerm idTerm = (FunctionTerm) ma.a.args.get(0);
				FunctionTerm argTerm = (FunctionTerm) ma.a.args.get(1);

				if (!rIts.containsKey(idTerm.functor)) {
					rIts.put(idTerm.functor, ConversionUtils.newIntentionalContent(probBound));
				}
				IntentionalContent itc = rIts.get(idTerm.functor);

				if (argTerm.functor.equals(IntentionManagementConstants.beliefLinkModality)) {
					String lingRef = ((FunctionTerm)argTerm.args.get(0)).functor;
					EpistemicStatus es = ConversionUtils.termToEpistemicStatus((FunctionTerm)argTerm.args.get(1));

					FunctionTerm action = (FunctionTerm)argTerm.args.get(2);

					if (action.functor.equals(ConversionUtils.functor_FEATVAL)) {
						String newId = ConversionUtils.foldIntoBeliefs(idGen, es, lingRef, usedRefs, action, bels_pre);
						if (newId != null) {
							dFormula refF = BeliefFormulaFactory.newModalFormula(IntentionManagementConstants.beliefLinkModality, BeliefFormulaFactory.newPointerFormula(new WorkingMemoryAddress(newId, commitSA)));
							itc.preconditions = ConversionUtils.combineDFormulas(itc.preconditions, refF);
						}
					}
/*
					else if (action.functor.equals("mark")) {
						String marking = ((FunctionTerm)action.args[0]).functor;
						String newId = foldIntoBeliefs(idGen, es, lingRef, usedRefs, action, bels_pre);
						if (newId != null) {
							dFormula refF = BeliefFormulaFactory.newModalFormula(IntentionManagement.beliefLinkModality, BeliefFormulaFactory.newPointerFormula(new WorkingMemoryAddress(newId, commitSA)));
							itc.preconditions = combineDFormulas(itc.preconditions, refF);
						}
					}
 */
				}
				if (argTerm.functor.equals(IntentionManagementConstants.stateModality) && argTerm.args.size() == 1) {
					dFormula stateF = BeliefFormulaFactory.newModalFormula(IntentionManagementConstants.stateModality, ConversionUtils.uniTermToFormula((FunctionTerm) argTerm.args.get(0)));
					itc.preconditions = ConversionUtils.combineDFormulas(itc.preconditions, stateF);
				}
			}
			else if (ma.a.predSym.equals(ConversionUtils.predsym_POSTCONDITION)) {
				FunctionTerm idTerm = (FunctionTerm) ma.a.args.get(0);
				FunctionTerm argTerm = (FunctionTerm) ma.a.args.get(1);

				if (!rIts.containsKey(idTerm.functor)) {
					rIts.put(idTerm.functor, ConversionUtils.newIntentionalContent(probBound));
				}
				IntentionalContent itc = rIts.get(idTerm.functor);

				if (argTerm.functor.equals(IntentionManagementConstants.beliefLinkModality)) {
					String lingRef = ((FunctionTerm)argTerm.args.get(0)).functor;
					EpistemicStatus es = ConversionUtils.termToEpistemicStatus((FunctionTerm)argTerm.args.get(1));

					FunctionTerm action = (FunctionTerm)argTerm.args.get(2);

					if (action.functor.equals(ConversionUtils.functor_FEATVAL)) {
						String newId = ConversionUtils.foldIntoBeliefs(idGen, es, lingRef, usedRefs, action, bels_post);
						if (newId != null) {
							dFormula refF = BeliefFormulaFactory.newModalFormula(IntentionManagementConstants.beliefLinkModality, BeliefFormulaFactory.newPointerFormula(new WorkingMemoryAddress(newId, dialogueSA)));
							itc.postconditions = ConversionUtils.combineDFormulas(itc.postconditions, refF);
						}
					}
				}
				if (argTerm.functor.equals(IntentionManagementConstants.stateModality) && argTerm.args.size() == 1) {
					dFormula stateF = BeliefFormulaFactory.newModalFormula(IntentionManagementConstants.stateModality, ConversionUtils.uniTermToFormula((FunctionTerm) argTerm.args.get(0)));
					itc.postconditions = ConversionUtils.combineDFormulas(itc.postconditions, stateF);
				}
			}
		}

		for (ModalisedAtom ma : rrs) {
			if (ma.m.size() == 1 && ma.m.get(0) == Modality.Intention && ma.a.predSym.equals(ConversionUtils.predsym_POINTER) && ma.a.args.size() == 3) {
				String featName = ((FunctionTerm)ma.a.args.get(0)).functor;
				String markFrom = ((FunctionTerm)ma.a.args.get(1)).functor;
				String markTo = ((FunctionTerm)ma.a.args.get(2)).functor;

				dBelief from = ConversionUtils.findMarkedBelief(bels_pre, markFrom);
//				if (from == null) {
//					from = findMarkedBelief(bels_post, markFrom);
//				}

				dBelief to = ConversionUtils.findMarkedBelief(bels_pre, markTo);
//				if (to == null) {
//					to = findMarkedBelief(bels_post, markTo);
//				}

				if (from != null && to != null) {
					ConversionUtils.addFeature(from, featName, BeliefFormulaFactory.newPointerFormula(new WorkingMemoryAddress(to.id, commitSA)));  // FIXME: this is *very* hacky!
				}
			}
		}

		// explicit topic switch
		for (ModalisedAtom ma : rrs) {
			if (ma.m.size() == 1 && ma.m.get(0) == Modality.Understanding && ma.a.predSym.equals(ConversionUtils.predsym_TOPIC_SWITCH_MARK) && ma.a.args.size() == 2
					&& ma.a.args.get(0) instanceof FunctionTerm && ma.a.args.get(1) instanceof FunctionTerm) {

				String nom = ((FunctionTerm) ma.a.args.get(0)).functor;
				NominalReference newTopic = null;

				dFormula inF = ConversionUtils.uniTermToFormula((FunctionTerm) ma.a.args.get(1));
				dFormula outF = null;

				outF = ConversionUtils.replacePointersInFormula(inF, bels_pre, commitSA);
				if (outF instanceof PointerFormula) {
					newTopic = new NominalReference(nom, outF);
				}

				if (newTopic == null) {
					outF = ConversionUtils.replacePointersInFormula(inF, bels_post, dialogueSA);
					if (outF instanceof PointerFormula) {
						newTopic = new NominalReference(nom, outF);
					}
				}

				if (newTopic == null) {
					log("got a topic swith, but don't know what to resolve it against");
				}
				else {
					ri.setNominalReference(newTopic);
				}
			}
		}

		Iterator<String> iter = rIts.keySet().iterator();
		while (iter.hasNext()) {
			Intention it = new Intention();
			it.id = idGen.newIdentifier();
			it.frame = new AbstractFrame();
			AttributedEpistemicStatus epst = new AttributedEpistemicStatus();
			epst.agent = thisAgent;
			epst.attribagents = new LinkedList<String>();
			epst.attribagents.add(attribAgent);
			it.estatus = epst;
			it.content = new LinkedList<IntentionalContent>();
			IntentionalContent itc = rIts.get(iter.next());
			it.content.add(itc);
			if (itc.agents != null && !itc.agents.isEmpty()) {

				// replace pointers in the formulas
				ConversionUtils.replacePointersInFormula(itc.preconditions, bels_pre, commitSA);
				ConversionUtils.replacePointersInFormula(itc.preconditions, bels_post, dialogueSA);
				ConversionUtils.replacePointersInFormula(itc.postconditions, bels_pre, commitSA);
				ConversionUtils.replacePointersInFormula(itc.postconditions, bels_post, dialogueSA);

				ri.getIntentions().add(it);
			}
			else {
				log("discarding [" + it.id + "]: incomplete (agent list empty)");
			}
		}

		for (dBelief b : bels_pre) {
			ConversionUtils.removeFeature(b, ConversionUtils.feat_MARK);
		}
		for (dBelief b : bels_post) {
			ConversionUtils.removeFeature(b, ConversionUtils.feat_MARK);
		}

		ri.getPreconditionBeliefs().addAll(bels_pre);
		ri.getPostconditionBeliefs().addAll(bels_post);

		return ri;
	}

	private void log(String s) {
		if (logger != null) {
			logger.debug(s);
		}
	}

}
