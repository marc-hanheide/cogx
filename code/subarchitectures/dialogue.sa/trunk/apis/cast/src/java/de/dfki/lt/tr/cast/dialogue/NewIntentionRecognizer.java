package de.dfki.lt.tr.cast.dialogue;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;

import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.architecture.WorkingMemoryWriterComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionToAct;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.PossibleInterpretedIntentions;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.interpret.AbducerUtils;
import de.dfki.lt.tr.dialogue.interpret.CASTResultWrapper;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.GeneratedWMAddressTranslator;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.interpret.InterpretedUserIntention;
import de.dfki.lt.tr.dialogue.interpret.InterpretedUserIntentionProofInterpreter;
import de.dfki.lt.tr.dialogue.interpret.MaximumReadingsTerminationCondition;
import de.dfki.lt.tr.dialogue.interpret.ResultCombinator;
import de.dfki.lt.tr.dialogue.interpret.ResultGatherer;
import de.dfki.lt.tr.dialogue.interpret.RobotCommunicativeAction;
import de.dfki.lt.tr.dialogue.interpret.TerminationCondition;
import de.dfki.lt.tr.dialogue.interpret.WMAddressTranslator;
import de.dfki.lt.tr.dialogue.interpret.WMAddressTranslatorFactory;
import de.dfki.lt.tr.dialogue.interpret.atoms.AssertedReferenceAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.FromLFAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.IntentionIDAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.NewBeliefAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.QUDOpenAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.QUDPolarAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.ReferentOfAtom;
import de.dfki.lt.tr.dialogue.interpret.atoms.TypeOfQUDAtom;
import de.dfki.lt.tr.dialogue.ref.BasicReferenceResolutionRequestExtractor;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequestExtractor;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.slice.asr.UnclarifiedPossibleInterpretedIntentions;
import de.dfki.lt.tr.dialogue.slice.interpret.InterpretationRequest;
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parseselection.SelectedLogicalForm;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.util.BasicNominalRemapper;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import de.dfki.lt.tr.dialogue.util.NominalRemapper;
import de.dfki.lt.tr.infer.abducer.engine.AbductionEnginePrx;
import de.dfki.lt.tr.infer.abducer.engine.FileReadErrorException;
import de.dfki.lt.tr.infer.abducer.engine.SyntaxErrorException;
import de.dfki.lt.tr.infer.abducer.lang.DisjointDeclaration;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.proof.AbstractAssertionSolver;
import de.dfki.lt.tr.infer.abducer.proof.AbstractProofInterpretationContext;
import de.dfki.lt.tr.infer.abducer.proof.AssertionSolverCascade;
import de.dfki.lt.tr.infer.abducer.proof.ContextUpdate;
import de.dfki.lt.tr.infer.abducer.proof.EngineProofExpander;
import de.dfki.lt.tr.infer.abducer.proof.ProofExpander;
import de.dfki.lt.tr.infer.abducer.proof.ProofInterpretationContext;
import de.dfki.lt.tr.infer.abducer.proof.ProofInterpreter;
import de.dfki.lt.tr.infer.abducer.proof.ProofPruner;
import de.dfki.lt.tr.infer.abducer.proof.pruners.LengthPruner;
import de.dfki.lt.tr.infer.abducer.util.AbductionEngineConnection;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;

public class NewIntentionRecognizer
extends AbstractAbductiveComponent<InterpretedUserIntention, String> {

	public static final WorkingMemoryAddress EMPTY_ADDRESS = new WorkingMemoryAddress("nobody", "home");
	public final String DEFAULT_ABD_SERVER_NAME = "AbducerServer";
	public final int DEFAULT_ABD_PORT = 9100;
	public final String DEFAULT_ABD_ENDPOINT_CONFIG = "default";
	public final String DEFAULT_ENGINE_NAME = "intention-recognition";
	public final int DEFAULT_TIMEOUT = 250;

	public final int DEFAULT_MAX_READINGS = 1;

	private String abd_serverName = DEFAULT_ABD_SERVER_NAME;
	private List<String> files = new LinkedList<String>();
	private String abd_endpoints;
	private String abd_engineName = DEFAULT_ENGINE_NAME;

	private String rulesetFile = "/dev/null";

	private int timeout = DEFAULT_TIMEOUT;

	private final Map<String, RemappedSLF> nomToLFMap;
//	private final Map<String, String> renamedNomToNom;
//	private final Map<String, NominalRemapper> nomToRemapper;

	private final ReferenceResolutionRequestExtractor rrExtractor;
	private final Map<WorkingMemoryAddress, ResultGatherer<ReferenceResolutionResultWrapper>> gatherers;

	private final IdentifierGenerator<String> idGen;
	private final IdentifierGenerator<WorkingMemoryAddress> wmaGen;
	private final String idPrefix = "irecog";
	private int idIndex = 0;
	private int remapIndex = 1;
	private TerminationCondition condition = null;
	private int maxReadings = DEFAULT_MAX_READINGS;

	private WMView<IntentionToAct> openIntentionsToAct = null;
	private final WMAddressTranslatorFactory translatorFactory;

	public NewIntentionRecognizer() {
		super();
		nomToLFMap = new HashMap<String, RemappedSLF>();
//		renamedNomToNom = new HashMap<String, String>();
//		nomToRemapper = new HashMap<String, NomToRemapper>();
		rrExtractor = new BasicReferenceResolutionRequestExtractor(getLogger());
		gatherers = new HashMap<WorkingMemoryAddress, ResultGatherer<ReferenceResolutionResultWrapper>>();
		idGen = new IdentifierGenerator<String>() {
			@Override
			public String newIdentifier() {
				return idPrefix + ":" + idIndex++;
			}
		};
		wmaGen = new IdentifierGenerator<WorkingMemoryAddress>() {
			@Override
			public WorkingMemoryAddress newIdentifier() {
				return new WorkingMemoryAddress(idGen.newIdentifier(), getSubarchitectureID());
			}
		};
		translatorFactory = new WMAddressTranslatorFactory() {
			@Override
			public WMAddressTranslator newTranslator() {
				return new GeneratedWMAddressTranslator(wmaGen);
			}
		};
	}

	@Override
	public void onConfigure(Map<String, String> args) {
		super.onConfigure(args);

		if (args.containsKey("--ruleset")) {
			rulesetFile = args.get("--ruleset");
		}
		if (args.containsKey("--timeout")) {
			String timeoutStr = args.get("--timeout");
			timeout = Integer.parseInt(timeoutStr);
		}
		if (args.containsKey("--max-readings")) {
			String readingsStr = args.get("--max-readings");
			maxReadings = Integer.parseInt(readingsStr);
		}

		getLogger().debug("will only look at the " + maxReadings + " best reading(s)");
		condition = new MaximumReadingsTerminationCondition(maxReadings);

		String abd_host = args.get("--abd-host");
		abd_endpoints = AbducerUtils.getAbducerServerEndpointString(abd_host, DEFAULT_ABD_PORT);

		files.addAll(AbducerUtils.getAbducerRuleFiles(getLogger(), rulesetFile));
	}

	@Override
	public AbductionEnginePrx initEngine() {
		getLogger().debug("initialising the abduction engine, servername=\"" + abd_serverName + "\", endpoints=\"" + abd_endpoints + "\"");
		AbductionEngineConnection connection = new AbductionEngineConnection();
		connection.connectToServer(abd_serverName, abd_endpoints);
		getLogger().debug("binding to the engine \"" + abd_engineName + "\"");
		connection.bindToEngine(abd_engineName);

		AbductionEnginePrx prx = connection.getEngineProxy();
		prx.clearContext();

		for (String filename : files) {
			try {
				prx.loadFile(filename);
			}
			catch (FileReadErrorException ex) {
				logException(ex);
			}
			catch (SyntaxErrorException ex) {
				logException(ex);
			}
		}

		return prx;
	}

	@Override
	protected ProofInterpretationContext<InterpretedUserIntention, String> initContext() {
		ProofPruner pruner = new LengthPruner(3);
		ProofExpander expander = new EngineProofExpander(getEngine(), timeout);

		AssertionSolverCascade solvers = new AssertionSolverCascade();
		solvers.addSolver(new ExpandLFAssertionSolver());
		solvers.addSolver(new IntentionIDAssertionSolver());
		solvers.addSolver(new NewBeliefAssertionSolver());
		solvers.addSolver(new ReferenceResolutionAssertionSolver());
		solvers.addSolver(new WhQUDAssertionSolver());
		solvers.addSolver(new PolarQUDAssertionSolver());
		solvers.addSolver(new TypeOfQUDAssertionSolver());

		ProofInterpreter<InterpretedUserIntention> interpreter = new InterpretedUserIntentionProofInterpreter(getLogger());

		final WorkingMemoryWriterComponent committer = this;

		return new AbstractProofInterpretationContext<InterpretedUserIntention, String>(pruner, expander, solvers, interpreter) {

			@Override
			public void onSuccessfulInterpretation(List<InterpretedUserIntention> listIpret, double asrConfidence, String arg) {
				getLogger().debug("got " + listIpret.size() + " interpretations.");
				getLogger().debug("NLP confidence = " + asrConfidence);
//				for (int i = 0; i < listIpret.size(); i++) {
//					getLogger().debug("interpretation " + (i + 1) + "/" + listIpret.size() + ": " + listIpret.get(i));
//				}

				if (!listIpret.isEmpty()) {
					PossibleInterpretedIntentions pii = createPossibleInterpretedIntentions(translatorFactory, listIpret);
					pii = prunePossibleInterpretedIntentions(pii);
					getLogger().debug("normalizing confidences");
					normalizeConfidences(pii);
					getLogger().debug("after pruning and normalization: " + pii.intentions.size() + " alternative intentions");
					getLogger().debug("adding the following to the WM (as unclarified):\n" + possibleInterpretedIntentionsToString(pii));
					try {
						addToWorkingMemory(newDataID(), new UnclarifiedPossibleInterpretedIntentions(pii, arg, (float) asrConfidence));
					}
					catch (SubarchitectureComponentException ex) {
						logException(ex);
					}
/*
					getLogger().debug("will now commit the first interpretation");
					InterpretedUserIntention ipret = listIpret.get(0);
					try {
						ipret.commit(committer);
					}
					catch (SubarchitectureComponentException ex) {
						logException(ex);
					}
 */
				}
				else {
					getLogger().warn("didn't get any interpretations at all!");
				}
			}

			@Override
			public void onNoInterpretation() {
				getLogger().warn("no interpretation found");
			}
			
		};
	}

	public PossibleInterpretedIntentions createPossibleInterpretedIntentions(WMAddressTranslatorFactory translatorFactory, List<InterpretedUserIntention> listIpret) {
		PossibleInterpretedIntentions pii = new PossibleInterpretedIntentions(
				new HashMap<WorkingMemoryAddress, InterpretedIntention>(),
				new HashMap<WorkingMemoryAddress, dBelief>(),
				EMPTY_ADDRESS);

		for (InterpretedUserIntention iui : listIpret) {
			WMAddressTranslator translator = translatorFactory.newTranslator();
			WorkingMemoryAddress wma = translator.translate(iui.getAddress());
			pii.intentions.put(wma, iui.toIntention(translator));
			pii.beliefs.putAll(iui.toBeliefs(translator));
		}
		
		return pii;
	}

	public String possibleInterpretedIntentionsToString(PossibleInterpretedIntentions pii) {
		String s = "(PossibleInterpretedIntentions\n";
		s += "intentions {\n";
		for (WorkingMemoryAddress wma : pii.intentions.keySet()) {
			InterpretedIntention ii = pii.intentions.get(wma);
			s += wmaToString(wma) + " ... " + InterpretedUserIntention.interpretedIntentionToString(ii) + "\n";
		}
		s += "}\n";
		s += "\n";
		s += "beliefs {\n";
		for (WorkingMemoryAddress wma : pii.beliefs.keySet()) {
			dBelief bel = pii.beliefs.get(wma);
			s += wmaToString(wma) + " ... " + BeliefIntentionUtils.beliefToString(bel) + "\n";
		}
		s += "}\n";
		s += ")";
		return s;
	}

	public void normalizeConfidences(PossibleInterpretedIntentions pii) {
		double sum = 0.0;
		for (InterpretedIntention iint : pii.intentions.values()) {
			sum += iint.confidence;
		}

		for (InterpretedIntention iint : pii.intentions.values()) {
			assert sum > 0.0;
			iint.confidence = (float) ((double) iint.confidence / sum);
		}
	}

	public PossibleInterpretedIntentions prunePossibleInterpretedIntentions(PossibleInterpretedIntentions pii) {
		PossibleInterpretedIntentions newPii = pii;

		getLogger().debug("will see whether the possible intentions need to be pruned");
		WorkingMemoryAddress addr = IntentionUnpacker.mostConfidentIntentionAddress(pii);
		InterpretedIntention iint = pii.intentions.get(addr);

		getLogger().debug("I'll look this intention:\n" + InterpretedUserIntention.interpretedIntentionToString(iint));

		if (
			(stringEquals(iint.stringContent.get("type"), "question")
					&& (stringEquals(iint.stringContent.get("subtype"), "open") || stringEquals(iint.stringContent.get("subtype"), "polar")))
				|| (stringEquals(iint.stringContent.get("type"), "assertion")
					&& (stringEquals(iint.stringContent.get("subtype"), "ascription")))) {

			// this is the case we can handle
			getLogger().debug("okay, this seems to be an open/polar question -> we should be able to handle multiple intentions here");
		}
		else {
			// we cannot handle anything else: prrrune!
			getLogger().debug("this is not an open/polar question -> will prune the PossibleInterpretedIntentions to be sure");
			newPii = extractFromRoot(addr, pii);
		}
		getLogger().debug("pruning finished");
		return newPii;
	}

	public static boolean stringEquals(String s1, String s2) {
		if (s1 != null && s2 != null) {
			return s1.equals(s2);
		}
		else {
			return false;
		}
	}

	public PossibleInterpretedIntentions extractFromRoot(WorkingMemoryAddress wma, PossibleInterpretedIntentions pii) {
		PossibleInterpretedIntentions newPii = new PossibleInterpretedIntentions(
				new HashMap<WorkingMemoryAddress, InterpretedIntention>(),
				new HashMap<WorkingMemoryAddress, dBelief>(), EMPTY_ADDRESS);

		InterpretedIntention iint = pii.intentions.get(wma);
		newPii.intentions.put(wma, iint);
		for (WorkingMemoryAddress addr : iint.addressContent.values()) {
			if (pii.beliefs.containsKey(addr)) {
				newPii.beliefs.put(addr, pii.beliefs.get(addr));
			}
		}
		
		return newPii;
	}

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				SelectedLogicalForm.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

							@Override
							public void execute(WorkingMemoryAddress addr) {
								try {
									getLogger().info("converting the SelectedLogicalForm to a partial interpretation");
									SelectedLogicalForm slf = getMemoryEntry(addr, SelectedLogicalForm.class);
									InterpretationRequest inprRequest = new InterpretationRequest(selectedLFToGoal(slf));
									addNewPartialInterpretation(addr, interpretationRequestToPartialInterpretation(getContext().getPruner(), addr, inprRequest, slf.lform.preferenceScore, slf.phonStringWordList, condition));
								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}
							
						});
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				ReferenceResolutionResult.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

							@Override
							public void execute(WorkingMemoryAddress addr) {
								try {
									getLogger().info("got a ReferenceResolutionResult");
									ReferenceResolutionResult result = getMemoryEntry(addr, ReferenceResolutionResult.class);
									ResultGatherer<ReferenceResolutionResultWrapper> gatherer = gatherers.get(result.requestAddress);
									if (gatherer != null) {
										gatherer.addResult(new ReferenceResolutionResultWrapper(result));
									}
									else {
										getLogger().error("gatherer is null!");
									}
								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}
						});
					}
				});

		openIntentionsToAct = WMView.create(this, IntentionToAct.class);
		try {
			openIntentionsToAct.start();
		}
		catch (UnknownSubarchitectureException ex) {
			logException(ex);
			scheduleOwnDeath();
		}
	}

	public class ExpandLFAssertionSolver extends AbstractAssertionSolver<FromLFAtom> {

		public ExpandLFAssertionSolver() {
			super(new FromLFAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(FromLFAtom a) {
			String nominal = a.getNominal();
			getLogger().info("solving an LF assertion for nominal \"" + nominal + "\"");
//			String originalNom = renamedNomToNom.get(nominal);
//			assert originalNom != null;
			RemappedSLF remapped = nomToLFMap.get(nominal);
			final LogicalForm lf = remapped.slf.lform;
			final NominalRemapper remapper = remapped.remapper;
			return new ContextUpdate() {

				@Override
				public void doUpdate(AbductionEnginePrx engine) {
					getLogger().debug("updating the abduction context for the solved LF assertion");
					for (ModalisedAtom fact : AbducerUtils.lfToFacts(new Modality[] {Modality.Truth}, lf, remapper)) {
						engine.addFact(fact);
					}
				}

			};
		}

	};

	public class IntentionIDAssertionSolver extends AbstractAssertionSolver<IntentionIDAtom> {

		public IntentionIDAssertionSolver() {
			super(new IntentionIDAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(IntentionIDAtom a) {
			final String nominal = a.getNominal();
			if (nominal == null) {
				getLogger().error("undetermined nominal");
				return null;
			}

			final WorkingMemoryAddress wma = newWorkingMemoryAddress();

			getLogger().info("generated a new intention WMA: " + wmaToString(wma));
			return new ContextUpdate() {

				@Override
				public void doUpdate(AbductionEnginePrx engine) {

					ModalisedAtom matom = TermAtomFactory.modalisedAtom(
							new Modality[] {
								Modality.Understanding,
								Modality.Truth
							},
							TermAtomFactory.atom("intention", new Term[] {
								TermAtomFactory.term(nominal),
								ConversionUtils.workingMemoryAddressToTerm(wma)
							}));

					engine.clearAssumabilityFunction("intention_creation");

					double weight = 1.0;
					getLogger().debug("adding intention ID atom to the assumability function: " + PrettyPrint.modalisedAtomToString(matom) + " @ " + weight);
					engine.addAssumable("intention_creation", matom, (float) weight);
				}
				
			};
		}

	};

	public class NewBeliefAssertionSolver extends AbstractAssertionSolver<NewBeliefAtom> {

		public NewBeliefAssertionSolver() {
			super(new NewBeliefAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(final NewBeliefAtom a) {
//			if (a.getIntentionAddress() == null) {
//				getLogger().error("intention address is null");
//				return null;
//			}
			if (a.getBeliefAddress() != null) {
				getLogger().error("working memory address is already set");
				return null;
			}

			final WorkingMemoryAddress newWma = newWorkingMemoryAddress();
			final NewBeliefAtom newAtom = new NewBeliefAtom(a.getIntentionAddress(), newWma, a.getEpistemicStatus());

			getLogger().info("generated a new belief WMA: " + wmaToString(newWma));

			return new ContextUpdate() {

				@Override
				public void doUpdate(AbductionEnginePrx engine) {
					ModalisedAtom matom = TermAtomFactory.modalisedAtom(
							new Modality[] {
								Modality.Understanding
							},
							TermAtomFactory.atom("new_belief_id", new Term[] {
								ConversionUtils.workingMemoryAddressToTerm(newWma),
								ConversionUtils.epistemicStatusToTerm(a.getEpistemicStatus())
							}));

					engine.clearAssumabilityFunction("belief_creation");

					double weight = 1.0;
					getLogger().debug("adding belief ID atom to the assumability function: " + PrettyPrint.modalisedAtomToString(matom) + " @ " + weight);
					engine.addAssumable("belief_creation", matom, (float) weight);

				}
				
			};
		}

	}

	public class ReferenceResolutionAssertionSolver extends AbstractAssertionSolver<AssertedReferenceAtom> {

		public ReferenceResolutionAssertionSolver() {
			super(new AssertedReferenceAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(AssertedReferenceAtom a) {
			String nominal = a.getNominal();
			if (nominal == null) {
				getLogger().error("nominal undetermined");
				return null;
			}

			RemappedSLF remapped = findSelectedLogicalFormByContainedNominal(nominal);
			if (remapped == null) {
				getLogger().error("no LF containing " + nominal + " (after unmapping) found");
				return null;
			}

			ReferenceResolutionRequest rr = rrExtractor.extractReferenceResolutionRequest(remapped.slf.lform, remapped.remapper.unremap(nominal), new TimeInterval(remapped.slf.ival));
			if (rr == null) {
				getLogger().error("extracted ReferenceResolutionRequest is null");
				return null;
			}
			rr.nom = remapped.remapper.remap(rr.nom);

			WorkingMemoryAddress wma = new WorkingMemoryAddress(newDataID(), getSubarchitectureID());
			ResultGatherer<ReferenceResolutionResultWrapper> gatherer = new ResultGatherer(wma, new MixingCombinator(2));
			assert gatherer != null;
			gatherers.put(wma, gatherer);

			ReferenceResolutionResultWrapper result = null;

			try {
				getLogger().info("adding a ReferenceResolutionRequest to the WM: " + ReferenceUtils.resolutionRequestToString(rr));
				addToWorkingMemory(wma, rr);
			}
			catch (SubarchitectureComponentException ex) {
				logException(ex);
			}

			result = gatherer.ensureStabilization(5, TimeUnit.SECONDS);
			final String nom = getNominalInTheRequest(wma);
			assert nom != null;
			stopGathererObservation(wma);

			getLogger().debug("got " + gatherer.getNumOfResults() + " results in total");
			if (gatherer.wasStabilizedByResult()) {
				getLogger().info("reason for resume: the results were good enough");
			}
			else {
				getLogger().info("reason for resume: timeouted while listening in hope for better results");
			}

			if (result != null) {
				final ReferenceResolutionResult refs = result.getResult();
				getLogger().info("this is the gathered reference resolution result:\n" + ReferenceUtils.resolutionResultToString(refs));
				return new ContextUpdate() {

					@Override
					public void doUpdate(AbductionEnginePrx engine) {
						getLogger().info("updating reference resolution");

						engine.clearAssumabilityFunction("reference_resolution");
						Map<String, Set<ModalisedAtom>> disj = new HashMap<String, Set<ModalisedAtom>>();

//						String nom = refs.nom;
						for (EpistemicReferenceHypothesis hypo : refs.hypos) {

							ModalisedAtom rma = ReferentOfAtom.newReferentOfAtom(nom, hypo.referent, hypo.epst).toModalisedAtom();

							getLogger().debug("adding reference hypothesis: " + PrettyPrint.modalisedAtomToString(rma) + " @ p=" + hypo.score);
							engine.addAssumable("reference_resolution", rma, (float) AbducerUtils.probToWeight(hypo.score));

							Set<ModalisedAtom> dj = disj.get(nom);
							if (dj != null) {
								dj.add(rma);
							}
							else {
								dj = new HashSet<ModalisedAtom>();
								dj.add(rma);
							}
							disj.put(nom, dj);
						}

						for (String n : disj.keySet()) {
							Set<ModalisedAtom> dj = disj.get(n);
							DisjointDeclaration dd = new DisjointDeclaration();
							dd.atoms = new ArrayList<ModalisedAtom>(dj);
							getLogger().debug("adding a disjoint declaration for " + n + " (" + dj.size() + " entries)");
							engine.addDisjointDeclaration(dd);
						}
						getLogger().info("done context update for reference resolution");
					}

				};
			}
			else {
				getLogger().error("result was null");
				return null;
			}
		}

	};

	public class WhQUDAssertionSolver extends AbstractAssertionSolver<QUDOpenAtom> {

		public WhQUDAssertionSolver() {
			super(new QUDOpenAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(QUDOpenAtom a) {
			final String nominal = a.getNominal();
			if (nominal == null) {
				getLogger().error("nominal undetermined");
				return null;
			}
			final WorkingMemoryAddress intWma = a.getIntentionAddress();
//			if (intWma == null) {
//				getLogger().error("intention address undetermined");
//				return null;
//			}

			final Set<WorkingMemoryAddress> wmas = openIntentionsToAct.keySet();
			getLogger().debug("got " + wmas.size() + " open IntentionsToAct");

			return new ContextUpdate() {

				@Override
				public void doUpdate(AbductionEnginePrx engine) {
					for (WorkingMemoryAddress wma : wmas) {
						IntentionToAct actint = openIntentionsToAct.get(wma);
						if (actint != null) {
							getLogger().debug("expanding IntentionToAct " + wmaToString(wma));
							ModalisedAtom matom = new QUDOpenAtom(wma, nominal, intWma, null).toModalisedAtom();
							engine.addFact(matom);
							for (ModalisedAtom fact : RobotCommunicativeAction.intentionToActToFacts(wma, actint)) {
								engine.addFact(fact);
							}
						}
					}
				}
				
			};
		}
		
	};

	public class PolarQUDAssertionSolver extends AbstractAssertionSolver<QUDPolarAtom> {

		public PolarQUDAssertionSolver() {
			super(new QUDPolarAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(QUDPolarAtom a) {
			final String nominal = a.getNominal();
			if (nominal == null) {
				getLogger().error("nominal undetermined");
				return null;
			}
			final WorkingMemoryAddress intWma = a.getIntentionAddress();
//			if (intWma == null) {
//				getLogger().error("intention address undetermined");
//				return null;
//			}

			final Set<WorkingMemoryAddress> wmas = openIntentionsToAct.keySet();
			getLogger().debug("got " + wmas.size() + " open IntentionsToAct");

			return new ContextUpdate() {

				@Override
				public void doUpdate(AbductionEnginePrx engine) {
					for (WorkingMemoryAddress wma : wmas) {
						IntentionToAct actint = openIntentionsToAct.get(wma);
						if (actint != null) {
							getLogger().debug("expanding IntentionToAct " + wmaToString(wma));
							ModalisedAtom matom = new QUDPolarAtom(null, null, wma, nominal, intWma, null, null, null).toModalisedAtom();
							engine.addFact(matom);
							for (ModalisedAtom fact : RobotCommunicativeAction.intentionToActToFacts(wma, actint)) {
								engine.addFact(fact);
							}
						}
					}
				}
				
			};
		}
		
	};

	public class TypeOfQUDAssertionSolver extends AbstractAssertionSolver<TypeOfQUDAtom> {

		public TypeOfQUDAssertionSolver() {
			super(new TypeOfQUDAtom.Matcher());
		}

		@Override
		public ContextUpdate solveFromParsed(TypeOfQUDAtom a) {
//			final String nominal = a.getNominal();
//			if (nominal == null) {
//				getLogger().error("nominal undetermined");
//				return null;
//			}
			final WorkingMemoryAddress intWma = a.getIntentionAddress();
//			if (intWma == null) {
//				getLogger().error("intention address undetermined");
//				return null;
//			}

			final Set<WorkingMemoryAddress> wmas = openIntentionsToAct.keySet();
			getLogger().debug("got " + wmas.size() + " open IntentionsToAct");

			return new ContextUpdate() {

				@Override
				public void doUpdate(AbductionEnginePrx engine) {
					for (WorkingMemoryAddress wma : wmas) {
						IntentionToAct actint = openIntentionsToAct.get(wma);
						String type = actint.stringContent.get("type");

						if (type != null && actint != null) {
							getLogger().debug("expanding IntentionToAct " + wmaToString(wma));
							ModalisedAtom matom = new TypeOfQUDAtom(wma, intWma, type).toModalisedAtom();
							getLogger().debug("will add the following fact: " + PrettyPrint.modalisedAtomToString(matom));
							engine.addFact(matom);
							for (ModalisedAtom fact : RobotCommunicativeAction.intentionToActToFacts(wma, actint)) {
								getLogger().debug("will add the following fact: " + PrettyPrint.modalisedAtomToString(fact));
								engine.addFact(fact);
							}
						}
					}
				}
				
			};
		}
		
	};

	@Deprecated
	public String getNominalInTheRequest(WorkingMemoryAddress wma) {
		try {
			ReferenceResolutionRequest request = getMemoryEntry(wma, ReferenceResolutionRequest.class);
			return request.nom;
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
			return null;
		}
	}

	public void stopGathererObservation(WorkingMemoryAddress wma) {
		gatherers.remove(wma);
		try {
			deleteFromWorkingMemory(wma);
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
		}
	}

	public ModalisedAtom selectedLFToGoal(SelectedLogicalForm slf) {
		LogicalForm lf = slf.lform;

		NominalRemapper remapper = new BasicNominalRemapper(remapIndex++);
		String nom = lf.root.nomVar;
		String newNom = remapper.remap(nom);
//		renamedNomToNom.put(newNom, nom);

		nomToLFMap.put(newNom, new RemappedSLF(slf, remapper));

		ModalisedAtom goal = TermAtomFactory.modalisedAtom(
				new Modality[] {
					Modality.Understanding,
					Modality.Event
				},
				TermAtomFactory.atom("utter", new Term[] {
					TermAtomFactory.term(IntentionManagementConstants.humanAgent),
					TermAtomFactory.term(IntentionManagementConstants.thisAgent),
					TermAtomFactory.term(newNom)
				}));

		return goal;
	}

	protected WorkingMemoryAddress newWorkingMemoryAddress() {
		return new WorkingMemoryAddress(idGen.newIdentifier(), getSubarchitectureID());
	}

	private RemappedSLF findSelectedLogicalFormByContainedNominal(String nominal) {
		RemappedSLF result = null;
		for (RemappedSLF remapped : nomToLFMap.values()) {
			LogicalForm lf = remapped.slf.lform;
			NominalRemapper remapper = remapped.remapper;
			Iterator<LFNominal> iter = LFUtils.lfGetNominals(lf);
			while (iter.hasNext()) {
				LFNominal lfn = iter.next();
				if (remapper.remap(lfn.nomVar).equals(nominal)) {
					return remapped;
				}
			}
		}
		return result;
	}

	private static class ReferenceResolutionResultWrapper extends CASTResultWrapper<ReferenceResolutionResult> {

		public ReferenceResolutionResultWrapper(ReferenceResolutionResult result) {
			super(result);
		}

		@Override
		public WorkingMemoryAddress getRequestAddress() {
			return getResult().requestAddress;
		}

	}

	private static class RemappedSLF {
		public final SelectedLogicalForm slf;
		public final NominalRemapper remapper;
		public RemappedSLF(SelectedLogicalForm slf, NominalRemapper remapper) {
			this.slf = slf;
			this.remapper = remapper;
		}
	}

	private class MixingCombinator implements ResultCombinator<ReferenceResolutionResultWrapper> {

		private final int minCount;
		private int count = 0;
		private ReferenceResolutionResultWrapper result;
		
		public MixingCombinator(int minCount) {
			this.minCount = minCount;
			ReferenceResolutionResult rr = new ReferenceResolutionResult(null, null, "combined", new LinkedList<EpistemicReferenceHypothesis>());
			result = new ReferenceResolutionResultWrapper(rr);
		}

		@Override
		synchronized public void addResult(ReferenceResolutionResultWrapper added) {
			getLogger().debug("got a new reference resolution result: " + ReferenceUtils.resolutionResultToString(added.getResult()));
			++count;
			if (result.getResult().nom == null) {
				result.getResult().nom = added.getResult().nom;
			}
			if (result.getResult().requestAddress == null) {
				result.getResult().requestAddress = added.getResult().requestAddress;
			}

			for (EpistemicReferenceHypothesis hypo : added.getResult().hypos) {
				addHypo(hypo);
			}
			adjustMaximum();
		}

		private void adjustMaximum() {
			double max = 0.0;
			for (EpistemicReferenceHypothesis h : result.getResult().hypos) {
				if (h.score > max) {
					max = h.score;
				}
			}
			if (max > 0.0 && max > 1.0) {
				for (EpistemicReferenceHypothesis h : result.getResult().hypos) {
					h.score /= max;
				}
			}
		}

		private void addHypo(EpistemicReferenceHypothesis hypo) {
			WorkingMemoryAddress hypoWMA = formulaToWMA(hypo.referent);

			if (hypoWMA != null) {
				boolean boosted = false;
				for (EpistemicReferenceHypothesis h : result.getResult().hypos) {
					if (!boosted) {
						WorkingMemoryAddress oldWMA = formulaToWMA(h.referent);
						if (oldWMA != null && oldWMA.equals(hypoWMA)) {
							h.score += hypo.score;
							boosted = true;
						}
					}
				}
				if (!boosted) {
					result.getResult().hypos.add(hypo);
				}
			}
			else {
				result.getResult().hypos.add(hypo);
			}
		}

		@Override
		public ReferenceResolutionResultWrapper toResult() {
			return result;
		}

		@Override
		public boolean resultsSufficient() {
			return count >= minCount;
		}
		
	}

	public static WorkingMemoryAddress formulaToWMA(dFormula f) {
		WorkingMemoryAddress wma = null;
		if (f instanceof PointerFormula) {
			PointerFormula pf = (PointerFormula) f;
			wma = pf.pointer;
		}
		return wma;
	}

}
