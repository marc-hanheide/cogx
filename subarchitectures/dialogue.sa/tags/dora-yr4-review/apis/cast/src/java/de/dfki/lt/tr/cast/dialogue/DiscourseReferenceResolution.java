package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.distribs.*;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionToAct;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.dialogue.DiscourseReferenceResolution.LastMentionedReferenceResolver;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolver;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.util.EpistemicStatusFactory;

public class DiscourseReferenceResolution
extends AbstractReferenceResolutionComponent<LastMentionedReferenceResolver> {

	private WorkingMemoryAddress lastMentioned;

	public DiscourseReferenceResolution() {
		super();
		lastMentioned = null;
	}

	@Override
	protected LastMentionedReferenceResolver initResolver() {
		return new LastMentionedReferenceResolver();
	}

	@Override
	protected void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				BaseIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

							@Override
							public void execute(WorkingMemoryAddress addr) {
								try {
									BaseIntention bint = getMemoryEntry(addr, BaseIntention.class);

									WorkingMemoryAddress aboutAddr = bint.addressContent.get("about");
									if (aboutAddr != null) {
										setLastMentioned(aboutAddr);
									}

								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}
							
						});
					}
				});
	}

	private void setLastMentioned(WorkingMemoryAddress wma) {
		lastMentioned = wma;
	}

	private WorkingMemoryAddress getLastMentioned() {
		return lastMentioned;
	}

        public static boolean isVisible(dBelief bel) {
            if ("visualobject".equals(bel.type)) {
                if (bel.content instanceof CondIndependentDistribs) {
                    CondIndependentDistribs d = (CondIndependentDistribs) bel.content;
                    ProbDistribution pd = d.distribs.get("presence");
                    if (pd != null && pd instanceof BasicProbDistribution) {
                        BasicProbDistribution presd = (BasicProbDistribution) pd;
                        if (presd.values instanceof FormulaValues) {
                            FormulaValues fvs = (FormulaValues) presd.values;
                            if (fvs.values.size() == 1) {
                                FormulaProbPair fpp = fvs.values.get(0);
                                if (fpp.val instanceof ElementaryFormula) {
                                    ElementaryFormula formula = (ElementaryFormula) fpp.val;
                                    return "visible".equals(formula.prop);
                                }
                            }
                        }
                    }
                }
            }
            return false;
        }

        private boolean isVisibleVisualObject(WorkingMemoryAddress wma) {
            try {
                dBelief bel = getMemoryEntry(wma, dBelief.class);
                return isVisible(bel);
            }
            catch (SubarchitectureComponentException ex) {
                logException(ex);
            }
            return false;
        }
        
	public class LastMentionedReferenceResolver implements ReferenceResolver {

		private final EpistemicStatus epst = EpistemicStatusFactory.newSharedEpistemicStatus(IntentionManagementConstants.thisAgent, IntentionManagementConstants.humanAgent);

		@Override
		public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr, WorkingMemoryAddress origin) {
			ReferenceResolutionResult result = ReferenceUtils.newEmptyResolutionResult(rr, origin, "last-mention");
			if (rr.sort.equals(ReferenceResolver.SORT_DISCOURSE)) {
				WorkingMemoryAddress qudAboutAddr = getLastMentioned();
				if (qudAboutAddr != null && isVisibleVisualObject(qudAboutAddr)) {
					dFormula referent = new PointerFormula(0, qudAboutAddr, "");
					double score = 10.0;
					result.hypos.add(new EpistemicReferenceHypothesis(epst, referent, score));
				}
			}
			return result;
		}
		
	}

}
