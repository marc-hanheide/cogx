package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.slice.distribs.*;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionToAct;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.dialogue.OthernessReferenceResolution.OthernessReferenceResolver;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.ref.*;
import de.dfki.lt.tr.dialogue.ref.util.ReferenceUtils;
import de.dfki.lt.tr.dialogue.util.EpistemicStatusFactory;
import eu.cogx.beliefs.slice.MergedBelief;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class OthernessReferenceResolution
extends AbstractReferenceResolutionComponent<OthernessReferenceResolver> {

	private WorkingMemoryAddress lastMentioned;
        private WMView<MergedBelief> view;

	public OthernessReferenceResolution() {
		super();
		lastMentioned = null;
                view = null;
	}

	@Override
	protected OthernessReferenceResolver initResolver() {
		return new OthernessReferenceResolver();
	}

	@Override
	protected void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				BaseIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new AbstractDialogueComponent.ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

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
                
            view = WMView.<MergedBelief>create(this, MergedBelief.class);
            try {
                view.start();
            }
            catch (UnknownSubarchitectureException ex) {
                logException(ex);
                scheduleOwnDeath();
            }
	}

	private void setLastMentioned(WorkingMemoryAddress wma) {
		lastMentioned = wma;
	}

	private WorkingMemoryAddress getLastMentioned() {
		return lastMentioned;
	}

        private boolean isVisibleVisualObject(WorkingMemoryAddress wma) {
            try {
                dBelief bel = getMemoryEntry(wma, dBelief.class);
                return CogXReferringUtils.canBeReferredTo(bel);
            }
            catch (SubarchitectureComponentException ex) {
                logException(ex);
            }
            return false;
        }

        public static boolean isOthernessRequest(ReferenceResolutionRequest rr) {
            boolean result = false;
//            if (ReferenceResolver.SORT_DISCOURSE.equals(rr.sort)) {
//                return false;
//            }
            for (Constraint c : rr.constraints) {
                if ("otherness".equals(c.feature)) {
                    result = true;
                }
            }
            return result;
        }
        
	public class OthernessReferenceResolver implements ReferenceResolver {

		private final EpistemicStatus epst = EpistemicStatusFactory.newSharedEpistemicStatus(IntentionManagementConstants.thisAgent, IntentionManagementConstants.humanAgent);

                protected List<EpistemicReferenceHypothesis> getHypotheses() {
                    List<EpistemicReferenceHypothesis> result = new LinkedList<EpistemicReferenceHypothesis>();

                    // TODO: verify that the iterator is thread-safe
                    Iterator<Map.Entry<WorkingMemoryAddress, MergedBelief>> it = view.entrySet().iterator();

                    while (it.hasNext()) {
                        Map.Entry<WorkingMemoryAddress, MergedBelief> entry = it.next();
                        WorkingMemoryAddress wma = entry.getKey();
                        dBelief bel = entry.getValue();
                        
                        if (CogXReferringUtils.canBeReferredTo(bel)) {
                            if (!wma.equals(lastMentioned)) {
                                getLogger().trace("belief " + wmaToString(wma) + " is a POSSIBLE REFERENT");
                                result.add(new EpistemicReferenceHypothesis(epst, new PointerFormula(0, wma, ""), 10.0));
                            }
                            else {
                                getLogger().trace("belief " + wmaToString(wma) + " not eligible: this is what we talked about last");
                            }
                        }
                        else {
                            getLogger().trace("belief " + wmaToString(wma) + " not eligible: cannot be referred to");
                        }
                    }
                    getLogger().trace("we're done here");

                    return result;
                }
                
               
		@Override
		public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr, WorkingMemoryAddress origin) {
                        ReferenceResolutionResult result = ReferenceUtils.newEmptyResolutionResult(rr, origin, "otherness");
                        if (isOthernessRequest(rr)) {
                            log("this is a request about otherness");
                            result.hypos.addAll(getHypotheses());
                        }
                        else {
                            log("request doesn't seem to be about otherness");
                        }
                        return result;
		}
		
	}

}
