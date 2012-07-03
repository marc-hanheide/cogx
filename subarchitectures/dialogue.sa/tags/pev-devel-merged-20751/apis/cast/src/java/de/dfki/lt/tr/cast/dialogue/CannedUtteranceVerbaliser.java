package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.slice.produce.CannedUtterance;
import de.dfki.lt.tr.dialogue.util.VerbalisationUtils;

public class CannedUtteranceVerbaliser
extends AbstractDialogueComponent {

    @Override
    public void onStart() {
        super.onStart();
        
        addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
            CannedUtterance.class, WorkingMemoryOperation.ADD),
            new WorkingMemoryChangeReceiver() {
                    @Override
                    public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                        addTask(new ProcessingTaskWithData<WorkingMemoryChange>(_wmc) {

                            @Override
                            public void execute(WorkingMemoryChange wmc) {
                                try {
                                    CannedUtterance utt = getMemoryEntry(wmc.address, CannedUtterance.class);
                                    if (utt.content != null) {
                                        VerbalisationUtils.verbaliseString(CannedUtteranceVerbaliser.this, utt.content);
                                    }
                                }
                                catch (SubarchitectureComponentException ex) {
                                    getLogger().error("exception in handling canned utterance", ex);
                                }
                            }
                        });
                    }
            });

    }
    
}
