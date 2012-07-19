package de.dfki.lt.tr.dialogue.intentions.inst;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.AbstractRichIntention;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;

public class AutonomousLearningNotificationIntention extends AbstractRichIntention {

    private final WorkingMemoryAddress about;
    private final String label;
    
    public AutonomousLearningNotificationIntention(WorkingMemoryAddress about, String label) {
        this.about = about;
        this.label = label;
    }
    
    public WorkingMemoryAddress getEntity() {
        return about;
    }
    
    public String getLabel() {
        return label;
    }
    
    public static class Transcoder
            extends AbstractBaseIntentionTranscoder<AutonomousLearningNotificationIntention> {

        public static final String TYPE = "autonomous-notification";
        public static final String SKEY_LABEL = "label";
        
        @Override
        protected void encodeContent(AutonomousLearningNotificationIntention rich, BaseIntention poor) {
            poor.stringContent.put(SKEY_TYPE, TYPE);
            poor.stringContent.put(SKEY_LABEL, rich.getLabel());
            poor.addressContent.put(PKEY_ABOUT, rich.getEntity());
        }

        public AutonomousLearningNotificationIntention tryDecode(BaseIntention poor) {
            try {
                DecodingUtils.stringCheck(poor, SKEY_TYPE, TYPE);
                String label = DecodingUtils.stringGet(poor, SKEY_LABEL);
                WorkingMemoryAddress about = DecodingUtils.addressGet(poor, PKEY_ABOUT);
                return new AutonomousLearningNotificationIntention(about, label);
            }
            catch (DecodingException ex) {
                    return null;
            }
        }
        
    }
    
}
