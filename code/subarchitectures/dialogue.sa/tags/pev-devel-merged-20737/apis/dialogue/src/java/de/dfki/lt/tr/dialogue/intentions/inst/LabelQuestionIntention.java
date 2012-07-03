package de.dfki.lt.tr.dialogue.intentions.inst;

import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;

public class LabelQuestionIntention extends QuestionIntention {

    protected final String label;

    public LabelQuestionIntention(String label) {
        super();
        this.label = label;
    }

    public String getLabel() {
        return label;
    }


    public static class Transcoder
            extends AbstractBaseIntentionTranscoder<LabelQuestionIntention> {

        public static final Transcoder INSTANCE = new Transcoder();

        public final String SKEY_LABEL = "label";

        @Override
        protected void encodeContent(LabelQuestionIntention rich, BaseIntention poor) {
            QuestionIntention.Transcoder.INSTANCE.encodeContent(rich, poor);
            poor.stringContent.put(SKEY_LABEL, rich.getLabel());
            poor.stringContent.put(SKEY_SUBTYPE, "polar");
        }

        @Override
        public LabelQuestionIntention tryDecode(BaseIntention poor) {
            try {
                QuestionIntention qint = QuestionIntention.Transcoder.INSTANCE.tryDecode(poor);
                if (qint != null) {
                    DecodingUtils.stringCheck(poor, SKEY_SUBTYPE, "polar");
                    String feature = DecodingUtils.stringGet(poor, SKEY_LABEL);
                    return new LabelQuestionIntention(feature);
                }
                else {
                    return null;
                }
            }
            catch (DecodingException ex) {
                return null;
            }
        }
    }

}
