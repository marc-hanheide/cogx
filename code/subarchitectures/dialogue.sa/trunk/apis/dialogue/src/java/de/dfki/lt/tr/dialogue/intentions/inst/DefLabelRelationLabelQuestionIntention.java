package de.dfki.lt.tr.dialogue.intentions.inst;

import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;

public class DefLabelRelationLabelQuestionIntention extends LabelQuestionIntention {

    protected final String relation;
    protected final String otherLabel;

    public DefLabelRelationLabelQuestionIntention(String label, String relation, String otherLabel) {
        super(label);
        this.relation = relation;
        this.otherLabel = otherLabel;
    }

    public String getRelation() {
        return relation;
    }
    
    public String getOtherLabel() {
        return otherLabel;
    }
    
    public static class Transcoder
            extends AbstractBaseIntentionTranscoder<DefLabelRelationLabelQuestionIntention> {

        public static final Transcoder INSTANCE = new Transcoder();

        public final String SKEY_RELATION = "relation";
        public final String SKEY_OTHERLABEL = "otherlabel";

        @Override
        protected void encodeContent(DefLabelRelationLabelQuestionIntention rich, BaseIntention poor) {
            LabelQuestionIntention.Transcoder.INSTANCE.encodeContent(rich, poor);
            poor.stringContent.put(SKEY_RELATION, rich.getRelation());
            poor.stringContent.put(SKEY_OTHERLABEL, rich.getOtherLabel());
        }

        @Override
        public DefLabelRelationLabelQuestionIntention tryDecode(BaseIntention poor) {
            try {
                LabelQuestionIntention qint = LabelQuestionIntention.Transcoder.INSTANCE.tryDecode(poor);
                if (qint != null) {
                    String relation = DecodingUtils.stringGet(poor, SKEY_RELATION);
                    String otherLabel = DecodingUtils.stringGet(poor, SKEY_OTHERLABEL);
                    return new DefLabelRelationLabelQuestionIntention(qint.getLabel(), relation, otherLabel);
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
