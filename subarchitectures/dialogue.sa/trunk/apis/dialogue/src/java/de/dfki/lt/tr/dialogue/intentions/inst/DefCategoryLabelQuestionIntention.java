package de.dfki.lt.tr.dialogue.intentions.inst;

import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;

public class DefCategoryLabelQuestionIntention extends LabelQuestionIntention {

    protected final String category;

    public DefCategoryLabelQuestionIntention(String label, String category) {
        super(label);
        this.category = category;
    }

    public String getCategory() {
        return category;
    }
    
    public static class Transcoder
            extends AbstractBaseIntentionTranscoder<DefCategoryLabelQuestionIntention> {

        public static final Transcoder INSTANCE = new Transcoder();

        public final String SKEY_CATEGORY = "category";

        @Override
        protected void encodeContent(DefCategoryLabelQuestionIntention rich, BaseIntention poor) {
            LabelQuestionIntention.Transcoder.INSTANCE.encodeContent(rich, poor);
            poor.stringContent.put(SKEY_CATEGORY, rich.getCategory());
        }

        @Override
        public DefCategoryLabelQuestionIntention tryDecode(BaseIntention poor) {
            try {
                LabelQuestionIntention qint = LabelQuestionIntention.Transcoder.INSTANCE.tryDecode(poor);
                if (qint != null) {
                    String category = DecodingUtils.stringGet(poor, SKEY_CATEGORY);
                    return new DefCategoryLabelQuestionIntention(qint.getLabel(), category);
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
