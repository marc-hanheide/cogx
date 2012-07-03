package de.dfki.lt.tr.dialogue.intentions.inst;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
import de.dfki.lt.tr.dialogue.intentions.AbstractBaseIntentionTranscoder;
import de.dfki.lt.tr.dialogue.intentions.DecodingException;
import de.dfki.lt.tr.dialogue.intentions.DecodingUtils;

public class HypothesisGenerationVerificationQuestionIntention extends QuestionIntention {

    protected final String label;
    protected final String relation;
    protected final WorkingMemoryAddress relatedEntity;
    protected final WorkingMemoryAddress about;

    public HypothesisGenerationVerificationQuestionIntention(String label, String relation, WorkingMemoryAddress relationTo, WorkingMemoryAddress about) {
        super();
        this.relation = relation;
        this.label = label;
        this.relatedEntity = relationTo;
        this.about = about;
    }

    public String getRelation() {
        return relation;
    }
    
    public String getLabel() {
        return label;
    }
    
    public WorkingMemoryAddress getRelatedEntity() {
        return relatedEntity;
    }
    
    public WorkingMemoryAddress getEntity() {
        return about;
    }
    
    public static class Transcoder
            extends AbstractBaseIntentionTranscoder<HypothesisGenerationVerificationQuestionIntention> {

        public static final Transcoder INSTANCE = new Transcoder();

        public final String SKEY_LABEL = "label";
        public final String SKEY_RELATION = "relation";
        public final String PKEY_RELATED_ENTITY = "relatedentity";
        
        @Override
        protected void encodeContent(HypothesisGenerationVerificationQuestionIntention rich, BaseIntention poor) {
            QuestionIntention.Transcoder.INSTANCE.encodeContent(rich, poor);
            poor.stringContent.put(SKEY_SUBTYPE, "polar");
            poor.stringContent.put(SKEY_SUBSUBTYPE, "hgv");
            poor.stringContent.put(SKEY_RELATION, rich.getRelation());
            poor.stringContent.put(SKEY_LABEL, rich.getLabel());
            poor.addressContent.put(PKEY_RELATED_ENTITY, rich.getRelatedEntity());
            poor.addressContent.put(PKEY_ABOUT, rich.getRelatedEntity());
        }

        @Override
        public HypothesisGenerationVerificationQuestionIntention tryDecode(BaseIntention poor) {
            try {
                QuestionIntention qint = QuestionIntention.Transcoder.INSTANCE.tryDecode(poor);
                    if (qint != null) {
                        DecodingUtils.stringCheck(poor, SKEY_SUBTYPE, "polar");
                        DecodingUtils.stringCheck(poor, SKEY_SUBSUBTYPE, "hgv");
                        String relation = DecodingUtils.stringGet(poor, SKEY_RELATION);
                        String label = DecodingUtils.stringGet(poor, SKEY_LABEL);
                        WorkingMemoryAddress relatedEntity = DecodingUtils.addressGet(poor, PKEY_RELATED_ENTITY);
                        WorkingMemoryAddress about = DecodingUtils.addressGet(poor, PKEY_ABOUT);
                        return new HypothesisGenerationVerificationQuestionIntention(label, relation, relatedEntity, about);
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
