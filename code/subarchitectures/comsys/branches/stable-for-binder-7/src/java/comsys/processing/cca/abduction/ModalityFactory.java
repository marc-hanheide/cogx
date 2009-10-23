package comsys.processing.cca.abduction;

import Abducer.AttStateModality;
import Abducer.EventModality;
import Abducer.GenerationModality;
import Abducer.InfoModality;
import Abducer.IntentionModality;
import Abducer.KModality;
import Abducer.ModalityType;
import Abducer.UnderstandingModality;
import beliefmodels.adl.AgentStatus;
import beliefmodels.adl.AttributedAgentStatus;
import beliefmodels.adl.MutualAgentStatus;
import beliefmodels.adl.PrivateAgentStatus;

public abstract class ModalityFactory {

	/**
	 * Return the "understanding" modality.
	 * 
	 * @return
	 */
	public static UnderstandingModality understandingModality() {
		UnderstandingModality m = new UnderstandingModality();
		m.type = ModalityType.Understanding;
		return m;
	}

	/**
	 * Return the "generation" modality.
	 * 
	 * @return
	 */
	public static GenerationModality generationModality() {
		GenerationModality m = new GenerationModality();
		m.type = ModalityType.Generation;
		return m;
	}

	/**
	 * Return the "event" modality for observed events.
	 * 
	 * @return
	 */
	public static EventModality eventModality() {
		EventModality m = new EventModality();
		m.type = ModalityType.Event;
		return m;
	}
	
	/**
	 * Return the "intention" modality for recognised intentions.
	 * 
	 * @return
	 */
	public static IntentionModality intentionModality() {
		IntentionModality m = new IntentionModality();
		m.type = ModalityType.Intention;
		return m;
	}

	/**
	 * Return the "info" modality for information content.
	 * 
	 * @return
	 */
	public static InfoModality infoModality() {
		InfoModality m = new InfoModality();
		m.type = ModalityType.Info;
		return m;
	}

	/**
	 * Return the "attention" modality for attention-state-related content.
	 * 
	 * @return
	 */
	public static AttStateModality attStateModality() {
		AttStateModality m = new AttStateModality();
		m.type = ModalityType.AttState;
		return m;
	}

	/**
	 * Return the corresponding abducer representation of an agent reference.
	 * @param ag agent
	 * @return
	 */
	private static Abducer.Agent toAbducerAgent(beliefmodels.adl.Agent ag) {
		// TODO: what exceptions does this raise?
		return Abducer.Agent.convert(ag.id);
	}

	/**
	 * Return the "K" modality for beliefs.
	 * 
	 * @param as agent status
	 * @return
	 */
	public static KModality kModality(AgentStatus as) {
		KModality m = new KModality();
		m.type = ModalityType.K;
		if (as instanceof AttributedAgentStatus) {
			m.share = Abducer.Sharing.Attribute;
			m.ag = toAbducerAgent(((AttributedAgentStatus) as).ag);
			m.ag2 = toAbducerAgent(((AttributedAgentStatus) as).ag2);
		}
		else if (as instanceof PrivateAgentStatus) {
			m.share = Abducer.Sharing.Private;
			m.ag = toAbducerAgent(((PrivateAgentStatus) as).ag);
			m.ag2 = Abducer.Agent.human;
		}
		else if (as instanceof MutualAgentStatus) {
			m.share = Abducer.Sharing.Mutual;
			m.ag = Abducer.Agent.robot;
			m.ag2 = Abducer.Agent.human;
		}
		else {
			//System.err.println("unknown AgentStatus in AbducerUtils.kModality() !");
			return null;
		}
		//System.err.println("AU.kMo: share=" + m.share.toString() + ", act=" + m.act.toString() + ", pat=" + m.pat.toString());
		return m;
	}

	
}
