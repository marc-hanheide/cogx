package motivation.components.generators;

import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import motivation.slice.Motive;

public abstract class AbstractBeliefMotiveGenerator<M extends Motive, T extends dBelief>
		extends AbstractWMEntryMotiveGenerator<M, T> {

	protected final String beliefType;

	public AbstractBeliefMotiveGenerator(String _beliefType,
			Class<M> _motiveClass, Class<T> _beliefClass) {
		super(_motiveClass, _beliefClass);
		beliefType = _beliefType;
	}

	protected boolean processEntry(T _entry) {
		return _entry.type.equals(beliefType);
	}
	
}
