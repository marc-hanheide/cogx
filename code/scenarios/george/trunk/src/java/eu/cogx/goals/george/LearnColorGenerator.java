/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.goals.george;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 *
 */
public class LearnColorGenerator extends LearnConceptGenerator {

	/* (non-Javadoc)
	 * @see eu.cogx.goals.george.LearnConceptGenerator#getConceptName()
	 */
	@Override
	protected String getConceptName() {
		return "color";
	}

	/* (non-Javadoc)
	 * @see eu.cogx.goals.george.LearnConceptGenerator#getGainPropertyName()
	 */
	@Override
	protected String getGainPropertyName() {
		return "gain-color";
	}

}
