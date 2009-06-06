/**
 * 
 */
package cast.core;

/**
 * 
 * Arbitrary pair of things, useful for temporary storage of related things.
 * 
 * @author nah
 * 
 */
public class Pair<First, Second> {

	public First m_first;

	public Second m_second;

	public Pair(First _a, Second _b) {
		m_first = _a;
		m_second = _b;
	}

}
