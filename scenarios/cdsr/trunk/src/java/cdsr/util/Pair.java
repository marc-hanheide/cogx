/**
 * 
 */
package cdsr.util;

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
	
	@Override
	public boolean equals(Object _obj) {
		if(_obj instanceof Pair) {
			Pair<?, ?> p = (Pair<?,?>) _obj;
			return p.m_first.equals(m_first) && p.m_second.equals(m_second);
		}
		else {
			return false;
		}
	}

}
