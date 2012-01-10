package cdsr.objects;

import java.io.Serializable;

/**
 * A relation (e.g. a spatial relation) between 2 objects.
 * 
 * @author nah
 * 
 */
public class ObjectRelation implements Serializable {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private final String m_id;
	private final String m_type;
	private final String m_start;
	private final String m_end;
	private final double m_strength;

	public ObjectRelation(String _id, String _type, String _start, String _end,
			double _strength) {
	  m_id = _id;
		m_type = _type;
		m_start = _start;
		m_end = _end;
		m_strength = _strength;
	}

	public String getID() {
	  return m_id;
	}
	
	public String getType() {
		return m_type;
	}

	public String getStart() {
		return m_start;
	}

	public String getEnd() {
		return m_end;
	}

	public double getStrength() {
		return m_strength;
	}

}
