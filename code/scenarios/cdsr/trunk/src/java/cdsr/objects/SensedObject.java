/**
 * 
 */
package cdsr.objects;

import java.awt.geom.Line2D;
import java.util.ArrayList;

/**
 * An object sensed by the robot.
 * 
 * @author nah
 * 
 */
public class SensedObject extends AbstractPhysicalObject {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private final String m_type;

	/**
	 * @param _id
	 * @param _edges
	 */
	public SensedObject(String _id, ArrayList<Line2D.Double> _edges, String _type) {
		super(_id, _edges);
		m_type = _type;
	}

	
	public String getType() {
		return m_type;
	}
}
