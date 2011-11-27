/**
 * 
 */
package cdsr.objects;

import java.awt.geom.Line2D.Double;
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
	 * @param _walls
	 */
	public SensedObject(String _id, ArrayList<Double> _walls, String _type) {
		super(_id, _walls);
		m_type = _type;
	}

	
	public String getType() {
		return m_type;
	}
}
