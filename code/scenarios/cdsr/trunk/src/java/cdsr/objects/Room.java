/**
 * 
 */
package cdsr.objects;

import java.awt.geom.Line2D;
import java.util.ArrayList;


/**
 * @author nah
 *
 */
public class Room extends AbstractPhysicalObject {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * @param _id
	 * @param _arrayList
	 */
	public Room(String _id, ArrayList<Line2D.Double> _walls) {
		super(_id, _walls);
	}

}
