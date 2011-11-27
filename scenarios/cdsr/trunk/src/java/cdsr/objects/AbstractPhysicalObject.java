package cdsr.objects;

import java.awt.geom.Line2D;
import java.awt.geom.Line2D.Double;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Iterator;

/**
 * 
 * An object in our CDSR world is simply a collection of, possibly non-connected, lines, plus an id.
 * 
 * @author nah
 *
 */
public abstract class AbstractPhysicalObject implements Serializable, Iterable<Line2D.Double> {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * The ID of the object.
	 */
	private final String m_id;

	/**
	 * The 2D lines used to construct the object
	 */
	private final ArrayList<Line2D.Double> m_lines;

	
	/**
	 * Construct an object from an ID and a list of walls.
	 * 
	 * @param _id
	 * @param _walls
	 */
	public AbstractPhysicalObject(String _id, ArrayList<Line2D.Double> _walls) {
		m_id = _id;
		m_lines = _walls;
	}

	/**
	 * Unique ID of the room.
	 * 
	 * @return
	 */
	public String getID() {
		return m_id;
	}
	
	public ArrayList<Line2D.Double> getLines() {
		return m_lines;
	}

	/**
	 * Iterator over walls in the room.
	 */
	@Override
	public Iterator<Double> iterator() {
		return m_lines.iterator();
	}

}
