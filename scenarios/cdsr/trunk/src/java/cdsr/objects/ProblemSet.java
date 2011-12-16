package cdsr.objects;

import java.io.Serializable;
import java.util.ArrayList;

/**
 * A collection of CDSR objects that define a complete problem set.
 * 
 * @author nah
 * 
 */
public class ProblemSet implements Serializable {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	private final Room m_room;
	
	private final ArrayList<SensedObject> m_objects;
	
	
	
	public Room getRoom() {
		return m_room;
	}

	public ArrayList<SensedObject> getObjects() {
		return m_objects;
	}

	public ArrayList<ObjectRelation> getRelations() {
		return m_relations;
	}

	private final ArrayList<ObjectRelation> m_relations;

	public ProblemSet(Room _room, ArrayList<SensedObject> _objects,
			ArrayList<ObjectRelation> _relations) {
		super();
		m_room = _room;
		m_objects = _objects;
		m_relations = _relations;
	}
	
	public void addObject(SensedObject _object) {
		m_objects.add(_object);
	}

	public void addObjects(ArrayList<SensedObject> _objects) {
		m_objects.addAll(_objects);
	}

}
