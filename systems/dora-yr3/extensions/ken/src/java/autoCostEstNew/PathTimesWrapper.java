package autoCostEstNew;

import java.io.Serializable;
import java.util.Vector;


/**
 * convenience class, wraps a set of PathTimes up for easier loading/saving
 * @author ken
 *
 */
public class PathTimesWrapper implements Serializable{

	
	
	private static final long serialVersionUID = 1L;
	private Vector<PathTimes> pathTimes;
	
	public PathTimesWrapper(Vector<PathTimes> pathTimes){
		this.pathTimes = pathTimes;
	}
	
	public Vector<PathTimes> getPathTimes(){
		return pathTimes;
	}
}
