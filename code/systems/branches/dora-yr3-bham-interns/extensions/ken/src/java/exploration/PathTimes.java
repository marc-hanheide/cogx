package exploration;

import java.io.Serializable;
import java.util.Vector;


/**
 * stores a edge, and all the runs across that edge
 * @author ken
 *
 */
public class PathTimes implements Serializable{

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private int a;//start point
	private int b;//end point
	private Vector<PathRun> runs;

	public PathTimes(int a, int b) {
		this.a = a;
		this.b = b;
		runs = new Vector<PathRun>();
	}

	public void add(PathRun run) {
		runs.add(run);
	}

	public PathRun getRun(int index) {
		return runs.get(index);
	}

	public Vector<PathRun> getRuns() {
		return runs;
	}

	public int getA() {
		return a;
	}

	public int getB() {
		return b;
	}
	@Override
	public String toString(){
		String returnV = " Path between "+ a + " & " + b;
		for(PathRun run: runs){
			returnV += "\n"+run.toString();
		}
		return returnV;
	}

}
