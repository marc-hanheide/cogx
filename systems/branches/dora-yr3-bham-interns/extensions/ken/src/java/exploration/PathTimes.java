package exploration;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Vector;

import displays.PathVisualization;

/**
 * stores a edge, and all the runs across that edge
 * 
 * @author ken
 * 
 */
public class PathTimes implements Serializable {

	public static void main(String[] args) {

		Vector<PathTimes> pathTimes = new Vector<PathTimes>();
		try {
			// ObjectInputStream in = new ObjectInputStream(
			// new BufferedInputStream(new FileInputStream("timings.txt")));
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream(
							"timings.txt")));
			pathTimes = ((PathTimesWrapper) (in.readObject())).getPathTimes();

			in.close();
		} catch (FileNotFoundException e) {
			System.out.println("unable to find file, load failed");

		} catch (IOException e) {
			System.out.println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			System.out.println(e);
			e.printStackTrace();

		}
		System.out.println(pathTimes);
		PathTimes.printList(pathTimes);
		// pathTimes.remove(pathTimes.get(pathTimes.size()-1));
		// pathTimes.remove(pathTimes.get(pathTimes.size()-1));
		// pathTimes.remove(pathTimes.get(pathTimes.size()-1));
		// pathTimes.remove(pathTimes.get(pathTimes.size()-1));
		// PathTimes.printList(pathTimes);
		// ObjectOutputStream out;
		// try {
		// File file = new File("timings.txt");
		// file.delete();
		// out = new ObjectOutputStream(new BufferedOutputStream(
		// new FileOutputStream("timings.txt")));
		// PathTimesWrapper wrap = new PathTimesWrapper(pathTimes);
		// out.writeObject(wrap);
		// out.close();
		//	
		// } catch (FileNotFoundException e) {
		//	
		// e.printStackTrace();
		// } catch (IOException e) {
		//	
		// e.printStackTrace();
		// }
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	private int a;// start point
	private int b;// end point
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
	public String toString() {
		int count = 0;
		String returnV = " \n Path between " + a + " & " + b;
		for (PathRun run : runs) {
			returnV += "\n" + run.toString();
			count++;
		}
		returnV += "\nthere are " + count + " runs ";
		return returnV;
	}

	public static void printList(Vector<PathTimes> pT) {
		System.out.println("path list size of " + pT.size());
		for (PathTimes p : pT) {
			System.out.println(p.getA() + " " + p.getB());
		}
	}

}
