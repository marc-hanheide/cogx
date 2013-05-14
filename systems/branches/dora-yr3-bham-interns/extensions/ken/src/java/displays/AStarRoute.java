package displays;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import navigation.AStarNode;
import navigation.AStarSearch;
import navigation.IntelligentPathSelector;
import navigation.Path;
import NavData.FNode;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

public class AStarRoute {

	public static void main(String[] args){
		int[] path = {600,610,620,630,640,650,660,670,680,690,700,710,720};
		for(int i=0;i<path.length;i++){
		AStarRoute a = new AStarRoute(0,38,path[i], false, 1);
		}
	//	AStarRoute a2 = new AStarRoute(17,14,770,false,1);
		//AStarRoute a = new AStarRoute(15,42,600+i*10, false, 1);
	}
	
	
	public AStarRoute(int start, int end, int time, boolean includeDay, int day) {
		Vector<PathTimes> pathTimes = new Vector<PathTimes>();
		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("howarth2.txt")));

			pathTimes = ((PathTimesWrapper) (in.readObject())).getPathTimes();

			in.close();
			BufferedReader f = new BufferedReader(
					new FileReader("tmpmap.graph"));
			String first = f.readLine();
			int length = Integer.valueOf(String.valueOf(first.toCharArray(), 6,
					first.length() - 6));
			//System.out.println(length);
			Vector<FNode> nodes = new Vector<FNode>();

			while (nodes.size() < length) {
				nodes.add(util.misc.string2FNode(f.readLine()));

//				System.out.println("node " + nodes.lastElement().nodeId
//						+ " pos " + nodes.lastElement().x + ", "
//						+ nodes.lastElement().y);
			}

			AStarNode.setRadTurn(5000);
			Vector<Path> p = util.misc
					.convertToSingleTime(pathTimes, new IntelligentPathSelector(), time, includeDay, day);
			//710 and any other time, 5-->10
			for(Path pa:p){
				System.out.println(pa.getA() + " to "+pa.getB() + " costs "+ pa.getCost()); 
			}
			System.out.println("graph is "+ p);
				Vector<Integer> path = AStarSearch.search(nodes, p, start, end);
				GraphicalDisplay g = new GraphicalDisplay(path,
						"Quickest path from " + path.get(0) + " to "
								+ path.get(path.size() - 1)+ " at "+ time);
				System.out.println("path is " + path);

			
		} catch (FileNotFoundException e) {
			System.out.println("unable to find file, load failed");

		} catch (IOException e) {
			System.out.println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			System.out.println(e);
			e.printStackTrace();

		}

	}
}
