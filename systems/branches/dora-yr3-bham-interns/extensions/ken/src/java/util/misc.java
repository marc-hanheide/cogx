package util;

import java.util.Vector;

import navigation.NaivePathSelector;
import navigation.Path;
import navigation.PathSelector;
import exploration.PathTimes;
import NavData.FNode;
import SpatialProperties.PathProperty;

public class misc {

	/**
	 * given a string will return a FNode FNode must be in the format ? nodeId x
	 * y theta AreaId width
	 * 
	 * @param s
	 * @return
	 */
	public static FNode string2FNode(String s) {
		String[] c = s.split(" ");

		FNode f = new FNode();
		f.nodeId = Integer.valueOf(c[1]);
		f.x = Double.valueOf(c[2]);
		f.y = Double.valueOf(c[3]);
		f.theta = Double.valueOf(c[4]);

		return f;
	}

	public static PathProperty string2PathProperty(String s) {
		String[] c = s.split(" ");
		PathProperty p = new PathProperty();
		p.place1Id = Integer.valueOf(c[0]);
		p.place2Id = Integer.valueOf(c[1]);
		return p;
	}

	/**
	 * converts the stored pathTimes into a Vector of paths each having the most
	 * likely time as their cost value
	 * 
	 * @return
	 */
	public static Vector<Path> convertToSingleTime(Vector<PathTimes> pathTimes, PathSelector p, int time) {
		return p.generateGraph(time);

	}
	
}
