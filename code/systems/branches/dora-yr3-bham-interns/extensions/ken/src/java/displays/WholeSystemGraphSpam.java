package displays;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import clustering.ProbGenerator;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

public class WholeSystemGraphSpam {
	private int boundOne;

	public static void main(String[] args) {
		

	//	WholeSystemGraphSpam g = new WholeSystemGraphSpam(600,700 ,false, 1);
	//	WholeSystemGraphSpam g2 = new WholeSystemGraphSpam(650,700 ,false, 1);
	//	TimingsChart t = new  TimingsChart(true);
		//PathVisualization.value=PathVisualization.values[2];
	
	//	WholeSystemGraphSpam g3 = new WholeSystemGraphSpam(600,700 ,false, 1);
		//WholeSystemGraphSpam g4 = new WholeSystemGraphSpam(650,700 ,false, 1);
		//TimingsChart t2 = new  TimingsChart(true);
		//PathVisualization.value=PathVisualization.values[1];
		
		WholeSystemGraphSpam g5 = new WholeSystemGraphSpam(600,700 ,false, 1);
		PathVisualization.value = PathVisualization.values[1];
		WholeSystemGraphSpam g6 = new WholeSystemGraphSpam(600,700 ,false, 1);
		PathVisualization.value = PathVisualization.values[2];
		WholeSystemGraphSpam g7 = new WholeSystemGraphSpam(600,700 ,false, 1);
		PathVisualization.value = PathVisualization.values[3];
		WholeSystemGraphSpam g8 = new WholeSystemGraphSpam(600,700 ,false, 1);
		
		//WholeSystemGraphSpam g6 = new WholeSystemGraphSpam(650,700 ,false, 1);
	
		
		//TimingsChart t3 = new  TimingsChart(true);
	//	TimingsChart t2 = new TimingsChart(false);
//		WholeSystemGraphSpam g = new WholeSystemGraphSpam(200, 450, false, 0);
	}

	public WholeSystemGraphSpam(int startTime, int endTime, boolean includeDay,
			int day) {

		GraphicalDisplay g = new GraphicalDisplay(new Vector<Integer>(),
				"Map");
		int num = numOfPaths();
		Vector<Integer> entr = new Vector<Integer>();
        Vector<Integer> timet = new Vector<Integer>();
		double[] entropies = new double[(endTime - startTime) / 10];
		double[] timeTaken = new double[entropies.length];
		for (int i = 0; i < num; i++) {
			int entPath = 0;
			int timing =0;
			ProbGenerator p = new ProbGenerator(i, false, false, includeDay,
					day);
			double[] ent = p.getEntropies();
			int[] est = p.getEstimates();
			if (boundOne == 0) {
				boundOne = p.getBounds(true).get(1);
			}
			for (int j = (startTime / boundOne); j < (endTime) / boundOne; j++) {
				if (ent[j] == Double.MAX_VALUE) {
				//	entropies[j - (startTime / boundOne)] += 10;
					entPath += 10;
				} else {
					// System.out.println("ent j is "+ent[j]);
					entropies[j - (startTime / boundOne)] += ent[j];
					entPath += ent[j];
				}
				timing+=est[j];
				timeTaken[j - (startTime / boundOne)] += est[j];
			}
			entPath/=((endTime-startTime)/boundOne);
			timing/=((endTime-startTime)/boundOne);
			entr.add(entPath);
			timet.add(timing);
			
		}
		System.out.println("entropies size is " + entropies.length);
		for (int i = 0; i < entropies.length; i++) {
			// System.out.println(entropies[i]);
			entropies[i] /= (num);
			timeTaken[i] /= num;
		}

		WholeSystemGraph eVis = new WholeSystemGraph(entropies, startTime,
				endTime, boundOne, includeDay, "Whole System Entropy between "+startTime + " and "+endTime, "Entropy");
		WholeSystemGraph tVis = new WholeSystemGraph(timeTaken, startTime,
				endTime, boundOne, includeDay, "Whole System Time Taken between "+startTime + " and "+endTime,"Time Taken");
		GraphicalEnt gEnt = new GraphicalEnt(entr, "Entropy Map between "+startTime + " and "+endTime,true);
		GraphicalEnt tEnt = new GraphicalEnt(timet, "Travel Times Map between "+startTime + " and "+endTime,false);

	}

	public int numOfPaths() {
		Vector<PathTimes> pathTimes = new Vector<PathTimes>();
		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("timings.txt")));

			pathTimes = ((PathTimesWrapper) (in.readObject())).getPathTimes();

		} catch (FileNotFoundException e) {
			System.out.println("unable to find file, load failed");

		} catch (IOException e) {
			System.out.println(e);
			e.printStackTrace();
		} catch (ClassNotFoundException e) {
			System.out.println(e);
			e.printStackTrace();

		}
		return pathTimes.size();
	}
}
