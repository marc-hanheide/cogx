package analysis;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import exploration.PathRun;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

/**
 * class that will take some path times, and then produce a set of paths/times
 * that need to be visited (those that are chosen will be there to see whether
 * or not a particular piece of data is a error or not)
 * 
 * @author kenslaptop
 * 
 */
public class Hypothesizer {

	private Vector<PathTimes> pathTimes;
	private Vector<Double> sd;
	private Vector<Double> means;

	public static void main(String[] arg) {
		Vector<PathTimes> pathTimes = new Vector<PathTimes>();
		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream("timings.txt")));

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
		Hypothesizer h = new Hypothesizer(pathTimes);
	//	System.out.println(h);
		h.greaterThanSd(2.5);
	}

	public Hypothesizer(Vector<PathTimes> pathTimes) {
		this.pathTimes = pathTimes;
		sd = new Vector<Double>();
		means = new Vector<Double>();
		for (PathTimes path : pathTimes) {
			
			double[] var = calculateVar(path);
			sd.add(var[0]);
			means.add(var[1]);
		
		}
		
	}

	/**
	 * returns an array where pos 0 is the sd and pos 1 is the mean
	 * @param path
	 * @return
	 */
	public double[] calculateVar(PathTimes path) {
		double sum = 0;
		double sumSqr = 0;
		for (PathRun pR : path.getRuns()) {
			double val = pR.timeTaken() / 1000;
			if (val != 0) {
				sum += val;
				sumSqr += (val * val);
			} else {
				sum--;
			}
			// .out.println("sumSqr "+sumSqr+ " sum "+sum);
		}
		double mean = sum / path.getRuns().size();

		// System.out.println("mean "+mean);
		double var = (sumSqr - sum * mean) / (path.getRuns().size() - 1);
		double[] r = new double[2];
		r[0]=Math.sqrt(var);
		r[1]= mean;
		return r;

	}
	
	public void greaterThanSd(double amt){
		for(int i=0;i<pathTimes.size();i++){
			System.out.println("pathTimes "+ pathTimes.get(i).getA() + " " + pathTimes.get(i).getB());
			for(PathRun pR:pathTimes.get(i).getRuns()){
				
				if((sd.get(i)*amt)<(Math.abs(pR.timeTaken()/1000-means.get(i)))){
					
					System.out.println(pR.timeTaken());
					System.out.println("sd "+sd.get(i));
					System.out.println("mean "+ means.get(i));
				}
			}
		}
		
	}
	
	@Override
	public String toString(){
		String returnV= "";
		
		for(int i=0;i<pathTimes.size();i++){
			returnV+= "for path between "+ pathTimes.get(i).getA()+ " "+ pathTimes.get(i).getB()
			+ " sd is "+ sd.get(i)+ " and mean is "+ means.get(i)+ "\n";
		}
		
		return returnV;
		
	}
	

}
