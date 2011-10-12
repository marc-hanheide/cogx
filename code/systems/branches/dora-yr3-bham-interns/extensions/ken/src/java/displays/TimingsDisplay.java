package displays;

import java.awt.Font;
import java.awt.Graphics;
import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Vector;

import javax.swing.JFrame;
import javax.swing.JPanel;

import exploration.PathRun;
import exploration.PathTimes;
import exploration.PathTimesWrapper;

public class TimingsDisplay extends JPanel {
	private Vector<PathTimes> pathTimes;
	private JFrame frame;
	private boolean compareAll;

	public static void main(String[] args) {
		TimingsDisplay dis = new TimingsDisplay(false, "seperate y Axis");
		TimingsDisplay dis2 = new TimingsDisplay(true, "relative y Axis");
	}

	/**
	 * boolean indicates whether the yAxis will be independent for each path or
	 * not (so if true, the axis will be all relative, but if false each extry
	 * will work off it's own axis)
	 * 
	 * @param averageOverAll
	 */
	public TimingsDisplay(boolean compareAll, String name) {
		frame = new JFrame(name);
		frame.setSize(800, 600);
		frame.add(this);
		load();
		frame.setVisible(true);
		this.compareAll = compareAll;
	}

	
	
	private void load() {
		pathTimes = new Vector<PathTimes>();
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

	}

	@Override
	public void paintComponent(Graphics g) {
		int xAxisInd = 15;
		int yAxisInd = 10;
		int xAxis = getSize().width - xAxisInd;
		int yAxis = getSize().height - yAxisInd * 2;
		int xValue = xAxis / pathTimes.size();
		String font = getToolkit().getFontList()[0];
		g.setFont(new Font(font, Font.BOLD, 5));
		long highest = Long.MIN_VALUE;

		if (compareAll) {

			for (int i = 0; i < pathTimes.size(); i++) {
				for (PathRun path : pathTimes.get(i).getRuns()) {
					if (path.timeTaken() > highest) {
						highest = path.timeTaken();
					}
				}
			}
			for (int i = 0; i < pathTimes.size(); i++) {
				System.out.println();
				System.out.println("position " + i);
				g.drawString(Integer.toString(pathTimes.get(i).getA()), i
						* xValue + xAxisInd, yAxis);
				g.drawString(Integer.toString(pathTimes.get(i).getB()), i
						* xValue + xAxisInd, (int) (yAxis + .5 * yAxisInd));
				Vector<PathRun> paths = pathTimes.get(i).getRuns();

				double highest2 = (double) highest;
				int count = 0;
				for (PathRun path : paths) {

					if (path.timeTaken() != 0) {

						g.fillOval(i * xValue + xAxisInd,
								(int) (yAxis * (1 - ((double) path.timeTaken())
										/ (highest2))), 3, 3);
						System.out.println("time taken " + path.timeTaken());
						System.out.println("highest " + highest2);
					} else {
						count++;
					}
				}
				g.drawString(Integer.toString(count), i * xValue + xAxisInd,
						yAxis + yAxisInd);
			}
		} else {

			for (int i = 0; i < pathTimes.size(); i++) {
				long highest3 = Long.MIN_VALUE;
				System.out.println();
				System.out.println("position " + i);
				g.drawString(Integer.toString(pathTimes.get(i).getA()), i
						* xValue + xAxisInd, yAxis);
				g.drawString(Integer.toString(pathTimes.get(i).getB()), i
						* xValue + xAxisInd, (int) (yAxis + .5 * yAxisInd));
				Vector<PathRun> paths = pathTimes.get(i).getRuns();

				for (PathRun path : pathTimes.get(i).getRuns()) {
					if (path.timeTaken() > highest3) {
						highest3 = path.timeTaken();
					}
				}
				double highest2 = (double) highest3;
				int count = 0;
				for (PathRun path : paths) {

					if (path.timeTaken() != 0) {

						g.fillOval(i * xValue + xAxisInd,
								(int) (yAxis * (1 - ((double) path.timeTaken())
										/ (highest2))), 3, 3);
						System.out.println("time taken " + path.timeTaken());
						System.out.println("highest " + highest2);
					} else {
						count++;
					}
				}
				g.drawString(Integer.toString(count), i * xValue + xAxisInd,
						yAxis + yAxisInd);

			}
		}

	}
}
