package displays;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.Date;
import java.util.Vector;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import exploration.PathTimes;
import exploration.PathTimesWrapper;

public class TimingsChart extends JFrame {

	JFreeChart chart;

	ChartPanel panel;

	public static void main(String[] args) {
		PathVisualization.value=PathVisualization.values[1];
		TimingsChart c = new TimingsChart(true);
		//PathVisualization.value=PathVisualization.values[1];
		//TimingsChart c1 = new TimingsChart(true);
		PathVisualization.value=PathVisualization.values[0];
		TimingsChart c2 = new TimingsChart(true);
		//TimingsChart c2 = new TimingsChart(false);
	}

	public TimingsChart(boolean val) {
		super("Timings chart");
		setSize(800, 600);
		load(val);
		panel = new ChartPanel(chart);
		this.add(panel);
		setVisible(true);
	}

	public void load(boolean val) {
		XYSeries pathTimes = new XYSeries("path times");

		try {
			ObjectInputStream in = new ObjectInputStream(
					new BufferedInputStream(new FileInputStream(PathVisualization.getVal())));

			Vector<PathTimes> pT = ((PathTimesWrapper) (in.readObject()))
					.getPathTimes();
			XYSeriesCollection series = new XYSeriesCollection();
			for (int i = 0; i < pT.size(); i++) {
				XYSeries current = new XYSeries(pT.get(i).getA() + " & "
						+ pT.get(i).getB());

				for (int j = 0; j < pT.get(i).getRuns().size(); j++) {
					if (val) {

						current.add(i, (double) (pT.get(i).getRun(j)
								.timeTaken()) / 1000);
					} else {
						Date time = pT.get(i).getRun(j).timeStarted();
						int timeDone = time.getHours() * 60 + time.getMinutes();
						current.add(i, timeDone);
					}
				}
				series.addSeries(current);
			}
			XYDataset xyDataset = series;

			chart = ChartFactory.createScatterPlot(null, // Title
					null, // X-Axis label
					(val)?"Time Taken (seconds)":"Time recorded", // Y-Axis label
					xyDataset, // Dataset

					PlotOrientation.VERTICAL, true, false, false);

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

}
