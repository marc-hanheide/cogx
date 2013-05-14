package displays;

import java.util.ArrayList;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import weka.core.Instance;

public class EntropyVis extends JFrame {
	JFreeChart chart;

	public EntropyVis(double[] entropies, int startTime, int endTime,
			int increment, boolean includeDay, String title) {
		super(title);
		setSize(800, 600);

		// XYSeries xy = new XYSeries("S'up");
		XYSeriesCollection xySeriesCollection = new XYSeriesCollection();
		XYSeries s = new XYSeries("entropies");
		for (int i = startTime; i < endTime; i++) {
			if (entropies[i] == Double.MAX_VALUE) {
				//s.add(i*increment, 10);
			} else {
				s.add(i * increment, entropies[i]);
//				System.out.println("i is "+i);
//				System.out.println("entropies is "+entropies[i]);
			}
		}
		xySeriesCollection.addSeries(s);
		chart = ChartFactory.createScatterPlot(title, "Time Of Day",
				"Entropy", xySeriesCollection, PlotOrientation.VERTICAL,
				true, true, false);

		final ChartPanel panel = new ChartPanel(chart, true);
		panel.setPreferredSize(new java.awt.Dimension(500, 270));

		setContentPane(panel);
		setVisible(true);
	}

}
