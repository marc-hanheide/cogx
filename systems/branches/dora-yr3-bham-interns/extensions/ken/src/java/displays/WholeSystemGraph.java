package displays;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class WholeSystemGraph extends JFrame {

	JFreeChart chart;

	public WholeSystemGraph(double[] entropies, int startTime, int endTime,
			int increment, boolean includeDay, String title, String yAxis) {
		super(title);
		setSize(800, 600);

		// XYSeries xy = new XYSeries("S'up");
		XYSeriesCollection xySeriesCollection = new XYSeriesCollection();
		
			XYSeries s = new XYSeries("entropies");
		
		for (int i = 0; i < (endTime - startTime) / increment; i++) {
			if (entropies[i] == Double.MAX_VALUE) {
				s.add(startTime + i * increment, 10);
			} else {
				s.add(startTime + i * increment, entropies[i]);
			}
		}
		xySeriesCollection.addSeries(s);
		chart = ChartFactory.createScatterPlot(title, "Time Of Day Recorded",
				yAxis, xySeriesCollection, PlotOrientation.VERTICAL,
				false, true, false);

		final ChartPanel panel = new ChartPanel(chart, true);
		panel.setPreferredSize(new java.awt.Dimension(500, 270));

		setContentPane(panel);
		setVisible(true);
	}

}
