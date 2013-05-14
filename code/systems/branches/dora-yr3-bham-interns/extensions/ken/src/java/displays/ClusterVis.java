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

public class ClusterVis extends JFrame {

	JFreeChart chart;
	
	
	public ClusterVis(ArrayList<ArrayList<Instance>> instances,
			 String title) {
		super(title);
		setSize(800, 600);
		
		XYSeries xy = new XYSeries("S'up");
		XYSeriesCollection xySeriesCollection = new XYSeriesCollection();
		for (ArrayList<Instance> ins : instances) {

			XYSeries s = new XYSeries("hypothesis"
					+ xySeriesCollection.getSeriesCount());
			for (Instance i : ins) {
									s.add(i.value(0), i.value(1));
				
			}
			xySeriesCollection.addSeries(s);
		}
		chart = ChartFactory.createScatterPlot(title,
				"Time Recorded", "Time Taken (ms)", xySeriesCollection,
				PlotOrientation.VERTICAL, true, true, false);
//		chart.getXYPlot().getRangeAxis().setLowerBound(4000);
//		chart.getXYPlot().getRangeAxis().setUpperBound(20000);
		
		final ChartPanel panel = new ChartPanel(chart, true);
		panel.setPreferredSize(new java.awt.Dimension(500, 270));
		
		setContentPane(panel);
		setVisible(true);
	}

}
