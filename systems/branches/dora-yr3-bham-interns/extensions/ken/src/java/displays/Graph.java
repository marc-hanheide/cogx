package displays;

import java.awt.Color;
import java.awt.Shape;
import java.awt.geom.Ellipse2D;

import javax.swing.JFrame;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYItemRenderer;
import org.jfree.data.xy.XYDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

public class Graph extends JFrame {
	JFreeChart chart;
	double[] data;
	double[] xS;
	ChartPanel panel;

	public Graph(double[] data, String title) {
		super(title);
		setSize(800, 600);
		this.data = data;
		generate();
		panel = new ChartPanel(chart);
		this.add(panel);
		setVisible(true);
	}
	
	public Graph(double[] data, double[] xS,String title) {
		super(title);
		setSize(800, 600);
		this.data = data;
		this.xS=xS;
		generate2();
		panel = new ChartPanel(chart);
		this.add(panel);
		setVisible(true);
	}
	

	private void generate2() {
		XYSeriesCollection series = new XYSeriesCollection();

		XYSeries current = new XYSeries("data");

		for (int i = 0; i < data.length; i++) {

			current.add(xS[i], data[i]);
		}
		series.addSeries(current);

		XYDataset xyDataset = series;

		
		
		
		chart = ChartFactory.createScatterPlot(null, // Title
				null, // X-Axis label
				"", // Y-Axis label
				xyDataset, // Dataset

				PlotOrientation.VERTICAL, true, false, false);
		int size = (int)Math.log10(data.length);
		size = 5-size;
		Shape shape  = new Ellipse2D.Double(0,0,size,size);
		XYPlot xyPlot = (XYPlot) chart.getPlot();
		XYItemRenderer renderer = xyPlot.getRenderer();
		renderer.setBaseShape(shape);
		renderer.setBasePaint(Color.red);
		renderer.setSeriesShape(0, shape);
		
		
	}
		
	

	private void generate() {

		XYSeriesCollection series = new XYSeriesCollection();

		XYSeries current = new XYSeries("data");

		for (int i = 0; i < data.length; i++) {

			current.add(i, data[i]);
		}
		System.out.println("graph series length is "+ data.length);
		series.addSeries(current);

		XYDataset xyDataset = series;

		chart = ChartFactory.createScatterPlot(null, // Title
				null, // X-Axis label
				"Time Taken (seconds)", // Y-Axis label
				xyDataset, // Dataset

				PlotOrientation.VERTICAL, true, false, false);

	}

}
