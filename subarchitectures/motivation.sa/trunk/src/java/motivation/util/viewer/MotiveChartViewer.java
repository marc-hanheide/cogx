package motivation.util.viewer;

import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.geom.Rectangle2D;
import java.util.Map;

import javax.swing.JFrame;

import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.util.WMMotiveSet;
import motivation.util.castextensions.WMEntrySet.ChangeHandler;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.annotations.CategoryAnnotation;
import org.jfree.chart.annotations.CategoryTextAnnotation;
import org.jfree.chart.axis.CategoryAxis;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.NumberAxis3D;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.chart.labels.StandardCategoryToolTipGenerator;
import org.jfree.chart.plot.CategoryPlot;
import org.jfree.chart.plot.CombinedDomainCategoryPlot;
import org.jfree.chart.renderer.category.BarRenderer;
import org.jfree.chart.renderer.category.LayeredBarRenderer;
import org.jfree.chart.renderer.category.LineAndShapeRenderer;
import org.jfree.chart.renderer.category.StackedBarRenderer;
import org.jfree.chart.renderer.category.StackedBarRenderer3D;
import org.jfree.chart.title.LegendTitle;
import org.jfree.data.category.CategoryDataset;
import org.jfree.data.category.DefaultCategoryDataset;
import org.jfree.ui.RefineryUtilities;

import Ice.ObjectImpl;

import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;

public class MotiveChartViewer extends ManagedComponent implements
		ChangeHandler {

	protected class Frame extends JFrame {

		DefaultCategoryDataset dsStatus;
		DefaultCategoryDataset dsInformationGain;
		DefaultCategoryDataset dsTries;
		CategoryPlot gainPlot;
		CategoryPlot statusPlot;
		CategoryPlot triesPlot;

		/**
		 * Creates a new demo instance.
		 * 
		 * @param title
		 *            the frame title.
		 */
		public Frame(final String title) {
			super(title);
			dsStatus = new DefaultCategoryDataset();
			dsInformationGain = new DefaultCategoryDataset();
			dsTries = new DefaultCategoryDataset();
			final ChartPanel chartPanel = new ChartPanel(createChart());
			chartPanel.setPreferredSize(new java.awt.Dimension(500, 270));
			setContentPane(chartPanel);
		}

		/**
		 * 
		 */
		private static final long serialVersionUID = -6559252766578349920L;

		/**
		 * Creates a dataset.
		 * 
		 * @return A dataset.
		 */
		public CategoryDataset createFakeDataset1() {

			final DefaultCategoryDataset dataset1 = new DefaultCategoryDataset();

			// row keys...
			final String series1 = "First";
			final String series2 = "Second";

			// column keys...
			final String type1 = "Type 1";
			final String type2 = "Type 2";
			final String type3 = "Type 3";
			final String type4 = "Type 4";
			final String type5 = "Type 5";
			final String type6 = "Type 6";
			final String type7 = "Type 7";
			final String type8 = "Type 8";

			dataset1.addValue(1.0, series1, type1);
			dataset1.addValue(4.0, series1, type2);
			dataset1.addValue(3.0, series1, type3);
			dataset1.addValue(5.0, series1, type4);
			dataset1.addValue(5.0, series1, type5);
			dataset1.addValue(7.0, series1, type6);
			dataset1.addValue(7.0, series1, type7);
			dataset1.addValue(8.0, series1, type8);

			dataset1.addValue(5.0, series2, type1);
			dataset1.addValue(7.0, series2, type2);
			dataset1.addValue(6.0, series2, type3);
			dataset1.addValue(8.0, series2, type4);
			dataset1.addValue(4.0, series2, type5);
			dataset1.addValue(4.0, series2, type6);
			dataset1.addValue(2.0, series2, type7);
			dataset1.addValue(1.0, series2, type8);

			return dataset1;

		}

		/**
		 * Creates a dataset.
		 * 
		 * @return A dataset.
		 */
		public CategoryDataset createFakeDataset2() {

			final DefaultCategoryDataset result = new DefaultCategoryDataset();

			// row keys...
			final String series1 = "Third";
			final String series2 = "Fourth";

			// column keys...
			final String type1 = "Type 1";
			final String type2 = "Type 2";
			final String type3 = "Type 3";
			final String type4 = "Type 4";
			final String type5 = "Type 5";
			final String type6 = "Type 6";
			final String type7 = "Type 7";
			final String type8 = "Type 8";

			result.addValue(11.0, series1, type1);
			result.addValue(14.0, series1, type2);
			result.addValue(13.0, series1, type3);
			result.addValue(15.0, series1, type4);
			result.addValue(15.0, series1, type5);
			result.addValue(17.0, series1, type6);
			result.addValue(17.0, series1, type7);
			result.addValue(18.0, series1, type8);

			result.addValue(15.0, series2, type1);
			result.addValue(17.0, series2, type2);
			result.addValue(16.0, series2, type3);
			result.addValue(18.0, series2, type4);
			result.addValue(14.0, series2, type5);
			result.addValue(14.0, series2, type6);
			result.addValue(12.0, series2, type7);
			result.addValue(11.0, series2, type8);

			return result;

		}

		// ****************************************************************************
		// * JFREECHART DEVELOPER GUIDE *
		// * The JFreeChart Developer Guide, written by David Gilbert, is
		// available *
		// * to purchase from Object Refinery Limited: *
		// * *
		// * http://www.object-refinery.com/jfreechart/guide.html *
		// * *
		// * Sales are used to provide funding for the JFreeChart project -
		// please *
		// * support us so that we can continue developing free software. *
		// ****************************************************************************

		/**
		 * Creates a chart.
		 * 
		 * @return A chart.
		 */
		private JFreeChart createChart() {

			final NumberAxis statusAxis = new NumberAxis3D("Status");
			statusAxis.setLowerBound(0);
			statusAxis.setUpperBound(5);
			statusAxis
					.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
			final StackedBarRenderer3D statusRender = new StackedBarRenderer3D();
			statusRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			statusPlot = new CategoryPlot(dsStatus, null, statusAxis,
					statusRender);
			statusPlot.setDomainGridlinesVisible(true);

			final NumberAxis gainAxis = new NumberAxis("Gain");
			gainAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
			final StackedBarRenderer3D gainRender = new StackedBarRenderer3D();
			gainRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			gainPlot = new CategoryPlot(dsInformationGain, null, gainAxis,
					gainRender);
			gainPlot.setDomainGridlinesVisible(true);

			final NumberAxis triesAxis = new NumberAxis("# tries");
			triesAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
			final StackedBarRenderer3D triesRender = new StackedBarRenderer3D();
			triesRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			triesPlot = new CategoryPlot(dsTries, null, triesAxis, triesRender);
			triesPlot.setDomainGridlinesVisible(true);

			final CategoryAxis domainAxis = new CategoryAxis("Category");
			final CombinedDomainCategoryPlot plot = new CombinedDomainCategoryPlot(
					domainAxis);
			plot.add(statusPlot, 3);
			plot.add(triesPlot, 2);
			plot.add(gainPlot, 1);

			final JFreeChart result = new JFreeChart("Motive Monitor",
					new Font("SansSerif", Font.BOLD, 10), plot, false);
			result.addLegend(new LegendTitle(statusPlot));
			return result;

		}

	}

	Frame guiFrame;
	WMMotiveSet motives;

	/**
	 * 
	 */
	public MotiveChartViewer() {
		super();
		motives = WMMotiveSet.create(this);
		motives.setHandler(this);
		guiFrame = new Frame(this.getClass().getSimpleName());

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {

		/* check if this is actually started in a cast system */
		if (this.getSubarchitectureID() != null) {
			super.start();
			motives.start();
		} else { // testmode

		}

		guiFrame.pack();
		RefineryUtilities.centerFrameOnScreen(guiFrame);
		guiFrame.setVisible(true);

	}

	/**
	 * Starting point for the demonstration application.
	 * 
	 * @param args
	 *            ignored.
	 * @throws InterruptedException
	 */
	public static void main(final String[] args) throws InterruptedException {

		final MotiveChartViewer demo = new MotiveChartViewer();
		demo.start();
		Thread.sleep(2000);
		// Thread.sleep(2000);
		// demo.dataset.setValue(10, "First", "Type 7");
		// Thread.sleep(2000);
		// demo.dataset.setValue(20, "Second", "Type 10");

	}

	@Override
	public void motiveChanged(Map<WorkingMemoryAddress, ObjectImpl> map,
			WorkingMemoryChange wmc, ObjectImpl newMotive, ObjectImpl oldMotive) {
		Motive motive = (Motive) newMotive;
		switch (wmc.operation) {
		case ADD:
			guiFrame.dsStatus.addValue(motive.status.value() + 1, motive
					.getClass().getSimpleName(), wmc.address.id);
			guiFrame.dsInformationGain.addValue(motive.informationGain, "",
					wmc.address.id);
			guiFrame.dsTries.addValue(motive.tries, "", wmc.address.id);
			break;
		case DELETE:
			guiFrame.dsStatus.removeValue(motive.getClass().getSimpleName(),
					wmc.address.id);
			guiFrame.dsInformationGain.removeValue("", wmc.address.id);
			guiFrame.dsTries.removeValue("", wmc.address.id);
			break;
		case OVERWRITE:
			guiFrame.dsStatus.setValue(motive.status.value() + 1, motive
					.getClass().getSimpleName(), wmc.address.id);
			guiFrame.dsInformationGain.setValue(motive.informationGain, "",
					wmc.address.id);
			guiFrame.dsTries.setValue(motive.tries, "", wmc.address.id);
			break;
		}
	}

}