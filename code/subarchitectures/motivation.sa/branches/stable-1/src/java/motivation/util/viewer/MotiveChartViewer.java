package motivation.util.viewer;

import java.awt.EventQueue;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.geom.Rectangle2D;
import java.util.Map;
import java.util.Vector;

import javax.swing.JFrame;

import motivation.slice.ExploreMotive;
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

		synchronized void update(WorkingMemoryChange wmc, final Motive motive) {
			switch (wmc.operation) {
			case ADD:
				dsStatus.addValue(motive.status.value() + 1, motive.getClass()
						.getSimpleName(), wmc.address.id);
				dsInformationGain.addValue(motive.informationGain, "",
						wmc.address.id);
				dsTries.addValue(motive.tries, "", wmc.address.id);
				break;
			case DELETE:
				dsStatus.removeValue(motive.getClass().getSimpleName(),
						wmc.address.id);
				dsInformationGain.removeValue("", wmc.address.id);
				dsTries.removeValue("", wmc.address.id);
				break;
			case OVERWRITE:
				dsStatus.setValue(motive.status.value() + 1, motive.getClass()
						.getSimpleName(), wmc.address.id);
				dsInformationGain.setValue(motive.informationGain, "",
						wmc.address.id);
				dsTries.setValue(motive.tries, "", wmc.address.id);
				break;
			}

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
		guiFrame.update(wmc, motive);
	}

}