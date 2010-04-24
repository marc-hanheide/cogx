package motivation.util.viewer;

import java.awt.Font;
import java.util.Map;

import javax.swing.JFrame;

import motivation.slice.Motive;
import motivation.util.WMMotiveSet;

import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.CategoryAxis;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.axis.SymbolAxis;
import org.jfree.chart.labels.StandardCategoryToolTipGenerator;
import org.jfree.chart.plot.CategoryPlot;
import org.jfree.chart.plot.CombinedDomainCategoryPlot;
import org.jfree.chart.renderer.category.BarRenderer3D;
import org.jfree.chart.renderer.category.LineRenderer3D;
import org.jfree.chart.renderer.category.StackedBarRenderer3D;
import org.jfree.chart.title.LegendTitle;
import org.jfree.data.category.DefaultCategoryDataset;
import org.jfree.ui.RefineryUtilities;

import Ice.ObjectImpl;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import castutils.castextensions.WMEntrySet.ChangeHandler;

public class MotiveChartViewer extends ManagedComponent implements
		ChangeHandler {

	protected class Frame extends JFrame {

		DefaultCategoryDataset dsStatus;
		DefaultCategoryDataset dsGainCosts;
		DefaultCategoryDataset dsTries;
		DefaultCategoryDataset dsRank;
		CategoryPlot gainCostsPlot;
		CategoryPlot statusPlot;
		CategoryPlot triesPlot;
		CategoryPlot rankPlot;

		/**
		 * Creates a new demo instance.
		 * 
		 * @param title
		 *            the frame title.
		 */
		public Frame(final String title) {
			super(title);
			dsStatus = new DefaultCategoryDataset();
			dsGainCosts = new DefaultCategoryDataset();
			dsTries = new DefaultCategoryDataset();
			dsRank = new DefaultCategoryDataset();
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
			String[] labels = {"","UNSURFACED","SURFACED","","","ACTIVATED"};
			final NumberAxis statusAxis = new SymbolAxis("Status",labels);
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

			final NumberAxis gainCostsAxis = new NumberAxis("Gain/costs");
			gainCostsAxis.setStandardTickUnits(NumberAxis
					.createIntegerTickUnits());
			final BarRenderer3D gainCostsRender = new BarRenderer3D();
			gainCostsRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			gainCostsPlot = new CategoryPlot(dsGainCosts, null, gainCostsAxis,
					gainCostsRender);
			gainCostsPlot.setDomainGridlinesVisible(true);

			final NumberAxis rankAxis = new NumberAxis("rank");
			gainCostsAxis.setStandardTickUnits(NumberAxis
					.createIntegerTickUnits());
			final LineRenderer3D rankRender = new LineRenderer3D();
			rankRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			rankPlot = new CategoryPlot(dsRank, null, rankAxis, rankRender);
			rankPlot.setDomainGridlinesVisible(true);

			final NumberAxis triesAxis = new NumberAxis("# tries / priority");
			triesAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
			final BarRenderer3D triesRender = new BarRenderer3D();
			triesRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			triesPlot = new CategoryPlot(dsTries, null, triesAxis, triesRender);
			triesPlot.setDomainGridlinesVisible(true);

			final CategoryAxis domainAxis = new CategoryAxis("WM Id");
			final CombinedDomainCategoryPlot plot = new CombinedDomainCategoryPlot(
					domainAxis);
			plot.add(statusPlot, 3);
			plot.add(rankPlot, 3);
			plot.add(triesPlot, 3);
			plot.add(gainCostsPlot, 3);

			final JFreeChart result = new JFreeChart("Motive Monitor",
					new Font("SansSerif", Font.BOLD, 10), plot, false);
			result.addLegend(new LegendTitle(statusPlot));
			result.addLegend(new LegendTitle(triesPlot));
			result.addLegend(new LegendTitle(gainCostsPlot));
			return result;

		}

		synchronized void update(WorkingMemoryChange wmc, final Motive motive) {
			switch (wmc.operation) {
			case ADD:
				dsStatus.addValue(motive.status.value() + 1, motive.getClass()
						.getSimpleName(), wmc.address.id);
				dsGainCosts.addValue(motive.informationGain, "gain",
						wmc.address.id);
				dsTries.addValue(motive.tries, "tries", wmc.address.id);
				dsGainCosts.addValue(motive.costs/10, "costs", wmc.address.id);
				dsTries.addValue(motive.priority.value(), "priority",
						wmc.address.id);
				dsRank.addValue(motive.rank, "", wmc.address.id);
				break;
			case DELETE:
				dsStatus.removeValue(motive.getClass().getSimpleName(),
						wmc.address.id);
				dsGainCosts.removeValue("gain", wmc.address.id);
				dsTries.removeValue("tries", wmc.address.id);
				dsGainCosts.removeValue("costs", wmc.address.id);
				dsTries.removeValue("priority", wmc.address.id);
				dsRank.removeValue("", wmc.address.id);
				break;
			case OVERWRITE:
				dsStatus.setValue(motive.status.value() + 1, motive.getClass()
						.getSimpleName(), wmc.address.id);
				dsGainCosts.setValue(motive.informationGain, "gain",
						wmc.address.id);
				dsTries.setValue(motive.tries, "tries", wmc.address.id);
				dsGainCosts.setValue(motive.costs/10, "costs", wmc.address.id);
				dsTries.setValue(motive.priority.value(), "priority",
						wmc.address.id);
				dsRank.setValue(motive.rank, "", wmc.address.id);
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
	public void entryChanged(Map<WorkingMemoryAddress, ObjectImpl> map,
			WorkingMemoryChange wmc, ObjectImpl newMotive, ObjectImpl oldMotive) {
		Motive motive = (Motive) newMotive;
		guiFrame.update(wmc, motive);
	}

}