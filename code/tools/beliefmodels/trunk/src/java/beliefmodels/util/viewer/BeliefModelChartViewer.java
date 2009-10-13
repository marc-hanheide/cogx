
// =========================================================
// PACKAGE 
// =========================================================

package beliefmodels.util.viewer;

// =========================================================
// IMPORTS
// =========================================================

// ---------------------------------------------------------
// BINDER, BELIEFMODELS imports
// ---------------------------------------------------------

import beliefmodels.adl.Belief;
import beliefmodels.adl.BeliefModel;
import beliefmodels.adl.AttributedAgentStatus;
import beliefmodels.adl.PrivateAgentStatus;
import beliefmodels.adl.MutualAgentStatus;


// ---------------------------------------------------------
// CAST imports
// ---------------------------------------------------------

import cast.architecture.ManagedComponent;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

// ---------------------------------------------------------
// ICE imports
// ---------------------------------------------------------

import Ice.ObjectImpl;

// ---------------------------------------------------------
// JAVA imports
// ---------------------------------------------------------

import java.awt.EventQueue;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.geom.Rectangle2D;
import java.util.Map;
import java.util.Vector;

import javax.swing.JFrame;

// ---------------------------------------------------------
// JFreeChart imports
// ---------------------------------------------------------

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
import org.jfree.chart.renderer.category.BarRenderer3D;
import org.jfree.chart.renderer.category.LayeredBarRenderer;
import org.jfree.chart.renderer.category.LineAndShapeRenderer;
import org.jfree.chart.renderer.category.StackedBarRenderer;
import org.jfree.chart.renderer.category.StackedBarRenderer3D;
import org.jfree.chart.title.LegendTitle;
import org.jfree.data.category.CategoryDataset;
import org.jfree.data.category.DefaultCategoryDataset;
import org.jfree.ui.RefineryUtilities;

/** 
The class <b>BeliefModelChartViewer</b> on-line charts the operations on a belief model: 
 
 <ul>
 <li> number of private (robot), attributed (robot[human]), and shared ({robot,human}) beliefs. 
 </ul>

 
 @author Geert-Jan Kruijff (gj@dfki.de)
 @author Marc Hanheide
 @started 091013
 @version 091013
 
 */ 

public class BeliefModelChartViewer 
	extends ManagedComponent { 

	protected class Frame extends JFrame {

		// private beliefs
		DefaultCategoryDataset dsPrivate;
		// attributed beliefs
		DefaultCategoryDataset dsAttributed;
		// shared beliefs
		DefaultCategoryDataset dsShared;
		
		CategoryPlot privatePlot;
		CategoryPlot attributedPlot;
		CategoryPlot sharedPlot;


		
		/**
		 * constructor initializes the GUI
		 */

		/**
		 * Creates a new frame instance.
		 * 
		 * @param title
		 *            the frame title.
		 */
		public Frame(final String title) {
			super(title);
			dsPrivate = new DefaultCategoryDataset();
			dsAttributed = new DefaultCategoryDataset();
			dsShared = new DefaultCategoryDataset();
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

			// set up the private beliefs
			final NumberAxis privateAxis = new NumberAxis3D("#Beliefs: {robot}");
			privateAxis.setLowerBound(0);
			privateAxis.setUpperBound(5);
			privateAxis
					.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
			final StackedBarRenderer3D privateRender = new StackedBarRenderer3D();
			privateRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			privatePlot = new CategoryPlot(dsPrivate, null, privateAxis,
					privateRender);
			privatePlot.setDomainGridlinesVisible(true);

			// set up the attributed beliefs
			final NumberAxis attributedAxis = new NumberAxis("#Beliefs: {robot[human]}");
			attributedAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
			final StackedBarRenderer3D attributedRender = new StackedBarRenderer3D();
			attributedRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			attributedPlot = new CategoryPlot(dsAttributed, null, attributedAxis,
					attributedRender);
			attributedPlot.setDomainGridlinesVisible(true);

			// set up the shared beliefs
			final NumberAxis sharedAxis = new NumberAxis("#Beliefs: {robot,human}");
			sharedAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
			final BarRenderer3D sharedRender = new BarRenderer3D();
			sharedRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			sharedPlot = new CategoryPlot(dsShared, null, sharedAxis, sharedRender);
			sharedPlot.setDomainGridlinesVisible(true);

			final CategoryAxis domainAxis = new CategoryAxis("WM Id");
			final CombinedDomainCategoryPlot plot = new CombinedDomainCategoryPlot(
					domainAxis);
			plot.add(privatePlot, 3);
			plot.add(attributedPlot, 2);
			plot.add(sharedPlot, 1);

			final JFreeChart result = new JFreeChart("BeliefModel Monitor",
					new Font("SansSerif", Font.BOLD, 10), plot, false);
			result.addLegend(new LegendTitle(privatePlot));
			result.addLegend(new LegendTitle(attributedPlot));			
			result.addLegend(new LegendTitle(sharedPlot));
			return result;

		}

		synchronized void update(WorkingMemoryChange wmc, final Belief belief) {
			switch (wmc.operation) {
			case ADD:
					if (belief.ags instanceof PrivateAgentStatus) {
						dsPrivate.addValue(1, "private", wmc.address.id);
					} else if (belief.ags instanceof AttributedAgentStatus) { 
						dsPrivate.addValue(1, "attributed", wmc.address.id);						
					} else if (belief.ags instanceof MutualAgentStatus) {
						dsPrivate.addValue(1, "shared", wmc.address.id);													   
					}
				break;
			case DELETE:
				dsPrivate.removeValue("private", wmc.address.id);
					if (belief.ags instanceof PrivateAgentStatus) {
						dsPrivate.removeValue("private", wmc.address.id);
					} else if (belief.ags instanceof AttributedAgentStatus) { 
						dsPrivate.removeValue("attributed", wmc.address.id);						
					} else if (belief.ags instanceof MutualAgentStatus) {
						dsPrivate.removeValue("shared", wmc.address.id);													   
					}
				break;
			case OVERWRITE:
					if (belief.ags instanceof PrivateAgentStatus) {
						dsPrivate.addValue(1, "private", wmc.address.id);
					} else if (belief.ags instanceof AttributedAgentStatus) { 
						dsPrivate.addValue(1, "attributed", wmc.address.id);						
					} else if (belief.ags instanceof MutualAgentStatus) {
						dsPrivate.addValue(1, "shared", wmc.address.id);													   
					}
				break;
			}

		}

	}

		
	// global variable for the gui
	Frame guiFrame;		
		
	public BeliefModelChartViewer() {
		super();
		guiFrame = new Frame(this.getClass().getSimpleName());			
	}
		
		
		
		
	/*
	 * start up the GUI, and register the change listeners for Belief and BeliefModel objects
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {

		/* check if this is actually started in a cast system */
		if (this.getSubarchitectureID() != null) {
			super.start();
		} else { // testmode

		}

		guiFrame.pack();
		RefineryUtilities.centerFrameOnScreen(guiFrame);
		guiFrame.setVisible(true);
		// 
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Belief.class,
																   WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {
						
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							Belief belief = getMemoryEntry(_wmc.address, Belief.class);
							guiFrame.update(_wmc, belief);
						}
						catch (Exception e) {
							e.printStackTrace();
						}
					} 
				});
		
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(BeliefModel.class,
																   WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
						
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						try {
							BeliefModel bmodel = getMemoryEntry(_wmc.address, BeliefModel.class);
							
						}
						catch (Exception e) {
							e.printStackTrace();
						}
					} 
				});
		
		
		

	}

	/**
	 * Starting point for the demonstration application.
	 * 
	 * @param args
	 *            ignored.
	 * @throws InterruptedException
	 */
	public static void main(final String[] args) throws InterruptedException {

		final BeliefModelChartViewer demo = new BeliefModelChartViewer();
		demo.start();
		Thread.sleep(2000);
		// Thread.sleep(2000);
		// demo.dataset.setValue(10, "First", "Type 7");
		// Thread.sleep(2000);
		// demo.dataset.setValue(20, "Second", "Type 10");

	}



}