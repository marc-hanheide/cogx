
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
import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.ContinualStatus;
import beliefmodels.domainmodel.cogx.ContinualFormula;
import beliefmodels.domainmodel.cogx.UncertainSuperFormula;



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
import java.util.HashMap;
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
 <li> number of beliefs (robot), certainty (robot[human]), and truth ({robot,human}) beliefs. 
 </ul>

 
 @author Geert-Jan Kruijff (gj@dfki.de)
 @author Marc Hanheide
 @started 091013
 @version 091013
 
 */ 

public class BeliefModelChartViewer 
	extends ManagedComponent { 

	protected class Frame extends JFrame {

		// beliefs beliefs
		DefaultCategoryDataset dsBeliefs;
		// certainty beliefs
		DefaultCategoryDataset dsCertainty;
		// truth beliefs
		DefaultCategoryDataset dsTruth;
		
		CategoryPlot beliefsPlot;
		CategoryPlot certaintyPlot;
		CategoryPlot truthPlot;


		
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
			dsBeliefs = new DefaultCategoryDataset();
			dsCertainty = new DefaultCategoryDataset();
			dsTruth = new DefaultCategoryDataset();
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

			// set up the beliefs beliefs
			final NumberAxis beliefsAxis = new NumberAxis3D("Beliefs");
			beliefsAxis.setLowerBound(0);
			beliefsAxis.setUpperBound(5);
			beliefsAxis
					.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
			final StackedBarRenderer3D beliefsRender = new StackedBarRenderer3D();
			beliefsRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			beliefsPlot = new CategoryPlot(dsBeliefs, null, beliefsAxis,
					beliefsRender);
			beliefsPlot.setDomainGridlinesVisible(true);

			// set up the certainty beliefs
			final NumberAxis certaintyAxis = new NumberAxis("Certainty");
			certaintyAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
			final StackedBarRenderer3D certaintyRender = new StackedBarRenderer3D();
			certaintyRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			certaintyPlot = new CategoryPlot(dsCertainty, null, certaintyAxis,
					certaintyRender);
			certaintyPlot.setDomainGridlinesVisible(true);

			// set up the truth beliefs
			final NumberAxis truthAxis = new NumberAxis("Status");
			truthAxis.setStandardTickUnits(NumberAxis.createIntegerTickUnits());
			final BarRenderer3D truthRender = new BarRenderer3D();
			truthRender
					.setBaseToolTipGenerator(new StandardCategoryToolTipGenerator());
			truthPlot = new CategoryPlot(dsTruth, null, truthAxis, truthRender);
			truthPlot.setDomainGridlinesVisible(true);

			final CategoryAxis domainAxis = new CategoryAxis("WM Id");
			final CombinedDomainCategoryPlot plot = new CombinedDomainCategoryPlot(
					domainAxis);
			plot.add(beliefsPlot, 3);
			plot.add(certaintyPlot, 2);
			plot.add(truthPlot, 1);

			final JFreeChart result = new JFreeChart("BeliefModel Monitor",
					new Font("SansSerif", Font.BOLD, 10), plot, false);
			result.addLegend(new LegendTitle(beliefsPlot));
			result.addLegend(new LegendTitle(certaintyPlot));			
			result.addLegend(new LegendTitle(truthPlot));
			return result;

		}

		synchronized void update(WorkingMemoryChange wmc, final Belief belief) {
			switch (wmc.operation) {
			case ADD:
					if (belief.ags instanceof PrivateAgentStatus) {
						dsBeliefs.addValue(1, "private: {robot}", wmc.address.id);
					} else if (belief.ags instanceof AttributedAgentStatus) { 
						dsBeliefs.addValue(1, "attributed: {robot[human]}", wmc.address.id);				
					} else if (belief.ags instanceof MutualAgentStatus) {
						dsBeliefs.addValue(1, "shared: {robot,human}", wmc.address.id);						
					}
					dsCertainty.setValue(((UncertainSuperFormula)belief.phi).prob, "certainty", wmc.address.id);
					updateTruth(wmc,belief);
					break;
			case DELETE:
				dsBeliefs.removeValue("beliefs", wmc.address.id);
					if (belief.ags instanceof PrivateAgentStatus) {
						System.out.println("DELETING HERE!!");
						dsBeliefs.removeValue("private: {robot}", wmc.address.id);
					} else if (belief.ags instanceof AttributedAgentStatus) { 
						dsBeliefs.removeValue("attributed: {robot[human]}", wmc.address.id);						
					} else if (belief.ags instanceof MutualAgentStatus) {
						dsBeliefs.removeValue("shared: {robot,human}", wmc.address.id);													   
					}
					dsTruth.removeColumn(wmc.address.id);
					dsCertainty.removeColumn(wmc.address.id);
				break;
			case OVERWRITE:
					if (belief.ags instanceof PrivateAgentStatus) {
						dsBeliefs.addValue(1, "private: {robot}", wmc.address.id);
					} else if (belief.ags instanceof AttributedAgentStatus) { 
						dsBeliefs.addValue(1, "attributed: {robot[human]}", wmc.address.id);						
					} else if (belief.ags instanceof MutualAgentStatus) {
						dsBeliefs.addValue(1, "shared: {robot,human}", wmc.address.id);
					}
					dsCertainty.setValue(((UncertainSuperFormula)belief.phi).prob, "certainty", wmc.address.id);
					updateTruth(wmc,belief);					
					break;
			}

		}

		/** cycles over the complex formula of a belief. if one formula has asserted content, the truth status of the belief is set to asserted; otherwise, fact. */
		
		synchronized void updateTruth (WorkingMemoryChange wmc, final Belief belief) { 
			try { 
			if (belief.phi instanceof ComplexFormula) { 
				ComplexFormula phi = (ComplexFormula) belief.phi;
				int polarity = 1;
				boolean asserted = false; 
				for (int i = 0; i < phi.formulae.length; i++) {
					ContinualFormula formula = (ContinualFormula) phi.formulae[i]; 
					if (formula.cstatus.equals(ContinualStatus.assertion)) { 
						asserted = true;
					} // end if
				} // end for
				if (asserted) { 
					dsTruth.setValue(((UncertainSuperFormula)belief.phi).prob * polarity, "asserted", wmc.address.id);
				} else { 
					dsTruth.setValue(((UncertainSuperFormula)belief.phi).prob * polarity, "factual", wmc.address.id);		
				}
			} else { 
				if (belief.phi instanceof ContinualFormula) { 
					int polarity = 1;
					if (((ContinualFormula)belief.phi).polarity == false) { polarity = -1; }
					if (((ContinualFormula)belief.phi).cstatus.equals(ContinualStatus.assertion)) {
						dsTruth.setValue(((UncertainSuperFormula)belief.phi).prob * polarity, "assertion", wmc.address.id);
					} else { 	
						dsTruth.setValue(((UncertainSuperFormula)belief.phi).prob * polarity, "factual", wmc.address.id);							
					} 
				} else { 
					dsTruth.setValue(((UncertainSuperFormula)belief.phi).prob, "factual", wmc.address.id);					
				} 
			} 
			} catch (ClassCastException e) { 
				System.out.println("[BMCW] cce on ["+(belief.phi).getClass().toString()+"]");
			} 
		} // end updateTruth
		
	}

		
	// global variable for the gui
	Frame guiFrame;		
		
	public BeliefModelChartViewer() {
		super();
		guiFrame = new Frame(this.getClass().getSimpleName());			
	}
		
		
	HashMap<String, Belief> hash = new HashMap<String, Belief>();

		
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
							if (existsOnWorkingMemory(_wmc.address)) {
							Belief belief = getMemoryEntry(_wmc.address, Belief.class);
							guiFrame.update(_wmc, belief);
							hash.put(_wmc.address.id, belief);
							}
							else {
								guiFrame.update(_wmc, hash.get(_wmc.address.id));
							}
							
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