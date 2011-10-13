package motivation.components.filters;

import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.Map;

import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;

import motivation.slice.AnalyzeProtoObjectMotive;
import motivation.slice.LearnObjectFeatureMotive;
import motivation.slice.LookAtViewConeMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.TutorInitiativeMotive;
import cast.CASTException;
import cast.cdl.WorkingMemoryChange;

public class GeorgeManualSelectFilter implements MotiveFilter {

	/**
	 * 
	 */
	public GeorgeManualSelectFilter() {
		getJFrame();
	}

	private MotiveFilterManager component; // @jve:decl-index=0:

	private JFrame jFrame = null; // @jve:decl-index=0:visual-constraint="1,21"
	private JPanel jContentPane = null;
	private JButton jButtonUpdate = null; // @jve:decl-index=0:visual-constraint="325,118"

	private JPanel jMotivesPanel = null;

//	private JSlider jcannedPrioritySlider = null;
	private JSlider jRobotInitiativePrioritySlider = null;
	private JSlider jTutorInitiativePrioritySlider = null;
	private JSlider jRobotNonSituatedPrioritySlider = null;

	private JSlider jAnalyzePOPrioritySlider = null;
	private JSlider jLearnVOPrioritySlider = null;
	private JSlider jLookAroundPrioritySlider = null;

	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {

		// if (motive instanceof CannedTextMotive)
		// return MotivePriority.values()[jcannedPrioritySlider.getValue()];
		// else if (motive instanceof RobotInitiativeMotive)
		// return MotivePriority.values()[jRobotInitiativePrioritySlider
		// .getValue()];
		// else
		if (motive instanceof TutorInitiativeMotive) {
			return MotivePriority.values()[jTutorInitiativePrioritySlider
					.getValue()];
		}
		// else if (motive instanceof RobotNonSituatedMotive)
		// return MotivePriority.values()[jRobotNonSituatedPrioritySlider
		// .getValue()];
		// else
		else if (motive instanceof AnalyzeProtoObjectMotive) {
			return MotivePriority.values()[jAnalyzePOPrioritySlider.getValue()];
		} else if (motive instanceof LearnObjectFeatureMotive) {
			return MotivePriority.values()[jLearnVOPrioritySlider.getValue()];
		} else if (motive instanceof LookAtViewConeMotive) {
			return MotivePriority.values()[jLookAroundPrioritySlider.getValue()];
		}
		else {
			return MotivePriority.NORMAL;
		}
	}

	/**
	 * This method initializes jFrame
	 * 
	 * @return javax.swing.JFrame
	 */
	private JFrame getJFrame() {
		if (jFrame == null) {
			jFrame = new JFrame();
			jFrame.setTitle("Motivation::ManualSelectFilter");
			jFrame.setContentPane(getJContentPane());
		}
		return jFrame;
	}

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {
		if (jContentPane == null) {
			jContentPane = new JPanel();
			jContentPane.setLayout(new GridLayout(0, 1));
			jContentPane.add(getJMotivesPanel());
			jMotivesPanel.add(getJButtonUpdate());
		}
		return jContentPane;
	}

	/**
	 * This method initializes jButton
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getJButtonUpdate() {
		if (jButtonUpdate == null) {
			jButtonUpdate = new JButton();
			jButtonUpdate.setText("recheck all");
			jButtonUpdate.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent e) {
					try {
						component.println("update everything");
						component.checkAll();
					} catch (CASTException e1) {
						component.println("unexpected exception in checkAll: ");
						e1.printStackTrace();
					}

				}
			});
		}
		return jButtonUpdate;
	}

	public void setManager(MotiveFilterManager motiveFilterManager) {
		component = motiveFilterManager;
	}

	/**
	 * This method initializes jMotivesPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJMotivesPanel() {
		if (jMotivesPanel == null) {
			GridLayout gridLayout = new GridLayout(0, 2);
			// gridLayout.setColumns(1);
			jMotivesPanel = new JPanel();
			jMotivesPanel.setLayout(gridLayout);
			// jMotivesPanel.add(new JLabel("canned Goal"));
			// jMotivesPanel.add(getJSlidercanned());
			// jMotivesPanel.add(new JLabel("Robot Init."));
			// jMotivesPanel.add(getJSliderRobotInitiative());
			jMotivesPanel.add(new JLabel("Tutor Init."));
			jMotivesPanel.add(getJSliderTutorInitiative());
			// jMotivesPanel.add(new JLabel("Robot NonSit."));
			// jMotivesPanel.add(getJSliderRobotNonSituated());
			jMotivesPanel.add(new JLabel("Analyse PO"));
			jMotivesPanel.add(getJSliderAnalyzePO());
			jMotivesPanel.add(new JLabel("Robot Init."));
			jMotivesPanel.add(getJSliderLearnVO());
			jMotivesPanel.add(new JLabel("Look Around"));
			jMotivesPanel.add(getJSliderLookAround());

			jMotivesPanel.add(getPresetPanel());
		}
		return jMotivesPanel;
	}

	private JPanel getPresetPanel() {
		JPanel presetPanel = new JPanel();
		presetPanel.add(new JLabel("presets:"));

		JButton nullButton = new JButton("unsurface all");
		nullButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				try {
					// default start-up slider values
					jAnalyzePOPrioritySlider.setValue(0);
					jLearnVOPrioritySlider.setValue(0);
					jLookAroundPrioritySlider.setValue(0);
					jTutorInitiativePrioritySlider.setValue(0);
					component.checkAll();
				} catch (CASTException e1) {
					component.println("unexpected exception in checkAll: ");
					e1.printStackTrace();
				}

			}
		});
		presetPanel.add(nullButton);

		JButton doraButton = new JButton("cogx default");
		doraButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				try {
					// default start-up slider values
					jAnalyzePOPrioritySlider.setValue(1);
					jLearnVOPrioritySlider.setValue(2);
					jLookAroundPrioritySlider.setValue(0);
					jTutorInitiativePrioritySlider.setValue(3);

					component.checkAll();
				} catch (CASTException e1) {
					component.println("unexpected exception in checkAll: ");
					e1.printStackTrace();
				}

			}
		});
		presetPanel.add(doraButton);

		return presetPanel;
	}

	private JSlider createPrioritySlider() {
		JSlider jSlider = new JSlider(JSlider.HORIZONTAL,
				MotivePriority.UNSURFACE.ordinal(),
				MotivePriority.HIGH.ordinal(),
				MotivePriority.UNSURFACE.ordinal());
		Dictionary<Integer, JComponent> labels;
		labels = new Hashtable<Integer, JComponent>();
		labels.put(MotivePriority.UNSURFACE.ordinal(), new JLabel(
				MotivePriority.UNSURFACE.name()));
		labels.put(MotivePriority.LOW.ordinal(),
				new JLabel(MotivePriority.LOW.name()));
		labels.put(MotivePriority.NORMAL.ordinal(), new JLabel(
				MotivePriority.NORMAL.name()));
		labels.put(MotivePriority.HIGH.ordinal(), new JLabel(
				MotivePriority.HIGH.name()));
		jSlider.setLabelTable(labels);
		jSlider.setPaintTicks(true);
		jSlider.setPaintLabels(true);
		jSlider.setSnapToTicks(true);
		return jSlider;

	}

//	/**
//	 * This method initializes jExplorePrioritySlider
//	 * 
//	 * @return javax.swing.JSlider
//	 */
//	private JSlider getJSlidercanned() {
//		if (jcannedPrioritySlider == null) {
//			jcannedPrioritySlider = createPrioritySlider();
//		}
//		return jcannedPrioritySlider;
//	}

	/**
	 * This method initializes jExplorePrioritySlider
	 * 
	 * @return javax.swing.JSlider
	 */
	private JSlider getJSliderRobotInitiative() {
		if (jRobotInitiativePrioritySlider == null) {
			jRobotInitiativePrioritySlider = createPrioritySlider();
		}
		return jRobotInitiativePrioritySlider;
	}

	/**
	 * This method initializes jExplorePrioritySlider
	 * 
	 * @return javax.swing.JSlider
	 */
	private JSlider getJSliderTutorInitiative() {
		if (jTutorInitiativePrioritySlider == null) {
			jTutorInitiativePrioritySlider = createPrioritySlider();
		}
		return jTutorInitiativePrioritySlider;
	}

	/**
	 * This method initializes jExplorePrioritySlider
	 * 
	 * @return javax.swing.JSlider
	 */
	private JSlider getJSliderRobotNonSituated() {
		if (jRobotNonSituatedPrioritySlider == null) {
			jRobotNonSituatedPrioritySlider = createPrioritySlider();
		}
		return jRobotNonSituatedPrioritySlider;
	}

	private JSlider getJSliderAnalyzePO() {
		if (jAnalyzePOPrioritySlider == null) {
			jAnalyzePOPrioritySlider = createPrioritySlider();
		}
		return jAnalyzePOPrioritySlider;
	}

	private JSlider getJSliderLearnVO() {
		if (jLearnVOPrioritySlider == null) {
			jLearnVOPrioritySlider = createPrioritySlider();
		}
		return jLearnVOPrioritySlider;
	}

	private JSlider getJSliderLookAround() {
		if (jLookAroundPrioritySlider == null) {
			jLookAroundPrioritySlider = createPrioritySlider();
		}
		return jLookAroundPrioritySlider;
	}

	@Override
	public void start() {
		jFrame.setVisible(true);
		jFrame.pack();
		jFrame.setSize(800, 600);

		// default start-up slider values
		jAnalyzePOPrioritySlider.setValue(1);
		jLearnVOPrioritySlider.setValue(0);
		jLookAroundPrioritySlider.setValue(0);
		jTutorInitiativePrioritySlider.setValue(3);

	}

	@Override
	public void configure(Map<String, String> arg0) {

	}

} // @jve:decl-index=0:visual-constraint="583,36"
