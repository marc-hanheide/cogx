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
import motivation.slice.CannedTextMotive;
import motivation.slice.CategorizeRoomMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.PatrolMotive;
import motivation.slice.RobotInitiativeMotive;
import motivation.slice.TutorInitiativeMotive;
import motivation.slice.RobotNonSituatedMotive;
import cast.CASTException;
import cast.cdl.WorkingMemoryChange;

public class ManualSelectFilter implements MotiveFilter {

	/**
	 * 
	 */
	public ManualSelectFilter() {
		getJFrame();
	}

	private MotiveFilterManager component; // @jve:decl-index=0:

	private JFrame jFrame = null; // @jve:decl-index=0:visual-constraint="1,21"
	private JPanel jContentPane = null;
	private JButton jButtonUpdate = null; // @jve:decl-index=0:visual-constraint="325,118"

	private JPanel jMotivesPanel = null;

	private JSlider jExplorePrioritySlider = null; // @jve:decl-index=0:visual-constraint="414,85"

	private JSlider jHomePrioritySlider = null;

	private JSlider jCategorizePrioritySlider = null;

	private JSlider jPatrolPrioritySlider = null;

	private JSlider jcannedPrioritySlider = null;
	private JSlider jRobotInitiativePrioritySlider = null;
	private JSlider jTutorInitiativePrioritySlider = null;
	private JSlider jRobotNonSituatedPrioritySlider = null;

	private JSlider jAnalyzePOPrioritySlider = null;
	
	
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {

		if (motive instanceof ExploreMotive)
			return MotivePriority.values()[jExplorePrioritySlider.getValue()];
		else if (motive instanceof HomingMotive)
			return MotivePriority.values()[jHomePrioritySlider.getValue()];
		else if (motive instanceof CategorizeRoomMotive)
			return MotivePriority.values()[jCategorizePrioritySlider.getValue()];
		else if (motive instanceof CannedTextMotive)
			return MotivePriority.values()[jcannedPrioritySlider.getValue()];
		else if (motive instanceof PatrolMotive)
			return MotivePriority.values()[jPatrolPrioritySlider.getValue()];
		else if (motive instanceof RobotInitiativeMotive)
			return MotivePriority.values()[jRobotInitiativePrioritySlider.getValue()];
		else if (motive instanceof TutorInitiativeMotive)
			return MotivePriority.values()[jTutorInitiativePrioritySlider.getValue()];
		else if (motive instanceof RobotNonSituatedMotive)
			return MotivePriority.values()[jRobotNonSituatedPrioritySlider.getValue()];
		else if (motive instanceof AnalyzeProtoObjectMotive)
			return MotivePriority.values()[jAnalyzePOPrioritySlider.getValue()];
		else
			return MotivePriority.NORMAL;
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
			jMotivesPanel.add(new JLabel("canned Goal"));
			jMotivesPanel.add(getJSlidercanned());
			jMotivesPanel.add(new JLabel("Categorize"));
			jMotivesPanel.add(getJSliderCategorize());
			jMotivesPanel.add(new JLabel("Explore"));
			jMotivesPanel.add(getJSliderExplore());
			jMotivesPanel.add(new JLabel("Patrol/Goto"));
			jMotivesPanel.add(getJSliderPatrol());
			// jMotivesPanel.add(new JLabel("Test"));
			// jMotivesPanel.add(getJSliderTest());
			jMotivesPanel.add(new JLabel("Homing"));
			jMotivesPanel.add(getJSliderHome());
			jMotivesPanel.add(new JLabel("Robot Init."));
			jMotivesPanel.add(getJSliderRobotInitiative());
			jMotivesPanel.add(new JLabel("Tutor Init."));
			jMotivesPanel.add(getJSliderTutorInitiative());
			jMotivesPanel.add(new JLabel("Robot NonSit."));
			jMotivesPanel.add(getJSliderRobotNonSituated());
			jMotivesPanel.add(new JLabel("Anal. PO"));
			jMotivesPanel.add(getJSliderAnalyzePO());
			jMotivesPanel.add(getPresetPanel());
		}
		return jMotivesPanel;
	}

	private JPanel getPresetPanel() {
		JPanel presetPanel = new JPanel();
		presetPanel.add(new JLabel("presets:"));
		JButton ecButton = new JButton("explore > categorize");
		ecButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				try {
					jExplorePrioritySlider.setValue(2);
					jCategorizePrioritySlider.setValue(1);
					component.checkAll();
				} catch (CASTException e1) {
					component.println("unexpected exception in checkAll: ");
					e1.printStackTrace();
				}

			}
		});

		// presetPanel.add(ecButton);

		JButton ceButton = new JButton("categorize > explore");
		ceButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				try {
					jExplorePrioritySlider.setValue(1);
					jCategorizePrioritySlider.setValue(2);
					component.checkAll();
				} catch (CASTException e1) {
					component.println("unexpected exception in checkAll: ");
					e1.printStackTrace();
				}

			}
		});
		// presetPanel.add(ceButton);

		JButton peButton = new JButton("patrol == explore");
		peButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				try {
					jExplorePrioritySlider.setValue(1);
					jPatrolPrioritySlider.setValue(1);
					component.checkAll();
				} catch (CASTException e1) {
					component.println("unexpected exception in checkAll: ");
					e1.printStackTrace();
				}

			}
		});
		// presetPanel.add(peButton);

		JButton nullButton = new JButton("unsurface all");
		nullButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				try {
					jExplorePrioritySlider.setValue(0);
					jCategorizePrioritySlider.setValue(0);
					jHomePrioritySlider.setValue(0);
					jPatrolPrioritySlider.setValue(0);
					jcannedPrioritySlider.setValue(0);
					jRobotInitiativePrioritySlider.setValue(0);
					jTutorInitiativePrioritySlider.setValue(0);
					jRobotNonSituatedPrioritySlider.setValue(0);
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
					jExplorePrioritySlider.setValue(2);
					jCategorizePrioritySlider.setValue(2);
					jHomePrioritySlider.setValue(0);
					jPatrolPrioritySlider.setValue(1);
					jcannedPrioritySlider.setValue(1);
					jRobotInitiativePrioritySlider.setValue(2);
					jTutorInitiativePrioritySlider.setValue(3);
					jRobotNonSituatedPrioritySlider.setValue(1);
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
				MotivePriority.UNSURFACE.ordinal(), MotivePriority.HIGH
						.ordinal(), MotivePriority.UNSURFACE.ordinal());
		Dictionary<Integer, JComponent> labels;
		labels = new Hashtable<Integer, JComponent>();
		labels.put(MotivePriority.UNSURFACE.ordinal(), new JLabel(
				MotivePriority.UNSURFACE.name()));
		labels.put(MotivePriority.LOW.ordinal(), new JLabel(MotivePriority.LOW
				.name()));
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

	/**
	 * This method initializes jExplorePrioritySlider
	 * 
	 * @return javax.swing.JSlider
	 */
	private JSlider getJSlidercanned() {
		if (jcannedPrioritySlider == null) {
			jcannedPrioritySlider = createPrioritySlider();
		}
		return jcannedPrioritySlider;
	}

	/**
	 * This method initializes jExplorePrioritySlider
	 * 
	 * @return javax.swing.JSlider
	 */
	private JSlider getJSliderExplore() {
		if (jExplorePrioritySlider == null) {
			jExplorePrioritySlider = createPrioritySlider();
		}
		return jExplorePrioritySlider;
	}

	/**
	 * This method initializes jExplorePrioritySlider
	 * 
	 * @return javax.swing.JSlider
	 */
	private JSlider getJSliderPatrol() {
		if (jPatrolPrioritySlider == null) {
			jPatrolPrioritySlider = createPrioritySlider();
		}
		return jPatrolPrioritySlider;
	}

	/**
	 * This method initializes jExplorePrioritySlider
	 * 
	 * @return javax.swing.JSlider
	 */
	private JSlider getJSliderCategorize() {
		if (jCategorizePrioritySlider == null) {
			jCategorizePrioritySlider = createPrioritySlider();
		}
		return jCategorizePrioritySlider;
	}

	/**
	 * This method initializes jExplorePrioritySlider
	 * 
	 * @return javax.swing.JSlider
	 */
	private JSlider getJSliderHome() {
		if (jHomePrioritySlider == null) {
			jHomePrioritySlider = createPrioritySlider();
		}
		return jHomePrioritySlider;
	}

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
	
	@Override
	public void start() {
		jFrame.setVisible(true);
		jFrame.pack();
		jFrame.setSize(800, 600);
		jTutorInitiativePrioritySlider.setValue(3);

		
	}

	@Override
	public void configure(Map<String, String> arg0) {
		// TODO Auto-generated method stub

	}

} // @jve:decl-index=0:visual-constraint="583,36"
