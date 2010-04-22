package motivation.components.filters;

import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Dictionary;
import java.util.Hashtable;

import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;

import spatial.motivation.PatrolPlaceGenerator;

import motivation.slice.CategorizeRoomMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.GeneralGoalMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.PatrolMotive;
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

	private JSlider jGeneralPrioritySlider = null;
	
	public MotivePriority checkMotive(Motive motive, WorkingMemoryChange wmc) {

		if (motive instanceof ExploreMotive)
			return MotivePriority.convert(jExplorePrioritySlider.getValue());
		else if (motive instanceof HomingMotive)
			return MotivePriority.convert(jHomePrioritySlider.getValue());
		else if (motive instanceof CategorizeRoomMotive)
			return MotivePriority.convert(jCategorizePrioritySlider.getValue());
		else if (motive instanceof GeneralGoalMotive)
			return MotivePriority.convert(jGeneralPrioritySlider.getValue());
		else if (motive instanceof PatrolMotive)
			return MotivePriority.convert(jPatrolPrioritySlider.getValue());
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
			GridLayout gridLayout = new GridLayout(0, 1);
			gridLayout.setColumns(1);
			jMotivesPanel = new JPanel();
			jMotivesPanel.setLayout(gridLayout);
			jMotivesPanel.add(new JLabel("General Goal"));
			jMotivesPanel.add(getJSliderGeneral());
			jMotivesPanel.add(new JLabel("Categorize"));
			jMotivesPanel.add(getJSliderCategorize());
			jMotivesPanel.add(new JLabel("Explore"));
			jMotivesPanel.add(getJSliderExplore());
			jMotivesPanel.add(new JLabel("Patrol"));
			jMotivesPanel.add(getJSliderPatrol());
			// jMotivesPanel.add(new JLabel("Test"));
			// jMotivesPanel.add(getJSliderTest());
			jMotivesPanel.add(new JLabel("Homing"));
			jMotivesPanel.add(getJSliderHome());
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

		presetPanel.add(ecButton);

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
		presetPanel.add(ceButton);

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
		presetPanel.add(peButton);
		
		
		JButton nullButton = new JButton("unsurface all");
		nullButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				try {
					jExplorePrioritySlider.setValue(0);
					jCategorizePrioritySlider.setValue(0);
					jHomePrioritySlider.setValue(0);
					jPatrolPrioritySlider.setValue(0);
					jGeneralPrioritySlider.setValue(0);
					component.checkAll();
				} catch (CASTException e1) {
					component.println("unexpected exception in checkAll: ");
					e1.printStackTrace();
				}

			}
		});
		presetPanel.add(nullButton);

		return presetPanel;
	}

	private JSlider createPrioritySlider() {
		JSlider jSlider = new JSlider(JSlider.HORIZONTAL,
				MotivePriority.UNSURFACE.value(), MotivePriority.HIGH.value(),
				MotivePriority.UNSURFACE.value());
		Dictionary<Integer, JComponent> labels;
		labels = new Hashtable<Integer, JComponent>();
		labels.put(MotivePriority.UNSURFACE.value(), new JLabel(
				MotivePriority.UNSURFACE.name()));
		labels.put(MotivePriority.LOW.value(), new JLabel(MotivePriority.LOW
				.name()));
		labels.put(MotivePriority.NORMAL.value(), new JLabel(
				MotivePriority.NORMAL.name()));
		labels.put(MotivePriority.HIGH.value(), new JLabel(MotivePriority.HIGH
				.name()));
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
	private JSlider getJSliderGeneral() {
		if (jGeneralPrioritySlider == null) {
			jGeneralPrioritySlider = createPrioritySlider();
		}
		return jGeneralPrioritySlider;
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

	@Override
	public void start() {
		jFrame.setVisible(true);
		jFrame.pack();
		jFrame.setSize(500, 500);
	}

} // @jve:decl-index=0:visual-constraint="583,36"
