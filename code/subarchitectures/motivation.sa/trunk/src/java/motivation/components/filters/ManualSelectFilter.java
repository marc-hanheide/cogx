package motivation.components.filters;

import java.awt.Dimension;
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

import motivation.slice.CategorizeRoomMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.slice.MotivePriority;
import motivation.slice.TestMotive;
import cast.CASTException;
import cast.cdl.WorkingMemoryChange;

public class ManualSelectFilter implements MotiveFilter {

	/**
	 * 
	 */
	public ManualSelectFilter() {
		getJFrame();
		jFrame.setVisible(true);
		jFrame.pack();
	}

	private MotiveFilterManager component;

	private JFrame jFrame = null; // @jve:decl-index=0:visual-constraint="1,21"
	private JPanel jContentPane = null;
	private JButton jButtonUpdate = null; // @jve:decl-index=0:visual-constraint="325,118"

	private JPanel jMotivesPanel = null;

	private JSlider jExplorePrioritySlider = null; // @jve:decl-index=0:visual-constraint="414,85"

	private JSlider jTestPrioritySlider =null;

	private JSlider jHomePrioritySlider = null;

	private JSlider jCategorizePrioritySlider = null;
	
	public MotivePriority shouldBeSurfaced(Motive motive, WorkingMemoryChange wmc) {
		
		if (motive instanceof ExploreMotive)
			return MotivePriority.convert(jExplorePrioritySlider.getValue());
		else if (motive instanceof TestMotive)
			return MotivePriority.convert(jTestPrioritySlider.getValue());
		else if (motive instanceof HomingMotive)
			return MotivePriority.convert(jHomePrioritySlider.getValue());
		else if (motive instanceof CategorizeRoomMotive)
			return MotivePriority.convert(jCategorizePrioritySlider.getValue());
		else
			return MotivePriority.NORMAL;
	}

	public boolean shouldBeUnsurfaced(Motive motive, WorkingMemoryChange wmc) {
		// if (motive.status==MotiveStatus.ACTIVE)
		// return false;

		if (motive instanceof ExploreMotive)
			return MotivePriority.convert(jExplorePrioritySlider.getValue())==MotivePriority.UNSURFACE;
		else if (motive instanceof TestMotive)
			return MotivePriority.convert(jTestPrioritySlider.getValue())==MotivePriority.UNSURFACE;
		else if (motive instanceof HomingMotive)
			return MotivePriority.convert(jHomePrioritySlider.getValue())==MotivePriority.UNSURFACE;
		else if (motive instanceof CategorizeRoomMotive)
			return MotivePriority.convert(jCategorizePrioritySlider.getValue())==MotivePriority.UNSURFACE;
		else
			return false;
	}

	/**
	 * This method initializes jFrame
	 * 
	 * @return javax.swing.JFrame
	 */
	private JFrame getJFrame() {
		if (jFrame == null) {
			jFrame = new JFrame();
			jFrame.setSize(new Dimension(618, 184));
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
			jMotivesPanel.add(new JLabel("Categorize"));
			jMotivesPanel.add(getJSliderCategorize());
			jMotivesPanel.add(new JLabel("Explore"));
			jMotivesPanel.add(getJSliderExplore());
			jMotivesPanel.add(new JLabel("Test"));
			jMotivesPanel.add(getJSliderTest());
			jMotivesPanel.add(new JLabel("Homing"));
			jMotivesPanel.add(getJSliderHome());
		}
		return jMotivesPanel;
	}

	private JSlider createPrioritySlider() {
		JSlider jSlider = new JSlider(JSlider.HORIZONTAL,
				MotivePriority.UNSURFACE.value(), MotivePriority.HIGH.value(), MotivePriority.UNSURFACE.value());
		Dictionary<Integer, JComponent> labels;
		labels = new Hashtable<Integer, JComponent>();
		labels.put(MotivePriority.UNSURFACE.value(), new JLabel(
				MotivePriority.UNSURFACE.name()));
		labels.put(MotivePriority.LOW.value(), new JLabel(
				MotivePriority.LOW.name()));
		labels.put(MotivePriority.NORMAL.value(), new JLabel(
				MotivePriority.NORMAL.name()));
		labels.put(MotivePriority.HIGH.value(), new JLabel(
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
	private JSlider getJSliderTest() {
		if (jTestPrioritySlider == null) {
			jTestPrioritySlider = createPrioritySlider();
		}
		return jTestPrioritySlider;
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

	
} // @jve:decl-index=0:visual-constraint="583,36"
