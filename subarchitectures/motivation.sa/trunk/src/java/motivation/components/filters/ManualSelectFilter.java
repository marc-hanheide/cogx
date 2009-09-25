package motivation.components.filters;

import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JPanel;

import motivation.slice.CategorizePlaceMotive;
import motivation.slice.ExploreMotive;
import motivation.slice.HomingMotive;
import motivation.slice.Motive;
import motivation.slice.MotiveStatus;
import motivation.slice.TestMotive;
import cast.CASTException;

public class ManualSelectFilter implements MotiveFilter {

	/**
	 * 
	 */
	protected ManualSelectFilter() {
		getJFrame();
		jFrame.setVisible(true);
		jFrame.pack();
	}

	private MotiveFilterManager component;

	private JFrame jFrame = null; // @jve:decl-index=0:visual-constraint="1,21"
	private JPanel jContentPane = null;
	private JCheckBox jCheckBoxExploreMotive = null; // @jve:decl-index=0:visual-constraint="367,125"
	private JCheckBox jCheckBoxHomingMotive = null; // @jve:decl-index=0:visual-constraint="365,163"
	private JCheckBox jCheckBoxTestMotive = null; // @jve:decl-index=0:visual-constraint="421,181"
	private JButton jButtonUpdate = null; // @jve:decl-index=0:visual-constraint="325,118"
	private JCheckBox jCheckBoxCategorizePlace = null; // @jve:decl-index=0:visual-constraint="427,60"

	public boolean shouldBeSurfaced(Motive motive) {
		if (motive instanceof ExploreMotive)
			return jCheckBoxExploreMotive.isSelected();
		else if (motive instanceof TestMotive)
			return jCheckBoxTestMotive.isSelected();
		else if (motive instanceof HomingMotive)
			return jCheckBoxHomingMotive.isSelected();
		else if (motive instanceof CategorizePlaceMotive)
			return jCheckBoxCategorizePlace.isSelected();
		else
			return true;
	}


	public boolean shouldBeUnsurfaced(Motive motive) {
		if (motive.status==MotiveStatus.ACTIVE)
			return false;
		
		if (motive instanceof ExploreMotive)
			return !jCheckBoxExploreMotive.isSelected();
		else if (motive instanceof TestMotive)
			return !jCheckBoxTestMotive.isSelected();
		else if (motive instanceof HomingMotive)
			return !jCheckBoxHomingMotive.isSelected();
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
			jFrame.setSize(new Dimension(318, 184));
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
			jContentPane.add(getJCheckBoxExploreMotive());
			jContentPane.add(getJCheckBoxTestMotive());
			jContentPane.add(getJCheckBoxHomingMotive());
			jContentPane.add(getJCheckBoxCategorizePlace());
			jContentPane.add(getJButtonUpdate());
		}
		return jContentPane;
	}

	/**
	 * This method initializes jCheckBox
	 * 
	 * @return javax.swing.JCheckBox
	 */
	private JCheckBox getJCheckBoxExploreMotive() {
		if (jCheckBoxExploreMotive == null) {
			jCheckBoxExploreMotive = new JCheckBox();
			jCheckBoxExploreMotive.setSize(new Dimension(257, 29));
			jCheckBoxExploreMotive.setText("let ExploreMotives pass");
		}
		return jCheckBoxExploreMotive;
	}

	/**
	 * This method initializes jCheckBox
	 * 
	 * @return javax.swing.JCheckBox
	 */
	private JCheckBox getJCheckBoxHomingMotive() {
		if (jCheckBoxHomingMotive == null) {
			jCheckBoxHomingMotive = new JCheckBox();
			jCheckBoxHomingMotive.setSize(new Dimension(257, 29));
			jCheckBoxHomingMotive.setText("let HomingMotives pass");
		}
		return jCheckBoxHomingMotive;
	}

	/**
	 * This method initializes jCheckBox1
	 * 
	 * @return javax.swing.JCheckBox
	 */
	private JCheckBox getJCheckBoxTestMotive() {
		if (jCheckBoxTestMotive == null) {
			jCheckBoxTestMotive = new JCheckBox();
			jCheckBoxTestMotive.setSize(new Dimension(125, 22));
			jCheckBoxTestMotive.setText("let TestMotives pass");
		}
		return jCheckBoxTestMotive;
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
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}

				}
			});
		}
		return jButtonUpdate;
	}

	/**
	 * This method initializes jCheckBoxCategorizePlace
	 * 
	 * @return javax.swing.JCheckBox
	 */
	private JCheckBox getJCheckBoxCategorizePlace() {
		if (jCheckBoxCategorizePlace == null) {
			jCheckBoxCategorizePlace = new JCheckBox();
			jCheckBoxCategorizePlace.setText("let CategorizePlaceMotives pass");
		}
		return jCheckBoxCategorizePlace;
	}

	public void setManager(MotiveFilterManager motiveFilterManager) {
		component = motiveFilterManager;
	}

} // @jve:decl-index=0:visual-constraint="583,36"
