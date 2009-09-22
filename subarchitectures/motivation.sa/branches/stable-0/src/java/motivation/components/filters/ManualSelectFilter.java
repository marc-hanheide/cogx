package motivation.components.filters;

import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JPanel;

import motivation.slice.ExploreMotive;
import motivation.slice.Motive;
import motivation.slice.TestMotive;
import javax.swing.JButton;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import cast.CASTException;

public class ManualSelectFilter extends AbstractFilter {

	/* (non-Javadoc)
	 * @see motivation.components.filters.AbstractFilter#start()
	 */
	@Override
	protected void start() {
		super.start();
		JFrame frame = getJFrame();
		frame.setVisible(true);
		frame.pack();
	}

	private JFrame jFrame = null;  //  @jve:decl-index=0:visual-constraint="0,-1"
	private JPanel jContentPane = null;
	private JCheckBox jCheckBoxExploreMotive = null;  //  @jve:decl-index=0:visual-constraint="367,125"
	private JCheckBox jCheckBoxTestMotive = null;  //  @jve:decl-index=0:visual-constraint="421,181"
	private JButton jButtonUpdate = null;  //  @jve:decl-index=0:visual-constraint="325,118"

	@Override
	protected boolean shouldBeSurfaced(Motive motive) {
		if (motive instanceof ExploreMotive)
			return jCheckBoxExploreMotive.isSelected();
		else if (motive instanceof TestMotive)
			return jCheckBoxTestMotive.isSelected();
		else
			return true;
	}

	@Override
	protected boolean shouldBeUnsurfaced(Motive motive) {
		if (motive instanceof ExploreMotive)
			return !jCheckBoxExploreMotive.isSelected();
		else if (motive instanceof TestMotive)
			return !jCheckBoxTestMotive.isSelected();
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
			jFrame.setSize(new Dimension(318, 256));
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
			jContentPane.setLayout(new GridLayout(0,1));
			jContentPane.add(getJCheckBoxExploreMotive());
			jContentPane.add(getJCheckBoxTestMotive());
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
			jButtonUpdate.setText("update all");
			jButtonUpdate.addActionListener(new ActionListener() {
				
				@Override
				public void actionPerformed(ActionEvent e) {
					try {
						println("update everything");
						checkAll();
					} catch (CASTException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					
				}
			});
		}
		return jButtonUpdate;
	}

}
