/**
 * 
 */
package comsys.fake;

import java.awt.GridLayout;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

/**
 * @author marc
 *
 */
public class FakeComsysFrame extends JFrame {

	private static final long serialVersionUID = 1L;
	private JPanel jContentPane = null;
	private JLabel jLabelQuestion = null;
	private JComboBox jComboBoxAnswer = null;
	private JPanel jPanelButtons = null;
	private JButton jButtonOK = null;
	private JButton jButtonCancel = null;
	/**
	 * This is the default constructor
	 */
	public FakeComsysFrame() {
		super();
		initialize();
	}

	/**
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {
		this.setSize(663, 136);
		this.setContentPane(getJContentPane());
		this.setTitle("ComSysFake");
	}

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {
		if (jContentPane == null) {
			GridLayout gridLayout = new GridLayout();
			gridLayout.setRows(3);
			gridLayout.setColumns(1);
			jLabelQuestion = new JLabel();
			jLabelQuestion.setText("Question to ask");
			jContentPane = new JPanel();
			jContentPane.setLayout(gridLayout);
			jContentPane.add(jLabelQuestion, null);
			jContentPane.add(getJComboBoxAnswer(), null);
			jContentPane.add(getJPanelButtons(), null);
		}
		return jContentPane;
	}

	/**
	 * This method initializes jComboBox	
	 * 	
	 * @return javax.swing.JComboBox	
	 */
	public JComboBox getJComboBoxAnswer() {
		if (jComboBoxAnswer == null) {
			jComboBoxAnswer = new JComboBox();
		}
		return jComboBoxAnswer;
	}

	/**
	 * This method initializes jPanel	
	 * 	
	 * @return javax.swing.JPanel	
	 */
	private JPanel getJPanelButtons() {
		if (jPanelButtons == null) {
			GridLayout gridLayout1 = new GridLayout();
			gridLayout1.setRows(1);
			gridLayout1.setColumns(2);
			jPanelButtons = new JPanel();
			jPanelButtons.setLayout(gridLayout1);
			jPanelButtons.add(getJButtonCancel(), null);
			jPanelButtons.add(getJButtonOK(), null);
		}
		return jPanelButtons;
	}

	/**
	 * This method initializes jButton	
	 * 	
	 * @return javax.swing.JButton	
	 */
	public JButton getJButtonOK() {
		if (jButtonOK == null) {
			jButtonOK = new JButton();
			jButtonOK.setText("OK");
			jButtonOK.addActionListener(new java.awt.event.ActionListener() {
				public void actionPerformed(java.awt.event.ActionEvent e) {
					System.out.println("actionPerformed()"); // TODO Auto-generated Event stub actionPerformed()
				}
			});
		}
		return jButtonOK;
	}

	/**
	 * This method initializes jButton1	
	 * 	
	 * @return javax.swing.JButton	
	 */
	public JButton getJButtonCancel() {
		if (jButtonCancel == null) {
			jButtonCancel = new JButton();
			jButtonCancel.setText("Cancel");
			jButtonCancel.addActionListener(new java.awt.event.ActionListener() {
				public void actionPerformed(java.awt.event.ActionEvent e) {
					System.out.println("actionPerformed()"); // TODO Auto-generated Event stub actionPerformed()
				}
			});
		}
		return jButtonCancel;
	}

	/**
	 * @return the jLabelQuestion
	 */
	public JLabel getjLabelQuestion() {
		return jLabelQuestion;
	}


}  //  @jve:decl-index=0:visual-constraint="10,10"
