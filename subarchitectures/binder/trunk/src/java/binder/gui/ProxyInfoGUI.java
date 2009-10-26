/*
 * Created by JFormDesigner on Mon Oct 26 11:17:16 CET 2009
 */

package binder.gui;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.border.*;

import binder.components.BinderMonitor;

/**
 * @author Pierre Lison
 */
public class ProxyInfoGUI extends JDialog {
	
	BinderMonitor bm;
	
	public ProxyInfoGUI(BinderGUI owner) {
		super(owner);
		bm = owner.bm;
		initComponents();
	}

	public ProxyInfoGUI(Dialog owner) {
		super(owner);
		initComponents();
	}

	private void button1ActionPerformed(ActionEvent e) {
		FeatureInfoGUI featureInfo = new FeatureInfoGUI(this);
		featureInfo.setVisible(true);
	}

	private void initComponents() {
		// JFormDesigner - Component initialization - DO NOT MODIFY  //GEN-BEGIN:initComponents
		// Generated using JFormDesigner Evaluation license - Pierre Lison
		dialogPane = new JPanel();
		panel2 = new JPanel();
		label3 = new JLabel();
		textField2 = new JTextField();
		label4 = new JLabel();
		textField3 = new JTextField();
		label5 = new JLabel();
		panel1 = new JPanel();
		slider1 = new JSlider();
		textField1 = new JTextField();
		label12 = new JLabel();
		label13 = new JLabel();
		panel9 = new JPanel();
		label6 = new JLabel();
		button1 = new JButton();
		panel10 = new JPanel();
		label15 = new JLabel();
		panel6 = new JPanel();
		label7 = new JLabel();
		label8 = new JLabel();
		buttonBar = new JPanel();
		okButton = new JButton();
		cancelButton = new JButton();

		//======== this ========
		setTitle("Insert new Proxy");
		Container contentPane = getContentPane();
		contentPane.setLayout(new BorderLayout());

		//======== dialogPane ========
		{
			dialogPane.setBorder(new EmptyBorder(12, 12, 12, 12));

			// JFormDesigner evaluation mark
			dialogPane.setBorder(new javax.swing.border.CompoundBorder(
				new javax.swing.border.TitledBorder(new javax.swing.border.EmptyBorder(0, 0, 0, 0),
					"JFormDesigner Evaluation", javax.swing.border.TitledBorder.CENTER,
					javax.swing.border.TitledBorder.BOTTOM, new java.awt.Font("Dialog", java.awt.Font.BOLD, 12),
					java.awt.Color.red), dialogPane.getBorder())); dialogPane.addPropertyChangeListener(new java.beans.PropertyChangeListener(){public void propertyChange(java.beans.PropertyChangeEvent e){if("border".equals(e.getPropertyName()))throw new RuntimeException();}});

			dialogPane.setLayout(new BorderLayout());

			//======== panel2 ========
			{
				panel2.setLayout(new GridBagLayout());
				((GridBagLayout)panel2.getLayout()).columnWidths = new int[] {351, 237, 0};
				((GridBagLayout)panel2.getLayout()).rowHeights = new int[] {0, 0, 0, 0, 0};
				((GridBagLayout)panel2.getLayout()).columnWeights = new double[] {1.0, 1.0, 1.0E-4};
				((GridBagLayout)panel2.getLayout()).rowWeights = new double[] {1.0, 1.0, 1.0, 1.0, 1.0E-4};

				//---- label3 ----
				label3.setText("Proxy identifier: ");
				panel2.add(label3, new GridBagConstraints(0, 0, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- textField2 ----
				textField2.setText("ProxyX");
				textField2.setColumns(12);
				panel2.add(textField2, new GridBagConstraints(1, 0, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- label4 ----
				label4.setText("Originating subarchitecture: ");
				panel2.add(label4, new GridBagConstraints(0, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- textField3 ----
				textField3.setText("subarchY");
				textField3.setColumns(12);
				panel2.add(textField3, new GridBagConstraints(1, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- label5 ----
				label5.setText("Prob ( exists | obs ): ");
				panel2.add(label5, new GridBagConstraints(0, 2, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//======== panel1 ========
				{
					panel1.setLayout(new BorderLayout());

					//---- slider1 ----
					slider1.setMaximum(1);
					panel1.add(slider1, BorderLayout.CENTER);

					//---- textField1 ----
					textField1.setText("0.75");
					panel1.add(textField1, BorderLayout.EAST);
				}
				panel2.add(panel1, new GridBagConstraints(1, 2, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- label12 ----
				label12.setText("     ");
				label12.setFont(new Font("Lucida Grande", Font.PLAIN, 8));
				panel2.add(label12, new GridBagConstraints(0, 3, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 0, 0), 0, 0));

				//---- label13 ----
				label13.setText("    ");
				label13.setFont(new Font("Lucida Grande", Font.PLAIN, 8));
				panel2.add(label13, new GridBagConstraints(1, 3, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 0, 0), 0, 0));
			}
			dialogPane.add(panel2, BorderLayout.NORTH);

			//======== panel9 ========
			{
				panel9.setLayout(new GridBagLayout());
				((GridBagLayout)panel9.getLayout()).columnWidths = new int[] {138, 0, 180, 0};
				((GridBagLayout)panel9.getLayout()).rowHeights = new int[] {0, 0, 0, 0, 0};
				((GridBagLayout)panel9.getLayout()).columnWeights = new double[] {0.0, 1.0, 0.0, 1.0E-4};
				((GridBagLayout)panel9.getLayout()).rowWeights = new double[] {0.0, 0.0, 0.0, 0.0, 1.0E-4};

				//---- label6 ----
				label6.setText("Included features: ");
				panel9.add(label6, new GridBagConstraints(0, 0, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 5), 0, 0));

				//---- button1 ----
				button1.setText("Add new feature");
				button1.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						button1ActionPerformed(e);
					}
				});
				panel9.add(button1, new GridBagConstraints(2, 0, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 0), 0, 0));

				//======== panel10 ========
				{
					panel10.setBorder(new BevelBorder(BevelBorder.LOWERED));
					panel10.setLayout(new GridBagLayout());
					((GridBagLayout)panel10.getLayout()).columnWidths = new int[] {0, 0, 0};
					((GridBagLayout)panel10.getLayout()).rowHeights = new int[] {0, 0, 0, 0};
					((GridBagLayout)panel10.getLayout()).columnWeights = new double[] {0.0, 0.0, 1.0E-4};
					((GridBagLayout)panel10.getLayout()).rowWeights = new double[] {0.0, 0.0, 0.0, 1.0E-4};

					//---- label15 ----
					label15.setText("No features");
					label15.setFont(label15.getFont().deriveFont(label15.getFont().getStyle() | Font.ITALIC));
					panel10.add(label15, new GridBagConstraints(0, 1, 1, 1, 0.0, 0.0,
						GridBagConstraints.CENTER, GridBagConstraints.BOTH,
						new Insets(0, 0, 5, 5), 0, 0));
				}
				panel9.add(panel10, new GridBagConstraints(0, 1, 3, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 0), 0, 0));

				//======== panel6 ========
				{
					panel6.setLayout(new BorderLayout());

					//---- label7 ----
					label7.setText("Total number of features:  ");
					panel6.add(label7, BorderLayout.CENTER);

					//---- label8 ----
					label8.setText("0 ");
					panel6.add(label8, BorderLayout.EAST);
				}
				panel9.add(panel6, new GridBagConstraints(2, 2, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 0), 0, 0));
			}
			dialogPane.add(panel9, BorderLayout.CENTER);

			//======== buttonBar ========
			{
				buttonBar.setBorder(new EmptyBorder(12, 0, 0, 0));
				buttonBar.setLayout(new GridBagLayout());
				((GridBagLayout)buttonBar.getLayout()).columnWidths = new int[] {0, 85, 80};
				((GridBagLayout)buttonBar.getLayout()).columnWeights = new double[] {1.0, 0.0, 0.0};

				//---- okButton ----
				okButton.setText("Insert");
				buttonBar.add(okButton, new GridBagConstraints(1, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 0, 5), 0, 0));

				//---- cancelButton ----
				cancelButton.setText("Cancel");
				buttonBar.add(cancelButton, new GridBagConstraints(2, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 0, 0), 0, 0));
			}
			dialogPane.add(buttonBar, BorderLayout.SOUTH);
		}
		contentPane.add(dialogPane, BorderLayout.CENTER);
		pack();
		setLocationRelativeTo(getOwner());
		// JFormDesigner - End of component initialization  //GEN-END:initComponents
		
		setSize(new Dimension(400, 350));
		setPreferredSize(new Dimension(400, 350));
	}

	// JFormDesigner - Variables declaration - DO NOT MODIFY  //GEN-BEGIN:variables
	// Generated using JFormDesigner Evaluation license - Pierre Lison
	private JPanel dialogPane;
	private JPanel panel2;
	private JLabel label3;
	private JTextField textField2;
	private JLabel label4;
	private JTextField textField3;
	private JLabel label5;
	private JPanel panel1;
	private JSlider slider1;
	private JTextField textField1;
	private JLabel label12;
	private JLabel label13;
	private JPanel panel9;
	private JLabel label6;
	private JButton button1;
	private JPanel panel10;
	private JLabel label15;
	private JPanel panel6;
	private JLabel label7;
	private JLabel label8;
	private JPanel buttonBar;
	private JButton okButton;
	private JButton cancelButton;
	// JFormDesigner - End of variables declaration  //GEN-END:variables
}
