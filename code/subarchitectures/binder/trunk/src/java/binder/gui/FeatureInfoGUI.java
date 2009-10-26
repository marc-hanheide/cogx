/*
 * Created by JFormDesigner on Mon Oct 26 15:31:29 CET 2009
 */

package binder.gui;

import java.awt.*;
import java.awt.event.*;

import javax.swing.*;
import javax.swing.border.*;
import javax.swing.table.*;

import cast.cdl.CASTTime;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.featvalues.BooleanValue;
import binder.autogen.featvalues.FloatValue;
import binder.autogen.featvalues.IntegerValue;
import binder.autogen.featvalues.StringValue;
import binder.autogen.featvalues.UnknownValue;
import binder.components.BinderMonitor;
import binder.constructors.ProxyConstructor;
import binder.utils.FeatureValueUtils;

/**
 * @author Pierre Lison
 */
public class FeatureInfoGUI extends JDialog {
	
	BinderMonitor bm;
	
	Feature curFeature;
	
	public boolean LOGGING = true;
	
	
	public FeatureInfoGUI(ProxyInfoGUI owner) {
		super(owner);
		bm = owner.bm;
		initComponents();
		
		curFeature = new Feature("", new FeatureValue[0]);
	}

	public FeatureInfoGUI(Dialog owner) {
		super(owner);
		initComponents();
	}

	private void button1ActionPerformed(ActionEvent e) {
		setSize(new Dimension(450, 450  + (30 * curFeature.alternativeValues.length)));
		setPreferredSize(new Dimension(450, 450  + (25 * curFeature.alternativeValues.length)));
		panel5.setVisible(true);
	}

	private void button4ActionPerformed(ActionEvent e) {
		panel5.setVisible(false);
		setSize(new Dimension(450, 300  + (30 * curFeature.alternativeValues.length)));
		setPreferredSize(new Dimension(450, 300  + (25 * curFeature.alternativeValues.length)));
	}

	private void button2ActionPerformed(ActionEvent e) {
		
		if (comboBox1.getSelectedItem().equals("StringValue")) {
			float prob = Float.parseFloat(textField3.getText());
			CASTTime time = bm.getCASTTimeInMonitor();
			String val = textField2.getText();
			StringValue sv = new StringValue(prob, time, val);
			ProxyConstructor.addFeatureValueToFeature(curFeature, sv);
			updateFeatureValuesFrame();
		}
		
		if (comboBox1.getSelectedItem().equals("IntegerValue")) {
			float prob = Float.parseFloat(textField3.getText());
			CASTTime time = bm.getCASTTimeInMonitor();
			String val = textField2.getText();
			try {
			IntegerValue sv = new IntegerValue(prob, time, Integer.parseInt(val));
			ProxyConstructor.addFeatureValueToFeature(curFeature, sv);
			updateFeatureValuesFrame();
			}
			catch (NumberFormatException ex) {
				log("sorry, wrong format");
			}
		}
		
		if (comboBox1.getSelectedItem().equals("FloatValue")) {
			float prob = Float.parseFloat(textField3.getText());
			CASTTime time = bm.getCASTTimeInMonitor();
			String val = textField2.getText();
			try {
				FloatValue sv = new FloatValue(prob, time, Float.parseFloat(val));
			ProxyConstructor.addFeatureValueToFeature(curFeature, sv);
			updateFeatureValuesFrame();
			}
			catch (NumberFormatException ex) {
				log("sorry, wrong format");
			}
		}
		if (comboBox1.getSelectedItem().equals("BooleanValue")) {
			float prob = Float.parseFloat(textField3.getText());
			CASTTime time = bm.getCASTTimeInMonitor();
			String val = textField2.getText();
			try {
				BooleanValue sv = new BooleanValue(prob, time, Boolean.parseBoolean(val));
			ProxyConstructor.addFeatureValueToFeature(curFeature, sv);
			updateFeatureValuesFrame();
			}
			catch (NumberFormatException ex) {
				log("sorry, wrong format");
			}
		}
		if (comboBox1.getSelectedItem().equals("AddressValue")) {
			float prob = Float.parseFloat(textField3.getText());
			CASTTime time = bm.getCASTTimeInMonitor();
			String val = textField2.getText();
			AddressValue sv = new AddressValue(prob, time, val);
			ProxyConstructor.addFeatureValueToFeature(curFeature, sv);
			updateFeatureValuesFrame();
		}
		if (comboBox1.getSelectedItem().equals("UnknownValue")) {
			float prob = Float.parseFloat(textField3.getText());
			CASTTime time = bm.getCASTTimeInMonitor();
			UnknownValue sv = new UnknownValue(prob, time);
			ProxyConstructor.addFeatureValueToFeature(curFeature, sv);
			updateFeatureValuesFrame();
		}
		
		panel5.setVisible(false);
		setSize(new Dimension(450, 300 + (30 * curFeature.alternativeValues.length)));
		setPreferredSize(new Dimension(450, 300  + (25 * curFeature.alternativeValues.length)));
	}
	
	
	private void updateFeatureValuesFrame() {
		log("WOOHOOO update feature values frame!!!!!!!");
		log("nb feat values: " + curFeature.alternativeValues.length);

		if (table1.getModel().getRowCount() < curFeature.alternativeValues.length) {
			
			log("Have to enlarge the panel and the table...");
			
			scrollPane1.setMinimumSize(new Dimension(50, 30 + (23* curFeature.alternativeValues.length)));
			
			table1.setModel(new DefaultTableModel(
					new Object[curFeature.alternativeValues.length][3] ,
					new String[] {
						"Entry", "Type", "Probability"
					}
				) {
					Class[] columnTypes = new Class[] {
						String.class, String.class, Float.class
					};
					boolean[] columnEditable = new boolean[] {
						false, false, false
					};
					@Override
					public Class<?> getColumnClass(int columnIndex) {
						return columnTypes[columnIndex];
					}
					@Override
					public boolean isCellEditable(int rowIndex, int columnIndex) {
						return columnEditable[columnIndex];
					}
				});
			
		}
			for (int i = 0 ; i <curFeature.alternativeValues.length ; i++) {
				String entry = FeatureValueUtils.toString(curFeature.alternativeValues[i]);
				float prob = curFeature.alternativeValues[i].independentProb;
				table1.getModel().setValueAt(entry, i, 0);
				table1.getModel().setValueAt(curFeature.alternativeValues[i].getClass().getSimpleName(), i, 1);
				table1.getModel().setValueAt(new Float(prob), i, 2);
			}
	}

	private void initComponents() {
		// JFormDesigner - Component initialization - DO NOT MODIFY  //GEN-BEGIN:initComponents
		// Generated using JFormDesigner Evaluation license - Pierre Lison
		dialogPane = new JPanel();
		panel1 = new JPanel();
		label1 = new JLabel();
		textField1 = new JTextField();
		label2 = new JLabel();
		panel2 = new JPanel();
		scrollPane1 = new JScrollPane();
		table1 = new JTable();
		button1 = new JButton();
		panel4 = new JPanel();
		label4 = new JLabel();
		label5 = new JLabel();
		panel6 = new JPanel();
		label10 = new JLabel();
		label11 = new JLabel();
		panel5 = new JPanel();
		label6 = new JLabel();
		comboBox1 = new JComboBox();
		label7 = new JLabel();
		textField2 = new JTextField();
		label8 = new JLabel();
		panel7 = new JPanel();
		slider1 = new JSlider();
		textField3 = new JTextField();
		button3 = new JButton();
		panel8 = new JPanel();
		button2 = new JButton();
		button4 = new JButton();
		buttonBar = new JPanel();
		okButton = new JButton();
		cancelButton = new JButton();

		//======== this ========
		setTitle("Insert new Feature in Proxy");
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

			//======== panel1 ========
			{
				panel1.setLayout(new GridBagLayout());
				((GridBagLayout)panel1.getLayout()).columnWidths = new int[] {121, 55, 166, 0};
				((GridBagLayout)panel1.getLayout()).rowHeights = new int[] {0, 30, 0};
				((GridBagLayout)panel1.getLayout()).columnWeights = new double[] {0.0, 1.0, 0.0, 1.0E-4};
				((GridBagLayout)panel1.getLayout()).rowWeights = new double[] {0.0, 0.0, 1.0E-4};

				//---- label1 ----
				label1.setText("Feature label: ");
				panel1.add(label1, new GridBagConstraints(0, 0, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 5), 0, 0));

				//---- textField1 ----
				textField1.setText("dgdfgsdf");
				textField1.setColumns(12);
				panel1.add(textField1, new GridBagConstraints(2, 0, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 0), 0, 0));

				//---- label2 ----
				label2.setText("Alternative feature values:");
				panel1.add(label2, new GridBagConstraints(0, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 0, 5), 0, 0));
			}
			dialogPane.add(panel1, BorderLayout.NORTH);

			//======== panel2 ========
			{
				panel2.setLayout(new GridBagLayout());
				((GridBagLayout)panel2.getLayout()).columnWidths = new int[] {0, 25, 208, 0};
				((GridBagLayout)panel2.getLayout()).rowHeights = new int[] {35, 0, 0, 0, 0, 0};
				((GridBagLayout)panel2.getLayout()).columnWeights = new double[] {0.0, 1.0, 0.0, 1.0E-4};
				((GridBagLayout)panel2.getLayout()).rowWeights = new double[] {1.0, 0.0, 0.0, 0.0, 0.0, 1.0E-4};

				//======== scrollPane1 ========
				{
					scrollPane1.setMinimumSize(new Dimension(50, 45));

					//---- table1 ----
					table1.setModel(new DefaultTableModel(
						new Object[][] {
							{" (No feature values)", " ", null},
						},
						new String[] {
							"Entry", "Type", "Probability"
						}
					) {
						Class[] columnTypes = new Class[] {
							String.class, String.class, Float.class
						};
						boolean[] columnEditable = new boolean[] {
							false, false, false
						};
						@Override
						public Class<?> getColumnClass(int columnIndex) {
							return columnTypes[columnIndex];
						}
						@Override
						public boolean isCellEditable(int rowIndex, int columnIndex) {
							return columnEditable[columnIndex];
						}
					});
					{
						TableColumnModel cm = table1.getColumnModel();
						cm.getColumn(1).setResizable(false);
						cm.getColumn(1).setMinWidth(100);
						cm.getColumn(1).setMaxWidth(200);
						cm.getColumn(1).setPreferredWidth(150);
						cm.getColumn(1).setCellEditor(new DefaultCellEditor(
							new JComboBox(new DefaultComboBoxModel(new String[] {
								" ",
								"StringValue",
								"IntegerValue",
								"FloatValue",
								"BooleanValue",
								"AddressValue",
								"UnknownValue"
							}))));
						cm.getColumn(2).setResizable(false);
						cm.getColumn(2).setMinWidth(60);
						cm.getColumn(2).setMaxWidth(100);
						cm.getColumn(2).setPreferredWidth(80);
					}
					table1.setBorder(new CompoundBorder(
						new BevelBorder(BevelBorder.LOWERED),
						new EmptyBorder(30, 30, 30, 30)));
					table1.setBackground(SystemColor.window);
					table1.setGridColor(Color.black);
					table1.setAutoResizeMode(JTable.AUTO_RESIZE_ALL_COLUMNS);
					table1.setIntercellSpacing(new Dimension(0, 0));
					table1.setRowHeight(25);
					scrollPane1.setViewportView(table1);
				}
				panel2.add(scrollPane1, new GridBagConstraints(0, 0, 3, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.HORIZONTAL,
					new Insets(5, 20, 10, 20), 0, 0));

				//---- button1 ----
				button1.setText("Add new feature value");
				button1.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						button1ActionPerformed(e);
						button1ActionPerformed(e);
					}
				});
				panel2.add(button1, new GridBagConstraints(0, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 20, 5, 5), 0, 0));

				//======== panel4 ========
				{
					panel4.setLayout(new BorderLayout());

					//---- label4 ----
					label4.setText("Total number of alternative feature values:  ");
					label4.setFont(new Font("Lucida Grande", Font.PLAIN, 10));
					panel4.add(label4, BorderLayout.WEST);

					//---- label5 ----
					label5.setText("0");
					label5.setFont(new Font("Lucida Grande", Font.BOLD, 10));
					panel4.add(label5, BorderLayout.EAST);
				}
				panel2.add(panel4, new GridBagConstraints(2, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.WEST, GridBagConstraints.VERTICAL,
					new Insets(0, 0, 5, 20), 0, 0));

				//======== panel6 ========
				{
					panel6.setLayout(new BorderLayout());

					//---- label10 ----
					label10.setText("       ");
					panel6.add(label10, BorderLayout.EAST);

					//---- label11 ----
					label11.setText("      ");
					panel6.add(label11, BorderLayout.WEST);

					//======== panel5 ========
					{
						panel5.setBorder(new CompoundBorder(
							new EtchedBorder(EtchedBorder.RAISED),
							new EmptyBorder(8, 8, 8, 8)));
						panel5.setLayout(new GridBagLayout());
						((GridBagLayout)panel5.getLayout()).columnWidths = new int[] {0, 25, 173, 25, 58, 121, 0, 20, 0};
						((GridBagLayout)panel5.getLayout()).rowHeights = new int[] {0, 0, 0, 29, 0, 0};
						((GridBagLayout)panel5.getLayout()).columnWeights = new double[] {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0E-4};
						((GridBagLayout)panel5.getLayout()).rowWeights = new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-4};

						//---- label6 ----
						label6.setText("Feature value type:  ");
						panel5.add(label6, new GridBagConstraints(2, 0, 1, 1, 0.0, 0.0,
							GridBagConstraints.CENTER, GridBagConstraints.BOTH,
							new Insets(0, 0, 5, 5), 0, 0));

						//---- comboBox1 ----
						comboBox1.setComponentPopupMenu(null);
						comboBox1.setModel(new DefaultComboBoxModel(new String[] {
							"StringValue",
							"IntegerValue",
							"FloatValue",
							"BooleanValue",
							"AddressValue",
							"UnknownValue"
						}));
						panel5.add(comboBox1, new GridBagConstraints(5, 0, 1, 1, 0.0, 0.0,
							GridBagConstraints.CENTER, GridBagConstraints.BOTH,
							new Insets(0, 0, 5, 5), 0, 0));

						//---- label7 ----
						label7.setText("Feature value:  ");
						panel5.add(label7, new GridBagConstraints(2, 1, 1, 1, 0.0, 0.0,
							GridBagConstraints.CENTER, GridBagConstraints.BOTH,
							new Insets(0, 0, 5, 5), 0, 0));
						panel5.add(textField2, new GridBagConstraints(5, 1, 1, 1, 0.0, 0.0,
							GridBagConstraints.CENTER, GridBagConstraints.BOTH,
							new Insets(0, 0, 5, 5), 0, 0));

						//---- label8 ----
						label8.setText("Probability:  ");
						panel5.add(label8, new GridBagConstraints(2, 2, 1, 1, 0.0, 0.0,
							GridBagConstraints.CENTER, GridBagConstraints.BOTH,
							new Insets(0, 0, 5, 5), 0, 0));

						//======== panel7 ========
						{
							panel7.setLayout(new BorderLayout());

							//---- slider1 ----
							slider1.setMaximum(1);
							panel7.add(slider1, BorderLayout.CENTER);

							//---- textField3 ----
							textField3.setText("0.0");
							panel7.add(textField3, BorderLayout.EAST);
						}
						panel5.add(panel7, new GridBagConstraints(5, 2, 1, 1, 0.0, 0.0,
							GridBagConstraints.CENTER, GridBagConstraints.BOTH,
							new Insets(0, 0, 5, 5), 0, 0));

						//---- button3 ----
						button3.setText("Reset");
						panel5.add(button3, new GridBagConstraints(2, 3, 1, 1, 0.0, 0.0,
							GridBagConstraints.WEST, GridBagConstraints.VERTICAL,
							new Insets(0, 0, 5, 5), 0, 0));

						//======== panel8 ========
						{
							panel8.setLayout(new BorderLayout());

							//---- button2 ----
							button2.setText("Add");
							button2.addActionListener(new ActionListener() {
								public void actionPerformed(ActionEvent e) {
									button2ActionPerformed(e);
								}
							});
							panel8.add(button2, BorderLayout.CENTER);

							//---- button4 ----
							button4.setText("Cancel");
							button4.addActionListener(new ActionListener() {
								public void actionPerformed(ActionEvent e) {
									button4ActionPerformed(e);
								}
							});
							panel8.add(button4, BorderLayout.EAST);
						}
						panel5.add(panel8, new GridBagConstraints(5, 3, 1, 1, 0.0, 0.0,
							GridBagConstraints.CENTER, GridBagConstraints.BOTH,
							new Insets(0, 0, 5, 5), 0, 0));
					}
					panel6.add(panel5, BorderLayout.CENTER);
				}
				panel2.add(panel6, new GridBagConstraints(0, 3, 3, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 0), 0, 0));
			}
			dialogPane.add(panel2, BorderLayout.CENTER);

			//======== buttonBar ========
			{
				buttonBar.setBorder(new EmptyBorder(12, 0, 0, 0));
				buttonBar.setLayout(new GridBagLayout());
				((GridBagLayout)buttonBar.getLayout()).columnWidths = new int[] {411, 85, 80};
				((GridBagLayout)buttonBar.getLayout()).columnWeights = new double[] {1.0, 0.0, 0.0};

				//---- okButton ----
				okButton.setText("Insert Feature");
				buttonBar.add(okButton, new GridBagConstraints(1, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 0, 5), 0, 0));

				//---- cancelButton ----
				cancelButton.setText("Cancel");
				buttonBar.add(cancelButton, new GridBagConstraints(2, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.EAST, GridBagConstraints.VERTICAL,
					new Insets(0, 0, 0, 0), 0, 0));
			}
			dialogPane.add(buttonBar, BorderLayout.SOUTH);
		}
		contentPane.add(dialogPane, BorderLayout.CENTER);
		pack();
		setLocationRelativeTo(getOwner());
		// JFormDesigner - End of component initialization  //GEN-END:initComponents
		
		setSize(new Dimension(450, 300));
		setPreferredSize(new Dimension(450, 300));
		panel5.setVisible(false);
	}

	// JFormDesigner - Variables declaration - DO NOT MODIFY  //GEN-BEGIN:variables
	// Generated using JFormDesigner Evaluation license - Pierre Lison
	private JPanel dialogPane;
	private JPanel panel1;
	private JLabel label1;
	private JTextField textField1;
	private JLabel label2;
	private JPanel panel2;
	private JScrollPane scrollPane1;
	private JTable table1;
	private JButton button1;
	private JPanel panel4;
	private JLabel label4;
	private JLabel label5;
	private JPanel panel6;
	private JLabel label10;
	private JLabel label11;
	private JPanel panel5;
	private JLabel label6;
	private JComboBox comboBox1;
	private JLabel label7;
	private JTextField textField2;
	private JLabel label8;
	private JPanel panel7;
	private JSlider slider1;
	private JTextField textField3;
	private JButton button3;
	private JPanel panel8;
	private JButton button2;
	private JButton button4;
	private JPanel buttonBar;
	private JButton okButton;
	private JButton cancelButton;
	// JFormDesigner - End of variables declaration  //GEN-END:variables


	private void log(String s) {
		if (LOGGING) {
			System.out.println("[FeatureInfoGUI] " + s);
		}
	}
}


final class FeatureValueCellRenderer 
       extends DefaultTableCellRenderer {
	
  public Component getTableCellRendererComponent(JTable table,
                                                 Object value,
                                                 boolean isSelected,
                                                 boolean hasFocus,
                                                 int row,
                                                 int column) {
    Component c = 
      super.getTableCellRendererComponent(table, value,
                                          isSelected, hasFocus,
                                          row, column);

    System.out.println("ROW: " + row);
    if (row == 1 ) {
       c.setFont(new Font("Lucida Grande", Font.BOLD, 13));
       // you may want to address isSelected here too
       c.setForeground(Color.WHITE);
       c.setBackground(Color.BLUE);
    }
    return c;
  }
  
}


