/*
 * Created by JFormDesigner on Mon Oct 26 15:31:29 CET 2009
 */

package binder.gui;

import java.awt.*;
import java.awt.event.*;
import java.beans.*;
import java.util.Vector;

import javax.swing.*;
import javax.swing.border.*;
import javax.swing.event.*;
import javax.swing.table.*;

import cast.cdl.CASTTime;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.featvalues.BooleanValue;
import binder.autogen.featvalues.FloatValue;
import binder.autogen.featvalues.IntegerValue;
import binder.autogen.featvalues.StringValue;
import binder.autogen.featvalues.UnknownValue;
import binder.components.Binder;
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

	ProxyInfoGUI proxyWindow;
	
	Proxy proxy;
	
	private enum OPERATION {INSERT, MODIFY, INSERT_FROM_MAINWINDOW };
	
	OPERATION optype;
	

	public FeatureInfoGUI(ProxyInfoGUI owner) {
		super(owner);
		proxyWindow = owner;
		bm = owner.bm;
		initComponents();

		curFeature = new Feature("", new FeatureValue[0]);
		((TitledBorder)((CompoundBorder)dialogPane.getBorder()).getOutsideBorder()).setTitle("");
		
		optype = OPERATION.INSERT;
	}
	
	

	public FeatureInfoGUI(BinderGUI owner, Proxy proxy) {
		super(owner);
		bm = owner.bm;
		initComponents();
		
		this.proxy = proxy;

		curFeature = new Feature("", new FeatureValue[0]);
		((TitledBorder)((CompoundBorder)dialogPane.getBorder()).getOutsideBorder()).setTitle("");
		
		optype = OPERATION.INSERT_FROM_MAINWINDOW;
	}
	
	
	public FeatureInfoGUI(ProxyInfoGUI owner, Feature feature) {
		super(owner);
		proxyWindow = owner;
		bm = owner.bm;
		initComponents();

		setTitle("Modify existing Feature in Proxy");
		okButton.setText("Modify Feature");
		curFeature = feature;
		
		textField1.setText(feature.featlabel);
		updateFeatureValuesFrame();
		
		((TitledBorder)((CompoundBorder)dialogPane.getBorder()).getOutsideBorder()).setTitle("");
		
		optype = OPERATION.MODIFY;
	}
	
	
	private void button1ActionPerformed(ActionEvent e) {
		if (!panel5.isVisible()) {
			setSize(new Dimension(520, 430  + (25 * curFeature.alternativeValues.length)));
			setPreferredSize(new Dimension(520, 430  + (25 * curFeature.alternativeValues.length)));
			panel5.setVisible(true);
			okButton.setEnabled(false);
			cancelButton.setEnabled(false);
		}
		else {
			button1.setEnabled(false);
		}
		comboBox1.setSelectedItem("StringValue");
		textField2.setText("");
		textField3.setText("0.75");
		button3.setVisible(false);
		button2.setText("Add");
	}

	private void button4ActionPerformed(ActionEvent e) {
		panel5.setVisible(false);
		setSize(new Dimension(520, 280  + (25 * curFeature.alternativeValues.length)));
		setPreferredSize(new Dimension(520, 280  + (25 * curFeature.alternativeValues.length)));
		button1.setEnabled(true);
		okButton.setEnabled(true);
		cancelButton.setEnabled(true);
	}

	private void button2ActionPerformed(ActionEvent e) {
		
		FeatureValue featval = constructFeatureValue();
		
		if (button2.getText().equals("Add") && featval != null) {
			addFeatureValue(featval);
		}
		else if (button2.getText().equals("Modify") && table1.getSelectedRow() >= 0 && 
				table1.getSelectedRow() < curFeature.alternativeValues.length && featval != null) {
			updateFeatureValue(featval, table1.getSelectedRow());
		}
	}
	
	
	private FeatureValue constructFeatureValue() {

		FeatureValue featval = null;
		if (comboBox1.getSelectedItem().equals("StringValue")) {
			float prob = Float.parseFloat(textField3.getText());
			CASTTime time = bm.getCASTTimeInMonitor();
			String val = textField2.getText();
			featval = new StringValue(prob, time, val);
			ProxyConstructor.setTimeStamp(featval, bm.getCASTTimeInMonitor());
		}

		if (comboBox1.getSelectedItem().equals("IntegerValue")) {
			float prob = Float.parseFloat(textField3.getText());
			CASTTime time = bm.getCASTTimeInMonitor();
			String val = textField2.getText();
			try {
				featval = new IntegerValue(prob, time, Integer.parseInt(val));
				ProxyConstructor.setTimeStamp(featval, bm.getCASTTimeInMonitor());
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
				featval = new FloatValue(prob, time, Float.parseFloat(val));
				ProxyConstructor.setTimeStamp(featval, bm.getCASTTimeInMonitor());
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
				featval = new BooleanValue(prob, time, Boolean.parseBoolean(val));
				ProxyConstructor.setTimeStamp(featval, bm.getCASTTimeInMonitor());
			}
			catch (NumberFormatException ex) {
				log("sorry, wrong format");
			}
		}
		if (comboBox1.getSelectedItem().equals("AddressValue")) {
			float prob = Float.parseFloat(textField3.getText());
			CASTTime time = bm.getCASTTimeInMonitor();
			String val = textField2.getText();
			featval = new AddressValue(prob, time, val);
			ProxyConstructor.setTimeStamp(featval, bm.getCASTTimeInMonitor());
			updateFeatureValuesFrame();
		}
		if (comboBox1.getSelectedItem().equals("UnknownValue")) {
			float prob = Float.parseFloat(textField3.getText());
			CASTTime time = bm.getCASTTimeInMonitor();
			featval = new UnknownValue(prob, time);
			ProxyConstructor.setTimeStamp(featval, bm.getCASTTimeInMonitor());
		}
		
		return featval;
	}
	
	public void addFeatureValue(FeatureValue featval) {
		ProxyConstructor.addFeatureValueToFeature(curFeature, featval);

		updateFeatureValuesFrame();
		
		panel5.setVisible(false);
		setSize(new Dimension(520, 280 + (30 * curFeature.alternativeValues.length)));
		setPreferredSize(new Dimension(520, 280  + (25 * curFeature.alternativeValues.length)));
		button1.setEnabled(true);
		okButton.setEnabled(true);
		cancelButton.setEnabled(true);
	}

	
	public void updateFeatureValue(FeatureValue featval, int index) {
		
		if (index >= 0 && index < curFeature.alternativeValues.length) {
			curFeature.alternativeValues[index] = featval;
		}
		updateFeatureValuesFrame();
		
		panel5.setVisible(false);
		setSize(new Dimension(520, 280 + (30 * curFeature.alternativeValues.length)));
		setPreferredSize(new Dimension(520, 280  + (25 * curFeature.alternativeValues.length)));
		button1.setEnabled(true);
		okButton.setEnabled(true);
		cancelButton.setEnabled(true);
	}
	
	
	private void slider1StateChanged(ChangeEvent e) {
		JSlider source = (JSlider)e.getSource();
		float fps = (source.getValue()/100.0f);
		double roundedProb = Math.round(fps*100.0) / 100.0;
		textField3.setText(""+roundedProb);
	}

	
	private void button3ActionPerformed(ActionEvent e) {
		
		if (curFeature.alternativeValues.length > 0) {
			Vector<FeatureValue> featvals = new Vector<FeatureValue>();
			for (int i = 0 ; i < curFeature.alternativeValues.length ; i++) {
				featvals.add(curFeature.alternativeValues[i]);
			}

			for (int i = 0 ; i < table1.getSelectedRows().length ; i++) {
				FeatureValue toBeRemoved = curFeature.alternativeValues[table1.getSelectedRows()[i]];
				featvals.remove(toBeRemoved);
			}

			curFeature.alternativeValues = new FeatureValue[featvals.size()];
			curFeature.alternativeValues = featvals.toArray(curFeature.alternativeValues);
			
			updateFeatureValuesFrame();
		}
	}
	

	private void cancelButtonActionPerformed(ActionEvent e) {
		dispose();
	}

	private void okButtonActionPerformed(ActionEvent e) {
		curFeature.featlabel = textField1.getText();
		if (verifyFeatureIsWellFormed()) {
			if (optype.equals(OPERATION.INSERT)) {
				proxyWindow.addFeatureToProxy(curFeature);
			}
			else if (optype.equals(OPERATION.MODIFY)){
				proxyWindow.updateFeaturesFrame();
			}
			else if (optype.equals(OPERATION.INSERT_FROM_MAINWINDOW)) {
				ProxyConstructor.addFeatureToProxy(proxy, curFeature);
				try {
					bm.overwriteWorkingMemory(proxy.entityID, Binder.BINDER_SA, proxy);
				}
				catch (Exception ex) {
					ex.printStackTrace();
				}
			}
			dispose();
		}
	}


	private void table1MouseClicked(MouseEvent e) {
		if (table1.isEnabled()) {
		if (!panel5.isVisible()) {
			setSize(new Dimension(520, 430  + (30 * curFeature.alternativeValues.length)));
			setPreferredSize(new Dimension(520, 430  + (25 * curFeature.alternativeValues.length)));
			panel5.setVisible(true);
			okButton.setEnabled(false);
			cancelButton.setEnabled(false);
		}

		int row = table1.rowAtPoint(e.getPoint());
		if (row >= 0 && row < curFeature.alternativeValues.length) {
			FeatureValue featval = curFeature.alternativeValues[row];
			comboBox1.setSelectedItem(featval.getClass().getSimpleName());
			textField2.setText(FeatureValueUtils.toString(featval));
			textField3.setText(""+featval.independentProb);
			slider1.setValue((int)(featval.independentProb * 100));
		}
		button3.setVisible(true);
		button2.setText("Modify");
		}
	}

	private void comboBox1ItemStateChanged(ItemEvent e) {
		if (comboBox1.getSelectedItem().equals("UnknownValue")) {
			textField2.setEnabled(false);
		}
		else {
			textField2.setEnabled(true);
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
		panel8 = new JPanel();
		button2 = new JButton();
		button3 = new JButton();
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
				((GridBagLayout)panel1.getLayout()).columnWidths = new int[] {131, 0, 74, 88, 0};
				((GridBagLayout)panel1.getLayout()).rowHeights = new int[] {0, 30, 0};
				((GridBagLayout)panel1.getLayout()).columnWeights = new double[] {0.0, 0.0, 1.0, 0.0, 1.0E-4};
				((GridBagLayout)panel1.getLayout()).rowWeights = new double[] {0.0, 0.0, 1.0E-4};

				//---- label1 ----
				label1.setText("Feature label: ");
				panel1.add(label1, new GridBagConstraints(0, 0, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 5), 0, 0));

				//---- textField1 ----
				textField1.setColumns(12);
				panel1.add(textField1, new GridBagConstraints(2, 0, 1, 1, 0.0, 0.0,
					GridBagConstraints.WEST, GridBagConstraints.VERTICAL,
					new Insets(0, 0, 5, 5), 0, 0));

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
				((GridBagLayout)panel2.getLayout()).columnWidths = new int[] {155, 56, 208, 0};
				((GridBagLayout)panel2.getLayout()).rowHeights = new int[] {50, 0, 0, 0, 0};
				((GridBagLayout)panel2.getLayout()).columnWeights = new double[] {0.0, 1.0, 0.0, 1.0E-4};
				((GridBagLayout)panel2.getLayout()).rowWeights = new double[] {1.0, 0.0, 0.0, 0.0, 1.0E-4};

				//======== scrollPane1 ========
				{
					scrollPane1.setMinimumSize(new Dimension(50, 55));

					//---- table1 ----
					table1.setModel(new DefaultTableModel(
						new Object[][] {
							{" (None)", " ", null},
						},
						new String[] {
							"Entry", "Type", "Probability"
						}
					) {
						Class[] columnTypes = new Class[] {
							String.class, String.class, Float.class
						};
						@Override
						public Class<?> getColumnClass(int columnIndex) {
							return columnTypes[columnIndex];
						}
					});
					{
						TableColumnModel cm = table1.getColumnModel();
						cm.getColumn(0).setMinWidth(160);
						cm.getColumn(0).setMaxWidth(500);
						cm.getColumn(0).setPreferredWidth(200);
						cm.getColumn(1).setMinWidth(80);
						cm.getColumn(1).setMaxWidth(160);
						cm.getColumn(1).setPreferredWidth(120);
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
						cm.getColumn(2).setMinWidth(40);
						cm.getColumn(2).setMaxWidth(70);
						cm.getColumn(2).setPreferredWidth(80);
					}
					table1.setBorder(new CompoundBorder(
						new BevelBorder(BevelBorder.LOWERED),
						new EmptyBorder(30, 30, 30, 30)));
					table1.setAutoResizeMode(JTable.AUTO_RESIZE_ALL_COLUMNS);
					table1.setIntercellSpacing(new Dimension(0, 0));
					table1.setRowHeight(25);
					table1.setRowSelectionAllowed(false);
					table1.setEnabled(false);
					table1.setMinimumSize(new Dimension(175, 25));
					table1.setMaximumSize(new Dimension(2147483647, 25));
					table1.addMouseListener(new MouseAdapter() {
						@Override
						public void mouseClicked(MouseEvent e) {
							table1MouseClicked(e);
						}
					});
					scrollPane1.setViewportView(table1);
				}
				panel2.add(scrollPane1, new GridBagConstraints(0, 0, 3, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(5, 20, 10, 20), 0, 0));

				//---- button1 ----
				button1.setText("Add new value");
				button1.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						button1ActionPerformed(e);
					}
				});
				panel2.add(button1, new GridBagConstraints(0, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 5), 0, 0));

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
						((GridBagLayout)panel5.getLayout()).columnWidths = new int[] {0, 25, 144, 25, 58, 188, 0, 0, 20, 0};
						((GridBagLayout)panel5.getLayout()).rowHeights = new int[] {0, 0, 0, 29, 0, 0};
						((GridBagLayout)panel5.getLayout()).columnWeights = new double[] {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0E-4};
						((GridBagLayout)panel5.getLayout()).rowWeights = new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 1.0E-4};

						//---- label6 ----
						label6.setText("Feature value type: ");
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
							slider1.setValue(80);
							slider1.addChangeListener(new ChangeListener() {
								public void stateChanged(ChangeEvent e) {
									slider1StateChanged(e);
								}
							});
							panel7.add(slider1, BorderLayout.CENTER);

							//---- textField3 ----
							textField3.setText("0.80");
							textField3.setColumns(3);
							panel7.add(textField3, BorderLayout.EAST);
						}
						panel5.add(panel7, new GridBagConstraints(5, 2, 1, 1, 0.0, 0.0,
							GridBagConstraints.CENTER, GridBagConstraints.BOTH,
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
							panel8.add(button2, BorderLayout.WEST);

							//---- button3 ----
							button3.setText("Delete");
							button3.addActionListener(new ActionListener() {
								public void actionPerformed(ActionEvent e) {
									button3ActionPerformed(e);
								}
							});
							panel8.add(button3, BorderLayout.CENTER);

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
					new Insets(0, 0, 0, 0), 0, 0));
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
				okButton.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						okButtonActionPerformed(e);
					}
				});
				buttonBar.add(okButton, new GridBagConstraints(1, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 0, 5), 0, 0));

				//---- cancelButton ----
				cancelButton.setText("Cancel");
				cancelButton.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						cancelButtonActionPerformed(e);
					}
				});
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

		setSize(new Dimension(520, 280));
		setPreferredSize(new Dimension(520, 280));
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
	private JPanel panel8;
	private JButton button2;
	private JButton button3;
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


	private void printWarningMessage(String mess) {
		((TitledBorder)((CompoundBorder)dialogPane.getBorder()).getOutsideBorder()).setTitle(mess);
		pack();
		setSize(new Dimension (getSize().width, getSize().height +1));
		setPreferredSize(getPreferredSize());
	}
	
	private boolean verifyFeatureIsWellFormed() {
		if (curFeature.featlabel.equals("")) {
			printWarningMessage("Failed: no feature label specified");	
			return false;
		}
		else if (curFeature.alternativeValues.length == 0) {
			printWarningMessage("Failed: no feature value specified");
			return false;
		}
		return true;
	}


	private DefaultTableModel createTableModelForSize (int size) {
		return new DefaultTableModel(
				new Object[size][3] ,
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
		};
	}

	private void updateFeatureValuesFrame() {

		if (curFeature.alternativeValues.length == 0) {
			table1.setModel(createTableModelForSize(1));
			table1.getModel().setValueAt(" (None) ", 0, 0);
			table1.setRowSelectionAllowed(false);
			table1.setEnabled(false);
		}

		else {

			table1.setRowSelectionAllowed(true);
			table1.setEnabled(true);

			scrollPane1.setMinimumSize(new Dimension(50, 30 + (23* curFeature.alternativeValues.length)));

			table1.setModel(createTableModelForSize(curFeature.alternativeValues.length));

		}
		for (int i = 0 ; i <curFeature.alternativeValues.length ; i++) {
			String entry = FeatureValueUtils.toString(curFeature.alternativeValues[i]);
			float prob = curFeature.alternativeValues[i].independentProb;
			table1.getModel().setValueAt(entry, i, 0);
			table1.getModel().setValueAt(curFeature.alternativeValues[i].getClass().getSimpleName(), i, 1);
			table1.getModel().setValueAt(new Float(prob), i, 2);
		}

		label5.setText(""+curFeature.alternativeValues.length);
	}
}


