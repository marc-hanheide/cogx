/*
 * Created by JFormDesigner on Mon Oct 26 11:17:16 CET 2009
 */

package binder.gui;

import java.awt.*;
import java.awt.event.*;
import java.util.Vector;

import javax.swing.*;
import javax.swing.border.*;
import javax.swing.event.*;
import javax.swing.table.*;

import cast.cdl.WorkingMemoryPointer;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.specialentities.PhantomProxy;
import binder.autogen.specialentities.RelationProxy;
import binder.components.Binder;
import binder.components.BinderMonitor;
import binder.constructors.ProxyConstructor;
import binder.utils.FeatureValueUtils;

/**
 * @author Pierre Lison
 */
public class ProxyInfoGUI extends JDialog {
	
	BinderMonitor bm;
	
	Proxy curProxy;
		
	public boolean LOGGING = true;

	
	private void init() {
		initComponents();		
		((TitledBorder)((CompoundBorder)dialogPane.getBorder()).getOutsideBorder()).setTitle("");	
	}
	
	public ProxyInfoGUI(BinderGUI owner) {
		super(owner);
		bm = owner.bm;
		init();
		
		WorkingMemoryPointer origin = ProxyConstructor.createWorkingMemoryPointer(textField3.getText(), "", "");
		textField2.setText(bm.newDataID());
		curProxy = ProxyConstructor.createNewProxy(origin, textField2.getText(), 0.0f);
		ProxyConstructor.setTimeStamp(curProxy, bm.getCASTTimeInMonitor());
	}
	
	
	

	public ProxyInfoGUI(BinderGUI owner, Proxy proxy) {
		super(owner);
		bm = owner.bm;
		init();
		curProxy = proxy;
		setTitle("Modify existing Proxy");
		comboBox1.setSelectedItem(proxy.getClass().getSimpleName());
		textField2.setText(proxy.entityID);
		textField3.setText(proxy.origin.address.subarchitecture);
		textField1.setText("" + proxy.probExists);
		slider1.setValue((int)(proxy.probExists * 100));
		updateFeaturesFrame();
		if (proxy.features.length > 0) {
			table1.setComponentPopupMenu(popupMenu1);
			
		}
		okButton.setText("Modify");
	}


	private void button1ActionPerformed(ActionEvent e) {
		FeatureInfoGUI featureInfo = new FeatureInfoGUI(this);
		featureInfo.setVisible(true);
	}

	private void slider1StateChanged(ChangeEvent e) {
		JSlider source = (JSlider)e.getSource();
		float fps = (source.getValue()/100.0f);
		double roundedProb = Math.round(fps*100.0) / 100.0;
		textField1.setText(""+roundedProb);
	}

	private void cancelButtonActionPerformed(ActionEvent e) {
		dispose();
	}

	
	
	private void okButtonActionPerformed(ActionEvent e) {
		if (verifyProxyWellFormed()) {
			try {
			ProxyConstructor.setTimeStamp(curProxy, bm.getCASTTimeInMonitor());
			curProxy.entityID = textField2.getText();
			curProxy.origin.address.subarchitecture = textField3.getText();
			curProxy.probExists = Float.parseFloat(textField1.getText());
	
			if (okButton.getText().equals("Insert")) {
			bm.addToWorkingMemory(curProxy.entityID, Binder.BINDER_SA, curProxy);
			}
			else if (okButton.getText().equals("Modify")) {
				bm.overwriteWorkingMemory(curProxy.entityID, Binder.BINDER_SA, curProxy);
			}
			}
			catch (Exception ex) {
				ex.printStackTrace();
			}
			dispose();
		}
	}

	private void menuItem1ActionPerformed(ActionEvent e) {
		if (curProxy.features.length > 0) {
			Vector<Feature> featvals = new Vector<Feature>();
			for (int i = 0 ; i < curProxy.features.length ; i++) {
				featvals.add(curProxy.features[i]);
			}

			for (int i = 0 ; i < table1.getSelectedRows().length ; i++) {
				Feature toBeRemoved = curProxy.features[table1.getSelectedRows()[i]];
				featvals.remove(toBeRemoved);
			}

			curProxy.features = new Feature[featvals.size()];
			curProxy.features = featvals.toArray(curProxy.features);
			updateFeaturesFrame();
		}
	}

	private void comboBox1ItemStateChanged(ItemEvent e) {
		
		if (comboBox1.getSelectedItem().equals("PhantomProxy")) {
			PhantomProxy phantom = ProxyConstructor.createNewPhantomProxy
				(curProxy.origin, curProxy.entityID, curProxy.probExists);
			curProxy = phantom;
		}
		else if (comboBox1.getSelectedItem().equals("RelationProxy")) {
			RelationProxy phantom = ProxyConstructor.createNewRelationProxy
				(curProxy.origin, curProxy.entityID, curProxy.probExists, new AddressValue[0], new AddressValue[0]);
			curProxy = phantom;
		}
		
	}

	private void menuItem2ActionPerformed(ActionEvent e) {
		FeatureInfoGUI featureInfo = new FeatureInfoGUI(this, curProxy.features[table1.getSelectedRow()]);
		featureInfo.setVisible(true);
	}

	private void initComponents() {
		// JFormDesigner - Component initialization - DO NOT MODIFY  //GEN-BEGIN:initComponents
		// Generated using JFormDesigner Evaluation license - Pierre Lison
		dialogPane = new JPanel();
		panel2 = new JPanel();
		label1 = new JLabel();
		comboBox1 = new JComboBox();
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
		scrollPane1 = new JScrollPane();
		table1 = new JTable();
		button1 = new JButton();
		panel6 = new JPanel();
		label7 = new JLabel();
		label8 = new JLabel();
		buttonBar = new JPanel();
		okButton = new JButton();
		cancelButton = new JButton();
		popupMenu1 = new JPopupMenu();
		menuItem2 = new JMenuItem();
		menuItem1 = new JMenuItem();

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
				((GridBagLayout)panel2.getLayout()).rowHeights = new int[] {0, 0, 0, 0, 0, 0};
				((GridBagLayout)panel2.getLayout()).columnWeights = new double[] {1.0, 1.0, 1.0E-4};
				((GridBagLayout)panel2.getLayout()).rowWeights = new double[] {0.0, 1.0, 1.0, 1.0, 1.0, 1.0E-4};

				//---- label1 ----
				label1.setText("Proxy type:");
				panel2.add(label1, new GridBagConstraints(0, 0, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- comboBox1 ----
				comboBox1.setModel(new DefaultComboBoxModel(new String[] {
					"Proxy",
					"PhantomProxy",
					"RelationProxy"
				}));
				comboBox1.addItemListener(new ItemListener() {
					public void itemStateChanged(ItemEvent e) {
						comboBox1ItemStateChanged(e);
					}
				});
				panel2.add(comboBox1, new GridBagConstraints(1, 0, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- label3 ----
				label3.setText("Proxy identifier: ");
				panel2.add(label3, new GridBagConstraints(0, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- textField2 ----
				textField2.setText("ProxyX");
				textField2.setColumns(12);
				panel2.add(textField2, new GridBagConstraints(1, 1, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- label4 ----
				label4.setText("Originating subarchitecture: ");
				panel2.add(label4, new GridBagConstraints(0, 2, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- textField3 ----
				textField3.setText("vision");
				textField3.setColumns(12);
				panel2.add(textField3, new GridBagConstraints(1, 2, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- label5 ----
				label5.setText("Prob ( exists | obs ): ");
				panel2.add(label5, new GridBagConstraints(0, 3, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//======== panel1 ========
				{
					panel1.setLayout(new BorderLayout());

					//---- slider1 ----
					slider1.setValue(80);
					slider1.addChangeListener(new ChangeListener() {
						public void stateChanged(ChangeEvent e) {
							slider1StateChanged(e);
						}
					});
					panel1.add(slider1, BorderLayout.CENTER);

					//---- textField1 ----
					textField1.setText("0.80");
					panel1.add(textField1, BorderLayout.EAST);
				}
				panel2.add(panel1, new GridBagConstraints(1, 3, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 3, 0), 0, 0));

				//---- label12 ----
				label12.setText("     ");
				label12.setFont(new Font("Lucida Grande", Font.PLAIN, 8));
				panel2.add(label12, new GridBagConstraints(0, 4, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 0, 0), 0, 0));

				//---- label13 ----
				label13.setText("    ");
				label13.setFont(new Font("Lucida Grande", Font.PLAIN, 8));
				panel2.add(label13, new GridBagConstraints(1, 4, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 0, 0), 0, 0));
			}
			dialogPane.add(panel2, BorderLayout.NORTH);

			//======== panel9 ========
			{
				panel9.setLayout(new GridBagLayout());
				((GridBagLayout)panel9.getLayout()).columnWidths = new int[] {138, 0, 180, 0};
				((GridBagLayout)panel9.getLayout()).rowHeights = new int[] {0, 50, 0, 0, 0};
				((GridBagLayout)panel9.getLayout()).columnWeights = new double[] {0.0, 1.0, 0.0, 1.0E-4};
				((GridBagLayout)panel9.getLayout()).rowWeights = new double[] {0.0, 1.0, 0.0, 0.0, 1.0E-4};

				//---- label6 ----
				label6.setText("Included features: ");
				panel9.add(label6, new GridBagConstraints(0, 0, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 5), 0, 0));

				//======== scrollPane1 ========
				{
					scrollPane1.setEnabled(false);

					//---- table1 ----
					table1.setModel(new DefaultTableModel(
						new Object[][] {
							{" (None)", null},
						},
						new String[] {
							"Feature label", "Feature values"
						}
					) {
						boolean[] columnEditable = new boolean[] {
							false, false
						};
						@Override
						public boolean isCellEditable(int rowIndex, int columnIndex) {
							return columnEditable[columnIndex];
						}
					});
					{
						TableColumnModel cm = table1.getColumnModel();
						cm.getColumn(0).setResizable(false);
						cm.getColumn(0).setMinWidth(60);
						cm.getColumn(0).setMaxWidth(100);
						cm.getColumn(0).setPreferredWidth(80);
						cm.getColumn(1).setResizable(false);
						cm.getColumn(1).setMinWidth(160);
						cm.getColumn(1).setMaxWidth(500);
						cm.getColumn(1).setPreferredWidth(220);
					}
					table1.setRowHeight(25);
					table1.setRowSelectionAllowed(false);
					table1.setEnabled(false);
					table1.setBorder(new BevelBorder(BevelBorder.LOWERED));
					scrollPane1.setViewportView(table1);
				}
				panel9.add(scrollPane1, new GridBagConstraints(0, 1, 3, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 0), 0, 0));

				//---- button1 ----
				button1.setText("Add new feature");
				button1.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						button1ActionPerformed(e);
					}
				});
				panel9.add(button1, new GridBagConstraints(0, 2, 1, 1, 0.0, 0.0,
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 5, 5), 0, 0));

				//======== panel6 ========
				{
					panel6.setLayout(new BorderLayout());

					//---- label7 ----
					label7.setText("Total number of features:  ");
					label7.setIcon(null);
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
					GridBagConstraints.CENTER, GridBagConstraints.BOTH,
					new Insets(0, 0, 0, 0), 0, 0));
			}
			dialogPane.add(buttonBar, BorderLayout.SOUTH);
		}
		contentPane.add(dialogPane, BorderLayout.CENTER);
		pack();
		setLocationRelativeTo(getOwner());

		//======== popupMenu1 ========
		{

			//---- menuItem2 ----
			menuItem2.setText("Modify Feature");
			menuItem2.addActionListener(new ActionListener() {
				public void actionPerformed(ActionEvent e) {
					menuItem2ActionPerformed(e);
				}
			});
			popupMenu1.add(menuItem2);

			//---- menuItem1 ----
			menuItem1.setText("Delete Feature");
			menuItem1.addActionListener(new ActionListener() {
				public void actionPerformed(ActionEvent e) {
					menuItem1ActionPerformed(e);
				}
			});
			popupMenu1.add(menuItem1);
		}
		// JFormDesigner - End of component initialization  //GEN-END:initComponents
		
		setSize(new Dimension(480, 340));
		setPreferredSize(new Dimension(480, 340));
	}

	// JFormDesigner - Variables declaration - DO NOT MODIFY  //GEN-BEGIN:variables
	// Generated using JFormDesigner Evaluation license - Pierre Lison
	private JPanel dialogPane;
	private JPanel panel2;
	private JLabel label1;
	private JComboBox comboBox1;
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
	private JScrollPane scrollPane1;
	private JTable table1;
	private JButton button1;
	private JPanel panel6;
	private JLabel label7;
	private JLabel label8;
	private JPanel buttonBar;
	private JButton okButton;
	private JButton cancelButton;
	private JPopupMenu popupMenu1;
	private JMenuItem menuItem2;
	private JMenuItem menuItem1;
	// JFormDesigner - End of variables declaration  //GEN-END:variables

	
	public void addFeatureToProxy (Feature feat) {
		ProxyConstructor.addFeatureToProxy(curProxy, feat);
		updateFeaturesFrame();
		setSize(new Dimension(480, 340 + (30 * curProxy.features.length)));
		setPreferredSize(new Dimension(480, 340 + (30 * curProxy.features.length)));
		table1.setComponentPopupMenu(popupMenu1);
	}
	
	

	private DefaultTableModel createTableModelForSize (int size) {
		return new DefaultTableModel(
				new Object[size][2] ,
					new String[] {
						"Feature label", "Feature values"
					}
				) {
					boolean[] columnEditable = new boolean[] {
						false, false
					};
					@Override
					public boolean isCellEditable(int rowIndex, int columnIndex) {
						return columnEditable[columnIndex];
					}
				};
	}

	
	private void printWarningMessage(String mess) {
		((TitledBorder)((CompoundBorder)dialogPane.getBorder()).getOutsideBorder()).setTitle(mess);
		setSize(new Dimension (getSize().width, getSize().height +1));
		setPreferredSize(getPreferredSize());
		pack();
		dialogPane.setVisible(true);
		log(mess);
	}
	
	
	private boolean verifyProxyWellFormed() {
		
		if (curProxy.entityID.equals("")) {
			printWarningMessage("Unable to insert proxy: proxy identifier must be specified");
			return false;
		}
		else if (curProxy.origin.address.subarchitecture.equals("")) {
			printWarningMessage("Failed: originating subarchitecture must be specified");
			return false;
		}
		else if (bm.existsOnWorkingMemory(curProxy.entityID) && okButton.getText().equals("Insert"))  {
			printWarningMessage("Failed: proxy identifier already exists on WM");
			return false;
		}
		else if (!bm.existsOnWorkingMemory(curProxy.entityID) && okButton.getText().equals("Modify"))  {
			printWarningMessage("Failed: proxy does not exists on WM anymore");
			return false;
		}
		else {
			try {
				Float.parseFloat(textField1.getText());
			}
			catch (NumberFormatException e) {
				printWarningMessage("Failed: existence probability must be a float");
				return false;
			}
		}
		
		
		return true;
	}
	
	
	public void updateFeaturesFrame() {

		if (curProxy.features.length == 0) {
			table1.setModel(createTableModelForSize(1));
			table1.getModel().setValueAt(" (None) ", 0, 0);
			table1.setRowSelectionAllowed(false);
			table1.setEnabled(false);
			table1.setComponentPopupMenu(null);
		}

		else {

			table1.setRowSelectionAllowed(true);
			table1.setEnabled(true);

			scrollPane1.setMinimumSize(new Dimension(50, 30 + (23* curProxy.features.length)));

			table1.setModel(createTableModelForSize(curProxy.features.length));

		}
		for (int i = 0 ; i < curProxy.features.length ; i++) {
			
			String featvalues = "";
			for (int j = 0 ; j < curProxy.features[i].alternativeValues.length ; j++) {
				featvalues += FeatureValueUtils.toString(curProxy.features[i].alternativeValues[j]);
				featvalues += " (P=" + curProxy.features[i].alternativeValues[j].independentProb + ")";
				if (j < (curProxy.features[i].alternativeValues.length -1)) {
					featvalues += ", ";
				}
			}
			table1.getModel().setValueAt(curProxy.features[i].featlabel, i, 0);
			table1.getModel().setValueAt(featvalues, i, 1);
		}
 
		label8.setText(""+curProxy.features.length);
	}
	
	

	private void log(String s) {
		if (LOGGING) {
			System.out.println("[FeatureInfoGUI] " + s);
		}
	}

}
