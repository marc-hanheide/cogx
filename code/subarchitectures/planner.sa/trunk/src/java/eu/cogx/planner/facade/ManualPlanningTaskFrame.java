/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.planner.facade;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.GridLayout;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.TableModel;
import javax.swing.table.TableRowSorter;
import javax.swing.text.BadLocationException;
import javax.swing.JTextArea;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class ManualPlanningTaskFrame extends JFrame {

	public static interface SubmitListener {
		String submit(TableModel beliefTableModel);
	}

	private static final long serialVersionUID = 1L;
	private JPanel jContentPane = null;
	private JPanel jPanel = null;
	private JScrollPane jScrollPane = null;
	private JTable jTable = null;
	private final SubmitListener listener;
	private TableModel beliefTableModel;
	private JPanel jPanel1 = null;
	private JTextField jTextField = null;
	private JButton jButtonSubmit = null;
	private JPanel jPanel2 = null;
	private JScrollPane jScrollPane1 = null;
	private JTable jTable1 = null;
	private JTextArea jStatus = null;

	/**
	 * This is the default constructor
	 */
	public ManualPlanningTaskFrame() {
		super();
		beliefTableModel = new DefaultTableModel(new String[] { "ID", "Type" },
				1);
		listener = new SubmitListener() {

			@Override
			public String submit(TableModel firstColValue) {
				return "";
			}
		};
		initialize();
	}

	/**
	 * This is the default constructor
	 */
	public ManualPlanningTaskFrame(TableModel model, SubmitListener listener) {
		super();
		this.listener = listener;
		beliefTableModel = model;
		initialize();
	}

	/**
	 * This method initializes this
	 * 
	 * @return void
	 */
	private void initialize() {
		this.setSize(500, 248);
		this.setContentPane(getJContentPane());
		this.setTitle("ManualPlanningTask");
		this.pack();
	}

	/**
	 * This method initializes jContentPane
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJContentPane() {
		if (jContentPane == null) {
			jContentPane = new JPanel();
			jContentPane.setLayout(new BorderLayout());
			jContentPane.add(getJPanel(), BorderLayout.WEST);
			jContentPane.add(getJPanel1(), BorderLayout.SOUTH);
			jContentPane.add(getJPanel2(), BorderLayout.CENTER);
		}
		return jContentPane;
	}

	/**
	 * This method initializes jPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJPanel() {
		if (jPanel == null) {
			jPanel = new JPanel();
			jPanel.setLayout(new BorderLayout());
			jPanel.add(getJScrollPane(), BorderLayout.CENTER);
		}
		return jPanel;
	}

	/**
	 * This method initializes jScrollPane
	 * 
	 * @return javax.swing.JScrollPane
	 */
	private JScrollPane getJScrollPane() {
		if (jScrollPane == null) {
			jScrollPane = new JScrollPane();
			jScrollPane.setPreferredSize(new Dimension(250, 300));
			jScrollPane.setViewportView(getJTable());
		}
		return jScrollPane;
	}

	/**
	 * This method initializes jTable
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getJTable() {
		if (jTable == null) {
			jTable = new JTable();
			jTable.setModel(beliefTableModel);

			jTable.addMouseListener(new java.awt.event.MouseAdapter() {
				public void mouseClicked(java.awt.event.MouseEvent e) {
					String text = (String) jTable.getValueAt(jTable
							.getSelectedRow(), 0);
					int pos = jTextField.getCaretPosition();
					try {
						jTextField.getDocument().insertString(pos,
								"'" + text + "'", null);
					} catch (BadLocationException e1) {
						e1.printStackTrace();
					}
				}
			});
			final TableRowSorter<TableModel> sorter;
			sorter = new TableRowSorter<TableModel>(beliefTableModel);
			sorter.setSortsOnUpdates(true);
			jTable.setRowSorter(sorter);
			jTable.setAutoResizeMode(JTable.AUTO_RESIZE_LAST_COLUMN);
			jTable.getColumnModel().getColumn(0).setPreferredWidth(60);
			jTable.getColumnModel().getColumn(1).setPreferredWidth(30);

		}
		return jTable;
	}

	/**
	 * This method initializes jPanel1
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJPanel1() {
		if (jPanel1 == null) {
			jPanel1 = new JPanel();
			jPanel1.setLayout(new GridLayout(0, 1));
			jPanel1.add(getJTextField());
			jPanel1.add(getJButtonSubmit());
			jPanel1.add(getJStatus(), null);
		}
		return jPanel1;
	}

	/**
	 * This method initializes jTextField
	 * 
	 * @return javax.swing.JTextField
	 */
	private JTextField getJTextField() {
		if (jTextField == null) {
			jTextField = new JTextField();
			jTextField.setText("");
			jTextField.getDocument().addDocumentListener(
					new DocumentListener() {

						public void change() {
							int row = jTable1.getSelectedRow();
							if (row > -1)
								jTable1
										.setValueAt(jTextField.getText(), row,
												0);
						}

						public void changedUpdate(DocumentEvent e) {
							change();
						}

						public void removeUpdate(DocumentEvent e) {
							change();
						}

						public void insertUpdate(DocumentEvent e) {
							change();
						}
					});

		}
		return jTextField;
	}

	/**
	 * This method initializes jButtonSubmit
	 * 
	 * @return javax.swing.JButton
	 */
	private JButton getJButtonSubmit() {
		if (jButtonSubmit == null) {
			jButtonSubmit = new JButton();
			jButtonSubmit.setText("Plan");
			jButtonSubmit
					.addActionListener(new java.awt.event.ActionListener() {
						public void actionPerformed(java.awt.event.ActionEvent e) {
							String ret = listener.submit(jTable1.getModel());
							if (ret!=null) {
								jStatus.setText(ret);
							}
						}
					});
		}
		return jButtonSubmit;
	}

	/**
	 * This method initializes jPanel2
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJPanel2() {
		if (jPanel2 == null) {
			jPanel2 = new JPanel();
			jPanel2.setLayout(new BorderLayout());
			jPanel2.add(getJScrollPane1(), BorderLayout.CENTER);
		}
		return jPanel2;
	}

	/**
	 * This method initializes jScrollPane1
	 * 
	 * @return javax.swing.JScrollPane
	 */
	private JScrollPane getJScrollPane1() {
		if (jScrollPane1 == null) {
			jScrollPane1 = new JScrollPane();
			jScrollPane1.setPreferredSize(new Dimension(600, 300));
			jScrollPane1.setViewportView(getJTable1());
		}
		return jScrollPane1;
	}

	/**
	 * This method initializes jTable1
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getJTable1() {
		if (jTable1 == null) {
			jTable1 = new JTable();
			jTable1.setModel(new DefaultTableModel(new String[] { "Goal",
					"importance" }, 10));
			jTable1.addMouseListener(new java.awt.event.MouseAdapter() {
				public void mouseClicked(java.awt.event.MouseEvent e) {
					jTextField.setText((String) jTable1.getValueAt(jTable1
							.getSelectedRow(), 0));

				}
			});

			// jTable1.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);
			jTable1.getColumnModel().getColumn(0).setPreferredWidth(900);
			jTable1.getColumnModel().getColumn(1).setPreferredWidth(100);
			jTable1.changeSelection(0, 0, true, false);
		}
		return jTable1;
	}

	/**
	 * This method initializes jStatus	
	 * 	
	 * @return javax.swing.JTextArea	
	 */
	private JTextArea getJStatus() {
		if (jStatus == null) {
			jStatus = new JTextArea();
			jStatus.setEditable(false);
		}
		return jStatus;
	}

} // @jve:decl-index=0:visual-constraint="10,10"
