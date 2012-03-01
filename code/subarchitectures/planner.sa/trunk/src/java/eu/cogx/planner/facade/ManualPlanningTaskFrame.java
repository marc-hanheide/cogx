/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package eu.cogx.planner.facade;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.TableModel;
import javax.swing.table.TableRowSorter;
import javax.swing.text.BadLocationException;


/**
* This code was edited or generated using CloudGarden's Jigloo
* SWT/Swing GUI Builder, which is free for non-commercial
* use. If Jigloo is being used commercially (ie, by a corporation,
* company or business for any purpose whatever) then you
* should purchase a license for each developer using Jigloo.
* Please visit www.cloudgarden.com for details.
* Use of Jigloo implies acceptance of these licensing terms.
* A COMMERCIAL LICENSE HAS NOT BEEN PURCHASED FOR
* THIS MACHINE, SO JIGLOO OR THIS CODE CANNOT BE USED
* LEGALLY FOR ANY CORPORATE OR COMMERCIAL PURPOSE.
*/
/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class ManualPlanningTaskFrame extends JFrame {

	{
		//Set Look & Feel
		try {
			javax.swing.UIManager.setLookAndFeel("com.sun.java.swing.plaf.gtk.GTKLookAndFeel");
		} catch(Exception e) {
			e.printStackTrace();
		}
	}


	public static interface SubmitListener {
		String submit(TableModel beliefTableModel, boolean execute);
	}

	private static final long serialVersionUID = 1L;
	private JPanel jContentPane = null;
	private JPanel jBeliefsPanel = null;
	private JScrollPane jBeliefsScrollPane = null;
	private JTable jBeliefsTable = null;
	private final SubmitListener listener;
	private TableModel beliefTableModel;
	public TableModel getBeliefTableModel() {
		return beliefTableModel;
	}

	public void setBeliefTableModel(TableModel beliefTableModel) {
		this.beliefTableModel = beliefTableModel;
	}


	private JPanel jButtonPanel = null;
	private JTextField jGoalTextEditField = null;
	public JTextField getjGoalTextEditField() {
		return jGoalTextEditField;
	}


	private JButton jButtonSubmit = null;
	private JPanel jGoalsPanel = null;
	private JScrollPane jGoalsScrollPane = null;
	private JTable jGoalsTable = null;
	private JTextArea jStatus = null;
	private JScrollPane jStatusScrollPane = null;
	private JCheckBox jExecuteCheckbox = null;
	private JPanel jPlanButtonsPanel = null;
    private List<String> goals = null;

	/**
	 * This is the default constructor
	 */
	public ManualPlanningTaskFrame() {
		super();
		beliefTableModel = new DefaultTableModel(new String[] { "ID", "Type" },
				1);
		listener = new SubmitListener() {

			@Override
			public String submit(TableModel firstColValue, boolean execute) {
				return "";
			}
		};
		initialize();
	}

	/**
	 * This is the default constructor
	 */
	public ManualPlanningTaskFrame(TableModel model, SubmitListener listener, List<String> goals) {
		super();
		this.listener = listener;
		beliefTableModel = model;
        this.goals = goals;
		initialize();
	}

	/**
	 * This method initializes jBeliefsPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJBeliefsPanel() {
		if (jBeliefsPanel == null) {
			jBeliefsPanel = new JPanel();
			jBeliefsPanel.setLayout(new BorderLayout());
			jBeliefsPanel.add(getJBeliefsScrollPane(), BorderLayout.CENTER);
		}
		return jBeliefsPanel;
	}

	/**
	 * This method initializes jBeliefsScrollPane
	 * 
	 * @return javax.swing.JScrollPane
	 */
	private JScrollPane getJBeliefsScrollPane() {
		if (jBeliefsScrollPane == null) {
			jBeliefsScrollPane = new JScrollPane();
			jBeliefsScrollPane.setPreferredSize(new Dimension(250, 300));
			jBeliefsScrollPane.setViewportView(getJBeliefsTable());
		}
		return jBeliefsScrollPane;
	}

	/**
	 * This method initializes jBeliefsTable
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getJBeliefsTable() {
		if (jBeliefsTable == null) {
			jBeliefsTable = new JTable();
			jBeliefsTable.setModel(beliefTableModel);

			jBeliefsTable.addMouseListener(new java.awt.event.MouseAdapter() {
				public void mouseClicked(java.awt.event.MouseEvent e) {
					String text = (String) jBeliefsTable.getValueAt(
							jBeliefsTable.getSelectedRow(), 0);
					int pos = jGoalTextEditField.getCaretPosition();
					try {
						jGoalTextEditField.getDocument().insertString(pos,
								"'" + text + "'", null);
					} catch (BadLocationException e1) {
						e1.printStackTrace();
					}
				}
			});
			final TableRowSorter<TableModel> sorter;
			sorter = new TableRowSorter<TableModel>(beliefTableModel);
			sorter.setSortsOnUpdates(true);
			jBeliefsTable.setRowSorter(sorter);
			jBeliefsTable.setAutoResizeMode(JTable.AUTO_RESIZE_LAST_COLUMN);
			jBeliefsTable.getColumnModel().getColumn(0).setPreferredWidth(60);
			jBeliefsTable.getColumnModel().getColumn(1).setPreferredWidth(30);

		}
		return jBeliefsTable;
	}

	/**
	 * This method initializes jButtonPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	public JPanel getJButtonPanel() {
		if (jButtonPanel == null) {
			GridLayout gridLayout = new GridLayout(0, 1);
			gridLayout.setColumns(1);
			jButtonPanel = new JPanel();
			jButtonPanel.setLayout(gridLayout);
			jButtonPanel.add(getJGoalTextEditField());
			jButtonPanel.add(getJPlanButtonsPanel(), null);
			jButtonPanel.add(getJStatusScrollPane(), null);
		}
		return jButtonPanel;
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
							String ret = listener
									.submit(jGoalsTable.getModel(), jExecuteCheckbox.isSelected());
							if (ret != null) {
								jStatus.setText(ret);
							}
						}
					});
		}
		return jButtonSubmit;
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
			jContentPane.add(getJBeliefsPanel(), BorderLayout.WEST);
			jContentPane.add(getJButtonPanel(), BorderLayout.SOUTH);
			jContentPane.add(getJGoalsPanel(), BorderLayout.CENTER);
		}
		return jContentPane;
	}

	/**
	 * This method initializes jExecuteCheckbox	
	 * 	
	 * @return javax.swing.JCheckBox	
	 */
	private JCheckBox getJExecuteCheckbox() {
		if (jExecuteCheckbox == null) {
			jExecuteCheckbox = new JCheckBox();
			jExecuteCheckbox.setText("execute this plan?");
		}
		return jExecuteCheckbox;
	}

	/**
	 * This method initializes jGoalsPanel
	 * 
	 * @return javax.swing.JPanel
	 */
	private JPanel getJGoalsPanel() {
		if (jGoalsPanel == null) {
			jGoalsPanel = new JPanel();
			jGoalsPanel.setLayout(new BorderLayout());
			jGoalsPanel.add(getJGoalsScrollPane(), BorderLayout.CENTER);
		}
		return jGoalsPanel;
	}

	/**
	 * This method initializes jGoalsScrollPane
	 * 
	 * @return javax.swing.JScrollPane
	 */
	private JScrollPane getJGoalsScrollPane() {
		if (jGoalsScrollPane == null) {
			jGoalsScrollPane = new JScrollPane();
			jGoalsScrollPane.setPreferredSize(new Dimension(600, 300));
			jGoalsScrollPane.setViewportView(getJGoalsTable());
		}
		return jGoalsScrollPane;
	}

	/**
	 * This method initializes jGoalsTable
	 * 
	 * @return javax.swing.JTable
	 */
	private JTable getJGoalsTable() {
		if (jGoalsTable == null) {
			jGoalsTable = new JTable();
			jGoalsTable.setModel(new DefaultTableModel(new String[] { "Goal",
					"importance" , "deadline" }, 10));
			jGoalsTable.addMouseListener(new java.awt.event.MouseAdapter() {
				public void mouseClicked(java.awt.event.MouseEvent e) {
					jGoalTextEditField.setText((String) jGoalsTable.getValueAt(
							jGoalsTable.getSelectedRow(), 0));

				}
			});

			// jTable1.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);
			jGoalsTable.getColumnModel().getColumn(0).setPreferredWidth(900);
			jGoalsTable.getColumnModel().getColumn(1).setPreferredWidth(100);
			jGoalsTable.getColumnModel().getColumn(2).setPreferredWidth(100);
			jGoalsTable.changeSelection(0, 0, true, false);
		}
		return jGoalsTable;
	}

	/**
	 * This method initializes jGoalTextEditField
	 * 
	 * @return javax.swing.JTextField
	 */
	private JTextField getJGoalTextEditField() {
		if (jGoalTextEditField == null) {
			jGoalTextEditField = new JTextField();
			jGoalTextEditField.setText("");
			jGoalTextEditField.getDocument().addDocumentListener(
					new DocumentListener() {

						public void change() {
							int row = jGoalsTable.getSelectedRow();
							if (row > -1)
								jGoalsTable.setValueAt(jGoalTextEditField
										.getText(), row, 0);
						}

						public void changedUpdate(DocumentEvent e) {
							change();
						}

						public void insertUpdate(DocumentEvent e) {
							change();
						}

						public void removeUpdate(DocumentEvent e) {
							change();
						}
					});

		}
		return jGoalTextEditField;
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
			jStatus.setLineWrap(true);
			jStatus
					.setText("This is a manual PlanningTask creator to test the oversubscription planning. The left pane shows all beliefs available to the planner. Click them to include them into the currently edited goal.");
		}
		return jStatus;
	}

	/**
	 * This method initializes jStatusScrollPane
	 * 
	 * @return javax.swing.JScrollPane
	 */
	private JScrollPane getJStatusScrollPane() {
		if (jStatusScrollPane == null) {
			jStatusScrollPane = new JScrollPane();
			jStatusScrollPane.setViewportView(getJStatus());
			jStatusScrollPane
					.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		}
		return jStatusScrollPane;
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
        if (goals != null && !goals.isEmpty()) {
            int i = 0;
            for (String g : goals) {
                getJGoalsTable().setValueAt(g, i, 0);
                if (i == 0) {
                    jGoalTextEditField.setText(g);
                }
                i++;
            }
        }
        else {
            //jGoalTextEditField.setText("(exists (?o - visualobject) (and (= (label ?o) cerealbox) (position-reported ?o)))");
            jGoalTextEditField.setText("(forall (?p - ProtoObject) (exists (?v - VisualObject) (= (po-is-associated-with ?p) ?v)))");
        }
	}

	/**
	 * This method initializes jPlanButtonsPanel	
	 * 	
	 * @return javax.swing.JPanel	
	 */
	private JPanel getJPlanButtonsPanel() {
		if (jPlanButtonsPanel == null) {
			jPlanButtonsPanel = new JPanel();
			jPlanButtonsPanel.setLayout(new GridLayout(1,2));
			jPlanButtonsPanel.add(getJExecuteCheckbox(), null);
			jPlanButtonsPanel.add(getJButtonSubmit(), null);
		}
		return jPlanButtonsPanel;
	}

} // @jve:decl-index=0:visual-constraint="10,10"
