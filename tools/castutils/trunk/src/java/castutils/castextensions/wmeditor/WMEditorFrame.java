package castutils.castextensions.wmeditor;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.UUID;

import javax.swing.ComboBoxModel;
import javax.swing.DefaultComboBoxModel;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JEditorPane;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.JTabbedPane;
import javax.swing.JTextField;
import javax.swing.ListModel;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

import org.apache.log4j.Logger;
import org.yaml.snakeyaml.error.YAMLException;

import cast.CASTException;
import cast.cdl.WorkingMemoryOperation;

/**
 * This code was edited or generated using CloudGarden's Jigloo SWT/Swing GUI
 * Builder, which is free for non-commercial use. If Jigloo is being used
 * commercially (ie, by a corporation, company or business for any purpose
 * whatever) then you should purchase a license for each developer using Jigloo.
 * Please visit www.cloudgarden.com for details. Use of Jigloo implies
 * acceptance of these licensing terms. A COMMERCIAL LICENSE HAS NOT BEEN
 * PURCHASED FOR THIS MACHINE, SO JIGLOO OR THIS CODE CANNOT BE USED LEGALLY FOR
 * ANY CORPORATE OR COMMERCIAL PURPOSE.
 */
public class WMEditorFrame extends javax.swing.JFrame {

	{
		// Set Look & Feel
		try {
			javax.swing.UIManager
					.setLookAndFeel("com.sun.java.swing.plaf.gtk.GTKLookAndFeel");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public interface EditorActionListener {
		void getFromWM() throws CASTException, YAMLException,
				ClassCastException;

		void writeToWM() throws CASTException, YAMLException,
				ClassCastException;

		void selectedTemplate();

		Logger getLogger();

		String getNewID();

		void addTemplate();

		void selectedEntry() throws CASTException, YAMLException,
				ClassCastException;

		void updateWMList();

		void renameTemplate(String newName);

		void deleteTemplate();

		void updateWMEntriesFilter();

		void updateTemplateFilter();
	}

	/**
	 * 
	 */
	private static final long serialVersionUID = 5228163719930021272L;

	/**
	 * @return the jIDTextField
	 */
	public JTextField getjIDTextField() {
		return jIDTextField;
	}

	/**
	 * @return the jnewIDButton
	 */
	public JButton getJnewIDButton() {
		return jnewIDButton;
	}

	/**
	 * @return the jGetButton
	 */
	public JButton getjGetButton() {
		return jGetButton;
	}

	/**
	 * @return the jTemplateRenameButton
	 */
	public JButton getjTemplateRenameButton() {
		return jTemplateRenameButton;
	}

	/**
	 * @return the jTemplateList
	 */
	public JList getjTemplateList() {
		return jTemplateList;
	}

	/**
	 * @return the jStatusBar
	 */
	public JTextField getjStatusBar() {
		return jStatusBar;
	}

	/**
	 * @return the jWriteButton
	 */
	public JButton getjWriteButton() {
		return jWriteButton;
	}

	/**
	 * @return the jOPComboBox
	 */
	public JComboBox getjOPComboBox() {
		return jOPComboBox;
	}

	/**
	 * @return the jSAComboBox
	 */
	public JComboBox getjSAComboBox() {
		return jSAComboBox;
	}

	private JTextField jIDTextField;
	private JButton jnewIDButton;
	private JButton jGetButton;
	private JEditorPane jEditorPane;
	private JScrollPane jScrollPane3;
	private JScrollPane jScrollPane2;
	private JTextField jWMFilterTextField;
	private JTextField jTemplateFilterTextField;
	private JButton jTemplateStoreButton;
	private JButton jTemplateDelButton;
	private JPanel jTemplatesButtonsPanel;
	private JButton jWMUpdateButton;
	private JList jWMList;
	private JPanel jWMEntriesPanel;
	private JTabbedPane jLeftTabbedPane;
	private JSplitPane jSplitPane1;
	private JButton jTemplateRenameButton;
	private JList jTemplateList;
	private JTextField jStatusBar;
	private JButton jWriteButton;
	private JComboBox jOPComboBox;
	private JComboBox jSAComboBox;
	private EditorActionListener listener;
	private JPanel jMainPanel;
	private JPanel jTemplatesPanel;

	/**
	 * Auto-generated main method to display this JFrame
	 */
	public static void main(String[] args) {
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				WMEditorFrame inst = new WMEditorFrame();
				inst.setLocationRelativeTo(null);
				inst.setVisible(true);
			}
		});
	}

	public WMEditorFrame(EditorActionListener wmEditorComponent) {
		super();
		listener = wmEditorComponent;
		initGUI();
	}

	public WMEditorFrame() {
		super();
		listener = new EditorActionListener() {

			@Override
			public void writeToWM() {
				// TODO Auto-generated method stub

			}

			@Override
			public void selectedTemplate() {
				// TODO Auto-generated method stub

			}

			@Override
			public void getFromWM() {
				// TODO Auto-generated method stub

			}

			@Override
			public Logger getLogger() {
				return Logger.getLogger(WMEditorFrame.class);
			}

			@Override
			public String getNewID() {
				return UUID.randomUUID().toString();
			}

			@Override
			public void addTemplate() {
				String[] testStr = new String[100];
				for (int i=0; i<testStr.length; i++) {
					testStr[i]=Integer.toString(i);
				}
				getjTemplateList().setListData(testStr);


			}

			@Override
			public void selectedEntry() {
				// TODO Auto-generated method stub

			}

			@Override
			public void updateWMList() {
				String[] testStr = new String[100];
				for (int i=0; i<testStr.length; i++) {
					testStr[i]=Integer.toString(i);
				}
				getjWMList().setListData(testStr);

			}

			@Override
			public void deleteTemplate() {
				// TODO Auto-generated method stub

			}

			@Override
			public void renameTemplate(String newName) {
				// TODO Auto-generated method stub

			}

			@Override
			public void updateTemplateFilter() {
				// TODO Auto-generated method stub

			}

			@Override
			public void updateWMEntriesFilter() {
				// TODO Auto-generated method stub

			}
		};
		initGUI();
	}

	private void initGUI() {
		try {
			BorderLayout thisLayout = new BorderLayout();
			thisLayout.setHgap(5);
			thisLayout.setVgap(1);
			setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
			getContentPane().setLayout(thisLayout);
			this.setPreferredSize(new java.awt.Dimension(1024, 600));
			this.setTitle("working memory editor");
			{
				jMainPanel = new JPanel();
				
				BorderLayout jPanel1Layout = new BorderLayout();
				jMainPanel.setLayout(jPanel1Layout);
				getContentPane().add(jMainPanel, BorderLayout.CENTER);
				{
					final JScrollPane jScrollPane1 = new JScrollPane();
					jMainPanel.add(jScrollPane1, BorderLayout.CENTER);
					{
						jEditorPane = new JEditorPane();
						jScrollPane1.setViewportView(jEditorPane);
						jEditorPane.setText("");
						jEditorPane.setFont(new java.awt.Font("DejaVu Sans Mono",0,9));
						jEditorPane.setBackground(Color.WHITE);
					}
				}
				{
					final JLabel jEditorLabel = new JLabel();
					jMainPanel.add(jEditorLabel, BorderLayout.NORTH);
					jEditorLabel.setText("WM entry");
				}
			}
			{
				final JPanel jTopPanel = new JPanel();
				GridLayout jTopPanelLayout = new GridLayout(1, 4);
				jTopPanelLayout.setHgap(5);
				jTopPanelLayout.setVgap(5);
				jTopPanelLayout.setColumns(4);
				jTopPanel.setLayout(jTopPanelLayout);
				getContentPane().add(jTopPanel, BorderLayout.NORTH);
				{
					final JPanel jIDPanel = new JPanel();
					BorderLayout jIDPanelLayout = new BorderLayout();
					jIDPanelLayout.setHgap(3);
					jIDPanel.setLayout(jIDPanelLayout);
					jTopPanel.add(jIDPanel);
					{
						final JLabel jIDLabel = new JLabel();
						jIDPanel.add(jIDLabel, BorderLayout.WEST);
						jIDLabel.setText("ID:");
					}
					{
						jIDTextField = new JTextField();
						jIDPanel.add(jIDTextField, BorderLayout.CENTER);
						jIDTextField.setText("the id");
						jIDTextField.setPreferredSize(new java.awt.Dimension(
								95, 22));
					}
					{
						jnewIDButton = new JButton();
						jIDPanel.add(jnewIDButton, BorderLayout.EAST);
						jnewIDButton.setText("new ID");
						jnewIDButton.addActionListener(new ActionListener() {
							public void actionPerformed(ActionEvent evt) {
								jIDTextField.setText(listener.getNewID());
								setStatus("new ID generated");
							}
						});
					}
				}
				{
					final JPanel jSAPanel = new JPanel();
					FlowLayout jSAPanelLayout = new FlowLayout();
					jSAPanelLayout.setAlignment(FlowLayout.LEFT);
					jSAPanel.setLayout(jSAPanelLayout);
					jTopPanel.add(jSAPanel);
					{
						final JLabel jSALabel = new JLabel();
						jSAPanel.add(jSALabel);
						jSALabel.setText("ID:");
					}
					{
						ComboBoxModel jSAComboBoxModel = new DefaultComboBoxModel(
								new String[] {});
						jSAComboBox = new JComboBox();
						jSAPanel.add(jSAComboBox);
						jSAComboBox.setModel(jSAComboBoxModel);
						jSAComboBox.setEditable(true);
						jSAComboBox.setPreferredSize(new java.awt.Dimension(
								134, 22));
					}
				}
				{
					final JPanel jOPPanel = new JPanel();
					FlowLayout jOPPanelLayout = new FlowLayout();
					jOPPanelLayout.setAlignment(FlowLayout.LEFT);
					jOPPanel.setLayout(jOPPanelLayout);
					jTopPanel.add(jOPPanel);
					{
						final JLabel jOPLabel = new JLabel();
						jOPPanel.add(jOPLabel);
						jOPLabel.setText("ID:");
					}
					{
						ComboBoxModel jOPComboBoxModel = new DefaultComboBoxModel(
								new Object[] { WorkingMemoryOperation.ADD,
										WorkingMemoryOperation.OVERWRITE,
										WorkingMemoryOperation.DELETE });
						jOPComboBox = new JComboBox();
						jOPPanel.add(jOPComboBox);
						jOPComboBox.setModel(jOPComboBoxModel);
						jOPComboBox.setSize(120, 22);
//						jOPComboBox.setPreferredSize(new java.awt.Dimension(
//								124, 22));
					}
				}
				{
					final JPanel jExecutePanel = new JPanel();
					GridLayout jExecutePanelLayout = new GridLayout(1, 1);
					jExecutePanelLayout.setHgap(5);
					jExecutePanelLayout.setVgap(5);
					jExecutePanelLayout.setColumns(1);
					jExecutePanel.setLayout(jExecutePanelLayout);
					jTopPanel.add(jExecutePanel);
					jExecutePanel.setBounds(131, 77, 119, 25);
					{
						jGetButton = new JButton();
						jExecutePanel.add(jGetButton);
						jGetButton.setText("get");
//						jGetButton.setPreferredSize(new java.awt.Dimension(133,
//								32));
						jGetButton.addActionListener(new ActionListener() {
							public void actionPerformed(ActionEvent evt) {
								jGetButtonActionPerformed(evt);
							}
						});
					}
					{
						jWriteButton = new JButton();
						jExecutePanel.add(jWriteButton);
						jWriteButton.setText("write");
						jWriteButton.addActionListener(new ActionListener() {
							public void actionPerformed(ActionEvent evt) {
								jWriteButtonActionPerformed(evt);
							}
						});
					}
				}
			}
			{
				jStatusBar = new JTextField();
				getContentPane().add(jStatusBar, BorderLayout.SOUTH);
				jStatusBar.setText("set the status here");
				jStatusBar.setBackground(new java.awt.Color(229, 229, 229));
				jStatusBar.setEditable(false);
			}
			{
				jSplitPane1 = new JSplitPane();
				getContentPane().add(jSplitPane1, BorderLayout.CENTER);
//				jSplitPane1.setPreferredSize(new java.awt.Dimension(1020, 508));
				jSplitPane1.add(jMainPanel, JSplitPane.RIGHT);
//				jMainPanel.setPreferredSize(new java.awt.Dimension(900, 507));
				{
					jLeftTabbedPane = new JTabbedPane();
					jSplitPane1.add(jLeftTabbedPane, JSplitPane.LEFT);
					jLeftTabbedPane.setPreferredSize(new java.awt.Dimension(
							180, 380));
					jLeftTabbedPane.setSize(185, 380);
					{
						jWMEntriesPanel = new JPanel();
						jLeftTabbedPane.addTab("WM entries", null,
								jWMEntriesPanel, null);
						BorderLayout jPanel1Layout1 = new BorderLayout();
						jWMEntriesPanel.setMinimumSize(new java.awt.Dimension(
								190, 10));
						jWMEntriesPanel
								.setPreferredSize(new java.awt.Dimension(180,
										380));
						jWMEntriesPanel.setLayout(jPanel1Layout1);
						jWMEntriesPanel.setSize(180, 380);
						{
							jWMFilterTextField = new JTextField();
							jWMEntriesPanel.add(jWMFilterTextField,
									BorderLayout.NORTH);
							jWMFilterTextField.getDocument().addDocumentListener(new DocumentListener() {
								
								@Override
								public void removeUpdate(DocumentEvent arg0) {
									listener.updateWMEntriesFilter();
									
								}
								
								@Override
								public void insertUpdate(DocumentEvent arg0) {
									listener.updateWMEntriesFilter();
									
								}
								
								@Override
								public void changedUpdate(DocumentEvent arg0) {
									listener.updateWMEntriesFilter();
									
								}
							});

						}
						{
							jScrollPane2 = new JScrollPane();
							jWMEntriesPanel.add(jScrollPane2, BorderLayout.CENTER);
							jScrollPane2.setPreferredSize(new java.awt.Dimension(190, 410));
							{
								ListModel jList1Model = new DefaultComboBoxModel(
										new String[] {});
								jWMList = new JList();
								jScrollPane2.setViewportView(jWMList);
								jWMList.setModel(jList1Model);
								jWMList.setFont(new java.awt.Font("DejaVu Sans Mono",0,9));
								jWMList.addMouseListener(new MouseAdapter() {
									public void mouseClicked(MouseEvent evt) {
										try {
											listener.selectedEntry();
										} catch (RuntimeException e) {
											setStatus(e);
										} catch (CASTException e) {
											setStatus(e);
										}
									}
								});
							}
						}
						{
							jWMUpdateButton = new JButton();
							jWMEntriesPanel.add(jWMUpdateButton,
									BorderLayout.SOUTH);
							jWMUpdateButton.setText("update");
							jWMUpdateButton
									.addActionListener(new ActionListener() {
										public void actionPerformed(
												ActionEvent evt) {
											listener.updateWMList();
										}
									});
						}
					}
					{
						jTemplatesPanel = new JPanel();
						jLeftTabbedPane.addTab("Templates", null,
								jTemplatesPanel, null);
						BorderLayout jLeftPanelLayout = new BorderLayout();
						jTemplatesPanel.setLayout(jLeftPanelLayout);
						jTemplatesPanel.setSize(158, 380);
						jTemplatesPanel.setMinimumSize(new java.awt.Dimension(
								160, 10));
						jTemplatesPanel
								.setPreferredSize(new java.awt.Dimension(160,
										380));
						{
							jScrollPane3 = new JScrollPane();
							jTemplatesPanel.add(jScrollPane3, BorderLayout.CENTER);
							jScrollPane3.setPreferredSize(new java.awt.Dimension(190, 417));
							{
								ListModel jTemplateListModel = new DefaultComboBoxModel(
										new String[] {});
								jTemplateList = new JList();
								jScrollPane3.setViewportView(jTemplateList);
								GridLayout jTemplateListLayout = new GridLayout(1,
										1);
								jTemplateListLayout.setHgap(5);
								jTemplateListLayout.setVgap(5);
								jTemplateListLayout.setColumns(1);
								jTemplateList.setModel(jTemplateListModel);
								jTemplateList.setLayout(null);

								jTemplateList.setFont(new java.awt.Font("DejaVu Sans Mono",0,9));
								jTemplateList.addMouseListener(new MouseAdapter() {
									public void mouseClicked(MouseEvent evt) {
										listener.selectedTemplate();
									}
								});
							}
						}
						{
							jTemplatesButtonsPanel = new JPanel();
							GridLayout jPanel1Layout2 = new GridLayout(1, 1);
							jPanel1Layout2.setHgap(5);
							jPanel1Layout2.setVgap(5);
							jPanel1Layout2.setColumns(1);
							jTemplatesButtonsPanel.setLayout(jPanel1Layout2);
							jTemplatesPanel.add(jTemplatesButtonsPanel,
									BorderLayout.SOUTH);
							jTemplatesButtonsPanel
									.setPreferredSize(new java.awt.Dimension(
											190, 22));
							{
								jTemplateStoreButton = new JButton();
								jTemplatesButtonsPanel
										.add(jTemplateStoreButton);
								jTemplateStoreButton.setText("store");
								jTemplateStoreButton
										.addActionListener(new ActionListener() {
											public void actionPerformed(
													ActionEvent evt) {
												try {
													listener.addTemplate();
												} catch (RuntimeException e) {
													setStatus(e);
												}
											}
										});
							}
							{
								jTemplateDelButton = new JButton();
								jTemplatesButtonsPanel.add(jTemplateDelButton);
								jTemplateDelButton.setText("del");
								jTemplateDelButton
										.addActionListener(new ActionListener() {
											public void actionPerformed(
													ActionEvent evt) {
												listener.deleteTemplate();
											}
										});
							}
							{
								jTemplateRenameButton = new JButton();
								jTemplatesButtonsPanel
										.add(jTemplateRenameButton);
								jTemplateRenameButton.setText("ren.");
								jTemplateRenameButton
										.addActionListener(new ActionListener() {
											public void actionPerformed(
													ActionEvent evt) {
												try {
													String newName = JOptionPane
															.showInputDialog(
																	"choose new name",
																	jTemplateList
																			.getSelectedValue());
													if (newName != null)
														listener
																.renameTemplate(newName);
												} catch (RuntimeException e) {
													setStatus(e);
												}
											}
										});
							}
						}
						{
							jTemplateFilterTextField = new JTextField();
							jTemplatesPanel.add(getJTemplateFilterTextField(),
									BorderLayout.NORTH);
							jTemplateFilterTextField.getDocument().addDocumentListener(new DocumentListener() {
								
								@Override
								public void removeUpdate(DocumentEvent arg0) {
									listener.updateTemplateFilter();
									
								}
								
								@Override
								public void insertUpdate(DocumentEvent arg0) {
									listener.updateTemplateFilter();
									
								}
								
								@Override
								public void changedUpdate(DocumentEvent arg0) {
									listener.updateTemplateFilter();
									
								}
							});
						}
					}
				}
				{

				}
			}
			pack();
			this.setSize(700, 500);
			setStatus("ready to go");
		} catch (Exception e) {
			listener.getLogger().error(e);
		}
	}

	/**
	 * @return the jEditorPane
	 */
	public JEditorPane getjEditorPane() {
		return jEditorPane;
	}

	private void setStatus(String string) {
		jStatusBar.setText(string);
		jStatusBar.setForeground(Color.BLACK);
		listener.getLogger().info(string);
		jStatusBar.setToolTipText(string);

	}

	private void jWriteButtonActionPerformed(ActionEvent evt) {
		try {
			listener.writeToWM();
			setStatus("written to WM");
		} catch (CASTException e) {
			setStatus(e);
		} catch (RuntimeException e) {
			setStatus(e);
		}

	}

	/**
	 * @param e
	 */
	protected void setStatus(Exception e) {
		String string = e.getClass().getSimpleName() + ": " + e.getMessage();
		jStatusBar.setText(string);
		jStatusBar.setForeground(Color.RED);
		jStatusBar.setToolTipText(e.getClass().getSimpleName());
		listener.getLogger().warn(string);
	}

	protected void setStatus(CASTException e) {
		String string = e.getClass().getSimpleName() + ": " + e.message;
		jStatusBar.setText(string);
		jStatusBar.setForeground(Color.RED);
		jStatusBar.setToolTipText(e.getClass().getSimpleName());
		listener.getLogger().warn(string);
	}

	private void jGetButtonActionPerformed(ActionEvent evt) {
		try {
			listener.getFromWM();
			setStatus("got from WM");
		} catch (CASTException e) {
			setStatus(e);
		} catch (RuntimeException e) {
			setStatus(e);
		}
	}

	public JTextField getJTemplateFilterTextField() {
		return jTemplateFilterTextField;
	}

	/**
	 * @return the jWMFilterTextField
	 */
	public JTextField getjWMFilterTextField() {
		return jWMFilterTextField;
	}

	/**
	 * @return the jTemplateFilterTextField
	 */
	public JTextField getjTemplateFilterTextField() {
		return jTemplateFilterTextField;
	}

	/**
	 * @return the jTemplateStoreButton
	 */
	public JButton getjTemplateStoreButton() {
		return jTemplateStoreButton;
	}

	/**
	 * @return the jTemplateDelButton
	 */
	public JButton getjTemplateDelButton() {
		return jTemplateDelButton;
	}

	/**
	 * @return the jTemplatesButtonsPanel
	 */
	public JPanel getjTemplatesButtonsPanel() {
		return jTemplatesButtonsPanel;
	}

	/**
	 * @return the jWMUpdateButton
	 */
	public JButton getjWMUpdateButton() {
		return jWMUpdateButton;
	}

	/**
	 * @return the jWMList
	 */
	public JList getjWMList() {
		return jWMList;
	}

	/**
	 * @return the jWMEntriesPanel
	 */
	public JPanel getjWMEntriesPanel() {
		return jWMEntriesPanel;
	}

	/**
	 * @return the jLeftTabbedPane
	 */
	public JTabbedPane getjLeftTabbedPane() {
		return jLeftTabbedPane;
	}

	/**
	 * @return the jSplitPane1
	 */
	public JSplitPane getjSplitPane1() {
		return jSplitPane1;
	}

	/**
	 * @return the jMainPanel
	 */
	public JPanel getjMainPanel() {
		return jMainPanel;
	}

	/**
	 * @return the jTemplatesPanel
	 */
	public JPanel getjTemplatesPanel() {
		return jTemplatesPanel;
	}

}
