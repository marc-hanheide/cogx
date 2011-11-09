/*
 * This file is part of Bayesian Network for Java (BNJ).
 * Version 3.3+
 *
 * BNJ is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * BNJ is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BNJ in LICENSE.txt file; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * BNJ Version History
 * ---------------------------------------------
 * BN tools Jan 2000-May 2002
 *
 *  prealpha- January 200 - June 2001
 *	Benjamin Perry, Haipeng Guo, Laura Haverkamp
 *  Version 1- June 2001 - May 2002
 * 	Haipeng Guo, Benjamin Perry, Julie A. Thornton BNJ
 *
 * Bayesian Network for Java (BNJ).
 *  Version 1 - May 2002 - July 2003
 *  	release: v1.03a 29 July 2003
 * 	Infrastructure - Roby Joehanes, Haipeng Guo, Benjamin Perry, Julie A. Thornton
 *	Modules - Sonal S. Junnarkar
 *  Version 2 - August 2003 - July 2004
 *  	release: v2.03a 08 July 2004
 * 	Infrastructure - Roby Joehanes, Julie A. Thornton
 *	Modules - Siddharth Chandak, Prashanth Boddhireddy, Chris H. Meyer, Charlie L. Thornton, Bart Peinter
 *  Version 3 - August 2004 - Present
 *     	Infrastructure - Jeffrey M. Barber
 *	Modules - William H. Hsu, Andrew L. King, Chris H. Meyer, Julie A. Thornton
 * ---------------------------------------------
 */package edu.ksu.cis.bnj.gui.dialogs;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.List;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.TabFolder;
import org.eclipse.swt.widgets.TabItem;
import org.eclipse.swt.custom.CLabel;
import org.eclipse.swt.graphics.Color;
import edu.ksu.cis.bnj.gui.tools.Constants;
import edu.ksu.cis.bnj.gui.tools.LanguageUnicodeParser;
import edu.ksu.cis.bnj.gui.tools.UndoContext;
import edu.ksu.cis.bnj.gui.tools.undo.UndoInternalChange;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.Continuous;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.core.Domain;
import org.eclipse.swt.widgets.Group;
import org.eclipse.swt.events.ModifyListener;
import org.eclipse.swt.events.ModifyEvent;
public class BeliefNodeEditorDlg
{
	private Text		textVariable;
	private Text		textNewOutcome;
	private List		listOutcomes;
	private List		listPossibleOutcomes;
	private Text		textName;
	private Group		groupDiscrete;
	private Button		buttonCont;
	private Button		buttonDiscrete;
	private BeliefNode	nodeInEdit	= null;
	private Label		labelCont;
	private UndoContext	undoContext	= null;
	public int			BNE_Width	= 300;
	public int			BNE_Height	= 375;
	public void moveUp()
	{
		int idx = listOutcomes.getSelectionIndex();
		if (idx < 0) return;
		int nidx = idx - 1;
		if (nidx < 0) nidx = 0;
		String e = listOutcomes.getItem(idx);
		listOutcomes.remove(idx);
		listOutcomes.add(e, nidx);
		listOutcomes.select(nidx);
	}
	public void moveDown()
	{
		int idx = listOutcomes.getSelectionIndex();
		if (idx < 0) return;
		if (idx >= listOutcomes.getItemCount() - 1) return;
		int nidx = idx + 1;
		if (nidx < 0) nidx = 0;
		String e = listOutcomes.getItem(idx);
		listOutcomes.remove(idx);
		listOutcomes.add(e, nidx);
		listOutcomes.select(nidx);
	}
	public void Remove()
	{
		int idx = listOutcomes.getSelectionIndex();
		if (idx < 0) return;
		listOutcomes.remove(idx);
	}
	public void Add(String txt)
	{
		if(listOutcomes.indexOf(txt)<0)
		{
			if(txt.length() >= 1)
			{
				listOutcomes.add(txt);
			}
		}
	}
	String	origName;
	public void Rename(String x)
	{
		nodeInEdit.setName(x);
	}
	BeliefNetwork	_network;
	public void Apply()
	{
		if (undoContext != null)
		{
			undoContext.register(new UndoInternalChange(_network));
		}
		if(nodeInEdit.getType() == BeliefNode.NODE_CHANCE)
		{
			if (buttonDiscrete.getSelection() && !buttonCont.getSelection())
			{
				Discrete D = new Discrete();
				for (int i = 0; i < listOutcomes.getItemCount(); i++)
				{
					String x = listOutcomes.getItem(i);
					D.addName(x);
				}
				_network.changeBeliefNodeDomain(nodeInEdit, D);
			}
			else if (buttonCont.getSelection() && !buttonDiscrete.getSelection())
			{
				Continuous C = new Continuous(textVariable.getText());
				_network.changeBeliefNodeDomain(nodeInEdit, C);
			}
		}
		if(nodeInEdit.getType() == BeliefNode.NODE_DECISION)
		{
			Discrete D = new Discrete();
			for (int i = 0; i < listOutcomes.getItemCount(); i++)
			{
				String x = listOutcomes.getItem(i);
				D.addName(x);
			}
			_network.changeBeliefNodeDomain(nodeInEdit, D);
		}
	}
	public void Cancel()
	{
		nodeInEdit.setName(origName);
	}
	private LanguageUnicodeParser	LUP;
	public void open(Shell shellParent, Constants constants, BeliefNode bnode, BeliefNetwork bn, UndoContext UC)
	{
		LUP = LanguageUnicodeParser.getInstance();
		undoContext = UC;
		_network = bn;
		origName = bnode.getName();
		nodeInEdit = bnode;
		final Shell shell = new Shell(shellParent, SWT.APPLICATION_MODAL | SWT.CLOSE | SWT.TITLE);
		shell.setBounds(shellParent.getLocation().x + shellParent.getSize().x / 2 - BNE_Width / 2, shellParent
				.getLocation().y
				+ shellParent.getSize().y / 2 - BNE_Height / 2, BNE_Width, BNE_Height);
		shell.setText(LUP.get("edittitle") + ": " + bnode.getName());
		{
			final TabFolder tabFolder = new TabFolder(shell, SWT.NONE);
			tabFolder.setBounds(0, 0, 290, 310);
			{
				final TabItem tabItem = new TabItem(tabFolder, SWT.NONE);
				tabItem.setText(LUP.get("properties"));
				{
					final Composite composite = new Composite(tabFolder, SWT.NONE);
					tabItem.setControl(composite);
					{
						final CLabel label = new CLabel(composite, SWT.SHADOW_OUT);
						label.setBackground(new Color[] { constants.Colors._NodeEditColorGradStart,
								constants.Colors._NodeEditColorGradEnd }, new int[] { 100 });
						label.setBounds(0, 0, BNE_Width, 20);
						label.setText(LUP.get("nodeproperties"));
					}
					{
						final Group group = new Group(composite, SWT.NONE);
						group.setBounds(5, 20, 275, 55);
						{
							final Label label = new Label(group, SWT.NONE);
							label.setBounds(5, 10, 200, 15);
							label.setText(LUP.get("networkname"));
						}
						{
							textName = new Text(group, SWT.BORDER);
							textName.addModifyListener(new ModifyListener()
							{
								public void modifyText(ModifyEvent e)
								{
									shell.setText(LUP.get("edittitle") + ": " + textName.getText());
									Rename(textName.getText());
								}
							});
							textName.setText(bnode.getName());
							textName.setBounds(5, 25, 260, 20);
						}
					}
				}
			}
			if(bnode.getType() != BeliefNode.NODE_UTILITY)
			{
				final TabItem tabItem = new TabItem(tabFolder, SWT.NONE);
				tabItem.setText(LUP.get("domainprop"));
				{
					final Composite composite = new Composite(tabFolder, SWT.NONE);
					tabItem.setControl(composite);
					{
						labelCont = new Label(composite, SWT.NONE);
						labelCont.setBounds(5, 75, 200, 15);
						labelCont.setText(LUP.get("continuousvariable"));
						labelCont.setVisible(false);
					}
					{
						textVariable = new Text(composite, SWT.BORDER);
						textVariable.setBounds(5, 90, 165, 20);
						textVariable.setVisible(false);
					}
					{
						final CLabel label = new CLabel(composite, SWT.SHADOW_OUT);
						label.setBackground(new Color[] { constants.Colors._NodeEditColorGradStart,
								constants.Colors._NodeEditColorGradEnd }, new int[] { 100 });
						label.setBounds(0, 0, BNE_Width, 20);
						label.setText(LUP.get("domain"));
					}
					{
						groupDiscrete = new Group(composite, SWT.NONE);
						groupDiscrete.setText(LUP.get("discreteoutcomes"));
						groupDiscrete.setVisible(false);
						groupDiscrete.setBounds(5, 75, 275, 205);
						{
							listOutcomes = new List(groupDiscrete, SWT.BORDER | SWT.V_SCROLL);
							listOutcomes.setBounds(5, 20, 165, 65);
						}
						//125
						{
							listPossibleOutcomes = new List(groupDiscrete, SWT.BORDER | SWT.V_SCROLL | SWT.MULTI);
							listPossibleOutcomes.setBounds(5, 90, 165, 55);
							listPossibleOutcomes.add("True");
							listPossibleOutcomes.add("False");
							
							BeliefNode[] bnodes = bn.getNodes();
							for(int i = 0; i < bnodes.length; i++)
							{
								Domain D = bnodes[i].getDomain();
								for(int k = 0; k < D.getOrder(); k++)
								{
									String sD = D.getName(k);
									if(listPossibleOutcomes.indexOf(sD) < 0)
									{
										listPossibleOutcomes.add(sD);
									}
								}
							}
						}
						{
							final Button buttonUp = new Button(groupDiscrete, SWT.NONE);
							buttonUp.addSelectionListener(new SelectionAdapter()
							{
								public void widgetSelected(SelectionEvent e)
								{
									moveUp();
								}
							});
							buttonUp.setBounds(185, 25, 55, 22);
							buttonUp.setText(LUP.get("up"));
						}
						{
							final Button buttonDown = new Button(groupDiscrete, SWT.NONE);
							buttonDown.addSelectionListener(new SelectionAdapter()
							{
								public void widgetSelected(SelectionEvent e)
								{
									moveDown();
								}
							});
							buttonDown.setBounds(185, 50, 55, 22);
							buttonDown.setText(LUP.get("down"));
						}
						{
							final Label label = new Label(groupDiscrete, SWT.NONE);
							label.setBounds(10, 150, 70, 20);
							label.setText(LUP.get("newoutcome"));
						}
						{
							textNewOutcome = new Text(groupDiscrete, SWT.BORDER);
							textNewOutcome.setBounds(5, 170, 165, 25);
						}
						{
							final Button buttonAdd = new Button(groupDiscrete, SWT.NONE);
							buttonAdd.addSelectionListener(new SelectionAdapter()
							{
								public void widgetSelected(SelectionEvent e)
								{
									Add(textNewOutcome.getText());
									textNewOutcome.setText("");
								}
							});
							buttonAdd.setBounds(185, 170, 55, 25);
							buttonAdd.setText(LUP.get("add"));
						}
						{
							final Button buttonRemove = new Button(groupDiscrete, SWT.NONE);
							buttonRemove.addSelectionListener(new SelectionAdapter()
							{
								public void widgetSelected(SelectionEvent e)
								{
									Remove();
								}
							});
							buttonRemove.setBounds(185, 75, 55, 22);
							buttonRemove.setText(LUP.get("remove"));
						}
						{
							final Button buttonCopy = new Button(groupDiscrete, SWT.NONE);
							buttonCopy.addSelectionListener(new SelectionAdapter()
							{
								public void widgetSelected(SelectionEvent e)
								{
									for(int i = 0; i < listPossibleOutcomes.getSelection().length; i++)
									{
										Add(listPossibleOutcomes.getSelection()[i]);
									}
									listPossibleOutcomes.select(-1);
								}
							});
							buttonCopy.setBounds(185, 125, 55, 22);
							buttonCopy.setText(LUP.get("copyprevoutcome"));
						}						
					}
					
					{
						final Group group = new Group(composite, SWT.NONE);
						group.setEnabled(bnode.getType()== BeliefNode.NODE_CHANCE);
						group.setText(LUP.get("domaintype"));
						group.setBounds(5, 20, 275, 50);
						{
							buttonCont = new Button(group, SWT.RADIO);
							buttonCont.addSelectionListener(new SelectionAdapter()
							{
								public void widgetSelected(SelectionEvent e)
								{
									if (buttonCont.getSelection())
									{
										//System.out.println("selected
										// continous");
										groupDiscrete.setVisible(false);
										textVariable.setVisible(true);
										labelCont.setVisible(true);
									}
								}
							});
							buttonCont.setBounds(140, 20, 125, 20);
							buttonCont.setText(LUP.get("continuousradio"));
						}
						{
							buttonDiscrete = new Button(group, SWT.RADIO);
							buttonDiscrete.addSelectionListener(new SelectionAdapter()
							{
								public void widgetSelected(SelectionEvent e)
								{
									if (buttonDiscrete.getSelection())
									{
										//System.out.println("selected
										// discrete");
										groupDiscrete.setVisible(true);
										textVariable.setVisible(false);
										labelCont.setVisible(false);
									}
								}
							});
							buttonDiscrete.setBounds(10, 20, 125, 20);
							buttonDiscrete.setText(LUP.get("discreteradio"));
						}
					}
				}
			}
			{
				final CLabel label_1 = new CLabel(tabFolder, SWT.SHADOW_OUT);
				label_1.setText("label");
			}
		}
		if(bnode.getType() != BeliefNode.NODE_UTILITY)
		{
			if (bnode.getDomain() instanceof Discrete)
			{
				if(bnode.getType() == BeliefNode.NODE_CHANCE)
				buttonDiscrete.setSelection(true);
				groupDiscrete.setVisible(true);
				labelCont.setVisible(false);
				Discrete D = (Discrete) bnode.getDomain();
				textVariable.setVisible(false);
				for (int i = 0; i < D.getOrder(); i++)
				{
					listOutcomes.add(D.getName(i));
				}
			}
			else if (bnode.getDomain() instanceof Continuous)
			{
				Continuous C = (Continuous) bnode.getDomain();
				buttonCont.setSelection(true);
				groupDiscrete.setVisible(false);
				labelCont.setVisible(true);
				textVariable.setVisible(true);
			}
		}
		{
			final Button buttonApply = new Button(shell, SWT.NONE);
			buttonApply.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					Apply();
				}
			});
			buttonApply.setBounds(160, 315, 65, 25);
			buttonApply.setText(LUP.get("apply"));
		}
		{
			final Button buttonOk = new Button(shell, SWT.NONE);
			buttonOk.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					Apply();
					shell.close();
					shell.dispose();
				}
			});
			buttonOk.setBounds(230, 315, 60, 25);
			buttonOk.setText(LUP.get("accept"));
		}
		{
			final Button buttonCancel = new Button(shell, SWT.NONE);
			buttonCancel.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					Cancel();
					shell.close();
					shell.dispose();
				}
			});
			buttonCancel.setBounds(0, 315, 75, 25);
			buttonCancel.setText(LUP.get("cancel"));
		}
		shell.open();
		//shell.setBounds(shellParent.getLocation().x + shellParent.getSize().x
		// / 2 - BNE_Width / 2,shellParent.getLocation().y +
		// shellParent.getSize().y / 2 - BNE_Height / 2,BNE_Width,BNE_Height);
		//shell.setLocation(shellParent.getLocation().x +
		// shellParent.getSize().x / 2 - 105,shellParent.getLocation().y +
		// shellParent.getSize().y / 2 - 120);
		/*
		 * while (!shell.isDisposed ()) { if
		 * (!shellParent.getDisplay().readAndDispatch ())
		 * shellParent.getDisplay().sleep (); }
		 */
	}
}