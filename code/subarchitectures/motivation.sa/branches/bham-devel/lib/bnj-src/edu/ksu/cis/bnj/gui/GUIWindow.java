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
 */package edu.ksu.cis.bnj.gui;
import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.FileDialog;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.ProgressBar;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.widgets.ToolBar;
import org.eclipse.swt.widgets.ToolItem;
import org.eclipse.swt.layout.FillLayout;
import org.eclipse.swt.layout.FormLayout;
import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.Menu;
import org.eclipse.swt.widgets.MenuItem;
import edu.ksu.cis.bnj.gui.NetworkEdit;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.dynamic.DynamicTyping;
import edu.ksu.cis.bnj.ver3.dynamic.Filtering;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Iterator;
import edu.ksu.cis.bnj.ver3.plugin.IOPlugInLoader;
import edu.ksu.cis.bnj.ver3.streams.Exporter;
import edu.ksu.cis.bnj.ver3.streams.Importer;
import edu.ksu.cis.bnj.ver3.streams.OmniFormatV1_Reader;
import edu.ksu.cis.bnj.ver3.streams.OmniFormatV1_Writer;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.events.TraverseListener;
import org.eclipse.swt.events.TraverseEvent;
import org.eclipse.swt.events.VerifyListener;
import org.eclipse.swt.events.VerifyEvent;
import org.eclipse.swt.widgets.Slider;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.events.ControlAdapter;
import org.eclipse.swt.events.ControlEvent;
import org.eclipse.swt.widgets.Composite;
import org.eclipse.swt.widgets.TabFolder;
import org.eclipse.swt.widgets.TabItem;
import org.eclipse.swt.events.FocusAdapter;
import org.eclipse.swt.events.FocusEvent;
import org.eclipse.swt.events.ShellAdapter;
import org.eclipse.swt.events.ShellEvent;
import edu.ksu.cis.bnj.gui.NetworkRun;
import edu.ksu.cis.bnj.gui.dialogs.AboutDlg;
import edu.ksu.cis.bnj.gui.dialogs.FeedbackDlg;
import edu.ksu.cis.bnj.gui.tools.FocusRoute;
import edu.ksu.cis.bnj.gui.tools.IconWheel;
import edu.ksu.cis.bnj.gui.tools.LanguageUnicodeParser;
import edu.ksu.cis.bnj.gui.wizards.DynamicUnroll;
import edu.ksu.cis.util.GlobalOptions;
import org.eclipse.swt.events.DisposeListener;
import org.eclipse.swt.events.DisposeEvent;
import org.eclipse.swt.graphics.Image;
import org.eclipse.swt.graphics.Rectangle;
public class GUIWindow implements FocusRoute
{
	private static int Counter		= 0;
	
	static public synchronized int getCounter()
	{
		return Counter;
	}
	
	static public synchronized void incCounter()
	{
		Counter ++;
	}
	
	static public synchronized void decCounter()
	{
		Counter --;
	}
	
	static ArrayList _Windows;
	
	private Text					textSpotEdit;
	public BeliefNetwork			_bn;
	private String					_filename	= "";
	public Shell					_shell;
	private TabFolder				tabFolder;
	private int						manualFocus	= 0;
	private int						oldFocus = 0;
	private IconWheel				iconWheel;
	
	private Composite				networkeditOwner;
	private NetworkEdit				networkedit;
	private ToolBar					toolBarEdit;
	
	private Slider					sliderEditH;
	private Slider					sliderEditV;

	private Composite				networkrunOwner;
	private NetworkRun				networkrun;
	private ToolBar					toolBarRun;
	
	private Slider					sliderRunH;
	private Slider					sliderRunV;
	
	private Composite				networkvisOwner;
	private NetworkVisualize		networkvis;
	private ToolBar					toolBarVis;

	private Slider					sliderVisH;
	private Slider					sliderVisV;
	
	private TabItem					networkrunTab;
	private LanguageUnicodeParser	LUP;
	public GUIWindow()
	{
		LUP = LanguageUnicodeParser.getInstance();
	}
	int	toolBarHeight	= 48;
	int sliderBarDim = 16;
	public void createSliders()
	{
		
		sliderEditH = new Slider(networkeditOwner, SWT.NONE);
		sliderEditV = new Slider(networkeditOwner, SWT.VERTICAL);
		sliderEditH.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.RegisterSlide();
					}
				});		
		sliderEditV.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.RegisterSlide();
					}
				});
		
		
		sliderRunH = new Slider(networkrunOwner, SWT.NONE);
		sliderRunV = new Slider(networkrunOwner, SWT.VERTICAL);
		sliderRunH.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkrun.RegisterSlide();
					}
				});
		sliderRunV.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkrun.RegisterSlide();
					}
				});
		
		sliderVisH = new Slider(networkvisOwner, SWT.NONE);
		sliderVisV = new Slider(networkvisOwner, SWT.VERTICAL);
		sliderVisH.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.RegisterSlide();
					}
				});
		sliderVisV.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.RegisterSlide();
					}
				});			
		
	}
	public void maintainsize()
	{
		networkedit.setBounds(0, toolBarHeight, tabFolder.getClientArea().width - sliderBarDim,tabFolder.getClientArea().height - toolBarHeight - sliderBarDim);
 		networkrun.setBounds(0, toolBarHeight, tabFolder.getClientArea().width - sliderBarDim,tabFolder.getClientArea().height - toolBarHeight - sliderBarDim);
		networkvis.setBounds(0, toolBarHeight, tabFolder.getClientArea().width - sliderBarDim,tabFolder.getClientArea().height - toolBarHeight - sliderBarDim);
		networkedit.setFocusRoute(this);
		networkrun.setFocusRoute(this);
		networkvis.setFocusRoute(this);

		sliderEditH.setBounds(0, tabFolder.getClientArea().height - sliderBarDim, tabFolder.getClientArea().width - sliderBarDim, sliderBarDim);
		sliderEditV.setBounds(tabFolder.getClientArea().width - sliderBarDim,  toolBarHeight, sliderBarDim, tabFolder.getClientArea().height - toolBarHeight -  sliderBarDim);
		sliderRunH.setBounds(0, tabFolder.getClientArea().height - sliderBarDim, tabFolder.getClientArea().width - sliderBarDim, sliderBarDim);
		sliderRunV.setBounds(tabFolder.getClientArea().width - sliderBarDim,  toolBarHeight, sliderBarDim, tabFolder.getClientArea().height - toolBarHeight -  sliderBarDim);

		sliderVisH.setBounds(0, tabFolder.getClientArea().height - sliderBarDim, tabFolder.getClientArea().width - sliderBarDim, sliderBarDim);
		sliderVisV.setBounds(tabFolder.getClientArea().width - sliderBarDim,  toolBarHeight, sliderBarDim, tabFolder.getClientArea().height - toolBarHeight -  sliderBarDim);

		networkedit.SyncH(sliderEditH);
		networkedit.SyncV(sliderEditV);

		networkrun.SyncH(sliderRunH);
		networkrun.SyncV(sliderRunV);
		
		networkvis.SyncH(sliderVisH);
		networkvis.SyncV(sliderVisV);

		toolBarEdit.setBounds(0, 0, tabFolder.getClientArea().width, toolBarHeight);
		toolBarRun.setBounds(0, 0, tabFolder.getClientArea().width, toolBarHeight);
		toolBarVis.setBounds(0, 0, tabFolder.getClientArea().width, toolBarHeight);

		networkedit.redraw();
		networkrun.redraw();
		networkvis.redraw();
		
		doFocus();
	}
	
	public void redoFocus()
	{
		maintainsize();
	}
	
	public void makeNetworkEditor()
	{
		networkedit = new NetworkEdit(networkeditOwner, SWT.BORDER);
		networkedit.setBounds(25, 50, 210, 130);
		//networkedit.init(_bn);
		{
			textSpotEdit = new Text(networkedit, SWT.BORDER);
			textSpotEdit.addVerifyListener(new VerifyListener()
			{
				public void verifyText(VerifyEvent e)
				{
					e.doit = networkedit.verify(e.text);
				}
			});
			textSpotEdit.addTraverseListener(new TraverseListener()
			{
				public void keyTraversed(TraverseEvent e)
				{
					if (e.character == SWT.TAB)
					{
						if ((e.stateMask & SWT.SHIFT) > 0)
						{
							if ((e.stateMask & SWT.CTRL) > 0)
							{
								networkedit.upkey();
							}
							else
							{
								networkedit.shifttab();
							}
							e.doit = false;
						}
						else
						{
							if ((e.stateMask & SWT.CTRL) > 0)
							{
								networkedit.downkey();
							}
							else
							{
								networkedit.tab();
							}
							e.doit = false;
						}
					}
					else if (e.detail == SWT.TRAVERSE_TAB_NEXT)
					{
						networkedit.tab();
						e.doit = false;
					}
					else if (e.detail == SWT.TRAVERSE_TAB_PREVIOUS)
					{
						networkedit.shifttab();
						e.doit = false;
					}
					if (e.keyCode == SWT.DOWN)
					{
						System.out.println("down?");
					}
					/*
					 * if(e.detail = SWT.TRAVERSE_TAB_NEXT) if((e.character &
					 * SWT.DOWN) > 0) { System.out.println("down?"); }
					 */
				}
			});
			textSpotEdit.setBounds(5, 5, 25, 15);
			networkedit.setSpotEdit(textSpotEdit);
		}
	}
	
	public void doFocus()
	{
		if      (manualFocus == 0) networkedit.setFocus();
		else if (manualFocus == 1) networkrun.setFocus();
		else if (manualFocus == 2) networkvis.setFocus();
	}
	
	public void forceRedraw()
	{
		if      (manualFocus == 0) networkedit.redraw();
		else if (manualFocus == 1) networkrun.redraw();
		else if (manualFocus == 2) networkvis.redraw();
	}
	
	
	public void New(boolean initThis)
	{
		_bn = new BeliefNetwork();
		if (initThis)
		{
			_filename = "";
			manualFocus = 0;
			doFocus();
			Rebuild(_bn);
		}
		else
		{
			GUIWindow window = new GUIWindow();
			GUIWindow._Windows.add(window);
			window.open(_bn);
		}
	}
	
	public void Rebuild(BeliefNetwork bn)
	{
		tabFolder.setSelection(0);
		networkedit.ReInit();
		networkedit.init(bn);
		networkrun.SetNetwork(networkedit.GetRenderNetwork());
		networkvis.SetNetwork(networkedit.GetRenderNetwork());
		networkrun.SetTransformation(networkedit.getTransformation());
		networkedit.redraw();
		networkedit.setSpotEdit(textSpotEdit);
	}
	
	private void Open(Importer I, FileInputStream FIS)
	{
		OmniFormatV1_Reader ofv1w = new OmniFormatV1_Reader();
		I.load(FIS, ofv1w);
		_bn = ofv1w.GetBeliefNetwork(0);
		if (_filename.equals(""))
		{
			Rebuild(_bn);
			
			doFocus();
			if (manualFocus == 1)
			{
				networkrun.Compile();
			}
		}
		else
		{
			GUIWindow window = new GUIWindow();
			GUIWindow._Windows.add(window);
			window.open(_bn);
		}		
	}
	
	public void ForceOpen(BeliefNetwork network)
	{
		GUIWindow window = new GUIWindow();
		GUIWindow._Windows.add(window);
		window.open(network);
	}
	
	public void Open(String f)
	{
		try
		{
			FileInputStream FIS = new FileInputStream(f);
			IOPlugInLoader pil = IOPlugInLoader.getInstance();
			Importer IMP = pil.GetImporterByExt(pil.GetExt(f));
			Open(IMP,FIS);
			_filename = f;
		}
		catch (FileNotFoundException fnfe)
		{
			System.out.println("fnf");
		}
	}
	public void Open()
	{
		FileDialog dialog = new FileDialog(_shell, SWT.OPEN);
		
		IOPlugInLoader pil = IOPlugInLoader.getInstance();
		ArrayList importers = pil.getImporters();
		String[] filtersNames = new String[importers.size()+1];
		String[] filtersExt = new String[importers.size()+1];
		String comp = "";
		for(int xxx = 0; xxx < filtersNames.length-1; xxx++)
		{
			filtersExt[xxx+1] = ((Importer)importers.get(xxx)).getExt();
			if(xxx!=0)
			{
				comp+=";";
			}
			comp+=filtersExt[xxx+1];
			filtersNames[xxx+1] = ((Importer)importers.get(xxx)).getDesc() + "(" + filtersExt[xxx+1] + ")";
		}
		filtersNames[0] = "All Network Files" + "(" + comp + ")";
		filtersExt[0] = comp;
		
		dialog.setFilterNames(filtersNames);
		dialog.setFilterExtensions(filtersExt);
		//dialog.setFilterNames(new String[] { "XML Bif Files (*.xml)", "All Files (*.*)" });
		//dialog.setFilterExtensions(new String[] { "*.xml;*.dll", "*.*" }); //Windows
		// cards
		dialog.setFilterPath("."); //Windows path
		String f = dialog.open();
		if (f != null && f != "")
		{
			Open(f);
		}
	}
	void CleanUp()
	{
		_shell.close();
		GlobalOptions.getInstance().save("settings.ini");
	}
	public void Exit()
	{
		ArrayList _lD = new ArrayList();
		for(Iterator it = GUIWindow._Windows.iterator(); it.hasNext();)
		{
			_lD.add(it.next());
		}
		for(Iterator it = _lD.iterator(); it.hasNext();)
		{
			GUIWindow gwin = (GUIWindow) it.next();
			gwin.AppClose();
		}
	}
	
	public void AppClose()
	{
		// present with dialog here
		GUIWindow._Windows.remove(this);
		CleanUp();
		Runtime.getRuntime().gc();
	}
	
	public void Save()
	{
		if (_filename == "")
		{
			SaveAs();
			return;
		}
		try
		{
			IOPlugInLoader pil = IOPlugInLoader.getInstance();
			Exporter EXP = pil.GetExportersByExt(pil.GetExt(_filename));
			EXP.save(new FileOutputStream(_filename));
			OmniFormatV1_Writer.Write( networkedit.GetRenderNetwork()._bn, EXP.getStream1());
		}
		catch (FileNotFoundException fnfe)
		{
			System.out.println("unable to save");
		}
	}
	
	public void SaveAs()
	{
		FileDialog dialog = new FileDialog(_shell, SWT.SAVE);
		IOPlugInLoader pil = IOPlugInLoader.getInstance();
		ArrayList exporters = pil.getExporters();
		String[] filtersNames = new String[exporters.size()];
		String[] filtersExt = new String[exporters.size()];
		for(int xxx = 0; xxx < filtersNames.length; xxx++)
		{
			filtersExt[xxx] = ((Exporter)exporters.get(xxx)).getExt();
			filtersNames[xxx] = ((Exporter)exporters.get(xxx)).getDesc() + "(" + filtersExt[xxx] + ")";
		}
		dialog.setFilterNames(filtersNames);
		dialog.setFilterExtensions(filtersExt);
		dialog.setFilterPath(".");
		String f = dialog.open();
		if (f != null && f != "")
		{
			_filename = f;
			Save();
		}
	}
	public void Print()
	{}
	
	/*
	 * 	final Display display = new Display();
	final int [] count = new int [] {4};
	final Image image = new Image(display, 300, 300);
	final Shell splash = new Shell(SWT.ON_TOP);
	final ProgressBar bar = new ProgressBar(splash, SWT.NONE);
	bar.setMaximum(count[0]);
	Label label = new Label(splash, SWT.NONE);
	label.setImage(image);
	FormLayout layout = new FormLayout();
	splash.setLayout(layout);
	FormData labelData = new FormData ();
	labelData.right = new FormAttachment (100, 0);
	labelData.bottom = new FormAttachment (100, 0);
	label.setLayoutData(labelData);
	FormData progressData = new FormData ();
	progressData.left = new FormAttachment (0, 5);
	progressData.right = new FormAttachment (100, -5);
	progressData.bottom = new FormAttachment (100, -5);
	bar.setLayoutData(progressData);
	splash.pack();
	Rectangle splashRect = splash.getBounds();
	Rectangle displayRect = display.getBounds();
	int x = (displayRect.width - splashRect.width) / 2;
	int y = (displayRect.height - splashRect.height) / 2;
	splash.setLocation(x, y);
	splash.open();
	
	 */
	public static void main(String[] args)
	{
		final Display display = new Display();
		final Shell splash = new Shell(SWT.ON_TOP);
		final Image image = new Image(display, "imgs/splash.gif");
		
		Label label = new Label(splash, SWT.NONE);
		label.setImage(image);
		
		final ProgressBar bar = new ProgressBar(splash, SWT.NONE);
		bar.setMaximum(100);
		label.setImage(image);

		FormLayout layout = new FormLayout();
		splash.setLayout(layout);
		/*
		FormData labelData = new FormData ();
		labelData.right = new FormAttachment (100, 0);
		labelData.bottom = new FormAttachment (100, 0);
		label.setLayoutData(labelData);
		
		FormData progressData = new FormData ();
		progressData.left = new FormAttachment (0, 5);
		progressData.right = new FormAttachment (100, -5);
		progressData.bottom = new FormAttachment (100, -5);
		bar.setLayoutData(progressData);
		*/
		splash.pack();
		Rectangle splashRect = splash.getBounds();
		Rectangle displayRect = display.getBounds();
		int x = (displayRect.width - splashRect.width) / 2;
		int y = (displayRect.height - splashRect.height) / 2;
		splash.setLocation(x, y);
		splash.open();
		
		
		_Windows = new ArrayList();
		//String filename = "asia.xml";
		GUIWindow window = new GUIWindow();
		_Windows.add(window);
		
		
		String filename = "";
		for (int i = 0; i < args.length; i++)
		{
			filename = args[i];
		}
		//LanguageUnicodeParser.getInstance().parse("russian.ini");
		GlobalOptions GO = GlobalOptions.getInstance();
		String lang = GO.getString("language", "default.ini");
		if (!lang.equals("default.ini"))
		{
			LanguageUnicodeParser.getInstance().parse(lang);
		}
		BeliefNetwork bn = null;

		if (!filename.equals(""))
		{
			window.Open(filename);
		}

		try	{
			IOPlugInLoader IOPL = IOPlugInLoader.getInstance();
			window.open(bn);
			Thread.sleep(1500);
			display.close();
			while(getCounter() > 0)
			{
				try
				{
					Thread.sleep(10);
				}
				catch (Exception e)
				{
					
				}
			}
			
			
		}
		catch (Exception e)
		{
			String message = "";
			StackTraceElement[] ste = e.getStackTrace();
			message += e.getMessage() + "\n\n";
			message += e + "\n";
			for (int i = 0; i < ste.length; i++)
			{
				message += ste[i].getFileName() + "[" + ste[i].getLineNumber() + "] ~ " + ste[i].getClassName() + "::"
						+ ste[i].getMethodName() + "\n";
				//				System.out.println(e.getMessage());
			}
			System.out.println(message);
			FeedbackDlg.Send(message);
		}
		
	}
	/*
	 * ToolBar bar = new ToolBar (shell, SWT.BORDER); for (int i=0; i <8; i++) {
	 * ToolItem item = new ToolItem (bar, SWT.PUSH); item.setText ("Item " + i); }
	 * bar.pack ();
	 */
	
	
	public Display display;
	public void open(BeliefNetwork bn)
	{
		GUIWindow.incCounter();
		GUIThread GT = new GUIThread(this, bn);
		GT.start();
	}
	
	GUIWindow _Host;
	public void run(BeliefNetwork bn)
	{
		_Host = this;
		display = new Display();
		toolBarHeight = GlobalOptions.getInstance().getInteger("toolbar_height", 48);
		final Shell shell = new Shell(display, SWT.RESIZE | SWT.MIN | SWT.MAX | SWT.MODELESS);
		shell.addDisposeListener(new DisposeListener()
		{
			public void widgetDisposed(DisposeEvent e)
			{
				AppClose();
			}
		});
		shell.setLayout(new FillLayout());
		_shell = shell;
		iconWheel = new IconWheel();
		iconWheel.create(display);
		shell.setImage(iconWheel.Ico16);
		shell.setText(LUP.get("title"));
		{
			Menu menubar = new Menu(shell, SWT.BAR);
			shell.setMenuBar(menubar);
			{
				final MenuItem menuFile = new MenuItem(menubar, SWT.CASCADE);
				menuFile.setText(LUP.get("file"));
				{
					Menu popupmenu = new Menu(menuFile);
					menuFile.setMenu(popupmenu);
					{
						final MenuItem menuFileNew = new MenuItem(popupmenu, SWT.NONE);
						menuFileNew.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								New(false);
							}
						});
						menuFileNew.setText(LUP.get("new"));
					}
					{
						final MenuItem menuFileOpen = new MenuItem(popupmenu, SWT.NONE);
						menuFileOpen.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								Open();
							}
						});
						menuFileOpen.setText(LUP.get("open") + " ...");
					}
					{
						final MenuItem menuFileClose = new MenuItem(popupmenu, SWT.NONE);
						menuFileClose.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								AppClose();
							}
						});
						menuFileClose.setText(LUP.get("close"));
					}
					{
						new MenuItem(popupmenu, SWT.SEPARATOR);
					}
					{
						final MenuItem menuFileSave = new MenuItem(popupmenu, SWT.NONE);
						menuFileSave.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								Save();
							}
						});
						menuFileSave.setText(LUP.get("save"));
					}
					{
						final MenuItem menuFileSaveAs = new MenuItem(popupmenu, SWT.NONE);
						menuFileSaveAs.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								SaveAs();
							}
						});
						menuFileSaveAs.setText(LUP.get("saveas") + " ...");
					}
					{
						final MenuItem menuItem = new MenuItem(popupmenu, SWT.SEPARATOR);
						menuItem.setText("Menu Item");
					}
					{
						final MenuItem menuExit = new MenuItem(popupmenu, SWT.NONE);
						menuExit.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								Exit();
							}
						});
						menuExit.setText(LUP.get("exit"));
					}
				}
			}
			{
				final MenuItem menuEdit = new MenuItem(menubar, SWT.CASCADE);
				menuEdit.setText(LUP.get("edit"));
				{
					Menu popupmenu = new Menu(menuEdit);
					menuEdit.setMenu(popupmenu);
					{
						final MenuItem menuEditUndo = new MenuItem(popupmenu, SWT.NONE);
						menuEditUndo.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								networkedit.undo();
							}
						});
						menuEditUndo.setText(LUP.get("undo"));
						menuEditUndo.setAccelerator(SWT.CTRL + 'Z');
					}
				}
			}
			{
				final MenuItem menuView = new MenuItem(menubar, SWT.CASCADE);
				menuView.setText(LUP.get("view"));
				{
					Menu popupmenu = new Menu(menuView);
					menuView.setMenu(popupmenu);
					{
						final MenuItem menuViewCenterNetwork = new MenuItem(popupmenu, SWT.NONE);
						menuViewCenterNetwork.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								if (manualFocus == 0) networkedit.mapCenter();
								else if (manualFocus == 1) networkrun.mapCenter();
								else if (manualFocus == 2) networkvis.mapCenter();
							}
						});
						menuViewCenterNetwork.setText(LUP.get("centernet"));
					}
					{
						final MenuItem menuViewZoom2Fit = new MenuItem(popupmenu, SWT.NONE);
						menuViewZoom2Fit.setText(LUP.get("fitnet"));
						menuViewZoom2Fit.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								if (manualFocus == 0) networkedit.mapZoom2Fit();
								else if (manualFocus == 1) networkrun.mapZoom2Fit();
								else if (manualFocus == 2) networkvis.mapZoom2Fit();
							}
						});
					}
					{
						new MenuItem(popupmenu, SWT.SEPARATOR);
					}
					{
						final MenuItem menuItemZoomIn = new MenuItem(popupmenu, SWT.NONE);
						menuItemZoomIn.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								if (manualFocus == 0) networkedit.mapZoomIn();
								else if (manualFocus == 1) networkrun.mapZoomIn();
								else if (manualFocus == 2) networkvis.mapZoomIn();
							}
						});
						menuItemZoomIn.setText(LUP.get("zoomin"));
					}
					{
						final MenuItem menuItemZoomOut = new MenuItem(popupmenu, SWT.NONE);
						menuItemZoomOut.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								if (manualFocus == 0) networkedit.mapZoomOut();
								else if (manualFocus == 1) networkrun.mapZoomOut();
								else if (manualFocus == 2) networkvis.mapZoomOut();								
							}
						});
						menuItemZoomOut.setText(LUP.get("zoomout"));
					}
					{
						final MenuItem menuItemZoom100 = new MenuItem(popupmenu, SWT.NONE);
						menuItemZoom100.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								if (manualFocus == 0) networkedit.mapZoom100();
								else if (manualFocus == 1) networkrun.mapZoom100();
								else if (manualFocus == 2) networkvis.mapZoom100();										
							}
						});
						menuItemZoom100.setText(LUP.get("zoom100"));
					}
				}
			}
			{
				final MenuItem menuWizards = new MenuItem(menubar, SWT.CASCADE);
				menuWizards.setText(LUP.get("wizards"));
				{
					Menu popupmenu = new Menu(menuWizards);
					menuWizards.setMenu(popupmenu);
					{
						final MenuItem menuItem = new MenuItem(popupmenu, SWT.NONE);
						
						menuItem.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								//(new FeedbackDlg()).show(shell);
								(new DynamicUnroll(_Host, _bn)).show(shell);
							}
						});
						menuItem.setText(LUP.get("unrollwiz"));
					}
					{
						final MenuItem menuItem = new MenuItem(popupmenu, SWT.NONE);
						
						menuItem.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
							    if(DynamicTyping.IsDynamic(networkedit.GetRenderNetwork()._bn,true))
							    {
							        ForceOpen(Filtering.BuildFilterNetwork(networkedit.GetRenderNetwork()._bn));
							    }
							    else
							    {
							        System.out.println("Error: Need a v3.2 DBN");
							    }
							}
						});
						menuItem.setText(LUP.get("totransitionnet"));
					}

				
				}
			}			
			{
				final MenuItem menuAbout = new MenuItem(menubar, SWT.CASCADE);
				menuAbout.setText(LUP.get("about"));
				{
					Menu popupmenu = new Menu(menuAbout);
					menuAbout.setMenu(popupmenu);
					{
						final MenuItem menuItem = new MenuItem(popupmenu, SWT.NONE);
						menuItem.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								(new FeedbackDlg()).show(shell);
							}
						});
						menuItem.setText(LUP.get("submitfeedback"));
					}
					{
						final MenuItem menuAboutAbout = new MenuItem(popupmenu, SWT.NONE);
						menuAboutAbout.addSelectionListener(new SelectionAdapter()
						{
							public void widgetSelected(SelectionEvent e)
							{
								(new AboutDlg()).show(shell);
							}
						});
						menuAboutAbout.setText(LUP.get("about"));
					}
				}
			}
		}
		{
			tabFolder = new TabFolder(shell, SWT.NONE);
			{
				final TabItem tabEdit = new TabItem(tabFolder, SWT.NONE);
				tabEdit.setText(LUP.get("modeedit"));
				{
					networkeditOwner = new Composite(tabFolder, SWT.NONE);
					tabEdit.setControl(networkeditOwner);
					{
						makeNetworkEditor();
					}
					makeEditToolBar();
					networkeditOwner.addControlListener(new ControlAdapter()
					{
						public void controlResized(ControlEvent e)
						{
							maintainsize();
						}
					});
				}
			}
			tabFolder.addControlListener(new ControlAdapter()
			{
				public void controlResized(ControlEvent e)
				{
					maintainsize();
				}
			});
			tabFolder.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					//System.out.println("focus changed:" + );
					if (((TabItem) e.item).getText().equals(LUP.get("modeedit")))
					{
						manualFocus = 0;
						networkedit.Enter();
					}
					if (((TabItem) e.item).getText().equals(LUP.get("moderun")))
					{
						manualFocus = 1;
						networkrun.Compile();
					}
					else
					{
						manualFocus = 2;
						networkvis.Stop();
					}
					doFocus();
				}
			});
			tabFolder.addFocusListener(new FocusAdapter()
			{
				public void focusGained(FocusEvent e)
				{
					doFocus();
				}
			});
			{
				makeRunToolBar();
			}
			{
				makeVisToolBar();
				networkrun.SetNetwork(networkedit.GetRenderNetwork());
				networkvis.SetNetwork(networkedit.GetRenderNetwork());
				networkrun.SetTransformation(networkedit.getTransformation());
				
			}
		}
		shell.addControlListener(new ControlAdapter()
		{
			public void controlResized(ControlEvent e)
			{
				maintainsize();
			}
			public void controlMoved(ControlEvent e)
			{
				maintainsize();
			}
		});
		shell.addShellListener(new ShellAdapter()
		{
			public void shellDeactivated(ShellEvent e)
			{
				//oldFocus = manualFocus;
				//manualFocus = -1;
			}
			public void shellActivated(ShellEvent e)
			{
				//manualFocus = oldFocus;
				maintainsize();
			}
		});
		shell.addFocusListener(new FocusAdapter()
		{
			public void focusLost(FocusEvent e)
			{
				oldFocus = manualFocus;
				manualFocus = -1;
			}
			
			public void focusGained(FocusEvent e)
			{
				manualFocus = oldFocus;
				doFocus();
			}
			
		});

		createSliders();
		//New();
		if (bn == null)
		{
			New(true);
		}
		else
		{
			_bn = bn;
			networkedit.init(_bn);
			networkedit.redraw();
			doFocus();
			if (manualFocus == 1)
			{
				networkrun.Compile();
			}
		}
		shell.open();
		//run();
try
{
	
	/*
			display.timerExec(50, new Runnable()
			{
				public void run()
				{
					Refresh();
				}
			});	
			*/
	// install the hook
		while (!_shell.isDisposed())
		{
			if (!display.readAndDispatch()) display.sleep();
		}
		
	}
	catch (Exception e)
	{
		String message = "";
		StackTraceElement[] ste = e.getStackTrace();
		message += e.getMessage() + "\n\n";
		message += e + "\n";
		for (int i = 0; i < ste.length; i++)
		{
			message += ste[i].getFileName() + "[" + ste[i].getLineNumber() + "] ~ " + ste[i].getClassName() + "::"
					+ ste[i].getMethodName() + "\n";
			//				System.out.println(e.getMessage());
		}
		
		System.out.println(message);
		FeedbackDlg.Send(message);
	}
		
		GUIWindow.decCounter();
	}

	/*
	public void gRefresh()
	{
		for(Iterator it = GUIWindow._Windows.iterator() ; it.hasNext(); )
		{
			GUIWindow gwin = (GUIWindow) it.next();
			gwin.forceRedraw();
			
		}
	}
	public void Refresh()
	{
		gRefresh();
		display.timerExec(50, new Runnable()
				{
					public void run()
					{
						Refresh();
					}
				});	
	}
	*/
	
	void makeRunToolBar()
	{
		final TabItem tabRun = new TabItem(tabFolder, SWT.NONE);
		tabRun.setText(LUP.get("moderun"));
		{
			networkrunOwner = new Composite(tabFolder, SWT.NONE);
			tabRun.setControl(networkrunOwner);
			networkrun = new NetworkRun(networkrunOwner, SWT.BORDER);
			networkrun.setBounds(25, 50, 210, 130);
			
			toolBarRun = new ToolBar(networkrunOwner, SWT.NONE);
			toolBarRun.setBounds(0, 0, 635, toolBarHeight);
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkrun.mapCenter();
					}
				});
				toolItem.setImage(iconWheel.Center);
				toolItem.setText(LUP.get("tool_center"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkrun.mapZoom2Fit();
					}
				});
				toolItem.setImage(iconWheel.Zoom2Fit);
				toolItem.setText(LUP.get("tool_zoom2fit"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkrun.mapZoomIn();
					}
				});
				toolItem.setImage(iconWheel.ZoomIn);
				toolItem.setText(LUP.get("tool_zoomin"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkrun.mapZoomOut();
					}
				});
				toolItem.setImage(iconWheel.ZoomOut);
				toolItem.setText(LUP.get("tool_zoomout"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkrun.mapZoom100();
					}
				});
				toolItem.setImage(iconWheel.Zoom100);
				toolItem.setText(LUP.get("tool_zoom100"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.SEPARATOR);
			}					
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkrun.mapStart();
					}
				});
				toolItem.setImage(iconWheel.RunBegin);
				toolItem.setText(LUP.get("tool_begin_sample"));
			}
			/*
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkrun.mapRandom();
					}
				});
				toolItem.setImage(iconWheel.Center);
				toolItem.setText(LUP.get("tool_random_sample"));
			}
			*/
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkrun.mapClearEvidence();
					}
				});
				toolItem.setImage(iconWheel.RunClear);
				toolItem.setText(LUP.get("tool_clearevidence"));
			}
			//tool_clearevidence
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkrun.mapSnapshot();
					}
				});
				toolItem.setImage(iconWheel.RunSnapshot);
				toolItem.setText(LUP.get("tool_snapshot"));
			}					
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						if(networkrun.canSaveEvidence())
						{
							FileDialog dialog = new FileDialog(_shell, SWT.SAVE);
							dialog.setFilterNames(new String[] { "XML Evidence  (*.xmlevidence)", "All Files (*.*)" });
							dialog.setFilterExtensions(new String[] { "*.xmlevidence", "*.*" }); //Windows
							dialog.setFilterPath("."); //Windows path
							String f = dialog.open();
							if (f != null && f != "")
							{
								networkrun.mapSave(f);
							}
						}
					}
				});
				toolItem.setImage(iconWheel.RunSave);
				toolItem.setText(LUP.get("tool_save_evidence"));
			}

			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.SEPARATOR);
			}	
			{
				final ToolItem toolItem = new ToolItem(toolBarRun, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
					    // Filter
					    networkrun.mapFilter();

					}
				});
				toolItem.setImage(iconWheel.RunFilter);
				toolItem.setText(LUP.get("tool_filter"));
			}

			
		}
		
	}
	
	void makeEditToolBar()
	{
		{
			toolBarEdit = new ToolBar(networkeditOwner, SWT.NONE);
			toolBarEdit.setBounds(0, 0, 635, toolBarHeight);
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.NONE);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						Save();
					}
				});
				//toolItem.setImage();
				toolItem.setImage(iconWheel.Save);
				toolItem.setText(LUP.get("tool_save"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.SEPARATOR);
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.NONE);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapNewNode();
					}
				});
				//toolItem.setImage();
				toolItem.setImage(iconWheel.NodeAdd);
				toolItem.setText(LUP.get("tool_addnode"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.NONE);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapNewDescNode();
					}
				});
				//toolItem.setImage();
				toolItem.setImage(iconWheel.DescAdd);
				toolItem.setText(LUP.get("tool_adddescnode"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.NONE);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapNewUtilNode();
					}
				});
				//toolItem.setImage();
				toolItem.setImage(iconWheel.UtilAdd);
				toolItem.setText(LUP.get("tool_addutilnode"));
			}			
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapNewEdge();
					}
				});
				toolItem.setImage(iconWheel.EdgeAdd);
				toolItem.setText(LUP.get("tool_addedge"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapEdgeDelete();
					}
				});
				toolItem.setImage(iconWheel.EdgeDel);
				toolItem.setText(LUP.get("tool_deleteedge"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapDeleteNode();
					}
				});
				toolItem.setImage(iconWheel.NodeDel);
				toolItem.setText(LUP.get("tool_deletenode"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.SEPARATOR);
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapCenter();
					}
				});
				toolItem.setImage(iconWheel.Center);
				toolItem.setText(LUP.get("tool_center"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapZoom2Fit();
					}
				});
				toolItem.setImage(iconWheel.Zoom2Fit);
				toolItem.setText(LUP.get("tool_zoom2fit"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapZoomIn();
					}
				});
				toolItem.setImage(iconWheel.ZoomIn);
				toolItem.setText(LUP.get("tool_zoomin"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapZoomOut();
					}
				});
				toolItem.setImage(iconWheel.ZoomOut);
				toolItem.setText(LUP.get("tool_zoomout"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapZoom100();
					}
				});
				toolItem.setImage(iconWheel.Zoom100);
				toolItem.setText(LUP.get("tool_zoom100"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.SEPARATOR);
			}
			// \todo add when chris finiishes
			/*
			{
				final ToolItem toolItem = new ToolItem(toolBarEdit, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkedit.mapCutset();
					}
				});
				toolItem.setImage(iconWheel.Cutset);
				toolItem.setText(LUP.get("tool_cutset"));
			}
			*/
		}
		
	}
	
	/***
	 */
	
	void makeVisToolBar()
	{
		final TabItem tabVis = new TabItem(tabFolder, SWT.NONE);
		tabVis.setText(LUP.get("modevis"));
		{
			networkvisOwner = new Composite(tabFolder, SWT.NONE);
			tabVis.setControl(networkvisOwner);
			networkvis = new NetworkVisualize(networkvisOwner, SWT.BORDER);
			
			networkvis.setBounds(25, 50, 210, 130);
			toolBarVis = new ToolBar(networkvisOwner, SWT.NONE);
			toolBarVis.setBounds(0, 0, 635, toolBarHeight);
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.Stop();
					}
				});
				toolItem.setImage(iconWheel.VisStop);
				toolItem.setText(LUP.get("stop"));
			}
			toolBarVis.setBounds(0, 0, 635, toolBarHeight);
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.SkipBack();
					}
				});
				toolItem.setImage(iconWheel.VisSkipBack);
				toolItem.setText(LUP.get("skipback"));
			}
			toolBarVis.setBounds(0, 0, 635, toolBarHeight);
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.StepBack();
					}
				});
				toolItem.setImage(iconWheel.VisStepBack);
				toolItem.setText(LUP.get("stepback"));
			}			

			toolBarVis.setBounds(0, 0, 635, toolBarHeight);
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.Back();
					}
				});
				toolItem.setImage(iconWheel.VisBack);
				toolItem.setText(LUP.get("back"));
			}
			toolBarVis.setBounds(0, 0, 635, toolBarHeight);
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.Forward();
					}
				});
				toolItem.setImage(iconWheel.VisForward);
				toolItem.setText(LUP.get("forward"));
			}			
			toolBarVis.setBounds(0, 0, 635, toolBarHeight);
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.StepForward();
					}
				});
				toolItem.setImage(iconWheel.VisStepForward);
				toolItem.setText(LUP.get("stepforward"));
			}			
			toolBarVis.setBounds(0, 0, 635, toolBarHeight);
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.Pause();
					}
				});
				toolItem.setImage(iconWheel.VisPause);
				toolItem.setText(LUP.get("pause"));
			}
			toolBarVis.setBounds(0, 0, 635, toolBarHeight);
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.SkipForward();
					}
				});
				toolItem.setImage(iconWheel.VisSkipForward);
				toolItem.setText(LUP.get("skipforward"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.SEPARATOR);
			}
			
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.mapCenter();
					}
				});
				toolItem.setImage(iconWheel.Center);
				toolItem.setText(LUP.get("tool_center"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.mapZoom2Fit();
					}
				});
				toolItem.setImage(iconWheel.Zoom2Fit);
				toolItem.setText(LUP.get("tool_zoom2fit"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.mapZoomIn();
					}
				});
				toolItem.setImage(iconWheel.ZoomIn);
				toolItem.setText(LUP.get("tool_zoomin"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.mapZoomOut();
					}
				});
				toolItem.setImage(iconWheel.ZoomOut);
				toolItem.setText(LUP.get("tool_zoomout"));
			}
			{
				final ToolItem toolItem = new ToolItem(toolBarVis, SWT.PUSH);
				toolItem.addSelectionListener(new SelectionAdapter()
				{
					public void widgetSelected(SelectionEvent e)
					{
						networkvis.mapZoom100();
					}
				});
				toolItem.setImage(iconWheel.Zoom100);
				toolItem.setText(LUP.get("tool_zoom100"));
			}

		}
		
	}
}