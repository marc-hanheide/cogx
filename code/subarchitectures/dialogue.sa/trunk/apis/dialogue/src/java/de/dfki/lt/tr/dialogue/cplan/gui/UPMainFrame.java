package de.dfki.lt.tr.dialogue.cplan.gui;


import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Window;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.InputEvent;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.Semaphore;

import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.JTextArea;
import javax.swing.JToolBar;
import javax.swing.KeyStroke;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;
import javax.swing.text.BadLocationException;

import de.dfki.lt.loot.gui.DrawingPanel;
import de.dfki.lt.loot.gui.adapters.EmptyModelAdapter;
import de.dfki.lt.loot.gui.adapters.ModelAdapter;
import de.dfki.lt.tr.dialogue.cplan.DagNode;
import de.dfki.lt.tr.dialogue.cplan.InteractivePlanner;
import de.dfki.lt.tr.dialogue.cplan.LoggingTracer;
import de.dfki.lt.tr.dialogue.cplan.RuleTracer;
import de.dfki.lt.tr.dialogue.cplan.util.Position;

/**
 * <code>UPMainFrame</code> defines the main window of the content planner
 * interactive GUI
 *
 * @author Bernd Kiefer, DFKI
 * @version
 */
@SuppressWarnings("serial")
public class UPMainFrame extends JFrame implements ActionListener {

  /** This contains the currently open frames. */
  private static List<UPMainFrame> openFrames = new ArrayList<UPMainFrame>();
  /** This Listener is called when the last Frame has been closed */
  private static List<CloseAllListener> clAll =
    new LinkedList<CloseAllListener>();

  private static final String _defaultFont = "Monospace";

  private static final Color NORMAL_COLOR = Color.white;
  private static final Color ERROR_COLOR = new Color(255, 150, 150);

  private static final int LOAD_BTN = 0;
  private static final int PROCESS_BTN = LOAD_BTN + 1;
  private static final int TRACE_BTN = PROCESS_BTN + 1;
  private static final int STEP_BTN = TRACE_BTN + 1;
  private static final int CONTINUE_BTN = STEP_BTN + 1;
  private static final int CLEAR_BTN = CONTINUE_BTN + 1;
  private static final int REALIZE_BTN = CLEAR_BTN + 1;
  // private static final int NR_BTN = REALIZE_BTN + 1;


  private final Object[][] actionSpecs = {
    {"Load", "edit-redo", "Reload Rules", "Reload Rules",
      new Runnable() {
      public void run() {
        if (_projectFile == null) {
          JOptionPane.showMessageDialog(UPMainFrame.this, "Error",
              "No Rule File Loaded", JOptionPane.ERROR_MESSAGE);
        }
        else {
          readProjectFile();
        }
      }
    }
    },
    {"Process", "gnome-run", "Process Input", "Process",
      new Runnable() { public void run() { processInput(); } }
    },
    {"Trace", "go-next", "Trace Processing", "Trace",
      new Runnable() {
      public void run() {
        _tracer = new GuiTracer(UPMainFrame.this,
            RuleTracer.DISPLAY_MODIFICATION);
        _stepModeSemaphore = null;
        processInput();
      }
    }
    },
    {"Step", "go-jump", "Next Trace Step", "Step",
      new Runnable() {
      public void run() {
        if (_stepModeSemaphore != null) _stepModeSemaphore.release();
      }
    }
    },
    {"Continue", "go-last", "Continue until end", "Continue",
      new Runnable() {
      public void run() {
        _tracer.setTracing(0);
        if (_stepModeSemaphore != null) _stepModeSemaphore.release();
      }
    }
    },
    {"Clear", "edit-clear", "Clear Input", "Clear",
      new Runnable() {
      public void run() {
        _inputArea.setBackground(NORMAL_COLOR);
        _inputArea.setText("");
      }
    }
    },
    {"Realize", "insert-text", "Realize output", "Realize",
      new Runnable() {
      public void run() {
        String result = _planner.realize(_output);
        _statusLine.setText(result);
      }
    }
    }
  };

  private final Object[][] menuSpecs =
  { { "New",
    KeyStroke.getKeyStroke(Character.valueOf('n'), InputEvent.ALT_DOWN_MASK),
    new Runnable() {
      public void run() {
        UPMainFrame newMf = new UPMainFrame("Empty Planner");
        newMf.selectProjectDialog();
      }
    }
  },
  {"Open", KeyEvent.VK_O,
    new Runnable() { public void run() { selectProjectDialog(); } }
  },
  {"Close",
    KeyStroke.getKeyStroke(Character.valueOf('w'),
        InputEvent.META_DOWN_MASK | InputEvent.SHIFT_DOWN_MASK),
    new Runnable() { public void run() { UPMainFrame.this.dispose(); } }
  },
  {"Close All",
    KeyStroke.getKeyStroke(Character.valueOf('a'),
        InputEvent.ALT_DOWN_MASK),
    new Runnable() {
      public void run() {
        synchronized (UPMainFrame.openFrames) {
          for (UPMainFrame oneFrame : UPMainFrame.openFrames) {
            oneFrame.close();
          }
        }
      }
    }
  }
  /*
  {"Select Font", null,
    new Runnable() {
      public void run() {
        FontChooser f = new FontChooser();
        f.getChosenFont();
      }
    }
  }
  */
  };

  /* *************************************************************************
   * fields for GUI elements
   * *************************************************************************/

  private String _iconPath = null;
  private Font _textFont = new Font(_defaultFont, 0, 12);

  // TODO: History function for input area that can be loaded/stored/cleared
  /** contains the input text */
  private JTextArea _inputArea;

  /** displays error and status information */
  private JLabel _statusLine;

  /** This displays the input lf */
  private DrawingPanel _inputDisplay;

  /** This displays the output lf */
  private DrawingPanel _outputDisplay;

  // TODO: Combine with editor to repair errors in rule files
  // TODO: Tracing in window that allows jumping to the rule that is connected
  //       to the match/action immediately, generally, a much better tracing
  //       facility
  /** This displays text trace output */
  // private JTextArea _traceDisplay;

  /** Action Buttons */
  private ArrayList<JButton> _actionButtons;

  /** The actions that are called when a button is pressed */
  HashMap<String, Runnable> _actions;

  /* *************************************************************************
   * fields for processing elements beyond the GUI
   * *************************************************************************/

  /** This contains the selected rules file and current directory. */
  private File _currentDir = null;
  private File _projectFile = null;

  /** The processing unit behind this frame */
  private InteractivePlanner _planner = null;

  /** RuleTracer objects to perform tracing */
  private RuleTracer _oldTracer = null;
  private RuleTracer _tracer = null;

  /** A semaphore to allow for trace/step mode in the GUI */
  private Semaphore _stepModeSemaphore = null;

  private DagNode _input, _output;
  private Thread _processingThread;

  private boolean _finished;

  /* **********************************************************************
   * Window closing functionality
   * ********************************************************************** */

  public interface CloseAllListener {
    public abstract void allClosed();
  }

  private class ReleaseSemaphoreOnCloseAll implements CloseAllListener {
    Semaphore mySem;
    public ReleaseSemaphoreOnCloseAll(Semaphore sem) {
      mySem = sem;
    }
    public void allClosed() {
      mySem.release();
    }
  }

  /** <code>Terminator</code> defines action to be done when closing a frame.
   */
  private class Terminator extends WindowAdapter {
    /** This creates a new instance of <code>Terminator</code>. */
    public Terminator() {
      super();
    }

    /** This overrides the <code>windowClosing</code> method of the super class.
     * @param we a <code>WindowEvent</code>
     * @see java.awt.event.WindowAdapter#windowClosing(java.awt.event.WindowEvent)
     */
    @Override
    public void windowClosing(WindowEvent we) {
      we.getWindow().dispose();
    }

    /** This overrides the <code>windowClosed</code> method of the super class.
     * @param we a <code>WindowEvent</code>
     * @see java.awt.event.WindowAdapter#windowClosed(java.awt.event.WindowEvent)
     */
    @Override
    public void windowClosed(WindowEvent we) {

      // find the frame for this window and remove it form the list;
      // make sure this is thread safe
      synchronized (UPMainFrame.openFrames) {
        Window win = we.getWindow();
        openFrames.remove(win);
      }
      // if this was the last frame, completely shut down the application
      if (UPMainFrame.openFrames.isEmpty()) {
        for (CloseAllListener caListener : clAll) {
          caListener.allClosed();
        }
      }
    }
  }

  public void registerCloseAllListener(CloseAllListener caListener) {
    clAll.add(0, caListener);
  }

  public void releaseOnAllClosed(Semaphore sem) {
    registerCloseAllListener(new ReleaseSemaphoreOnCloseAll(sem));
  }

  /** a method to close a frame from within the program */
  public void close() {
    this.dispatchEvent(new WindowEvent(this, WindowEvent.WINDOW_CLOSING));
  }

  /* **********************************************************************
   * Constructors
   * ********************************************************************** */

  public UPMainFrame(String title) {
    this(title, new InteractivePlanner(), null);
  }

  public UPMainFrame(String title, InteractivePlanner ip, String ruleFile) {
    super(title);
    _planner = ip;
    _actions = new HashMap<String, Runnable>();
    initPanel();
    setProjectFile(ruleFile == null ? null : new File(ruleFile));
    setFinished(true);
  }

  /* **********************************************************************
   *  For GuiTracer, and other objects to communicate things to the user
   * ********************************************************************** */

  /** Clear the status line (a line at the bottom of the window for status
   *  messages).
   */
  public void clearStatusLine() {
    setStatusLine(" ");
  }

  /** Put the given message into the status line with black (default) text color
   */
  public void setStatusLine(String msg) {
    setStatusLine(msg, Color.BLACK);
  }

  /** Put the given message into the status line with the text color given by
   *  col
   */
  public void setStatusLine(String msg, Color col) {
    _statusLine.setForeground(col);
    _statusLine.setText(msg);
  }

  /** display the given data structure in the input area */
  public void setInputDisplay(DagNode dag) {
    _inputDisplay.setModel(dag);
  }

  /** display the given data structure in the output area */
  public void setOutputDisplay(DagNode dag) {
    _outputDisplay.setModel(dag);
  }

  /** Wait until either the Step or Continue buttons have been hit */
  public Semaphore waitForStepTrace(Semaphore toWaitFor) {
    setFinished(_finished);
    _stepModeSemaphore = toWaitFor;
    try {
      _stepModeSemaphore.acquire();  // acquire the only available permit
    } catch (InterruptedException e) {
    }
    return _stepModeSemaphore;
  }


  /* **********************************************************************
   * Action methods
   * ********************************************************************** */

  /** Read the file that contains paths to the rule files to be loaded */
  public void readProjectFile() {
    _planner.readProjectFile(_projectFile);
    if (! _planner.getPlanner().getErrors().isEmpty()) {
      int errs = _planner.getPlanner().getErrors().size();
      setStatusLine( String.format("WARNING: %1d error%2s in rule files",
                                    errs, (errs == 1 ? "" : "s")),
                     Color.RED);
    }
    this.setTitle(_projectFile.getPath());
  }

  /** Set the project file to the given file */
  private void setProjectFile(File projectFile) {
    if (projectFile == null) {
      _projectFile = null;
      _currentDir = new File(".");
    }
    else {
      _projectFile = projectFile;
      _currentDir = _projectFile.getAbsoluteFile().getParentFile();
      readProjectFile();
      setStatusLine("rule files reloaded");
    }
    /** Enable/Disable reload rules, process, start trace
     *  depending on existence of rule file
     */
    setFinished(true);
  }

  /** Open a file dialog to choose a new project file */
  private void selectProjectDialog() {
    // create file chooser for txt files
    JFileChooser fc = new JFileChooser();
    /*
    fc.addChoosableFileFilter(
        new FileNameExtensionFilter("txt/xml files only", "txt", "xml"));
     */
    fc.setCurrentDirectory(_currentDir);
    int returnVal = fc.showOpenDialog(UPMainFrame.this);
    File newProjectFile = null;
    do {
      if (returnVal == JFileChooser.APPROVE_OPTION) {
        // set the rule file
        newProjectFile = fc.getSelectedFile();
      }
    }
    while (newProjectFile == null && returnVal != JFileChooser.CANCEL_OPTION);
    if (newProjectFile != null) {
      setProjectFile(newProjectFile);
    }
  }

  /** Sets the _finished flag to what, which signal if the processor is
   *  processing input or not, and enable/disable buttons in the tool bar
   *  depending on this flag (and other conditions.
   */
  private void setFinished(boolean what) {
    _finished = what;
    _actionButtons.get(LOAD_BTN).setEnabled(_projectFile != null);
    _actionButtons.get(PROCESS_BTN).setEnabled(_projectFile != null && _finished);
    /** EnDisable gui tracing when text tracing was enabled */
    _actionButtons.get(TRACE_BTN).setEnabled(_projectFile != null && _finished);
    /** Enable/Disable step tracing */
    _actionButtons.get(STEP_BTN).setEnabled(! _finished);
    _actionButtons.get(CONTINUE_BTN).setEnabled(! _finished);
  }

  /** Call this method when the processing of new input starts */
  private void processingStarts() {
    if (_tracer != null && _oldTracer == null) {
      _oldTracer = _planner.getPlanner().getTracing();
      _planner.getPlanner().setTracing(_tracer);
    }
    setFinished(false);
  }

  /** Call this method when the processing of input has finished */
  private void processingEnds() {
    _tracer = null;
    if (_oldTracer != null) {
      _planner.getPlanner().setTracing(_oldTracer);
      _oldTracer = null;
    }
    setFinished(true);
  }

  /** This method takes the input string from the input area and tries to
   *  process it. If parsing the input reveals a syntax error, the caret is
   *  put to the error position and the error is signaled by using a reddish
   *  color as background for the input area. Otherwise, the processing is
   *  started in a new thread.
   */
  private void processInput() {
    clearStatusLine();
    _inputArea.setBackground(NORMAL_COLOR);
    String currentText = _inputArea.getText();
    if (currentText.isEmpty())
      return;

    _input = _planner.getPlanner().parseLfString(currentText);
    setInputDisplay(_input);
    if (_input != null) {
      // add currentText to the history
      // should be in its own thread, with a listener that sets the
      // output, and an indicator that it's running
      _processingThread = new Thread(
          new Runnable() {
            public void run() {
              if (_finished) {
                try {
                  processingStarts();
                  _output = _planner.getPlanner().process(_input);
                  setInputDisplay(_input);
                  setOutputDisplay(_output);
                }
                finally {
                  processingEnds();
                }
              }
            }
          });
      _processingThread.start();
    }
    else {
      setOutputDisplay(null);
      Position errorPos = _planner.getPlanner().getLastLFErrorPosition();
      _inputArea.setBackground(ERROR_COLOR);
      if (errorPos.line >= 0) {
        setStatusLine(errorPos.msg);
        try {
          int offset =
            _inputArea.getLineStartOffset(errorPos.line -1) + errorPos.column -1;
          // _inputArea.insert("\u26A1", offset);
          _inputArea.setCaretPosition(offset);
          _inputArea.requestFocus();
        }
        catch (BadLocationException blex) {
          // just ignore;
          System.out.println("" + blex + errorPos.line + errorPos.column);
        }
      }
    }
  }

  /* ***********************************************************************
   * Action Listener implementation
   * ***********************************************************************/

  /** Implement the ActionListener interface by looking up the appropriate
   *  actions in the _actions hash table and running it, if available.
   *  _actions contains the actions for the tool bar buttons and the menu
   *  items of the main menu bar.
   */
  @Override
  public void actionPerformed(ActionEvent e) {
    Runnable runnable = _actions.get(e.getActionCommand());
    if (runnable != null) {
      clearStatusLine();
      runnable.run();
    }
  }

  /* **********************************************************************
   * Initialization / Creation
   * ********************************************************************** */

  private DrawingPanel newLFPanel() {
    //return new DrawingPanel(new CompactLayout(), new LFModelAdapter());
    return new DrawingPanel(new LFLayout(),
                            new EmptyModelAdapter(ModelAdapter.MAP));
  }

  protected JButton newButton(String imageName,
      String actionCommand,
      String toolTipText,
      String altText) {
    //  Look for the image.
    String imgLocation = _iconPath + "24x24/actions/" + imageName + ".png";
    //URL imageURL = UPMainFrame.class.getResource(imgLocation);
    String imageURL = null;
    if (new File(imgLocation).exists()) {
      imageURL = imgLocation;
    }

    //  Create and initialize the button.
    JButton button = new JButton();
    button.setActionCommand(actionCommand);
    button.setToolTipText(toolTipText);
    button.addActionListener(this);

    if (imageURL != null) {                      //image found
      button.setIcon(new ImageIcon(imageURL, altText));
    } else {                                     //no image found
      button.setText(altText);
      // System.err.println("Resource not found: " + imgLocation);
    }

    return button;
  }

  private JToolBar newMainToolBar() {
    _actionButtons = new ArrayList<JButton>();
    JToolBar toolBar = new JToolBar("Still draggable");
    for (Object[] spec : actionSpecs) {
      _actions.put((String) spec[0], (Runnable) spec[4]);
      if (spec[1] != null) {
        JButton newButton =
          newButton((String) spec[1],(String) spec[0],
              (String) spec[2],(String) spec[3]);
        _actionButtons.add(newButton);
        toolBar.add(newButton);
      }
    }
    return toolBar;
  }

  private JScrollPane newInputArea() {
     // The area to input lf's
    _inputArea = new JTextArea();
    _inputArea.setFont(_textFont);
    _inputArea.setRows(5);
    //_inputArea.setColumns(70);

    return new JScrollPane(_inputArea);
  }


  /** Create a new menu item out from name and key spec and add it to the
   *  given menu
   */
  private void newMenuItem(JMenu menu, String name, Object key) {
    JMenuItem newItem = new JMenuItem(name);
    if (key instanceof Integer)
      newItem.setMnemonic((Integer) key);
    if (key instanceof KeyStroke)
      newItem.setAccelerator((KeyStroke) key);
    newItem.addActionListener(this);
    menu.add(newItem);
  }

  /** This initializes the menu bar. */
  private void newMainMenuBar() {
    // create the menu bar
    JMenuBar menuBar = new JMenuBar();
    menuBar.setOpaque(true);
    menuBar.setPreferredSize(new Dimension(400, 20));

    // create the 'File' menu
    JMenu menu = new JMenu("File");
    menu.setMnemonic(KeyEvent.VK_F);
    menuBar.add(menu);

    for (Object[] spec : menuSpecs) {
      newMenuItem(menu, (String) spec[0], spec[1]);
      _actions.put((String) spec[0], (Runnable) spec[2]);
    }

    // add menu bar to main frame
    this.setJMenuBar(menuBar);
  }

  private void initPanel() {
    try {
      String lookAndFeel = UIManager.getSystemLookAndFeelClassName();
      UIManager.setLookAndFeel(lookAndFeel);
      if (lookAndFeel.contains("GTK")) {
        _iconPath = "/usr/share/icons/gnome/";
      }
    } catch (ClassNotFoundException e) {
      // well, we're content with everything we get
    } catch (InstantiationException e) {
      // well, we're content with everything we get
    } catch (IllegalAccessException e) {
      // well, we're content with everything we get
    } catch (UnsupportedLookAndFeelException e) {
      // well, we're content with everything we get
    }

    // create content panel and add it to the frame
    JPanel contentPane = new JPanel(new BorderLayout());
    contentPane.setLayout(new BorderLayout());
    this.setContentPane(contentPane);
    // create scrollable display areas
    _inputDisplay = newLFPanel();
    _outputDisplay = newLFPanel();

    // create menu bar
    this.newMainMenuBar();
    add(newMainToolBar(), BorderLayout.PAGE_START);
    JScrollPane inputDisplay = new JScrollPane(_inputDisplay);
    JScrollPane outputDisplay = new JScrollPane(_outputDisplay);
    JSplitPane displayPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,
        inputDisplay, outputDisplay);
    displayPane.setOneTouchExpandable(true);
    JSplitPane splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT,
        newInputArea(), displayPane);
    //splitPane.setOneTouchExpandable(true);
    contentPane.add(splitPane, BorderLayout.CENTER);
    _statusLine = new JLabel();
    contentPane.add(_statusLine, BorderLayout.SOUTH);
    clearStatusLine();
    // add this frame to the list of open frames
    UPMainFrame.openFrames.add(this);
    // use native windowing system to position new frames
    this.setLocationByPlatform(true);
    this.setPreferredSize(new Dimension(800, 500));
    // set handler for closing operations
    this.addWindowListener(new Terminator());
    // display the frame
    this.pack();
    displayPane.setDividerLocation(.5);
    int unitIncrement = _inputDisplay.getDefaultTextHeight();
    // display the frame
    inputDisplay.getHorizontalScrollBar().setUnitIncrement(unitIncrement);
    inputDisplay.getVerticalScrollBar().setUnitIncrement(unitIncrement);
    outputDisplay.getHorizontalScrollBar().setUnitIncrement(unitIncrement);
    outputDisplay.getVerticalScrollBar().setUnitIncrement(unitIncrement);
    this.setVisible(true);
  }

  public void setTracing(int traceFlags) {
    if (traceFlags != 0) {
      //_traceDisplay = new JTextArea();
      _planner.getPlanner().setTracing(new LoggingTracer(traceFlags));
    }
    else {
      //_traceDisplay = null;
    }
  }
}
