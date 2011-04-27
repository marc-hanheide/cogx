package de.dfki.lt.tr.dialogue.cplan;

import java.io.File;
import java.io.IOException;
import java.util.Enumeration;
import java.util.List;

import jline.ConsoleReader;
import joptsimple.OptionException;
import joptsimple.OptionParser;
import joptsimple.OptionSet;
import opennlp.ccg.grammar.Grammar;
import opennlp.ccg.realize.Edge;
import opennlp.ccg.realize.Realizer;
import opennlp.ccg.synsem.LF;

import org.apache.log4j.Appender;
import org.apache.log4j.ConsoleAppender;
import org.apache.log4j.Logger;
import org.apache.log4j.PatternLayout;
import org.apache.log4j.SimpleLayout;

import de.dfki.lt.j2emacs.J2Emacs;

import de.dfki.lt.tr.dialogue.cplan.gui.LFModelAdapter;
import de.dfki.lt.tr.dialogue.cplan.gui.UPMainFrame;

public class InteractivePlanner implements UPMainFrame.CloseAllListener {

  private Realizer _realizer = null;

  private UtterancePlanner _up;

  private String _compilationBufferName = "*cplanner*";
  private J2Emacs _j2e = null;

  public InteractivePlanner() {
    _up = new UtterancePlanner();
    LFModelAdapter.init();
  }

  public void initializeProcessor(List<String> nonOptionArgs) {
    if (nonOptionArgs.isEmpty()) {
      System.out.println("Rule file required for interactive mode");
      System.exit(1);
    }
    _up.readProjectFile(new File(nonOptionArgs.get(0)));
  }

  /**/
  public void readProjectFile(File projectFile) {
    if (_j2e != null) {
      _j2e.clearBuffer(_compilationBufferName);
    }
    _up.readProjectFile(projectFile);
    if (_j2e != null) {
      _j2e.markAsProjectFiles(_up.getDirectory(), _up.getRuleFiles());
    }
  }
  /**/

  public UtterancePlanner getPlanner() {
    return _up;
  }

  /*
  private BufferedReader input = null;
  private String getQueryFromTerminalSimple() {
    if (input == null) {
      input = new BufferedReader(new InputStreamReader(System.in));
    }
    System.out.print("query> ");
    String queryString = "";
    try {
      String str;
      while ((str = input.readLine()) != null && (str.length() > 0)) {
        queryString += str + "\n";
      }
    } catch (IOException i) {
      return "";
    }
    return queryString;
  }
  */

  private ConsoleReader cinput = null;
  private String getInputFromTerminal() {
    /*
    if (input == null) {
      input = new BufferedReader(new InputStreamReader(System.in));
    }
    */
    String prompt = ">> ";
    String queryString = "";
    String str;
    try {
      if (cinput == null) {
        cinput = new ConsoleReader();
        cinput.setBellEnabled(false);
        //cinput.setDebug(new PrintWriter(new FileWriter("writer.dbg", true)));
        /*
        String[] commands = { "select", "ask", "describe", "insert", "delete" };
        String[] keywords = { "where", "optional", "filter" };
        List<Completor> completors = new LinkedList<Completor>();
        completors.add(new SimpleCompletor(commands));
        completors.add(new SimpleCompletor(keywords));
        ArgumentCompletor comp = new ArgumentCompletor(completors);
        comp.setStrict(false);
        cinput.addCompletor(comp);
        */
      }
      while ((str = //input.readLine()
        cinput.readLine(prompt)
      ) != null && (str.length() > 0)) {
        prompt = "/ ";
        queryString += str + "\n";
      }
    } catch (IOException i) {
      return "";
    }
    return queryString;
  }

  public void interactive(int traceFlags) {
    if (traceFlags != 0) {
      _up.setTracing(new LoggingTracer(traceFlags));
    }
    String input = getInputFromTerminal();
    while(! input.isEmpty()) {
      DagNode lf = _up.parseLfString(input);
      if (lf != null) {
        System.out.println(lf);
        DagNode result = _up.process(lf);
        System.out.println(result);
      }
      input = getInputFromTerminal();
    }
  }

  public String realize(DagNode dagLf) {
    String result = "";
    if (_realizer != null && dagLf != null) {
      DagEdge content = dagLf.getEdge(DagNode.getFeatureId("Content"));
      if (false && content != null) {
        dagLf = content.getValue();
      }
      LF lf = DagToLF.convertToLF(dagLf);
      System.out.println(lf);
      Edge resEdge = _realizer.realize(lf);
      result = resEdge.getSign().getOrthography();
      //}
    }
    return result;
  }

  private class LoadProjectAction implements J2Emacs.Action {
    private UPMainFrame _mf;

    LoadProjectAction(UPMainFrame mf) { _mf = mf; }

    @Override public void execute(String... args) { _mf.readProjectFile(); }
  }

  void startGui(int traceFlags, List<String> ruleFiles) {
    UPMainFrame mf =
      new UPMainFrame("ContentPlanner (no rules loaded)", this,
          (ruleFiles.isEmpty() ? null : ruleFiles.get(0)));
    mf.setTracing(traceFlags);
    mf.registerCloseAllListener(this);
    if (_j2e != null) {
      _j2e.registerAction("reload", new LoadProjectAction(mf));
    }
  }

  public void allClosed() {
    if (_j2e != null) { _j2e.close(); }
  }

  private void startEmacsConnection() {
    _j2e = new J2Emacs("CPlanner");
    // make an EmacsBufferAppender in j2e-compilation mode
    Appender ea = _j2e.new EmacsBufferAppender(_compilationBufferName, true);
    Logger uplogger = Logger.getLogger("UtterancePlanner");
    uplogger.removeAllAppenders();
    uplogger.setAdditivity(false);
    uplogger.addAppender(ea);
  }


  private static void usage(String msg) {
    String[] usage = {
        "Usage: UPDebugger [-b<atch> inputfile] [-c<ompileonly>] [-d<ebugdags>]",
        "                  [-g<ui>] [-t<race>={1,2,3}]", // [-e<macsConsole>]",
        "                  [-C<CG> grammardir ] <rulefile>",
        "      -c : only load rule files and exit, to check syntactic correctness",
        "      -t : bit 1: trace match, bit2 : trace modification"
    };
    System.out.println(msg);
    for (String us : usage) System.out.println(us);
    System.exit(1);
  }

  @SuppressWarnings({ "unchecked", "null" })
  public static void main(String[] args) {
    Logger uplogger = Logger.getLogger("UtterancePlanner");
    Enumeration<Appender> apps = uplogger.getAllAppenders();
    Enumeration<Appender> rapps = Logger.getRootLogger().getAllAppenders();
    if (! apps.hasMoreElements() && ! rapps.hasMoreElements()) {
      uplogger.addAppender( new ConsoleAppender(new PatternLayout("%m%n")));
      uplogger.setAdditivity(false);
      Logger.getRootLogger().addAppender(
          new ConsoleAppender(new SimpleLayout(), "System.err"));
    }

    OptionParser parser = new OptionParser("edcC:t::gb:");
    OptionSet options = null;
    try {
      options = parser.parse(args);
    }
    catch (OptionException ex) {
      usage("Error parsing options: " + ex.getLocalizedMessage());
      System.exit(1);
    }

    List<String> nonOptionArgs = options.nonOptionArguments();

    @SuppressWarnings("unused")
    String optionArg = null;

    char what = 'i';
    // x and T are only for test purposes
    String[] actionOptions = { "b", "c", "g" };
    for (String action : actionOptions) {
      if (options.has(action)) {
        if (what != 'i')
          usage("Only one of -b allowed.") ;
        what = action.charAt(0);
        optionArg = (String) options.valueOf(action);
      }
    }

    // trace flags
    int traceFlags = 0;
    if (options.has("t")) {
      traceFlags = Integer.parseInt((String) options.valueOf("t"));
    }

    InteractivePlanner ip = new InteractivePlanner();

    if (options.has("C")) {
      try {
        ip._realizer = new Realizer(new Grammar((String)options.valueOf("C")));
      } catch (IOException e) {
        ip._realizer = null;
        System.err.println("CCG grammar could not be loaded: " + e);
      }
    }

    if (options.has("e")) {
      ip.startEmacsConnection();
    }

    if (options.has("d")) {
      DagNode.useDebugPrinter();
    }
    switch (what) {
    case 'b':
      // batchProcess(optionArg);
      break;
    case 'g':
      ip.startGui(traceFlags, nonOptionArgs);
      break;
    case 'c': // only "compile" rules
      break;
    default:
      ip.initializeProcessor(nonOptionArgs);
      ip.interactive(traceFlags);
      ip.allClosed();
      break;
    }
  }
}
