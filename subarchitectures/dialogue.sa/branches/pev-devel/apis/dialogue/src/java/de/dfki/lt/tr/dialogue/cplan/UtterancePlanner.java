package de.dfki.lt.tr.dialogue.cplan;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Logger;

import de.dfki.lt.tr.dialogue.cplan.functions.FunctionFactory;
import de.dfki.lt.tr.dialogue.cplan.util.Position;

public class UtterancePlanner {

  /** This logger should be used only for messages concerning rule loading and
   *  processing issues. It is meant to be the console/error output of this
   *  content planner.
   */
  private Logger logger = Logger.getLogger("UtterancePlanner");

  /** The processing engine of this content planner */
  private ParallelProcessor _processor;
  /** The lexer to use in logical form parsing */
  private Lexer _lfLexer;
  /** The parser for logical forms. */
  private LFParser _lfParser;

  /** The lexer to use in rule and logical form parsing */
  private Lexer _ruleLexer;

  /** All the errors from the last round of rule reading */
  private List<Position> _errors;

  /** The directory used for resolving relative rule file names */
  private File _ruleRoot;
  /** All the files from the last round of rule reading */
  private List<File> _ruleFiles;

  static {
    DagNode.init();
  }

  public UtterancePlanner(File pluginDirectory) {
    FunctionFactory.init(pluginDirectory);
    _errors = new ArrayList<Position>();
    _ruleFiles = new ArrayList<File>();

    _ruleLexer = new Lexer();
    _ruleLexer.setErrorLogger(logger);

    _lfLexer = new Lexer();
    _lfParser = new LFParser(_lfLexer);
    _lfParser.errorVerbose = true;
  }

  public UtterancePlanner() {
    this(null);
  }

  public void initializeProcessor(List<Rule> rules) {
    _processor = new ParallelProcessor(rules);
  }

  public void setTracing(AbstractTracer rt) {
    _processor.setTracing(rt);
  }

  public AbstractTracer getTracing() {
    return _processor.getTracing();
  }

  private void readRules(Reader r, String inputDescription, List<Rule> rules)
  throws IOException {
    RuleParser ruleParser = new RuleParser(_ruleLexer);
    ruleParser.errorVerbose = true;
    ruleParser.setDebugLevel(0);
    ruleParser.reset(inputDescription, r);
    ruleParser.parse();
    List<Rule> fileRules = ruleParser.getRules();
    if (rules != null) {
      rules.addAll(fileRules);
    }
    _errors.addAll(_ruleLexer.getAllErrorPositions());
  }

  private void readRulesFromFile(File f, List<Rule> rules) {
    try {
      readRules(new FileReader(f), f.getPath(), rules);
      logger.info("Reading rule file: " + f);
    }
    catch (FileNotFoundException fnfex) {
      logger.warn("Could not find rule file: " + f);
    }
    catch (IOException ioex) {
      logger.warn("Could not read rule file: " + f + " (" + ioex +")");
    }
  }

  public List<File> readProjectFile(File ruleFile) {
    _ruleFiles.clear();
    _errors.clear();
    List<Rule> rules = new ArrayList<Rule>();
    _ruleRoot = ruleFile.getParentFile();
    try {
      BufferedReader in = new BufferedReader(new FileReader(ruleFile));
      String nextLine = null;
      while ((nextLine = in.readLine()) != null) {
        nextLine = nextLine.trim();
        if (! nextLine.isEmpty()) {
          File nextFile = new File(nextLine);
          if (! nextFile.isAbsolute()) {
            nextFile = new File(_ruleRoot, nextLine);
          }
          _ruleFiles.add(nextFile);
          readRulesFromFile(nextFile, rules);
        }
      }
    }
    catch (IOException ioex) {
      logger.warn(ioex);
    }
    initializeProcessor(rules);
    return _ruleFiles;
  }

  public List<Position> getErrors() {
    return _errors;
  }

  public List<File> getRuleFiles() {
    return _ruleFiles;
  }

  public File getDirectory() {
    return _ruleRoot;
  }

  /** This function is only present to support testing, therefore it has
   *  package visibility. It should not be used from the outside.
   */
  List<Rule> readRulesFromString(String ruleString) {
    List<Rule> rules = new ArrayList<Rule>();
    try {
      readRules(new StringReader(ruleString), "input", rules);
    } catch (IOException e) {
      // this will never be thrown
      e.printStackTrace();
    }
    return rules;
  }

  /** Run the processor until there is no more change */
  private DagNode computeFixpoint(DagNode input) {
    Bindings bindings = new Bindings();
    boolean changed = true;
    while (changed) {
      try {
        DagNode result = _processor.applyRules(input.cloneFS(), bindings);
        changed = (! result.equals(input) || bindings.globalBindingsChanged());
        input = result;
      }
      catch (InterruptedException ex) {
        break;
      }
    }
    return input;
  }

  /** Process the input with the loaded rules */
  public DagNode process(DagNode lf) {
    return computeFixpoint(lf);
  }

  /** Convert the given input string, which contains a (partial) logical form,
   *  into an internal data structure for processing.
   */
  public DagNode parseLfString(String input) {
    if (input.isEmpty())
      return null;
    _lfParser.reset("Console", new StringReader(input));
    try {
      if (_lfParser.parse()) {
        return _lfParser.getResultLF();
      }
    }
    catch (IOException ioex) {
      // this will hardly ever been thrown
      ioex.printStackTrace();
    }
    catch (ArrayIndexOutOfBoundsException ex) {
      // may occur during parsing of LF in LFParser. Just die silently.
    }
    return null;
  }

  /** Get the error position of the last LF parse, or null, if there is no
   *  such error.
   */
  public Position getLastLFErrorPosition() {
    return _lfLexer.getLastErrorPosition();
  }

}
