package de.dfki.lt.tr.dialogue.cplan;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.List;

import org.apache.log4j.Logger;

import de.dfki.lt.tr.dialogue.cplan.util.Position;

public class UtterancePlanner {

  private static Logger logger = Logger.getLogger("UtterancePlanner");

  private ParallelProcessor _processor;
  private Lexer _lexer;
  private LFParser _lfparser;

  private List<Position> _errors;

  static {
    DagNode.init();
  }

  public UtterancePlanner() {
    _lexer = new Lexer();
    _lfparser = new LFParser(_lexer);
    _errors = new ArrayList<Position>();
  }

  public void initializeProcessor(List<Rule> rules) {
    _processor = new ParallelProcessor(rules);
  }

  private List<Rule> readRuleFile(File f, List<Rule> rules) {
    try {
      logger.info("Reading rule file " + f);
      RuleParser parser = new RuleParser(_lexer);
      parser.reset(f.getPath(), new FileReader(f));
      parser.errorVerbose = true;
      parser.parse();
      List<Rule> fileRules = parser.getRules();
      if (rules != null) {
        rules.addAll(fileRules);
      }
      _errors.addAll(_lexer.getAllErrorPositions());
    }
    catch (FileNotFoundException fnfex) {
      logger.warn("Could not find rule file: " + f);
    }
    catch (IOException ioex) {
      logger.warn("Could not read rule file: " + f + "(" + ioex +")");
    }
    return rules;
  }

  public List<Position> readRuleFile(File ruleFile) {
    List<Rule> rules = new ArrayList<Rule>();
    _errors.clear();
    File ruleRoot = ruleFile.getParentFile();
    try {
      BufferedReader in = new BufferedReader(new FileReader(ruleFile));
      String nextLine = null;
      while ((nextLine = in.readLine()) != null) {
        nextLine = nextLine.trim();
        if (! nextLine.isEmpty()) {
          File nextFile = new File(nextLine);
          if (nextFile.isAbsolute()) {
            readRuleFile(nextFile, rules);
          }
          else {
            readRuleFile(new File(ruleRoot, nextLine), rules);
          }
        }
      }
    }
    catch (IOException ioex) {
      logger.warn(ioex);
    }
    initializeProcessor(rules);
    return _errors;
  }

  public List<Rule> readRuleString(String ruleString) {
    RuleParser parser = new RuleParser(new Lexer());
    parser.reset(ruleString, new StringReader(ruleString));
    try {
      parser.parse();
    } catch (IOException e) {
      // this will never be thrown
      e.printStackTrace();
    }
    List<Rule> rules = new ArrayList<Rule>();
    rules.addAll(parser.getRules());
    return rules;
  }

  private DagNode computeFixpoint(DagNode input) {
    Bindings bindings = new Bindings();
    int globalBindings = 0;
    boolean changed = true;
    while (changed) {
      DagNode result = _processor.applyRules(input.cloneFS(), bindings);
      changed = (! result.equals(input) ||
          globalBindings != bindings.getNumberOfGlobalBindings());
      globalBindings = bindings.getNumberOfGlobalBindings();
      input = result;
    }
    return input;
  }

  public DagNode parseLfString(String input) {
    _lfparser.errorVerbose = true;
    _lexer.setInputReader("Console", new StringReader(input));
    try {
      if (_lfparser.parse()) {
        return _lfparser.getResultLF();
      }
    }
    catch (IOException ioex) {
      // this will hardly ever been thrown
      ioex.printStackTrace();
    }
    catch (ArrayIndexOutOfBoundsException ex) {
    }
    return null;
  }

  public Position getLastErrorPosition() {
    return _lexer.getLastErrorPosition();
  }

  public DagNode process(DagNode lf) {
    return computeFixpoint(lf);
  }
}
