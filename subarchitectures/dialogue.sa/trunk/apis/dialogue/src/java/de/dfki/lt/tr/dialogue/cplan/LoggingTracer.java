package de.dfki.lt.tr.dialogue.cplan;

import java.util.List;

import org.apache.log4j.BasicConfigurator;
import org.apache.log4j.ConsoleAppender;
import org.apache.log4j.Logger;
import org.apache.log4j.PatternLayout;
import org.apache.log4j.spi.RootLogger;

import de.dfki.lt.tr.dialogue.cplan.actions.Action;

public class LoggingTracer implements RuleTracer {

  private int _ruleTracing = 0;
  private static Logger logger = Logger.getLogger("UtterancePlanner");


  public LoggingTracer(int bitmask) {
    if (bitmask != 0) {
      BasicConfigurator.resetConfiguration();
      RootLogger.getRootLogger().addAppender(
          new ConsoleAppender(new PatternLayout("%m%n")));
    }
    _ruleTracing = bitmask;
  }

  @Override
  public void traceMatch(DagEdge current, List<VarMatch> matches,
      Bindings bindings) {
    if ((_ruleTracing & DISPLAY_MATCHING) != 0) {
      StringBuilder sb = new StringBuilder();
      sb.append("\nMATCH: ");
      Rule.matchesToString(sb, matches);
      sb.append("\n       ").append(current);
      logger.info(sb.toString());
    }
  }

  @Override
  public void traceBeforeApplication(DagEdge current, Action action) {
    // TODO Auto-generated method stub
    if ((_ruleTracing & DISPLAY_MODIFICATION) != 0) {
      logger.info("\nAPPLY  " + action + "\nTO     " + current);
    }
  }

  @Override
  public void traceAfterApplication(DagEdge current, Action action) {
    // TODO Auto-generated method stub
    if ((_ruleTracing & DISPLAY_MODIFICATION) != 0) {
        logger.info("GETS   " + current);
    }
  }


}
