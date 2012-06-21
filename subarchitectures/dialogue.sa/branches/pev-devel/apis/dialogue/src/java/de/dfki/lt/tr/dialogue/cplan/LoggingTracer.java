package de.dfki.lt.tr.dialogue.cplan;

import org.apache.log4j.Logger;

public class LoggingTracer extends AbstractTracer {

  private int _ruleTracing = 0;
  private Logger logger = Logger.getLogger("TraceLogger");

  public LoggingTracer(int bitmask) {
    setTracing(bitmask);
  }

  @Override
  public void traceMatch(DagEdge root, DagEdge current, Rule r,
      Bindings bindings) throws InterruptedException {
    checkInterrupt();
    if ((_ruleTracing & MATCHING) != 0) {
      StringBuilder sb = new StringBuilder();
      sb.append("\nMATCH: ");
      r.appendMatches(sb).append("\n       ").append(current);
      logger.info(sb.toString());
    }
  }

  @Override
  public void traceBeforeApplication(DagEdge root, DagEdge current, Rule r,
      Bindings bindings){
    if ((_ruleTracing & MODIFICATION) != 0) {
      logger.info("\nAPPLY  " + r.appendActions(new StringBuilder()).toString()
          + "\nTO     " + current);
    }
  }

  @Override
  public void traceAfterApplication(DagEdge root, DagEdge current, Rule r,
      Bindings bindings){
    if ((_ruleTracing & MODIFICATION) != 0) {
        logger.info("GETS   " + current);
    }
  }

  @Override
  public void setTracing(int bitmask) {
    _ruleTracing = bitmask;
  }
}
