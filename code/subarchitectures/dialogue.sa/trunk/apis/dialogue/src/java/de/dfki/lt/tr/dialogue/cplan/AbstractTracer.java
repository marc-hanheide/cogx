package de.dfki.lt.tr.dialogue.cplan;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Semaphore;

public abstract class AbstractTracer {
  public static final int MATCHING = 1;
  public static final int MODIFICATION = MATCHING << 1;
  public static final int ALL = MATCHING | MODIFICATION;

  public interface SuspendListener {
    public abstract void suspended(boolean isSuspended);
  }

  private List<SuspendListener> _listeners = new ArrayList<SuspendListener>();

  private Semaphore _suspendSemaphore = new Semaphore(0);
  protected int _interrupted = 0;

  public void addListener(SuspendListener l) {
    _listeners.add(l);
  }

  public void removeListener(SuspendListener l) {
    _listeners.remove(l);
  }

  private void informListeners(boolean value) {
    for(SuspendListener l : _listeners) {
      l.suspended(value);
    }
  }

  protected void checkInterrupt() throws InterruptedException {
    int value = _interrupted; _interrupted = 0;
    switch (value) {
    case 1: {
      try {
        informListeners(true);
        _suspendSemaphore.acquire();
      } catch (InterruptedException ex) {
      }
    } break;
    case 2: throw new InterruptedException();
    }
  }

  public void setTracing(int bitmask) { }

  public abstract void traceMatch(DagEdge root, DagEdge current, Rule r,
      Bindings bindings) throws InterruptedException ;

  public void traceBeforeApplication(DagEdge root, DagEdge current,
      Rule r, Bindings bindings) {}

  public abstract void traceAfterApplication(DagEdge root, DagEdge current,
      Rule r, Bindings bindings) ;

  public void suspendProcessing() {
    _interrupted = 1;
  }

  public void continueProcessing() {
    if (isSuspended()) {
      _suspendSemaphore.release();
      informListeners(false);
    }
  }

  public void stopProcessing() {
    _interrupted = 2;
  }

  public boolean isSuspended() {
    return (! _suspendSemaphore.tryAcquire());
  }
}
