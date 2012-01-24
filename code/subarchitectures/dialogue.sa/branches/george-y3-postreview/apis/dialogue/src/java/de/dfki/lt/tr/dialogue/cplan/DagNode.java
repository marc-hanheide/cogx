package de.dfki.lt.tr.dialogue.cplan;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;

import org.apache.log4j.Logger;

import de.dfki.lt.tr.dialogue.cplan.io.DagPrinter;
import de.dfki.lt.tr.dialogue.cplan.io.LFDagPrinter;
import de.dfki.lt.tr.dialogue.cplan.io.LFDebugPrinter;
import de.dfki.lt.tr.dialogue.cplan.util.IntIDMap;
import de.dfki.lt.tr.dialogue.cplan.util.ShortIDMap;

public class DagNode {

  public static Logger logger = Logger.getLogger("DagNode");

  public static Map<DagNode, FailType> forwardFailures =
    new HashMap<DagNode, FailType>();
  public static Map<DagNode, FailType> backwardFailures =
    new HashMap<DagNode, FailType>();

  public static final int THIS_MORE_GENERAL = 1;
  public static final int ARG_MORE_GENERAL = 2;

  protected static boolean recordFailures = false;

  protected static final int THIS_MORE_GENERAL_MASK = ~ THIS_MORE_GENERAL;
  protected static final int ARG_MORE_GENERAL_MASK = ~ ARG_MORE_GENERAL;

  protected static short NO_FEAT = Short.MAX_VALUE;

  public static final String TOP_TYPE = "*TOP*";
  public static final int TOP_ID = 0;
  public static final int BOTTOM_ID = -1;

  // We decided to overwrite instead of unify, which has the nice side effect
  // that there are no failures.
  private static final boolean UNIFY_TYPES = false;

  /** Special feature name for the id of a nominal */
  public static short ID_FEAT_ID = -1;

  /** Special feature name for the preposition of a nominal */
  public static short PROP_FEAT_ID = -1;

  /** Special feature name for the type of a nominal */
  public static short TYPE_FEAT_ID = -1;

  private static final String[] featureOrder =
    { "__ID", "__TYPE", "__PROP", "Cop-Restr", "Cop-Scope" };

  private static final int _useLfPrinter = 1;

  public static void init() {
    mapTypeToNumber(TOP_TYPE);
    for (String feat : featureOrder) {
      mapFeatureToNumber(feat);
    }
    ID_FEAT_ID = mapFeatureToNumber(featureOrder[0]);
    TYPE_FEAT_ID = mapFeatureToNumber(featureOrder[1]);
    PROP_FEAT_ID = mapFeatureToNumber(featureOrder[2]);
    switch (_useLfPrinter) {
    case 1: usePrettyPrinter(); break;
    case 2: useDebugPrinter(); break;
    }
  }

  public static void usePrettyPrinter() {
    registerPrinter(new LFDagPrinter());
  }

  public static void useDebugPrinter() {
    registerPrinter(new LFDebugPrinter());
  }

  public static int totalNoNodes = 0, totalNoArcs = 0;

  static int currentGenerationMax = 1;
  static int currentGeneration = 1;

  private static IntIDMap<String> nameToType = new IntIDMap<String>();

  private static ShortIDMap<String> nameToFeature = new ShortIDMap<String>();

  /* ******************** PUBLIC CONSTANTS ******************** */

  /** Use internal codes or external names for printing */
  public static boolean PRINT_READABLE = true;

  /** A printer used to change the default toString method to print the right
   *  format
   */
  private static DagPrinter _DEFAULT_PRINTER = null;

  /** Which type of failure occured?
   *  unification failures: Type clash, cycle, wellformedness unification
   *  subsumption failures: type mismatch, missing feature, missing variable
   */
  public enum FailType { SUCCESS, TYPE, CYCLE, FEATURE, VARIABLE, WELLFORMED }

  /** Restrictor constants */
  public enum RESTRICT { NO, KEEP, REMOVE, DELETE, FULL }

  /* ******************** PRIVATE FIELDS AND CLASSES ******************** */

  // The type of this node
  protected int _typeCode;
  // The feature-value list
  ArrayList<DagEdge> _outedges;
  // is this node a complex node or just a prop/type
  private boolean _isNominal;

  // a generation counter to (in)validate the following scratch fields
  int _generation;
  // these are the generation-protected scratch slots
  private int _newType;
  private DagNode _forward;
  private DagNode _copy;
  ArrayList<DagEdge> _compArcs;

  /** so that we don't have to return null when the edges list is empty */
  private static Iterator<DagEdge> emptyEdges =
    new Iterator<DagEdge>() {
    public boolean hasNext() { return false; }
    public DagEdge next() { throw new NoSuchElementException(); }
    public void remove() {
      throw new UnsupportedOperationException();
    }
  };

  /** This iterator iterates over the edges of a node correctly, be it a
   * `complete' or `temporary' dag, i.e., one that is currently involved in
   * unification as has a maybe non-empty compArcs list.
   */
  public class EdgeIterator {
    private int cursorArcs = -1;
    private int cursorCompArcs = -1;

    EdgeIterator() {
      if (_outedges != null && _outedges.isEmpty()) {
        _outedges = null;
      }
      cursorArcs = (DagNode.this._outedges != null) ? 0 : -1;
      cursorCompArcs = (_generation == currentGeneration &&
                        DagNode.this._compArcs != null) ? 0 : -1 ;
    }

    public boolean hasNext() {
      return cursorArcs != -1 || cursorCompArcs != -1;
    }

    public DagEdge next() {
      if (cursorArcs != -1) {
        // cursorArcs != -1
        if (cursorCompArcs == -1 ||
            (DagNode.this._compArcs.get(cursorCompArcs)._feature >
             DagNode.this._outedges.get(cursorArcs)._feature)) {
          int curr = cursorArcs++;
          if (cursorArcs == DagNode.this._outedges.size()) {
            cursorArcs = -1;
          }
          return DagNode.this._outedges.get(curr);
        }
      }
      // if (cursorCompArcs == -1) throw new NoSuchElementException();
      int curr = cursorCompArcs++;
      if (cursorCompArcs == DagNode.this._compArcs.size()) {
        cursorCompArcs = -1;
      }
      return DagNode.this._compArcs.get(curr);
    }

    public void add(DagEdge arc) {
      if (cursorCompArcs != -1) {
        // add it at the cursor and move it to the next element.
        if (cursorCompArcs == 0 ||
            (DagNode.this._compArcs.get(cursorCompArcs - 1)._feature
             < arc._feature)) {
          DagNode.this._compArcs.add(cursorCompArcs, arc);
        } else {
          DagNode.this._compArcs.add(cursorCompArcs - 1, arc);
        }
        ++cursorCompArcs;
      } else {
        // if cursorCompArcs is -1, it is ALWAYS at the end of the compArcs
        // list, either because the list was empty in the beginning, or the
        // compArcs have all been visited. Otherwise, the cursor would have a
        // non-negative value.
        if (DagNode.this._compArcs == null) {
          DagNode.this._compArcs = new ArrayList<DagEdge>();
          DagNode.this._compArcs.add(arc);
        } else {
          int lastIndex =  DagNode.this._compArcs.size() - 1;
          if (DagNode.this._compArcs.get(lastIndex)._feature > arc._feature) {
            DagNode.this._compArcs.add(lastIndex, arc);
          } else {
            DagNode.this._compArcs.add(arc);
          }
        }
      }
      /*
        int i = 0;
        while (i < DagNode.this.compArcs.size()) {
        if (i > 0 &&
        DagNode.this.compArcs.get(i-1).feature >=
        DagNode.this.compArcs.get(i).feature) {
        System.out.println("what");
        }
        ++i;
        }
      */
    }
  }


  public static void invalidate() {
    if (currentGeneration > currentGenerationMax)
      currentGenerationMax = currentGeneration;
    currentGeneration = ++currentGenerationMax;
  }

  // *************************************************************************
  // Constructors
  // *************************************************************************

  protected DagNode(int typeIdent) {
    _typeCode = typeIdent;
    _outedges = null;
    _isNominal = false;
    _generation = 0;
  }

  public DagNode(String string, DagNode dagNode) {
    this(TOP_ID);
    addEdge(new DagEdge(mapFeatureToNumber(string), dagNode));
  }

  public DagNode(short featureId, DagNode dagNode) {
    this(TOP_ID);
    addEdge(new DagEdge(featureId, dagNode));
  }

  public DagNode(String type) {
    this(mapTypeToNumber(type));
  }

  public DagNode() {
    this(TOP_ID);
  }

  protected DagNode clone(int type) {
    DagNode result = new DagNode(type);
    result._isNominal = _isNominal;
    return result;
  }

  // *************************************************************************
  // Grammar interface
  // *************************************************************************

  public static int getTypeId(String value) {
    return mapTypeToNumber(value);
  }

  public static short getFeatureId(String name) {
    return mapFeatureToNumber(name);
  }

  protected static short mapFeatureToNumber(String name) {
    if (nameToFeature.contains(name))
      return nameToFeature.getId(name);
    else
      return nameToFeature.register(name);
  }

  protected static int mapTypeToNumber(String name) {
    if (nameToType.contains(name))
      return nameToType.getId(name);
    else
      return nameToType.register(name);
  }

  public static String getFeatureName(short feature) {
    if (feature >= 0)
      return nameToFeature.fromId(feature);
    else
      return "ILL";
  }
  public static String getTypeName(int type) {
    return nameToType.fromId(type);
  }

  protected static int unifyTypes(int type1, int type2) {
    // TODO replace by type unification proper. We currently don't need this.
    return (type1 == type2 ? type1 : BOTTOM_ID);
  }

  protected boolean keepFeature(short feat) {
    return true;
  }

  /** does typeId1 subsumes (is more general than or equal to) typeId2 *
  public static boolean subsumesType(int typeId1, int typeId2) {
  }
  */

  // *************************************************************************
  // Operations specific to tomabechi dag representation
  // *************************************************************************

  public DagNode dereference() {
    if (_generation != currentGeneration || _forward == null) return this;
    return (_forward).dereference();
  }

  private void setForward(DagNode fs) {
    if (_generation != currentGeneration) {
      _newType = _typeCode;
      _copy = null;
      _compArcs = null;
      _generation = currentGeneration;
    }
    _forward = fs;
  }

  public DagNode getForward() {
    return ((_generation != currentGeneration) ? null : this._forward);
  }

  private void setCopy(DagNode fs) {
    if (_generation != currentGeneration) {
      _newType = _typeCode;
      _forward = null;
      _compArcs = null;
      _generation = currentGeneration;
    }
    _copy = fs;
  }

  private DagNode getCopy() {
    return (_generation != currentGeneration) ? null : this._copy;
  }

  /** An iterator that works for complete as well as transitional (unified
   *  but not copied) dags correctly.
   *  Printers may have to have access to this special iterator, therefore
   *  the visibility is default.
   * @return An iterator iterating over all edges of this dag
   */
  EdgeIterator getNewEdgeIterator() {
    return this.new EdgeIterator();
  }

  public boolean newEdgesAreEmpty() {
    return (_outedges == null &&
            ((_generation != currentGeneration) || this._compArcs == null));
  }

  public int getNewType() {
    return (_generation != currentGeneration) ? this._typeCode : this._newType;
  }

  private void setNewType(int what) {
    if (_generation != currentGeneration) {
      _forward = null;
      _copy = null;
      _compArcs = null;
      _generation = currentGeneration;
    }
    _newType = what;
  }

  private void setVisited(int what) { setNewType(what); }

  private int visited() {
    if (_generation != currentGeneration) return -1;
    return _newType;
  }

  // the next two only make sense in UtterancePlanner version
  public DagNode setNominal() {
    _isNominal = true;
    return this;
  }

  public boolean isNominal() {
    return _isNominal;
  }

  // *************************************************************************
  // END Operations specific to tomabechi dag representation
  // *************************************************************************

  public int getType() {
    return this._typeCode;
  }

  public void setType(int typeIdent) {
    this._typeCode = typeIdent;
  }

  public String getTypeName() {
    return getTypeName(this.getNewType());
  }

  public static void recordFailures(boolean state) {
    recordFailures = state;
  }

  public static void registerPrinter(DagPrinter printer) {
    _DEFAULT_PRINTER = printer;
  }

  private DagNode cloneFSRec() {
    DagNode newCopy = getCopy();
    if (newCopy == null) {
      newCopy = this.clone(getNewType());
      setCopy(newCopy);
      if (getEdges() != null) {
        for (DagEdge e : getEdges()) {
          newCopy.addEdge(e._feature, e._value.cloneFSRec());
        }
      }
    }
    return newCopy;
  }

  public DagNode cloneFS() {
    int generationSave = currentGeneration;
    currentGeneration = ++currentGenerationMax;
    // need a new generation to make a fresh copy: special invalidation
    DagNode clone = cloneFSRec();
    currentGeneration = generationSave;
    return clone;
  }

  public DagNode derefFS() {
    return dereference();
  }

  /** recursive helper function for copyResult() */
  private DagNode copyResultRec(boolean deleteDaughters) {
    DagNode in = this.dereference();
    DagNode newCopy = in.getCopy();
    if (newCopy != null) {
      return newCopy;
    }

    newCopy = clone(in.getNewType());
    in.setCopy(newCopy);

    int newsize = 0;
    int cursorArcs = -1, cursorCompArcs = -1;
    if (in._outedges != null && ! in._outedges.isEmpty()) {
      cursorArcs = 0;
      newsize = in._outedges.size();
    }
    if (in._generation == currentGeneration
        && (in._compArcs != null && ! in._compArcs.isEmpty())) {
      cursorCompArcs = 0;
      newsize += in._compArcs.size();
    }

    if (cursorArcs != -1 || cursorCompArcs != -1) {
      newCopy._outedges = new ArrayList<DagEdge>(newsize);
    }
    while (cursorArcs != -1 || cursorCompArcs != -1) {
      DagEdge arc = null ;
      if (cursorArcs != -1 &&
          (cursorCompArcs == -1
           || (in._compArcs.get(cursorCompArcs)._feature
               > in._outedges.get(cursorArcs)._feature))) {
        int curr = cursorArcs;
        if (++cursorArcs == in._outedges.size()) {
          cursorArcs = -1;
        }
        arc = in._outedges.get(curr);
      } else {
        int curr = cursorCompArcs;
        if (++cursorCompArcs == in._compArcs.size()) {
          cursorCompArcs = -1;
        }
        arc = in._compArcs.get(curr);
      }

      short feat = arc._feature;
      if (!deleteDaughters || keepFeature(feat)) {
        newCopy._outedges.add(
            new DagEdge(feat, arc._value.copyResultRec(false)));
      }
    }
    in._compArcs = null;
    // if resetting the copy slot is really necessary, it must be done AFTER
    // copying has finished in a new recursive walkthrough
    //in.setCopy(null);
    return newCopy;
  }


  /** Copy the result after a series of unifications.
   * @param deleteDaughters delete some top level features only concerned with
   *                        building the constituent tree (grammar specified)
   * @return a copied result independent from the input dag
   */
  public DagNode copyResult(boolean deleteDaughters) {
    // Return a copied result using the scratch buffer of this node
    DagNode result = copyResultRec(deleteDaughters);
    invalidate();
    return result;
  }

  /*
  private boolean makeWellformed(int unifiedType) {
    typeDag = typeDag.cloneFS();
    if (typeDag._outedges != null) {
      return unifyFS1(typeDag);
    }
    return true;
  }
  */

  /** Will always return true. Otherwise, the whole system breaks, because
   *  already applied changes could not be rolled back.
   */
  @SuppressWarnings("null")
  public boolean add(DagNode arg) {
    DagNode in1 = this.dereference();
    DagNode in2 = arg.dereference();
    if (in1 == in2) return true;

    int type1 = in1.getNewType();
    int type2 = in2.getNewType();

    in1._isNominal |= in2._isNominal;

    int unifType;
    // NO TYPE UNIFICATION --> NO FAILURE, this is intended.
    if (UNIFY_TYPES) {
      unifType = unifyTypes(type1, type2);
      if (unifType == BOTTOM_ID) {
        if (recordFailures)
          forwardFailures.put(this, FailType.TYPE);
        return false;
      }
    }
    else {
      // overwrite types
      unifType = type2;
    }


    in1.setNewType(unifType); // this makes all scratch slots of in1 current
    // when will nothing happen in wff unification (according to pet)
    // a) if the new type is the same as both old types
    // b) if the edge lists of both nodes are empty (including compArcs)
    // c) if the one that changed type has no edges (including compArcs)
    //    if ((type1 == unifType && type2 == unifType) ||
    //        (in1.newEdgesAreEmpty() && in2.newEdgesAreEmpty()) ||
    //        (type1 != unifType && in1.newEdgesAreEmpty()) ||
    //        (type2 != unifType && in2.newEdgesAreEmpty())) {
    // i was more picky, but that resulted in extreme performance degradation
    // if the type has changed, the edge lists of both must be empty
    /*
    if ((type1 == unifType && type2 == unifType)
        || (in1.newEdgesAreEmpty() && in2.newEdgesAreEmpty())
        || (type1 == unifType && ! in1.newEdgesAreEmpty())
        || (type2 == unifType && ! in2.newEdgesAreEmpty())) {
    } else {
      if (! in1.makeWellformed(unifType)) {
        if (recordFailures)
          forwardFailures.put(this, FailType.WELLFORMED);
        return false;
      }
      in1 = in1.dereference();
    }
    */
    in2.setForward(in1);  // this makes all scratch slots of in2 current

    EdgeIterator arc1It = in1.getNewEdgeIterator();
    EdgeIterator arc2It = in2.getNewEdgeIterator();

    // the test if the iterators are null is not necessary here (can't occur
    // with a call to getNewEdgeIterator)
    DagEdge arc1 = null, arc2 = null;
    short feat1 = ((arc1It != null && arc1It.hasNext())
                   ? (arc1 = arc1It.next())._feature : NO_FEAT);
    short feat2 = ((arc2It != null && arc2It.hasNext())
                   ? (arc2 = arc2It.next())._feature : NO_FEAT);

    while (feat1 != NO_FEAT || feat2 != NO_FEAT) {
      while (feat1 < feat2) {
        // feature in 1 but not in 2: skip
        feat1 = (arc1It.hasNext() ? (arc1 = arc1It.next())._feature : NO_FEAT);
      }
      while (feat1 > feat2) { // feature in 2 missing in 1: add to compArcs
        arc1It.add(arc2);
        if (arc2._value._isNominal) {
          this._isNominal = true;
        }
        feat2 = (arc2It.hasNext() ? (arc2 = arc2It.next())._feature : NO_FEAT);
      }
      if (feat1 == feat2 && feat1 != NO_FEAT) {
        // this differs from ordinary unification
        DagNode subThis = arc1._value;
        DagNode subArg = arc2._value;
        if (subThis._isNominal || subArg._isNominal) {
          // "relation" arcs
          if (! (subThis._isNominal && subArg._isNominal)) {
            logger.warn("Status of relation/feature unclear " +
                "during unification: " + getFeatureName(feat1));
          }
          arc1It.add(arc2);
        }
        else { // "feature" arcs
          if (subThis.getClass() == DagNode.class) {
            if (! subThis.add(subArg))
              return false;;
          }
        }

        feat1 = (arc1It.hasNext() ? (arc1 = arc1It.next())._feature : NO_FEAT);
        feat2 = (arc2It.hasNext() ? (arc2 = arc2It.next())._feature : NO_FEAT);
      }
    }

    return true;
  }


  /** This method is called on the replace part of the rules. It
   *  therefore can contain var nodes, etc., which are not part
   *  of ordinary dags.
   */
  public final DagNode expandVars(Bindings bindings) {
    DagNode here = dereference();
    if (here._outedges != null) {
      List<DagNode> varDags = new LinkedList<DagNode>();
      Iterator<DagEdge> it = _outedges.iterator();
      // collect all ID edges with special nodes and evaluate their content.
      // the original edges are deleted
      while (it.hasNext()) {
        DagEdge edge = it.next();
        DagNode sub = edge._value;
        if (edge._feature == ID_FEAT_ID &&
            sub instanceof SpecialDagNode) {
          varDags.add(((SpecialDagNode)sub).evaluate(this, bindings));
          it.remove();
        }
        else {
          // either other feature or relation. call recursively
          edge._value = sub.expandVars(bindings);
        }
      }
      for(DagNode varDag : varDags) {
        // unify the contents of the var into this node
        here.add(varDag);
      }
    } else {
      // __TYPE or __PROP
      if (this instanceof SpecialDagNode) {
        return ((SpecialDagNode)this).evaluate(this, bindings);
      }
    }
    return here;
  }


  private int countCorefsRec(int maxCoref) {
    if (visited() < 0) {
      setVisited(0);
      for(EdgeIterator edgeIt = new EdgeIterator(); edgeIt.hasNext();) {
        DagEdge fvpair = edgeIt.next();
        maxCoref = fvpair.getValue().countCorefsRec(maxCoref);
      }
    } else {
      if (visited() == 0) {
        setVisited(++maxCoref);
      }
    }
    return maxCoref;
  }

  public int countCorefs() {
    int result = countCorefsRec(0);
    return result;
  }

  /*
  @SuppressWarnings("null")
  private int subsumesBiRec(DagNode in2, int result) {
    { DagNode fs1 = this.getForward();
      if ((result & THIS_MORE_GENERAL) != 0) {
        if (fs1 == null) {
          this.setForward(in2);
        } else {
          if (fs1 != in2) { // forward = false
            if (recordFailures)
              forwardFailures.put(this, FailType.VARIABLE);
            if ((result &= THIS_MORE_GENERAL_MASK) == 0) return 0;
          }
        }
      }
    }
    { DagNode fs2 = in2.getCopy();
      if ((result & ARG_MORE_GENERAL) != 0) {
        if (fs2 == null) {
          in2.setCopy(this);
        } else {
          if (fs2 != this) {  // backward = false
            if (recordFailures)
              backwardFailures.put(this, FailType.VARIABLE);
            if ((result &= ARG_MORE_GENERAL_MASK) == 0) return 0;
          }
        }
      }
    }
    int type1 = this.getNewType();
    int type2 = in2.getNewType();
    if (type1 != type2) {
      if (! subsumesType(type1, type2)) {
        if (recordFailures)
          forwardFailures.put(this, FailType.TYPE);
        if ((result &= THIS_MORE_GENERAL_MASK) == 0) return 0;
      }
      if (! subsumesType(type2, type1)) {
        if (recordFailures)
          backwardFailures.put(this, FailType.TYPE);
        if ((result &= ARG_MORE_GENERAL_MASK) == 0) return 0;
      }
    }

    List<DagEdge> edges1 = this.getEdges();
    List<DagEdge> edges2 = in2.getEdges();
    if (edges1 == null || edges2 == null) {
      if (edges1 != edges2) {
        if (edges1 == null) {
          if (recordFailures)
            backwardFailures.put(this, FailType.FEATURE);
          if ((result &= ARG_MORE_GENERAL_MASK) == 0) return 0;
        } else {
          if (recordFailures)
            forwardFailures.put(this, FailType.FEATURE);
          if ((result &= THIS_MORE_GENERAL_MASK) == 0) return 0;
        }
      }
      return result;
    }

    Iterator<DagEdge> arc1It = edges1.iterator();
    Iterator<DagEdge> arc2It = edges2.iterator();
    DagEdge arc1 = null, arc2 = null;
    int feat1 = (arc1It.hasNext() ? (arc1 = arc1It.next())._feature : NO_FEAT);
    int feat2 = (arc2It.hasNext() ? (arc2 = arc2It.next())._feature : NO_FEAT);
    while (feat1 != NO_FEAT && feat2 != NO_FEAT) {
      if (feat1 < feat2) { // feature in 1 missing in 2: no forward
        if (recordFailures)
          forwardFailures.put(this, FailType.FEATURE);
        if ((result &= THIS_MORE_GENERAL_MASK) == 0) return 0;
        while (feat1 < feat2) {
          feat1 = (arc1It.hasNext() ? (arc1 = arc1It.next())._feature : NO_FEAT);
        }
      }
      if (feat1 > feat2) { // feature in 2 missing in 1: no backward
        if (recordFailures)
          backwardFailures.put(this, FailType.FEATURE);
        if ((result &= ARG_MORE_GENERAL_MASK) == 0) return 0;
        while (feat1 > feat2) {
          feat2 = (arc2It.hasNext() ? (arc2 = arc2It.next())._feature : NO_FEAT);
        }
      }
      if (feat1 == feat2 && feat1 != NO_FEAT) {
        if ((result = arc1._value.subsumesBiRec(arc2._value, result)) == 0)
          return 0;
        feat1 = (arc1It.hasNext() ? (arc1 = arc1It.next())._feature : NO_FEAT);
        feat2 = (arc2It.hasNext() ? (arc2 = arc2It.next())._feature : NO_FEAT);
      }
    }
    if (feat1 != feat2) {
      if (feat1 == NO_FEAT) { // more features in arg: this is more general
        if (recordFailures)
          backwardFailures.put(this, FailType.FEATURE);
        result &= ARG_MORE_GENERAL_MASK;
      } else {
        if (recordFailures)
          forwardFailures.put(this, FailType.FEATURE);
        result &= THIS_MORE_GENERAL_MASK;
      }
    }

    return result;
  }


  /** compute the subsumption relation between this and fs in both directions:
   * FORWARD subsumption means that `this' subsumes (is a more general)
   * structure than `fs', while
   * BACKWARD subsumption means that `this' is subsumed by (more informative
   * than) `fs'
   *
  public int subsumesBi(DagNode fs) {
    if (recordFailures) {
      forwardFailures.clear();
      backwardFailures.clear();
    }
    int result = subsumesBiRec(fs, THIS_MORE_GENERAL + ARG_MORE_GENERAL);
    invalidate();
    return result;
  }

  @SuppressWarnings("null")
  private boolean subsumesRec(DagNode in2) {
    { DagNode fs1 = this.getForward();
      if (fs1 == null) {
        this.setForward(in2);
      } else {
        if (fs1 != in2) { // forward = false
          return false;
        }
      }
    }

    int type1 = this.getNewType();
    int type2 = in2.getNewType();
    if (type1 != type2) {
      if (! subsumesType(type1, type2)) {
        return false;
      }
    }

    List<DagEdge> edges1 = this.getEdges();
    List<DagEdge> edges2 = in2.getEdges();
    if (edges2 == null) {
      return (edges1 == edges2);
    }
    if (edges1 == null) {
      return true;
    }

    Iterator<DagEdge> arc1It = edges1.iterator();
    Iterator<DagEdge> arc2It = edges2.iterator();
    DagEdge arc1 = null, arc2 = null;
    int feat1 = (arc1It.hasNext() ? (arc1 = arc1It.next())._feature : NO_FEAT);
    int feat2 = (arc2It.hasNext() ? (arc2 = arc2It.next())._feature : NO_FEAT);
    while (feat1 != NO_FEAT && feat2 != NO_FEAT) {
      if (feat1 < feat2) { // feature in 1 missing in 2: no forward
        return false;
      }
      if (feat1 > feat2) { // feature in 2 missing in 1: no backward
        while (feat1 > feat2) {
          feat2 = (arc2It.hasNext() ? (arc2 = arc2It.next())._feature : NO_FEAT);
        }
      }
      if (feat1 == feat2 && feat1 != NO_FEAT) {
        if (! arc1._value.subsumesRec(arc2._value)) return false;
        feat1 = (arc1It.hasNext() ? (arc1 = arc1It.next())._feature : NO_FEAT);
        feat2 = (arc2It.hasNext() ? (arc2 = arc2It.next())._feature : NO_FEAT);
      }
    }
    return ((feat1 == feat2) || (feat1 == NO_FEAT));
  }

  /** Return true if `this' is more general than fs *
  public boolean subsumes(DagNode fs) {
    boolean result = subsumesRec(fs);
    invalidate();
    return result;
  }


  /** return true if fs is more general than `this' *
  public boolean isSubsumedBy(DagNode fs) {
    boolean result = (fs).subsumesRec(this);
    invalidate();
    return result;
  }
  */

  /** Check all possible combinations of edge values for equality. Since we
   *  may have edges with identical features, we have to do this expensive
   *  check.
   */
  private boolean
  equalsCrossCheck(List<DagNode> equals1, List<DagNode> equals2,
      IdentityHashMap<DagNode, IdentityHashMap<DagNode, Boolean>> eqClasses) {
    boolean result = false;
    for (int i = 0; i < equals1.size(); ++i) {
      for (int j = i; j < equals1.size(); ++j) {
        boolean localResult =
          equals1.get(i).equalsRecMulti(equals2.get(j), eqClasses);
        result |= localResult;
      }
    }
    return result;
  }

  /** recursive helper function for equals with multiple edges with the same
   *  feature. This only tests local failures. In the end the validity of
   *  variable bindings has to be checked by computing a perfect matching of
   *  the corresponding nodes of dag1 and dag2 from eqClasses.
   */
  @SuppressWarnings("null")
  private boolean equalsRecMulti(DagNode in2,
      IdentityHashMap<DagNode, IdentityHashMap<DagNode, Boolean>> eqClasses) {
    IdentityHashMap<DagNode, Boolean> eqToThis = eqClasses.get(this);
    if (eqToThis == null) {
      eqToThis = new IdentityHashMap<DagNode, Boolean>();
      eqClasses.put(this, eqToThis);
    } else {
      if (eqToThis.containsKey(in2))
        return eqToThis.get(in2);
    }

    int type1 = this.getType();
    int type2 = in2.getType();
    if (type1 != type2) {
      eqToThis.put(in2, false);
      return false;
    }


    List<DagEdge> edges1 = this.getEdges();
    List<DagEdge> edges2 = in2.getEdges();
    /* because we ignore ID
    if (edges1 == null || edges2 == null) {
      if (edges1 != edges2) {
        eqToThis.put(in2, false);
        return false;
      }
      eqToThis.put(in2, true);
      return true;
    }

    /* because we ignore ID
    if (edges1.size() != edges2.size()) {
      eqToThis.put(in2, false);
      return false;
    }
    */
    // temporarily, to avoid infinite recursion in case of cycle

    eqToThis.put(in2, true);

    DagEdge arc1 = null, arc2 = null;
    int feat1 = NO_FEAT, feat2 = NO_FEAT;
    Iterator<DagEdge> arc1It = null, arc2It = null;
    if (edges1 != null) {
      arc1It = edges1.iterator();
      feat1 = (arc1It.hasNext() ? (arc1 = arc1It.next())._feature : NO_FEAT);
    }
    if (edges2 != null) {
      arc2It = edges2.iterator();
      feat2 = (arc2It.hasNext() ? (arc2 = arc2It.next())._feature : NO_FEAT);
    }
    /** FIXME assuming ID is the first feature */
    if (feat1 == ID_FEAT_ID)
      feat1 = (arc1It.hasNext() ? (arc1 = arc1It.next())._feature : NO_FEAT);
    if (feat2 == ID_FEAT_ID)
      feat2 = (arc2It.hasNext() ? (arc2 = arc2It.next())._feature : NO_FEAT);

    while (feat1 == feat2 &&  feat1 != NO_FEAT) {
      /** Collect all edges on both sides that have this feature */
      int feat = feat1;
      List<DagNode> equals1 = new ArrayList<DagNode>();
      List<DagNode> equals2 = new ArrayList<DagNode>();
      while (feat1 == feat && feat2 == feat) {
        equals1.add(arc1.getValue());
        equals2.add(arc2.getValue());
        feat1 = (arc1It.hasNext() ? (arc1 = arc1It.next())._feature : NO_FEAT);
        feat2 = (arc2It.hasNext() ? (arc2 = arc2It.next())._feature : NO_FEAT);
      }
      if (feat1 != feat2         // avoid unnecessary cross checks
          || ! equalsCrossCheck(equals1, equals2, eqClasses)) {
        eqToThis.put(in2, false);
        return false;
      }
    }
    if (feat1 != feat2) {
      eqToThis.put(in2, false);
      return false;
    }
    return true;
  }


  @Override
  public boolean equals(Object obj) {
    if (! (obj instanceof DagNode)) return false;
    IdentityHashMap<DagNode, IdentityHashMap<DagNode, Boolean>> eqClasses =
      new IdentityHashMap<DagNode, IdentityHashMap<DagNode, Boolean>>();
    boolean result = equalsRecMulti((DagNode) obj, eqClasses);
    if (result) {
      int size = eqClasses.size();
      // check if we find a proper match for eqClasses
      // create a bipartite graph from the equality classes
      BipartiteGraph bg = new BipartiteGraph(size, size);
      IdentityHashMap<DagNode, Integer> nodeNumbering =
        new IdentityHashMap<DagNode, Integer>();
      int nextSourceNumber = 0;
      for(DagNode source : eqClasses.keySet()) {
        int nodeNumber = nextSourceNumber++;
        IdentityHashMap<DagNode, Boolean> targets = eqClasses.get(source);
        for (DagNode target : targets.keySet()) {
          int targetNumber = 0;
          if (nodeNumbering.containsKey(target)) {
            targetNumber = nodeNumbering.get(target);
          }
          else {
            targetNumber = nodeNumbering.size();
            nodeNumbering.put(target, targetNumber);
            if (targetNumber > size) {
              logger.warn("More target that source nodes");
              return false;
            }
          }
          if (targets.get(target)) {
            bg.add(nodeNumber, targetNumber);
          }
        }
      }
      assert(nextSourceNumber == eqClasses.size());
      if (nodeNumbering.size() != eqClasses.size()) {
        result = false;
      }
      else {
        int match = bg.hopcroft_karp();
        result = (match == eqClasses.size());
      }
    }
    invalidate();
    return result;
  }

  /** We assume the feature list is either equal to null or not empty. Other
   *  code can rely on that
   */
  private void addEdge(DagEdge undumpArc) {
    if (null == _outedges) _outedges = new ArrayList<DagEdge>(3);
    _outedges.add(undumpArc);
  }

  public void addEdge(short featCode, DagNode fs) {
    addEdge(new DagEdge(featCode, fs));
  }

  public void sortEdges() {
    Collections.sort(_outedges,
        new Comparator<DagEdge>(){
          @Override
          public int compare(DagEdge arg0, DagEdge arg1) {
            return arg0._feature - arg1._feature;
          }
    });
  }

  private ArrayList<DagEdge> getEdges() {
    return _outedges;
  }

  public Iterator<DagEdge> getEdgeIterator() {
    return _outedges == null ? emptyEdges : _outedges.iterator();
  }

  public EdgeIterator getTransitionalEdgeIterator() {
    return new EdgeIterator();
  }

  /* return the substructure under feature, if existent, null otherwise
   * could be improved using binary or interpolation search.
   * Works correctly only on non-temporary dags.
   */
  public DagEdge getEdge(short feature) {
    if (_outedges == null) return null;
    for (DagEdge edge : _outedges) {
      int f = edge.getFeature();
      if (f == feature)
        return edge;
      if (f > feature)
        break;
    }
    return null;
  }

  private class EdgesIterator implements Iterator<DagEdge> {
    private Iterator<DagEdge> _impl;
    DagEdge _curr = null;

    public EdgesIterator(short feature) {
      if (_outedges != null) {
        _impl = _outedges.iterator();
        while (_impl.hasNext() &&
               (_curr = _impl.next())._feature < feature) {}
      }
      if (_curr != null && _curr._feature != feature) {
        _curr = null;
      }
    }

    @Override public boolean hasNext() { return _curr != null; }

    @Override public DagEdge next() {
      DagEdge now = _curr;
      if (_impl.hasNext()) {
        _curr = _impl.next();
        if (_curr._feature != now._feature) _curr = null;
      }
      else {
        _curr = null;
      }
      return now;
    }

    @Override
    public void remove() { throw new UnsupportedOperationException(); }
  }

  /**
   * this function is not present in the standard version because features
   * are unique there
   */
  public Iterator<DagEdge> getEdges(short feature) {
    return new EdgesIterator(feature);
  }

  // clear the edges slot if it is not needed
  protected void edgesAreEmpty() {
    if (_outedges != null && _outedges.size() == 0) _outedges = null;
  }

  // **************************************************************************
  // BEGIN general convenience functions, path/arg access, etc.
  // **************************************************************************

  /** assign coref numbers to coreferenced nodes. Only nodes that are referred
   *  to more than once get a number greater that zero, all other nodes get
   *  zero.
   *  @return the number of nodes that were referenced more than once.
   */
  public int
  countCorefsLocal(IdentityHashMap<DagNode, Integer> corefs, int nextCorefNo) {
    DagNode here = dereference();
    if (! corefs.containsKey(here)) { // visited for the first time
      corefs.put(here, 0);
      EdgeIterator fvListIt = here.getNewEdgeIterator();
      if (fvListIt != null) {
        while(fvListIt.hasNext()) {
          DagNode sub = fvListIt.next().getValue();
          if (sub != null)
            nextCorefNo = sub.countCorefsLocal(corefs, nextCorefNo);
        }
      }
    } else {
      int corefNo = corefs.get(here);
      if (corefNo == 0) { // visited for the second time at least
        corefs.put(here, ++nextCorefNo);
      }
    }
    return nextCorefNo;
  }

  /** Get the node under the given path.
   *  Works correctly only on non-temporary dags.
   */
  public DagNode getSubNode(Iterator<Short> path) {
    DagNode current = this;
    while (path.hasNext() && current != null) {
      DagEdge next = current.getEdge(path.next());
      current = (next == null) ? null : next.getValue();
    }
    return current;
  }

  /** Remove the edge with the given feature. Works correctly only on
   * non-temporary dags.
   */
  public void removeEdge(short feature) {
    DagNode here = dereference();
    Iterator<DagEdge> it;
    if (here._outedges != null) {
      it = here._outedges.iterator();
      while (it.hasNext()) {
        DagEdge edge = it.next();
        if (edge.getFeature() == feature) {
          it.remove();
          return;
        }
      }
    }
    if (here._compArcs != null) {
      it = here._compArcs.iterator();
      while (it.hasNext()) {
        DagEdge edge = it.next();
        if (edge.getFeature() == feature) {
          it.remove();
          return;
        }
      }
    }
  }

  // *************************************************************************
  // Begin Restrictor and Error Case Reduction
  // *************************************************************************

  private RESTRICT getRestrictorType() {
    return RESTRICT.values()[getType()];
  }

  @SuppressWarnings("null")
  public void restrict(DagNode restrictor) {
    RESTRICT restrictType = restrictor.getRestrictorType();
    // don't touch anything beyond this node
    if (restrictType == RESTRICT.FULL) return;

    Iterator<? extends DagEdge> arc1It = this.getEdgeIterator();
    Iterator<? extends DagEdge> arc2It = restrictor.getEdgeIterator();
    DagEdge arc1 = null, arc2 = null;
    short feat1 = ((arc1It != null && arc1It.hasNext())
                   ? (arc1 = arc1It.next()).getFeature() : NO_FEAT);
    short feat2 = ((arc2It != null && arc2It.hasNext())
                   ? (arc2 = arc2It.next()).getFeature() : NO_FEAT);

    while (feat1 != NO_FEAT || feat2 != NO_FEAT) {
      while (feat1 < feat2) {
        // feature in this dag but not in restrictor
        if (restrictType == RESTRICT.KEEP) {
          arc1It.remove(); // delete the not mentioned feature
        }
        feat1 = (arc1It.hasNext()
                 ? (arc1 = arc1It.next()).getFeature() : NO_FEAT);
      }
      while (feat1 > feat2) { // feature in restrictor missing in this dag
        feat2 = (arc2It.hasNext()
                 ? (arc2 = arc2It.next()).getFeature() : NO_FEAT);
      }
      if (feat1 == feat2 && feat1 != NO_FEAT) {
        if (restrictType == RESTRICT.REMOVE ||
            arc2.getValue().getRestrictorType() == RESTRICT.DELETE) {
          arc1It.remove();
        } else {
          arc1.getValue().restrict(arc2.getValue());
        }
        feat1 = (arc1It.hasNext()
                 ? (arc1 = arc1It.next()).getFeature() : NO_FEAT);
        feat2 = (arc2It.hasNext()
                 ? (arc2 = arc2It.next()).getFeature() : NO_FEAT);
      }
    }
    edgesAreEmpty();
  }

  /*
  private void restrictSimpleRec(HashSet<DagNode> visited) {
    Iterator<? extends DagEdge> arcIt = this.getEdgeIterator();
    if (visited.contains(this) || arcIt == null) return;
    visited.add(this);
    while (arcIt.hasNext()) {
      DagEdge arc = arcIt.next();
      if (! keepFeature(arc.getFeature())) {
        arcIt.remove();
      } else {
        arc.getValue().restrict();
      }
    }
    edgesAreEmpty();
  }

  public void restrict() {
    restrictSimpleRec(new HashSet<DagNode>());
  }

  public void reduceRec(TFSErrorProducer ep, DagNode resSubNode,
                        HashSet<DagNode> visited) {
    if (visited.contains(this)) return;
    visited.add(this);
    Iterator<? extends DagEdge> currentEdgeIt = this.getEdgeIterator();
    if (currentEdgeIt == null) return;
    // while (featureToRemove still available)
    while (currentEdgeIt.hasNext()) {
      // check error with enhanced restrictor (add featureToRemove)
      DagEdge currentEdge = currentEdgeIt.next();
      resSubNode.addEdge(currentEdge.getFeature(),
                         new DagNode(RESTRICT.DELETE.ordinal()));
      // if error persists, keep restrictor, otherwise remove last feature
      if (ep.errorPersists()) {
        // error is still there, proceed to the next feature
        // nothing to do here
      } else {
        // first remove current edge from restrictor
        resSubNode.removeEdge(currentEdge.getFeature());
        DagNode resSubNext =
          new DagNode(RESTRICT.NO.ordinal());
        // try it one level deeper
        resSubNode.addEdge(currentEdge.getFeature(), resSubNext);
        currentEdge.getValue().reduceRec(ep, resSubNext, visited);
      }
      // get next featureToRemove
    }
  }
  */


  // *************************************************************************
  // Begin construct jxchg string representation from (permanent) dag
  // *************************************************************************

  private void toStringRec(boolean readable, StringBuilder sb,
                           IdentityHashMap<DagNode, Integer> corefs) {
    DagNode here = this.dereference();
    int corefNo = corefs.get(here);
    if (corefNo < 0) { // already printed, only coref
      sb.append(" #").append(-corefNo).append(' ');
      return;
    }

    if (corefNo > 0) {
      sb.append(" #").append(corefNo).append(' ');
      corefs.put(here, -corefNo);
    }

    sb.append('[');
    sb.append(readable ? here.getTypeName() : here.getType());
    EdgeIterator fvListIt = here.getNewEdgeIterator();
    if (fvListIt != null && fvListIt.hasNext()) {
      while(fvListIt.hasNext()) {
        DagEdge edge = fvListIt.next();
        sb.append(' ');
        sb.append(readable ? edge.getName() : edge.getFeature());
        edge.getValue().toStringRec(readable, sb, corefs);
      }
    }
    else {
      sb.append('@').append(here.getTypeName()).append('@');
    }
    sb.append(']');
  }

  /** print fs in jxchg format */
  @Override
  public final String toString() {
    IdentityHashMap<DagNode, Integer> corefMap =
      new IdentityHashMap<DagNode, Integer>();
    int corefs = 0;
    corefs = countCorefsLocal(corefMap, corefs);
    StringBuilder sb = new StringBuilder();
    if (this instanceof SpecialDagNode) {
      ((SpecialDagNode)this).toStringSpecial(sb);
    }
    else {
      if (_DEFAULT_PRINTER != null) {
        _DEFAULT_PRINTER.getCorefs(this);
        _DEFAULT_PRINTER.toStringRec(this, PRINT_READABLE, sb);
      }
      else {
        toStringRec(PRINT_READABLE, sb, corefMap);
      }
    }
    return sb.toString();
  }


}
