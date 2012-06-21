package de.dfki.lt.tr.dialogue.cplanwrapper;

// the planner proper
import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.LinkedList;
import java.util.List;

import org.apache.log4j.BasicConfigurator;

import de.dfki.lt.tr.dialogue.cplan.DagEdge;
import de.dfki.lt.tr.dialogue.cplan.DagNode;
import de.dfki.lt.tr.dialogue.cplan.UtterancePlanner;
import de.dfki.lt.tr.dialogue.slice.lf.Feature;
import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.slice.lf.LFRelation;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.util.LFUtils;

public class CPlanWrapper {

  private UtterancePlanner _planner;

  /** Initialize the content planner by providing it with a file that contains
   *   the rule files to load.
   */
  public CPlanWrapper(File topRuleFile, File pluginDirectory) {
    // make this into a configuration file with a rules and a plugins field.
    initialize(topRuleFile, pluginDirectory);
  }

  private void initialize(File ruleFile, File pluginDirectory) {
    _planner = new UtterancePlanner(pluginDirectory);
    _planner.readProjectFile(ruleFile);
  }

  private static LFNominal
  collectNominals(DagNode dag, List<LFNominal> nominals,
                  IdentityHashMap<DagNode, LFNominal> coreferences) {
    assert(dag.isNominal());

    // collect the `special' tokens for nominals
    DagNode prop = null ;
    DagNode id = null;
    DagNode type = null;

    DagNode.EdgeIterator it = dag.getTransitionalEdgeIterator();
    DagEdge edge = null;
    while (it.hasNext()) {
      edge = it.next();
      short feature = edge.getFeature();
      if (feature == DagNode.ID_FEAT_ID) {
        id = edge.getValue().dereference(); edge = null;
      } else if (feature == DagNode.TYPE_FEAT_ID) {
        type = edge.getValue().dereference(); edge = null;
      } else if (feature == DagNode.PROP_FEAT_ID) {
        prop = edge.getValue().dereference(); edge = null;
      } else {
        break;
      }
    }

    LFNominal newNominal = new LFNominal();
    newNominal.nomVar =
      ((id == null) ? "nom" + nominals.size() : id.getTypeName());
    // type == null should not happen
    if (type != null) {
      newNominal.sort = type.getTypeName();
    }
    // prop == null should not happen
    if (prop != null) {
      newNominal.prop = LFUtils.newProposition(prop.getTypeName());
    }
    else {
      newNominal.prop = LFUtils.newProposition();
    }

    nominals.add(newNominal);
    coreferences.put(dag, newNominal);

    List<Feature> features = new LinkedList<Feature>();
    List<LFRelation> relations = new LinkedList<LFRelation>();

    while(edge != null) {
      DagNode val = edge.getValue();
      if (val.isNominal()) {
        LFRelation rel = new LFRelation();
        rel.head = newNominal.nomVar;
        rel.mode = edge.getName();
        if (coreferences.containsKey(val)) {
          rel.dep = coreferences.get(val).nomVar;
          //rel.coIndexedDep = true;
          // FIXME what about this
        }
        else { // recursive call, not yet transformed
          LFNominal sub = collectNominals(val, nominals, coreferences);
          rel.dep = sub.nomVar;
          //rel.coIndexedDep = false;
        }
        relations.add(rel);
      }
      else {
        DagNode featVal = val.getEdge(DagNode.PROP_FEAT_ID).getValue();
        features.add(new Feature(edge.getName(), featVal.getTypeName()));
      }

      if (it.hasNext()) {
        edge = it.next();
      } else {
        edge = null;
      }
    }
    newNominal.feats = features.toArray(new Feature[features.size()]);
    newNominal.rels = relations.toArray(new LFRelation[relations.size()]);

    return newNominal;
  }

  /** Convert inner to outer form, i.e., a dag into a LF */
  static LogicalForm dagNodeToLf(DagNode dag) {
    LogicalForm result = new LogicalForm();

    IdentityHashMap<DagNode, LFNominal> coreferences =
      new IdentityHashMap<DagNode, LFNominal>();
    // recursively collect all "Nominals" into a list
    List<LFNominal> nominals = new LinkedList<LFNominal>();
    LFNominal root = collectNominals(dag, nominals, coreferences);

    result.noms = nominals.toArray(new LFNominal[nominals.size()]);
    result.root = root;
    result.logicalFormId = "";
    result.stringPos = 0;

    return result;
  }


  private static DagNode nominalToDagNode(LFNominal nom) {
    DagNode result = new DagNode();
    result.setNominal();
    result.addEdge(DagNode.ID_FEAT_ID, new DagNode(nom.nomVar));
    result.addEdge(DagNode.TYPE_FEAT_ID, new DagNode(nom.sort));
    //if (nom.prop.connective != ConnectiveType.NONE) {
    result.addEdge(DagNode.PROP_FEAT_ID, new DagNode(nom.prop.prop));
    //}
    for (Feature feat : nom.feats) {
      DagNode valNode = new DagNode();
      valNode.addEdge(DagNode.PROP_FEAT_ID, new DagNode(feat.value));
      result.addEdge(DagNode.getFeatureId(feat.feat), valNode);
    }
    return result;
  }

  /** Convert outer to inner form, i.e., a LF into a dag */
  static DagNode lfToDagNode(LogicalForm lf) {
    HashMap<String, DagNode> nameToNom = new HashMap<String, DagNode>();
    // create a DagNode for each nominals where the relations are still missing
    for (LFNominal nom : lf.noms) {
      nameToNom.put(nom.nomVar, nominalToDagNode(nom));
    }
    // now add the relation edges
    for (LFNominal nom : lf.noms) {
      DagNode dag = nameToNom.get(nom.nomVar);
      for (LFRelation rel : nom.rels) {
        dag.addEdge(DagNode.getFeatureId(rel.mode), nameToNom.get(rel.dep));
      }
      dag.sortEdges();
    }
    return nameToNom.get(lf.root.nomVar);
  }

  /** call the content planner */
  public LogicalForm callContentPlanner(LogicalForm input) {

    DagNode inputDag = lfToDagNode(input);

    System.out.println("[CPlan:InputDag] " + inputDag);

    DagNode planningResult = _planner.process(inputDag);
    System.out.println("[CPlan:outputDag] " + planningResult);

    return (planningResult == null) ? null : dagNodeToLf(planningResult);
  }

  public static void main(String[] args) {
    BasicConfigurator.configure();
    CPlanWrapper cp =
      new CPlanWrapper(new File(args[0]),
          (args.length > 1 ? new File(args[1]) : null));

    BufferedReader stdin = new BufferedReader(new InputStreamReader(System.in));
    String line;
    try {
      while ((line = stdin.readLine()) != null) {
        LogicalForm lf = LFUtils.convertFromString(line);

        LogicalForm result = cp.callContentPlanner(lf);

        System.out.println(CPlanWrapper.lfToDagNode(result).toString());

      }
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

}
