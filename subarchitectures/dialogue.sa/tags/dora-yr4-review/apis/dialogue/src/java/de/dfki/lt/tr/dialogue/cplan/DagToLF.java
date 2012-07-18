package de.dfki.lt.tr.dialogue.cplan;

import java.io.IOException;
import java.io.StringReader;
import java.util.IdentityHashMap;

import opennlp.ccg.realize.Realizer;
import opennlp.ccg.synsem.LF;

import org.apache.log4j.Logger;
import org.jdom.DocType;
import org.jdom.Document;
import org.jdom.Element;

public class DagToLF {
  /** This method creates a new diamond-element representing a feature of
   *  the given LFNominal. It appends this new Element to the given one and
   *  returns the combined Element. It is a help-method for creating an
   *  XML-document that represents a given LogicalForm
   *
   * @return Element that represents the parent nominal
   * @param sub the sub-dag containing the value of the feature under PROP
   * @param feature the feature name
   * @param el the XML element to add the new diamond-element to (parent)
   * Element to which will be appended
   */
  private static Element editFeatures(Element el, DagNode sub, String feature){

    // create root diamond node
    Element diamond = new Element("diamond");
    diamond = diamond.setAttribute("mode",feature);

    // create its proposition node
    Element prop = new Element("prop");
    prop = prop.setAttribute("name",
        sub.getEdge(DagNode.PROP_FEAT_ID).getValue().getTypeName());

    // add all in a tree-like manner
    diamond = diamond.addContent(prop);
    el = el.addContent(diamond);
    return el;
  } // end editFeatures

  /** This method handles the innards of a nominal node. There are subtle
   *  differences depending on whether the node is the top node or not. If the
   *  root node is processed, relMode has to be <code>null</code>
   *  @return the modified diamond node
   *  @param sub the sub-node that represents the diamond node
   *  @param relMode the name of the relation under which this node is put
   *  @param coreferences a hash map containing the names of all nominals that
   *        have been converted already. If sub represents such a nominal, only
   *        the name will be added, not the sub-features or -relations
   */
  private static Element handleContent(Element diamond, DagNode sub,
      String relMode, IdentityHashMap<DagNode, String> coreferences) {

    if (coreferences.containsKey(sub)) {
      // create its nom node containing dependent and type
      Element nom = new Element("nom");
      nom = nom.setAttribute("name", coreferences.get(sub));
      // add only the name
      diamond = diamond.addContent(nom);
    }
    else {
      // collect the `special' tokens for nominals
      DagNode propDag = null ;
      DagNode id = null;
      DagNode type = null;

      DagNode.EdgeIterator it = sub.getTransitionalEdgeIterator();
      DagEdge edge = null;
      while (it.hasNext()) {
        edge = it.next();
        short feature = edge.getFeature();
        if (feature == DagNode.ID_FEAT_ID) {
          id = edge.getValue().dereference(); edge = null;
        } else if (feature == DagNode.TYPE_FEAT_ID) {
          type = edge.getValue().dereference(); edge = null;
        } else if (feature == DagNode.PROP_FEAT_ID) {
          propDag = edge.getValue().dereference(); edge = null;
        } else {
          break;
        }
      }

      String nomVar =
        ((id == null) ? "nom" + coreferences.size() : id.getTypeName());
      // type == null should not happen
      String sort = (type == null ? "top" : type.getTypeName());
      String propName = (propDag == null ? "" : propDag.getTypeName());

      // name of nom = "id:type"
      String s = nomVar + ":" + sort;
      coreferences.put(sub, s);
      // add the nominal, in case of root node, it's the name attribute
      if (relMode == null) {
        diamond = diamond.setAttribute("nom",s);
      } else {
        // create its nom node containing dependent and type
        Element nom = new Element("nom");
        nom = nom.setAttribute("name",s);
        diamond = diamond.addContent(nom);
      }

      // create its proposition node
      Element prop = null;
      if (propName.length() != 0) {
        prop = new Element("prop");
        prop = prop.setAttribute("name", propName);
      }
      if (relMode == null ||
          (!relMode.equals("Subject") && !relMode.equals("Scope"))) {
        if (relMode != null && relMode.equals("Wh-Restr")) {
          if (sort.equals("specifier")) {
            if (prop != null) {
              diamond = diamond.addContent(prop);
            }
          } // end
        } else if (prop != null) {
          diamond = diamond.addContent(prop);
        }

        // if there are features and relations, deal with them properly
        //----------------------------------------------
        while (edge != null) {
          DagNode val = edge.getValue();
          if (val.isNominal()) {
            diamond = editRelations(diamond, val, edge.getName(), coreferences);
          }
          else {
            diamond = editFeatures(diamond, val, edge.getName());
          }
          edge = it.hasNext() ? it.next() : null;
        }
      }
    }
    return diamond;
  }

  /** This method creates a new diamond-element representing the given relation
   *  of the Nominal given in sub. If the relation itself has relations,
   *  recursively call this method to append the relations to the current
   *  element. The this new Element is appended to the originally given one and
   *  that combined Element is returned.
   *  It is a help-method for creating an XML-document that represents a given
   *  LogicalForm as dag
   * @return Element that represents the parent nominal
   * @param sub the sub-dag containing the value of the feature under PROP
   * @param relMode the mode of the relation to add
   * @param el the XML element to add the new diamond-element to (parent)
   * @param coreferences a hash map containing the names of all nominals that
   *        have been converted already. If sub represents such a nominal, only
   *        the name will be added, not the sub-features or -relations
   */
  private static Element editRelations(Element el, DagNode sub, String relMode,
      IdentityHashMap<DagNode, String> coreferences) {

    // create root diamond node
    Element diamond = new Element("diamond");
    diamond = diamond.setAttribute("mode", relMode);
    diamond = handleContent(diamond, sub, relMode, coreferences);

    el = el.addContent(diamond);
    return el;
  } // end editRelations



  /**
	 * Returns an OpenCCG LF object, constructed from a dag.
	 */
	public static LF convertToLF(DagNode root) {
	  IdentityHashMap<DagNode, String> coreferences =
	    new IdentityHashMap<DagNode, String>();

		Document doc;
		LF lf = null;
		try {
			// LogicalForm as dag to xml-File
			// Make it a proper XML document, with a document type and an indication
		  // of the root element.
			DocType doctype = new DocType("xml");
			Element root1 = new Element("xml");
			doc = new Document(root1,doctype);
			// basic Element "lf"
			Element lf1 = new Element("lf");
			Element sat = new Element("satop");
			// null as relMode indicates that sat is the root
			sat = handleContent(sat, root, null, coreferences);

			// now we supposedly have a complete structure
			lf1 = lf1.addContent(sat);
			root1 = doc.getRootElement().addContent(lf1);
			Element targetElt = new Element("target");
			targetElt = targetElt.setText("*** dummy target***");
			root1 = root1.addContent(targetElt);
			// call ccg.Realizer with XML-document to extract the contained LF and
			// return it
			Logger.getLogger(DagToLF.class).info(doc.toString());
			lf = Realizer.getLfFromDoc(doc);
		} catch (Exception e) {
			System.err.println("error in LF: " + e.getMessage());
		}
		// return resulting LF
		return lf;
	}

	public static String test(String input) {
	  Lexer lex = new Lexer();
	  LFParser lfp = new LFParser(lex);
	  lfp.reset("String", new StringReader(input));
	  try {
      if (lfp.parse()) {
        DagNode result = lfp.getResultLF();
        LF lf = DagToLF.convertToLF(result);
        return lf.toString();
      }
    } catch (IOException e) { // will never happen
      assert(false);
    }
	  return "";
	}
}