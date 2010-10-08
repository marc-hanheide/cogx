// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Pierre Lison (plison@dfki.de)
// Geert-Jan M. Kruijff (gj@dfki.de)
//                                                                                                                          
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION 
package de.dfki.lt.tr.dialogue.asr;

//=================================================================
// IMPORTS

// Java 
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.util.Vector;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.StringTokenizer;

// Dialogue API slice
import de.dfki.lt.tr.dialogue.slice.asr.PhonString;

// Dialogue API util
import de.dfki.lt.tr.dialogue.util.DialogueInvalidOperationException;


/**
 * The class builds up a word recognition lattice, given a vector of strings as recognized by 
 * a speech recognition engine. 
 * <p>
 * The lattice is built upon construction of the object: 
 * 
 * <pre> 
 * 		Vector recognizedStrings = new Vector<PhonString>();
 * 		// get the n-best recognized strings from an ASR engine ...
 * 		// create an identifier for the lattice
 * 		String latticeId = "lattice01"; 
 * 		WordRecognitionLattice lattice = new WordRecognitionLattice(recognizedStrings,latticeId);
 * </pre><p>
 * 
 *
 * @author 	Pierre Lison (plison@dfki.de)
 * @author  Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100607
 * @since   080909 
 */

public class WordRecognitionLattice {

	// The start node of the lattice
	Node startNode;
	// The end node of the lattice
	Node endNode;
	// Vector of all nodes in the lattice
	Vector<Node> currentNodes;
	// The identifier of the word recognition lattice
	String Id;
	// Stores the n-best list of PhonString objects, from which the lattice has been constructed
	Vector<PhonString> rawresults;
	// The maximum length of the stored recognized strings
	long maxLength = 0;

	/** 
	 * The constructor takes a vector of recognized strings, and an Id for the word lattice itself. 
	 * The lattice is constructed from the vector of recognized strings. The strings are used as 
	 * they are, and are not edited for error correction or contraction expansion. 
	 *  
	 * @param lines The recognized strings to construct the lattice from
	 * @param Id	The identifier to be assigned to the word recognition lattice
	*/ 
	public WordRecognitionLattice(Vector<PhonString> lines, String Id) {
		this.Id = Id;
		rawresults = lines;
		constructLattice(lines);
	} // end constructor
	
	/**
	 * Internal method for calling the construction of the lattice
	 * @param lines The strings from which the lattice is to be constructed
	 */
	private void constructLattice (Vector<PhonString> lines) 
	throws DialogueInvalidOperationException 
	{
		for (Enumeration<PhonString> e = lines.elements(); e.hasMoreElements();) {
			PhonString phon = e.nextElement();
			if (phon.length > maxLength) {
				maxLength = phon.length;
			} // end if
		} // end for
		if (maxLength == 0) 
		{
			throw new DialogueInvalidOperationException("Trying to construct a lattice from recognized strings of length zero");
		}
		startNode = extractLattice(lines);
		endNode.cleanPaths();
		currentNodes = new Vector<Node>();
		currentNodes.add(startNode);		
	} // end constructLattice
	
	/**
	 * Internal top-level method for extracting the lattice from a list of recognized strings. 
	 * For each string, a tokenizer is constructed. The resulting vector of tokenizers
	 * is then provided to another method doing the actual construction. 
	 * 
	 * @param 	phons 	The strings from which to construct the lattice
	 * @return	Node	Node pointing to the constructed lattice
	 */
	
	private Node extractLattice(Vector<PhonString> phons) {
		startNode = new Node();
		endNode = new Node();
		Vector<StringTokenizer> tokenizers = new Vector<StringTokenizer>();
		for (Enumeration<PhonString> e = phons.elements(); e.hasMoreElements();) {
			PhonString phon = e.nextElement();
			StringTokenizer tokenizer = new StringTokenizer(phon.wordSequence);
			tokenizers.add(tokenizer);
		} // end for
		Node node =  extractLattice(tokenizers, new Vector<Node>());
		return node;
	} // end extractLattice
	
	/**
	 * The internal method that does the actual construction of the lattice from the 
	 * recognized strings, using a list of string tokenizers defined on these strings. 
	 * First, the method constructs a map from words to tokenizers in which the word occurs 
	 * next (parallellism between recognized strings). Then, this map is used to construct 
	 * the nodes and edges making up the lattice. This method is called recursively. 
	 * 
	 * @param 	tokenizers A vector of tokenizers, one for each recognized string
	 * @param 	nodes A list of nodes making up the lattice constructed so far
	 * @return 	Pointer to the lattice
	 */

	private Node extractLattice(Vector<StringTokenizer> tokenizers , Vector<Node> nodes) 
	{	
		Node curnode = new Node();
		Hashtable<String, Vector<StringTokenizer>> hash = 
			new Hashtable<String, Vector<StringTokenizer>>();
		for (Enumeration<StringTokenizer> e = tokenizers.elements(); e.hasMoreElements();) {
			StringTokenizer tokenizer = e.nextElement();
			if (tokenizer.hasMoreElements()) {
				String token = tokenizer.nextToken();
				if (hash.containsKey(token)) {
					Vector<StringTokenizer> existingtoks = hash.get(token);
					if (tokenizer.hasMoreElements()) {
						existingtoks.add(tokenizer);
						hash.put(token, existingtoks);
					}
					else {
						existingtoks.add(new StringTokenizer(""));
						hash.put(token, existingtoks);
					}
				}
				else {
					if (tokenizer.hasMoreElements()) {
						Vector<StringTokenizer> newtoks = new Vector<StringTokenizer>();
						newtoks.add(tokenizer);
						hash.put(token, newtoks);
					}
					else {
						Vector<StringTokenizer> newtoks = new Vector<StringTokenizer>();
						newtoks.add(new StringTokenizer(""));
						hash.put(token, newtoks);
					}					
				}
			}	
			else {
				return endNode;
			}
		}

		for (Enumeration<String> e = hash.keys(); e.hasMoreElements();) {
			String firstword = e.nextElement();
			Node newnode = extractLattice(hash.get(firstword), nodes);
			Node similarnode = null;
			for (Enumeration<Node> f = nodes.elements(); f.hasMoreElements();) {
				Node node = f.nextElement();
				if (node.similarPaths(newnode)) {
					similarnode = node;
				}
			}	
			if (similarnode != null) {
				newnode = similarnode;
				Edge edge = new Edge(curnode, newnode, firstword, 0.0f);
				if (!curnode.outEdges.contains(edge)) {
					curnode.addOutEdge(edge);
					newnode.addInEdge(edge);
				}
			}
			else {
				Edge edge = new Edge(curnode, newnode, firstword, 0.0f);
				nodes.add(newnode);
				curnode.addOutEdge(edge);
				newnode.addInEdge(edge);
			}
		}
		return curnode;
	} // end extractLattice

	
	/**
	 * Returns a vector of the strings from which the lattice is built
	 * @return Vector<PhonString> The strings from which the lattice is built
	 */
	
	public Vector<PhonString> getAllResults() {
		return rawresults;
	} // end getAllResults
	
	/**
	 * Returns a vector consisting of a single element, being the first recognized string 
	 * in the vector from which the lattice has been constructed
	 * @return Vector<PhonString> A vector with one element only, namely the first recognized string
	 */
	
	public Vector<PhonString> getFirstResult() {
		Vector<PhonString> result = new Vector<PhonString>();
		result.add(rawresults.elementAt(0));
		return result;
	}
	
	/**
	 * Returns a vector consisting of strings read out from the lattice
	 * @return Vector<PhonString> A vector of constructed strings, computed from the lattice
	 */
		
	public Vector<PhonString> getNextResults() {
		Vector<PhonString> results = new Vector<PhonString>();
		Vector<Node> newnodes = new Vector<Node>();
		for (Enumeration<Node> c = currentNodes.elements() ; c.hasMoreElements();) {
			Node n = c.nextElement();
			for (Enumeration<Edge> d = n.outEdges.elements(); d.hasMoreElements();) {
				Edge edge = d.nextElement();
				Node n2 = edge.out;
				Vector<String> paths = n2.getBackwardPaths();
				for (Enumeration<String> e = paths.elements(); e.hasMoreElements();) {
					PhonString phon = new PhonString();
					phon.wordSequence = e.nextElement();
					results.add(phon);
				}
				newnodes.add(n2);
			}
		}
		currentNodes = newnodes;
		return results;
	} // end getNextResults

	/**
	 * Returns the maximum length of a recognized string, as stored in the lattice	
	 * @return long The max length
	 */
	
	public long getMaximumLength() {
		return maxLength;
	} // end getMaximumLength

	/**
	 * Returns the identifier of the word lattice
	 * @return String The identifier
	 */
	public String getId() { return Id; }

	/**
	 * Returns the stored recognition results, from which the lattice has been constructed. The results 
	 * are returned as a single String, separating the individual results using linebreaks "\n". 
	 * @return String A concatenation of the recognized strings 
	 */

	public String toString() {
		String str = "";
		str += "Stored recognition results:\n" ;
		for (Enumeration<PhonString> e = rawresults.elements(); e.hasMoreElements();) {
			PhonString phon = e.nextElement();
			str += phon.wordSequence + "\n";
		}
		return str;
	} // end toString

	static int NodeIncrement = 1;

	public final class Node {

		Vector<Edge> outEdges;
		Vector<Edge> inEdges;
		String nodeId;


		public Node() {
			inEdges = new Vector<Edge>();
			outEdges = new Vector<Edge>();
			nodeId = "node" + NodeIncrement;
			NodeIncrement++;
		}

		public void addOutEdge(Edge edge) {
			outEdges.add(edge);
		}
		
		public void addInEdge(Edge edge) {
			inEdges.add(edge);
		}

		public Vector<Edge> getOutEdges() {return outEdges; }
		
		public Vector<Edge> getInEdges() {return inEdges; }

		public boolean similarPaths(Node node) {
			Vector<String> ourpaths = getForwardPaths();
			Vector<String> theirpaths = node.getForwardPaths();
			if (ourpaths.size() != theirpaths.size()) {
				return false;
			}
			else {
				for (Enumeration<String> o = ourpaths.elements(); o.hasMoreElements();) {
					boolean found = false;
					String ourpath = o.nextElement();
					for (Enumeration<String> t = theirpaths.elements(); t.hasMoreElements();) {
						String theirpath = t.nextElement();
						if (ourpath.equals(theirpath)) {
							found = true;
						}
					}
					if (!found) {
						return false;
					}
				}
			}
			return true;
		}

		public Vector<String> getBackwardPaths() {
			Vector<String> results = new Vector<String>();
			for (Enumeration<Edge> e = inEdges.elements(); e.hasMoreElements();) {
				Edge edge = e.nextElement();
				Vector<String> previouspaths = edge.in.getBackwardPaths();
				if (previouspaths.size() == 0) {
					results.add(edge.label);
				}
				else {
					for (Enumeration<String> f = previouspaths.elements(); f.hasMoreElements();) {
						String previouspath = f.nextElement();
						String path = previouspath + " " + edge.label;
						results.add(path);
					}
				}
			}
			return results;
		}
	

		public Vector<String> getForwardPaths() {
			Vector<String> results = new Vector<String>();
			for (Enumeration<Edge> e = outEdges.elements(); e.hasMoreElements();) {
				Edge edge = e.nextElement();
				Vector<String> futurepaths = edge.out.getForwardPaths();
				if (futurepaths.size() == 0) {
					results.add(edge.label);
				}
				else {
					for (Enumeration<String> f = futurepaths.elements(); f.hasMoreElements();) {
						String futurepath = f.nextElement();
						String path = edge.label + " " + futurepath;
						results.add(path);
					}
				}
			}
			return results;
		}
		

		public void cleanPaths() {
			
			Vector<Edge> toRemove = new Vector<Edge>();
			for (Enumeration<Edge> e = inEdges.elements(); e.hasMoreElements();) {
				Edge edge = e.nextElement();
				
				if(edge.in.inEdges.size() == 0 && !edge.in.equals(startNode)) {
					edge.out.outEdges.remove(edge);
					toRemove.add(edge);
				}	
				edge.in.cleanPaths();
			}
			inEdges.removeAll(toRemove);
		}
	}



	public final class Edge {

		String label = "";
		float confidenceScore = 0.0f;
		Node in ;
		Node out;

		public Edge(Node in, Node out, String label, float confidenceScore) {
			this.in = in;
			this.out = out;
			this.label = label;
			this.confidenceScore = confidenceScore;
		}
		
		public boolean equals(Object other) {
			Edge otherEdge = (Edge) other;
			if (in.equals(otherEdge.in) && out.equals(otherEdge.out) 
					&& label.equals(otherEdge.label) && 
					confidenceScore == otherEdge.confidenceScore) {
				return true;
			}
			return false;
		}
	}




	
} // end class