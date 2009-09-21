///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-7 Jason Baldridge, Gann Bierner and Michael White
// 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//////////////////////////////////////////////////////////////////////////////

package opennlp.ccg.parse;

import opennlp.ccg.grammar.*;
import opennlp.ccg.synsem.*;
import gnu.trove.*;
import java.util.*;

/**
 * An implementation of the table (or chart) used for chart parsers like CKY.
 * Special functions are provided for combining cells of the chart into another
 * cell.
 * 
 * @author Jason Baldridge
 * @author Gann Bierner
 * @author Michael White
 * @version $Revision: 1.11 $, $Date: 2007/12/21 05:13:37 $
 */
public class Chart {

    // maps edges to representative edges, according to their cats, sans LFs
    @SuppressWarnings("unchecked")
    private static Map<Edge,Edge> createEdgeMap() {
    	return new THashMap(
			11,
	        new TObjectHashingStrategy() {
				private static final long serialVersionUID = 1L;
				public int computeHashCode(Object o) {
	                Edge edge = (Edge) o;
	                return edge.sign.getCategory().hashCodeNoLF(); 
	            }
	            public boolean equals(Object o1, Object o2) {
	                Edge edge1 = (Edge) o1; Edge edge2 = (Edge) o2;
	                return edge1.sign.getCategory().equalsNoLF(edge2.sign.getCategory());
	            }
	        }
    	);
    }
    
    /** The chart. */
	protected Map<Edge,Edge>[][] _table;

	/** Its size. */
	public int _size;

	/** The rules. */
	protected RuleGroup _rules;

	/** A map (identity-based) from signs to their edges. */
	protected Map<Sign,Edge> signMap = new IdentityHashMap<Sign,Edge>();
	
	/** The sign scorer (defaults to the null scorer). */
	protected SignScorer _signScorer = SignScorer.nullScorer;
	
	/** The "n" for n-best pruning (or 0 if none). */
	protected int _pruneVal = 0;
	
	
	/** Constructor. */
	@SuppressWarnings("unchecked")
	public Chart(int s, RuleGroup _R) {
		_rules = _R; _size = s;
		_table = new Map[_size][_size];
	}

	/** Sets the sign scorer. */
	public void setSignScorer(SignScorer signScorer) { _signScorer = signScorer; }
	
	/** Sets the n-best pruning val. */
	public void setPruneVal(int n) { _pruneVal = n; }
	
	
	//-----------------------------------------------------------
	// Chart construction
	
	/** Inserts a sign at the given cell.  Returns true if the number of edges has increased. */
	public boolean insert(int x, int y, Sign w) {
		if (_table[x][y] == null) _table[x][y] = createEdgeMap();
		int s = _table[x][y].size();
		// make edge, add to sign map
		Edge edge = new Edge(w); signMap.put(w, edge);
		// get representative edge
		Edge rep = _table[x][y].get(edge);
		// if none, add as representative
		if (rep == null) {
			edge.initAltEdges();
			_table[x][y].put(edge, edge);
		}
		// otherwise add as an alternative
		else {
			rep.altEdges.add(edge);
		}
		// done
		return _table[x][y].size() > s;
	}

	/** Returns the edge map for a given cell (ensuring non-null). */
	protected Map<Edge,Edge> get(int x, int y) {
		if (_table[x][y] == null) _table[x][y] = createEdgeMap();
		return _table[x][y];
	}
	
	public void set(int x, int y, SignHash wh) {
		_table[x][y] = createEdgeMap();
		for (Iterator<Sign> e = wh.asSignSet().iterator() ; e.hasNext();) {
			insert(x, y, e.next());
		}
	}

	/** Returns the signs for a given cell (ensuring non-null). */
	public SignHash getSigns(int x, int y) {
		Map<Edge,Edge> edgeMap = get(x, y);
		SignHash retval = new SignHash();
		for (Edge edge : edgeMap.values()) retval.insert(edge.sign);
		return retval;
	}

	/** Inserts edges into (x,y) that result from applying unary rules to those already in (x,y). */ 
	protected void insertCell(int x, int y) {
		if (_table[x][y] == null) return;
		Map<Edge,Edge> a = _table[x][y];
		List<Sign> inputs = new ArrayList<Sign>(a.size());
		for (Edge edge : a.values()) inputs.add(edge.sign);
		List<Sign> nextInputs = new ArrayList<Sign>(inputs.size());
		// repeat until no more inputs
		while (inputs.size() > 0) {
			// apply rules
			for (Sign sign : inputs) {
				List<Sign> results = _rules.applyUnaryRules(sign);
				for (Sign result : results) {
					// check for unary rule cycle; skip result if found
					if (!result.getDerivationHistory().containsCycle()) {
						// insert result
						boolean newEdgeClass = insert(x, y, result);
						// add to next inputs if it yielded a new equiv class
						if (newEdgeClass) nextInputs.add(result);
					}
				}
			}
			// move all results to inputs
			inputs.clear();
			inputs.addAll(nextInputs);
			nextInputs.clear();
		}
	}

	/** Inserts edges into (x3,y3) resulting from combining those in (x1,y1) and (x2,y2). */
	protected void insertCell(int x1, int y1, int x2, int y2, int x3, int y3) {
		if (_table[x1][y1] == null) return;
		if (_table[x2][y2] == null) return;
		Map<Edge,Edge> a = _table[x1][y1];
		Map<Edge,Edge> b = _table[x2][y2];
		for (Edge edge1 : a.values()) {
			for (Edge edge2 : b.values()) {
				List<Sign> results = _rules.applyBinaryRules(edge1.sign, edge2.sign);
				for (Sign result : results)
					insert(x3, y3, result);
			}
		}
	}

	
	//-----------------------------------------------------------
	// Unpacking (nb: this is drawn from the realizer, in simplified form)
	
	/** Unpacks the edges in the given cell, returning all alternatives. */
	public EdgeHash unpack(int x, int y) {
		Map<Edge,Edge> edgeMap = get(x, y);
		// recursively unpack each edge
	    @SuppressWarnings("unchecked")
        Set<Edge> unpacked = new THashSet(new TObjectIdentityHashingStrategy());
		for (Edge edge : edgeMap.values()) unpack(edge, unpacked); // (debug)
		// collect results
		EdgeHash retval = new EdgeHash();
		for (Edge edge : edgeMap.values()) {
			for (Edge alt : edge.altEdges) retval.insert(alt);
		}
		return retval;
	}
	
	// recursively unpack edge, unless already visited
	private void unpack(Edge edge, Set<Edge> unpacked) {
        if (unpacked.contains(edge) || edge.altEdges == null) return;
        // OR: recursively unpack alts, merging resulting alts
        EdgeHash merged = new EdgeHash();
        for (Edge alt : edge.altEdges) {
            // AND: unpack inputs, make alts, add to merged
            unpackAlt(alt, unpacked, merged);
        }
        // score
        boolean complete = (edge.sign.getWords().size() == _size);
        for (Edge m : merged.asEdgeSet()) { m.setScore(_signScorer.score(m.sign, complete)); }
        // sort
        List<Edge> mergedList = new ArrayList<Edge>(merged.asEdgeSet());
        Collections.sort(mergedList, edgeComparator);
        // prune
        if (_pruneVal > 0) {
        	while (mergedList.size() > _pruneVal) mergedList.remove(mergedList.size()-1);
        }
        // replace edge's alts
        edge.altEdges.clear(); edge.altEdges.addAll(mergedList);
        // update signMap (for debugging, potentially)
        for (Edge mergedEdge : mergedList) {
        	if (!signMap.containsKey(mergedEdge.sign))
        		signMap.put(mergedEdge.sign, mergedEdge);
        }
        // add to unpacked set
        unpacked.add(edge);
    }
    
    // recursively unpack inputs, make alt combos and add to merged
    private void unpackAlt(Edge alt, Set<Edge> unpacked, EdgeHash merged) {
        // unpack via input signs
        DerivationHistory history = alt.sign.getDerivationHistory(); 
        Sign[] inputSigns = history.getInputs();
        // base case: no inputs
        if (inputSigns == null) {
            merged.insert(alt); return;
        }
        // otherwise recursively unpack
        Edge[] inputEdges = new Edge[inputSigns.length];
        for (int i = 0; i < inputSigns.length; i++) {
            inputEdges[i] = signMap.get(inputSigns[i]); // get input edge using signMap
            unpack(inputEdges[i], unpacked);
        }
        // then make edges for new combos, using same rule, and add to merged (if unseen)
        Rule rule = history.getRule();
        List<Sign[]> altCombos = inputCombos(inputEdges, 0);
        List<Sign> results = new ArrayList<Sign>(1);
        for (Sign[] combo : altCombos) {
        	// use this alt for same combo
        	if (sameSigns(inputSigns, combo)) {
        		merged.insert(alt); continue;
        	}
        	results.clear();
        	((AbstractRule)rule).applyRule(combo, results);
        	if (results.isEmpty()) continue; // (rare?)
            Sign sign = results.get(0); // assuming single result
            merged.insert(new Edge(sign)); // make edge for new alt
        }
	}
	
    // returns a list of sign arrays, with each array of length inputEdges.length - i, 
    // representing all combinations of alt signs from i onwards
    private List<Sign[]> inputCombos(Edge[] inputEdges, int index) {
        Edge edge = inputEdges[index];
        // base case, inputEdges[last]
        if (index == inputEdges.length-1) {
            List<Edge> altEdges = edge.altEdges; 
            if (altEdges == null) altEdges = new ArrayList<Edge>();
            List<Sign[]> retval = new ArrayList<Sign[]>(altEdges.size());
            for (Edge alt : altEdges) {
                retval.add(new Sign[] { alt.sign });
            }
            return retval;
        }
        // otherwise recurse on index+1
        List<Sign[]> nextCombos = inputCombos(inputEdges, index+1);
        // and make new combos
        List<Edge> altEdges = edge.altEdges; 
        if (altEdges == null) altEdges = new ArrayList<Edge>();
        List<Sign[]> retval = new ArrayList<Sign[]>(altEdges.size() * nextCombos.size());
        for (Edge alt : altEdges) {
            for (int i = 0; i < nextCombos.size(); i++) {
                Sign[] nextSigns = nextCombos.get(i);
                Sign[] newCombo = new Sign[nextSigns.length+1];
                newCombo[0] = alt.sign;
                System.arraycopy(nextSigns, 0, newCombo, 1, nextSigns.length);
                retval.add(newCombo);
            }
        }
        return retval;
    }

    // checks for same signs
    private boolean sameSigns(Sign[] a, Sign[] b) {
    	if (a.length != b.length) return false;
    	for (int i=0; i < a.length; i++)
    		if (a[i] != b[i]) return false;
    	return true;
    }
    
    /** Compares edges based on their relative score, in descending order. */
    public static final Comparator<Edge> edgeComparator = new Comparator<Edge>() {
        public int compare(Edge edge1, Edge edge2) {
            return -1 * Double.compare(edge1.score, edge2.score);
        }
    };

    
	//-----------------------------------------------------------
    
	/** Returns the number of entries in each cell in the chart. */
	public String toString() {
		StringBuffer sb = new StringBuffer();
		for (int i = 0; i < _size; i++) {
			for (int j = 0; j < _size; j++) {
				sb.append(get(i, j).size()).append('\t');
			}
			sb.append('\n');
		}
		return sb.toString();
	}

	/** Prints the signs in the chart to System.out. */
	public void printChart() {
		int[] sizes = new int[_size];
		int rows = 0;
		for (int i = 0; i < _size; i++) {
			for (int j = i; j < _size; j++)
				if (get(i, j).size() > sizes[i])
					sizes[i] = get(i, j).size();
			rows += sizes[i];
		}

		String[][] toprint = new String[rows][_size];
		int maxwidth = 0;

		for (int i = 0, row = 0; i < _size; row += sizes[i++]) {
			for (int j = 0; j < _size; j++)
				for (int s = 0; s < sizes[i]; s++) {
					SignHash cell = getSigns(i, j);
					if (cell.size() >= s + 1) {
						toprint[row + s][j] = ((Sign) cell.toArray()[s])
								.getCategory().toString();
						if (toprint[row + s][j].length() > maxwidth)
							maxwidth = toprint[row + s][j].length();
					}
				}
		}

		int fullwidth = _size * (maxwidth + 3) - 1;
		System.out.print("|");
		for (int p = 0; p < fullwidth; p++)
			System.out.print("-");
		System.out.print("| ");
		System.out.println();

		for (int i = 0, entry = sizes[0], e = 0; i < rows; i++) {
			if (i == entry) {
				System.out.print("|");
				for (int p = 0; p < fullwidth; p++)
					System.out.print("-");
				System.out.print("|");
				System.out.println();
				entry += sizes[++e];
			}
			System.out.print("| ");

			for (int j = 0; j < _size; j++) {
				int pad = 1 + maxwidth;
				if (toprint[i][j] != null) {
					System.out.print(toprint[i][j]);
					pad -= toprint[i][j].length();
				}
				for (int p = 0; p < pad; p++)
					System.out.print(" ");
				System.out.print("| ");
			}
			System.out.println();
		}
		System.out.print("|");
		for (int p = 0; p < fullwidth; p++)
			System.out.print("-");
		System.out.print("| ");
		System.out.println();
	}
}
