///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003 Jason Baldridge, Gann Bierner and 
//                    University of Edinburgh (Michael White)
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

import java.util.*;

/**
 * An implementation of the table (or chart) used for chart parsers like CKY.
 * Special functions are provided for combining cells of the chart into another
 * cell.
 * 
 * @author Jason Baldridge
 * @author Gann Bierner
 * @author Michael White
 * @version $Revision: 1.5 $, $Date: 2005/10/20 18:49:42 $
 */
public class Chart {

	protected SignHash[][] _table;

	protected int _size;

	RuleGroup _rules;

	public Chart(int s, RuleGroup _R) {
		_rules = _R;
		_size = s;
		_table = new SignHash[_size][_size];
	}

	public boolean insert(int x, int y, Sign w, Sign[] old) {
		if (_table[x][y] == null) {
			_table[x][y] = new SignHash();
		}
		int s = _table[x][y].size();
		_table[x][y].insert(w);
		return _table[x][y].size() > s;
	}

	public SignHash get(int x, int y) {
		if (_table[x][y] == null) {
			_table[x][y] = new SignHash();
		}
		return _table[x][y];
	}

	public void set(int x, int y, SignHash wh) {
		_table[x][y] = wh;
	}

	protected void insertCell(int x, int y) {
		if (_table[x][y] == null) {
			return;
		}
		
		SignHash a = _table[x][y];
		Sign[] sign = new Sign[1];
		SignHash newHash = new SignHash();
		List<Sign> inputs = new ArrayList<Sign>(a.asSignSet());
		List<Sign> nextInputs = new ArrayList<Sign>(inputs.size());
		// repeat until no more inputs
			
		while (inputs.size() > 0) {
				
			// apply rules		
			for (Iterator<Sign> i = inputs.iterator(); i.hasNext();) {

				sign[0] = i.next();
				List<Sign> results = _rules.applyRules(sign);
				for (Iterator<Sign> resultIt = results.iterator(); resultIt
						.hasNext();) {
					Sign signresult = resultIt.next();
					newHash.insert(signresult);
				}
				nextInputs.addAll(results);
			}
			// move all results to inputs
			inputs.clear();
			inputs.addAll(nextInputs);
			nextInputs.clear();
		}
		
		
		for (Iterator i = newHash.iterator(); i.hasNext();) {
			insert(x, y, (Sign) i.next(), sign);
		}
	}

	protected void insertCell(int x1, int y1, int x2, int y2, int x3, int y3) {
		if (_table[x1][y1] == null)
			return;
		if (_table[x2][y2] == null)
			return;
		SignHash a = _table[x1][y1];
		SignHash b = _table[x2][y2];
		Sign[] signs = new Sign[2];
		for (Iterator i = a.iterator(); i.hasNext();) {
			signs[0] = (Sign) (i.next());
			for (Iterator j = b.iterator(); j.hasNext();) {
				signs[1] = (Sign) (j.next());
				List<Sign> results = _rules.applyRules(signs);
				for (Iterator<Sign> resultIt = results.iterator(); resultIt.hasNext();) {
					Sign result = resultIt.next();
					insert(x3, y3, (Sign) result, signs);
				}
			}
		}
	}

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
					SignHash cell = get(i, j);
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

