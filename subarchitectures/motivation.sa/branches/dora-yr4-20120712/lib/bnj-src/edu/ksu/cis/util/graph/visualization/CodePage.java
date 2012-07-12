/*
 * This file is part of Bayesian Network for Java (BNJ).
 * Version 3.3+
 *
 * BNJ is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * BNJ is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BNJ in LICENSE.txt file; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * BNJ Version History
 * ---------------------------------------------
 * BN tools Jan 2000-May 2002
 *
 *  prealpha- January 200 - June 2001
 *	Benjamin Perry, Haipeng Guo, Laura Haverkamp
 *  Version 1- June 2001 - May 2002
 * 	Haipeng Guo, Benjamin Perry, Julie A. Thornton BNJ
 *
 * Bayesian Network for Java (BNJ).
 *  Version 1 - May 2002 - July 2003
 *  	release: v1.03a 29 July 2003
 * 	Infrastructure - Roby Joehanes, Haipeng Guo, Benjamin Perry, Julie A. Thornton
 *	Modules - Sonal S. Junnarkar
 *  Version 2 - August 2003 - July 2004
 *  	release: v2.03a 08 July 2004
 * 	Infrastructure - Roby Joehanes, Julie A. Thornton
 *	Modules - Siddharth Chandak, Prashanth Boddhireddy, Chris H. Meyer, Charlie L. Thornton, Bart Peinter
 *  Version 3 - August 2004 - Present
 *     	Infrastructure - Jeffrey M. Barber
 *	Modules - William H. Hsu, Andrew L. King, Chris H. Meyer, Julie A. Thornton
 * ---------------------------------------------
 */package edu.ksu.cis.util.graph.visualization;
import java.util.Iterator;
import java.util.Stack;
import java.util.TreeMap;
/*!
 * \file CodePage.java
 * \author Jeffrey M. Barber
 */
public class CodePage
{
	private String[]	code;
	private String[]	codeProc;
	private int			active;
	private int			index;
	private TreeMap		_globalenv;
	private Stack		_envstack;
	private TreeMap		_localenv;
	/*! set the index fo this codepage
	 * \param idx
	 */
	public void setIndex(int idx)
	{
		index = idx;
	}
	/*!
	 * \return the index of this codepage
	 */
	public int getIndex()
	{
		return index;
	}
	/*!
	 * \param[in] CODE
	 */
	public CodePage(String[] CODE)
	{
		code = CODE;
		codeProc = new String[CODE.length];
		active = -1;
		_envstack = new Stack();
		_globalenv = new TreeMap();
		_localenv = new TreeMap();
	}
	/*! query the local environment
	 * \param[in] key
	 * \return
	 */
	public String getLocal(String key)
	{
		if (!_localenv.containsKey(key)) return null;
		return (String) _localenv.get(key);
	}
	/*! set local environment
	 * \param[in] key the key
	 * \param[in] value the value to map from key
	 */
	public void setLocal(String key, String value)
	{
		if (value == null)
			_localenv.remove(key);
		else
			_localenv.put(key, value);
	}
	/*! get a global string
	 * \param[in] key the key to lookup in global environment
	 * \return the value
	 */
	public String getGlobal(String key)
	{
		if (!_globalenv.containsKey(key)) return null;
		return (String) _globalenv.get(key);
	}
	/*! map key to value in the global environment
	 * @param[in] key
	 * @param[in] value
	 */
	public void setGlobal(String key, String value)
	{
		_globalenv.put(key, value);
	}
	/*! push local environment
	 */
	public void push()
	{
		_envstack.push(_localenv);
		_localenv = new TreeMap();
	}
	/*! push an environment, set current to r
	 * \param r the new environment
	 */
	public void push(TreeMap r)
	{
		_envstack.push(_localenv);
		_localenv = r;
	}
	/*! pop an environment
	 * \return an environment
	 */
	public TreeMap pop()
	{
		TreeMap old = _localenv;
		_localenv = (TreeMap) _envstack.pop();
		return old;
	}
	/*! get the code after substitution
	 * \return the code
	 */
	public String[] getCode()
	{
		for (int i = 0; i < codeProc.length; i++)
		{
			String l = code[i];
			for (Iterator it = _localenv.keySet().iterator(); it.hasNext();)
			{
				String key = (String) it.next();
				String val = (String) _localenv.get(key);
				if (val != null) l = l.replaceAll("`" + key + "`", key + "=" + val);
			}
			for (Iterator it = _globalenv.keySet().iterator(); it.hasNext();)
			{
				String key = (String) it.next();
				String val = (String) _globalenv.get(key);
				if (val != null) l = l.replaceAll("`" + key + "`", key + "=" + val);
			}
			codeProc[i] = l;
		}
		return codeProc;
	}
	/*! get the active line number
	 * \return the active line number
	 */
	public int getActive()
	{
		return active;
	}
	/*! set the active line number
	 * \param[in] line
	 */
	public void setActve(int line)
	{
		active = line;
	}
	/*! moralization
	 * \return the moralization algo
	 */
	public static CodePage getMoralizationCode()
	{
		String[] c = new String[11];
		c[0] = "set e";
		c[1] = "for each v in G.V";
		c[2] = "   for each (`p1`,`v`) in G";
		c[3] = "      for each (`p2`,`v`) in G";
		c[4] = "         if `p1`!=`p2` and (`p1`,`p2`) is not in G.E and (`p2`,`p1`) is not in G.E then";
		c[5] = "         	e = e union (`p1`,`p2`)";
		c[6] = "         end";
		c[7] = "      next";
		c[8] = "   next";
		c[9] = "next";
		c[10] = "G.E = G.E union e";
		return new CodePage(c);
	}
	/*! remove directionality
	 * \return the rd code
	 */
	public static CodePage getRemoveDirectionalityCode()
	{
		String[] c = new String[5];
		c[0] = "set `u`";
		c[1] = "for each e in G.E";
		c[2] = "    e = (`s`,`d`)";
		c[3] = "   `u` = `u` union (`d`,`s`)";
		c[4] = "next";
		return new CodePage(c);
	}
	/*! mcs see the purple book
	 * \return the mcs algo
	 */
	public static CodePage getMaxCardinalitySearch()
	{
		String[] c = new String[36];
		c[0] = "Constant: V = set of all vertices in graph;";
		c[1] = "          n = number of vertices in graph;";
		c[2] = "          E = set of all edges in the graph;";
		c[3] = " Var i,j: integer;";
		c[4] = "     v,w: vertex;";
		c[5] = "     set: array[0,n-1] of subset of vertices;";
		c[6] = "    size: array[V] of integer;";
		c[7] = "   alpha: array[V] of integer;";
		c[8] = "alphainv: array[1...n] of vertex;";
		c[9] = "Begin";
		c[10] = "  for i := 0 to (n-1) do";
		c[11] = "    set[i] := 0;";
		c[12] = "  for each v in V";
		c[13] = "    begin";
		c[14] = "      size[`v`] := 0;";
		c[15] = "      add `v` to set[0];";
		c[16] = "    end;";
		c[17] = "  i := 1; j:= 0;";
		c[18] = "  while `i` <= n do";
		c[19] = "    begin";
		c[20] = "      `v` := delete any from set[`j`]; // BNJ 3.0 picks a vertex with highest degree";
		c[21] = "      alpha[`v`] := `i`; alphainv[`i`] := `v`;";
		c[22] = "      size[`v`] := -1;";
		c[23] = "      for w is in V - {`v`} do";
		c[24] = "        if (`v`,`w`) is in E and size[`w`] >= 0 then";
		c[25] = "           begin;";
		c[26] = "              delete w from set[size[`w`]];";
		c[27] = "              size[`w`] := size[`w`] + 1;";
		c[28] = "              add `w` to set[size[`w`]];";
		c[29] = "           end;";
		c[30] = "      i := `i` + 1;";
		c[31] = "      j := `j` + 1;";
		c[32] = "      while j >= 0 and set[`j`] = 0 do";
		c[33] = "         j := `j` - 1;";
		c[34] = "      end;";
		c[35] = "  end;";
		return new CodePage(c);
	}
	/*! fill-in see the purple book
	 * \return the fillin algo
	 */
	public static CodePage getFillIn()
	{
		String[] c = new String[30];
		c[0] = "Constant: V = set of all vertices in graph;";
		c[1] = "          n = number of vertices in graph;";
		c[2] = "          E = set of all edges in the graph;";
		c[3] = "          alpha = array[V] of integer;";
		c[4] = "          alphainv = array[1...n] of vertex;";
		c[5] = " Var   i: integer;";
		c[6] = "   v,w,x: vertex;";
		c[7] = "       f: array[V] of V;";
		c[8] = "   index: array[V] of integer:";
		c[9] = "F(alpha): set of new edges";
		c[10] = "Begin";
		c[11] = "  for i := `n` to 1 do";
		c[12] = "    begin";
		c[13] = "      w := alphainv[i]; f(`w`) = `w`; index(`w`) := `i`;";
		c[14] = "      for each v in V do";
		c[15] = "        if (`v`,`w`) exists and alpha(`v`) > `i` then";
		c[16] = "          begin";
		c[17] = "            x := `v`;";
		c[18] = "            while index(`x`) > `i` do";
		c[19] = "              begin";
		c[20] = "                index(`x`) := `i`";
		c[21] = "                add (`x`,`w`) to F(alpha);";
		c[22] = "                x := f(`x`)";
		c[23] = "              end";
		c[24] = "            if f(`x`) = `x`";
		c[25] = "              then f(`x`) = `w`";
		c[26] = "          end";
		c[27] = "    end";
		c[28] = "end";
		c[29] = "E = E union F(alpha)";
		return new CodePage(c);
	}
	/*! build clique tree A
	 * \return the bctaalgo
	 */
	public static CodePage getBuildCliqueTree_A()
	{
		String[] c = new String[24];
		c[0] = " Var: Cliques,CliquesFinal : list of cliques";
		c[1] = "      Cx,Cy : temporary clique";
		c[2] = "      exec : boolean";
		c[3] = "for i = n to 1";
		c[4] = "  Cx = new Clique";
		c[5] = "  v = alphainv[`i`];";
		c[6] = "  p = G.getParents(`v`);";
		c[7] = "  add `v` to Cx;";
		c[8] = "  for each w in p;";
		c[9] = "    if alpha[`w`] < i then add `w` to Cx";
		c[10] = "      and `w` to Cx";
		c[11] = "    end";
		c[12] = "  next";
		c[13] = "  add Cs to Cliques";
		c[14] = "next";
		c[15] = "for each Cx in Cliques";
		c[16] = "  contain = false";
		c[17] = "  for each Cy in Cliques as long as `contain` = `false`";
		c[18] = "    if Cy contains Cx";
		c[19] = "      exec = true;";
		c[20] = "    end";
		c[21] = "  next";
		c[22] = "  if `contain`=false then add Cx to CliquesFinal";
		c[23] = "next";
		return new CodePage(c);
	}
	/*! build clique tree B
	 * \return the bctb algo
	 */
	public static CodePage getBuildCliqueTree_B()
	{
		String[] c = new String[14];
		c[0] = "Total = {}";
		c[1] = "First = first clique";
		c[2] = "First.S = {}";
		c[3] = "Total = Total union `First.nodes`";
		c[4] = "for each clique C ! First";
		c[5] = "  C.S = {}";
		c[6] = "  S = S union `C.nodes`";
		c[7] = "  S = S intersect Total";
		c[8] = "  Total = Total union `C.nodes`";
		c[9] = "  for each clique L < C";
		c[10] = "    if S is subset of L.nodes";
		c[11] = "       E = E union (L,C)";
		c[12] = "  next";
		c[13] = "next";
		return new CodePage(c);
	}
	/*! LS message passing
	 * \return the ls message passing algorithm
	 */
	public static CodePage getLSMessagePassing()
	{
		String[] c = new String[25];
		c[0] = "`C`";
		c[1] = "OnLambdaPropigation()";
		c[2] = "  lambda = P.extract(S);";
		c[3] = "  P = P / `lambda`";
		c[4] = "  if(hasParent)";
		c[5] = "    SendLambdaMessage(par, `lambda`);";
		c[6] = "`C`";
		c[7] = "OnLambdaMessage(CPF `lambda`)";
		c[8] = "   NumberRecv = `NumberRecv` + 1;";
		c[9] = "   if(NumberRecv < Number of Children) return;";
		c[10] = "   for each incoming Lambda message, LI";
		c[11] = "      P = P * `LI`;";
		c[12] = "   OnLambdaPropigation();";
		c[13] = "   if( Number of Parents == 0)";
		c[14] = "     OnPiPropigation();";
		c[15] = "`C`";
		c[16] = "OnPiPropigation()";
		c[17] = "   for each child, X";
		c[18] = "      pi = P.extract(X.S);";
		c[19] = "      SendPiMessage(X, `pi`);";
		c[20] = "`C`";
		c[21] = "OnPiMessage(CPF `pi`)";
		c[22] = "   P = P * `pi';";
		c[23] = "   if(NumChildren > 0)";
		c[24] = "      OnPiPropigation();";
		return new CodePage(c);
	}
	/*! AIS Hueristic Stage One 
	 * \return the AIS Stage One Hueristic algorithm
	 * @author Andrew King
	 */
	public static CodePage getAISStageOne(){
		String[] c = new String[25];
		c[0] = "POE := set of parents of evidence";
		c[1] = "Network := set of all nodes in network (iterator)";
		c[2] = "for each node in Network:";
		c[3] = "  if('name' has evidence)";
		c[4] = "   	Parents = getParents('name')";
		c[5] = "    for each node in Parents";
		c[6] = "       if POE !contain 'parent_name'";
		c[7] = "          POE.add('parent_name')";
		c[8] = "    end loop";
		c[9] = "end loop";
		c[10] = "//Now compute the average of the probabilities";
		c[11] = "//for all events that have evidence instantiated.";
		c[12] = "for each evidence node";
		c[13] = "   for each prob entry in 'node'";
		c[14] = "      'sum' = 'old_sum' + this prob entry";
		c[15] = "    end loop";
		c[16] = "    'avg' = 'sum' / 'num_pos' //averages over the possibilities for that event";
		c[17] = "    if( 'avg' < 1 / (1 / 2 * 'num_outcomes'";
		c[18] = "      for each parent of evidence";
		c[19] = "       for each entry in ICPT for 'this_node'";
		c[20] = "          set ICPT('this_node') to 1 / 'num_pos'";
		c[21] = "       end loop";
		c[22] = "    end loop";
		c[23] = "end loop";
		c[24] = "normalize ICPT";
		
		return new CodePage(c);
	}
	/*! AIS Hueristic Stage Two
	 * \return the AIS Stage Two Hueristic
	 * @author Andrew King
	 */
	public static CodePage getAISStageTwo(){
		String[] c = new String[14];
		c[0] = "StageTwoHeuristic()";
		c[1] = "for each node in network";
		c[2] = "   for each prob in 'node'";
		c[3] = "      if 'prob' < 0.04";
		c[4] = "         ICPT  prob entry for 'node' is set to (0.4)^2";
		c[5] = "      end if";
		c[6] = "      if ICPT for 'node' was modified";
		c[7] = "         find the max prob for that event";
		c[8] = "            reset the 'max_prob' to 'max_prob' - (0.04 - 'small_prob')";
		c[9] = "         end find";
		c[10] = "     end if";
		c[11] = "   end for";
		c[12] = "end for";
		c[13] = "normalize ICPT entries";
		
		return new CodePage(c);
	}
	/*!AIS Main Loop (sample generation and prob update
	 * 
	 * @author Andrew King
	 *
	 *\return the AIS main loop 
	 */
	public static CodePage getAISMain(){
		String[] c = new String[51];
		c[0] = "WIscore = 0";
		c[1] = "total_weight = 0";
		c[2] = "AISStageOne()";
		c[3] = "AISStageTwo()";
		c[4] = "samples := set of whole network samples for last interval";
		c[5] = "onesample := the last whole network sample generated";
		c[6] = "for each node in network";
		c[7] = "   if( 'node' has evidence )";
		c[8] = "      nodes_toupdate.add('node'.getParents())";
		c[9] = "   end if";
		c[10] = "end loop";
		c[11] = "for( 0 =< 'i' < 'num_samples')";
		c[12] = "   if( 'i' % 'interval' == 0 & 'i' != 0 )";
		c[13] = "      if( 'samples_so_far' < 'samples' / 'interval' )";
		c[14] = "         weight = 0";
		c[15] = "      else";
		c[16] = "         weight = 1";
		c[17] = "      end if";
		c[18] = "      'kmax' = 'samples' / 'interval' // kmax = samples / interval";
		c[19] = "      learnrate = ( 'kmax' - 'samples_so_far' ) / 'kmax'";
		c[20] = "      for each node in network";
		c[21] = "         if 'node' is an ICPT node";
		c[22] = "            for each entry in ICPT for 'node'";
		c[23] = "               new_prob = 'old_prob' + 'learnrate' * ( 'approx_prob' - 'old_prob' )";
		c[24] = "            end for";
		c[25] = "         end if";
		c[26] = "      end for";
		c[27] = "      normalize ICPTs";
		c[28] = "      clear all samples from last sampling interval";
		c[29] = "   end if";
		c[30] = "   clear onesample";
		c[31] = "   for each node in network";
		c[32] = "      if( 'node' has evidence )";
		c[33] = "         instantiate 'node's value in the sample to that of the evidence";
		c[34] = "      else";
		c[35] = "         parent_val_list := list of what the parent instantiations are";
		c[36] = "         for each parent of 'node'";
		c[37] = "            add 'parent's value to parent_val_list";
		c[38] = "         end for";
		c[39] = "         build prob interval according to parent_val_list and CPF for 'node'";
		c[40] = "         randomly generate number on the prob interval";
		c[41] = "         instantiate in sample for 'node' from outcome determined from prob interval hit";
		c[42] = "      end if";
		c[43] = "      add this instantiation to this sample";
		c[44] = "   end for";
		c[45] = "   add onesample to samples";
		c[46] = "   WIscore = calcsimpscore(onesample)";
		c[47] = "   if(no evidence in the network) WIscore = 1";
		c[48] = "   total_weight = total_weight + WIscore";
		c[49] = "   update the frequency tables with onesample";
		c[50] = "end for";
		
		
		return new CodePage(c);
	}
	
	
	 
	
}