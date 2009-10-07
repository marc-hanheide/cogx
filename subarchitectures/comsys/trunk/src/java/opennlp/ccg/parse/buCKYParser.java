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

import opennlp.ccg.grammar.Grammar;

/**
 * The parser is a CKY chart parser for CCG.
 *
 * (buCKYParser just specifies the class implements the CCGParserInterface)
 * 
 * @author      Jason Baldridge
 * @author      Gann Bierner
 * @author      Michael White
 * @version     $Revision: 1.8 $, $Date: 2005/10/20 18:49:42 $
 */
public class buCKYParser 
	extends opennlp.ccg.parse.Parser
	implements CCGParserInterface
{

	public buCKYParser (Grammar grammar) { 
		super(grammar);
	} // end constructor


} // end class