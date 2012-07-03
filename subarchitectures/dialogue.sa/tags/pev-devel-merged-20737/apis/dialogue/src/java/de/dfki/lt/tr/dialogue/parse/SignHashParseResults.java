//=================================================================
// Copyright (C) 2010 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
package de.dfki.lt.tr.dialogue.parse;

//=================================================================
// IMPORTS

// OpenCCG
import opennlp.ccg.parse.ParseResults;
import opennlp.ccg.synsem.SignHash;


public class SignHashParseResults 
implements ParseResults
{

	public SignHash hash = null; 

	public SignHashParseResults (SignHash h) { 
		hash = h;
	} // end constructor

} // end class
