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
 */package edu.ksu.cis.bnj.gui.tools;

import java.io.FileOutputStream;
import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;
import edu.ksu.cis.bnj.ver3.core.Discrete;
import edu.ksu.cis.bnj.ver3.core.DiscreteEvidence;
import edu.ksu.cis.bnj.ver3.core.Domain;
import edu.ksu.cis.bnj.ver3.streams.Evidence_Reader;
import edu.ksu.cis.bnj.ver3.streams.Evidence_Writer;
import edu.ksu.cis.bnj.ver3.streams.xml.Converter_xmlevidence;

/*!
 * \file EvidenceRecorder.java
 * \author Jeffrey M. Barber
 */
public class EvidenceRecorder
{
	private BeliefNetwork _Network;
	private Evidence_Writer _writer;
	private Evidence_Reader _storage;
	private int _timeStamp;
	public void Start(BeliefNetwork _bn)
	{
		_Network = _bn;
		_writer = new Evidence_Writer();
		_storage = new Evidence_Reader();
		_writer.BeginCollectSamples(_storage);
		_timeStamp = 0;
		
	}
	
	public void Snap()
	{
		_writer.WriteSample(_Network, _storage, _timeStamp);
	}
	
	public void Randomize()
	{
		BeliefNode[] _nodes = _Network.getNodes();
		for(int i = 0; i < _nodes.length; i++)
		{
			Domain D = _nodes[i].getDomain();
			if(D instanceof Discrete)
			{
				//todo replace with a better random number generate
				int r = ((int)((Math.random() * D.getOrder() * D.getOrder()))) % D.getOrder();
				_nodes[i].setEvidence(new DiscreteEvidence(r));
			}
		}
	}
	
	public void Save(String name)
	{
		try
		{
			_writer.EndCollectSamples(_storage);
			_storage.End();

			// open the file
			FileOutputStream FOS = new FileOutputStream(name);
			// load the format
			Converter_xmlevidence cxmlevidence = new Converter_xmlevidence();
			// select the file to save tp
			cxmlevidence.save(FOS);
			// write it out
			_writer.WriteEvidenceList(_storage.get(0), cxmlevidence);
			
			
/*		
			// open the file
			FileInputStream FIS = new FileInputStream(name);
			// create reader
			Evidence_Reader ER = new Evidence_Reader();
			// set the file & load
			cxmlevidence.load(FIS, ER);
			// read it by doing something with
			// ER.get(0),

			cxmlevidence.save(new FileOutputStream("test2.xml"));
			_writer.WriteEvidenceList(ER.get(0), cxmlevidence);
*/			
			
			// we write write to a file
		} catch (Exception e)
		{
			System.out.println("can't save");
			System.out.println(e.getMessage());
		}
	}
	
	public void Clear()
	{
		BeliefNode[] _nodes = _Network.getNodes();
		for(int i = 0; i < _nodes.length; i++)
		{
			_nodes[i].setEvidence(null);
		}		
	}
}
