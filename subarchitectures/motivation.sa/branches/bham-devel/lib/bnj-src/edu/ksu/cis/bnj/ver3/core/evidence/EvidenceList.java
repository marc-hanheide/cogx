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
 */package edu.ksu.cis.bnj.ver3.core.evidence;
/*!
 * \file EvidenceList.java
 * \author Jeffrey M. Barber
 */
public class EvidenceList
{
	private EvidenceSample[]	_Samples;
	private int					_SampleSize;
	/*! construct an empty evidence list
	 * 
	 */
	public EvidenceList()
	{
		_Samples = new EvidenceSample[2000];
		_SampleSize = 0;
	}
	/*! augment the buffer
	 * \param[in] b
	 */
	private void buffer(int b)
	{
		int newSize = _Samples.length + b;
		EvidenceSample[] newSamples = new EvidenceSample[newSize];
		for (int i = 0; i < _Samples.length; i++)
			newSamples[i] = _Samples[i];
		_Samples = newSamples;
	}
	/*! add a sample to the list
	 * \param[in] ev evidence sample
	 */
	public void add(EvidenceSample ev)
	{
		if (_SampleSize + 2 >= _Samples.length) buffer(1000);
		_Samples[_SampleSize] = ev;
		_Samples[_SampleSize].setLOC(_SampleSize);
		_SampleSize++;
	}
	/*! remove a sample
	 * \param[in] ev evidence sample
	 */
	public void remove(EvidenceSample ev)
	{
		if (ev.loc() < 0) return;
		for (int k = ev.loc(); k < size() - 1; k++)
		{
			_Samples[k] = _Samples[k + 1];
			_Samples[k].setLOC(k);
		}
		_Samples[size() - 1] = null;
		_SampleSize--;
	}
	/*! query on a sample's id
	 * \param[in] k the k'th sample
	 * \return the sample at k
	 */
	public EvidenceSample getSample(int k)
	{
		return _Samples[k];
	}
	/*! process all samples, and cache their discrete values according to this structure
	 * \param[in] ES
	 */
	public void CacheDiscrete(EvidenceStructure ES)
	{
		int N = size();
		for (int k = 0; k < N; k++)
		{
			_Samples[k].createDiscreteCache(ES.getNodes());
		}
	}
	/*! query the number of samples
	 * \return the number of samples
	 */
	public int size()
	{
		return _SampleSize;
	}
}