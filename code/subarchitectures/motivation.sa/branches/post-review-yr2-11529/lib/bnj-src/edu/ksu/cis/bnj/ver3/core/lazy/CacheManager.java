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
 */package edu.ksu.cis.bnj.ver3.core.lazy;
import edu.ksu.cis.util.GlobalOptions;
/*!
 * \file CacheManager.java
 * \author Jeffrey M. Barber
 */
public class CacheManager
{
	static private CacheManager	_instance	= new CacheManager();
	private int					units;
	private int					max;
	/*! singleton access controller
	 * \return
	 */
	static public CacheManager getInstance()
	{
		return _instance;
	}
	/*! construct the cache manager
	 */
	public CacheManager()
	{
		reset();
	}
	/*! reset the cache according to the GlobalOptions
	 */
	public void reset()
	{
		units = GlobalOptions.getInstance().getInteger("lazyeval_totalcpfvalues", 20000000);
		max = GlobalOptions.getInstance().getInteger("lazyeval_maxpercpf", 2000000);
	}
	/*! get the maximum number of units available for grabbing
	 * \return the number of units (rep. num of cpf cells)
	 */
	public int getMax()
	{
		if (max < units) return max;
		return units;
	}
	/*! grab du number of units
	 * \param du
	 */
	public void grab(int du)
	{
		units -= du;
	}
	/*! grab du number of units
	 * \param du
	 */
	public void give(int du)
	{
		units += du;
	}
}