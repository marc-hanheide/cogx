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
 */package edu.ksu.cis.bnj.ver3.inference.exact.mtls;
/*!
 * \file MessageQueue.java
 * \author Jeffrey M. Barber
 */
public class MessageQueue
{
	private QueueElement _Head;
	private QueueElement _Tail;
	
	class QueueElement
	{
		public Message _M;
		public QueueElement _Next;
	}
	
	/*! Add this message to the queue
	 * \param[in] M
	 */
	public synchronized void add(Message M)
	{
		QueueElement QE = new QueueElement();
		QE._M = M;
		QE._Next = null;
		
		if(_Tail == null)
		{
			_Head = QE;
			_Tail = QE;
		}
		else
		{
			_Tail._Next = QE;
			_Tail = QE;
		}
	}
	/*! Construct an empty Message Queue
	 */
	public MessageQueue()
	{
		_Head = null;
		_Tail = null;
	}
	
	/*! get a message from the queue that is available
	 * \param[in] CheckOut the clique checkout array
	 * \param[in] Proc the procid
	 * \return the message
	 */
	public synchronized Message get(MClique[] CheckOut, int Proc)
	{
		QueueElement _Search = _Head;
		QueueElement _SearchLast = null;
		
		while(_Search != null)
		{
			Message M = _Search._M;
			if (M.getClique().getProcessor() < 0)
			{
				CheckOut[Proc] = M.getClique();
				CheckOut[Proc].assignProcessor(Proc);
				if( _SearchLast == null )
				{
					_Head = _Search._Next;
					if(_Head == null)
					{
						_Tail = null;
					}
				}
				else
				{
					_SearchLast._Next = _Search._Next;
					if(_SearchLast._Next==null)
					{
						_Tail = _SearchLast;
					}
				}
				return M;
			}
			_SearchLast = _Search;
			_Search = _Search._Next; 
		}
		return null;
		
	}
}
