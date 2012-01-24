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
import java.util.ArrayList;
import edu.ksu.cis.util.graph.core.*;
import edu.ksu.cis.util.graph.layout.LayPolyTree;
import edu.ksu.cis.util.graph.visualization.operators.CallForLayoutPolyTree;
import edu.ksu.cis.util.graph.visualization.operators.CodePageSelectLine;
import edu.ksu.cis.util.graph.visualization.operators.ColorLegendMap;
import edu.ksu.cis.util.graph.visualization.operators.FlushMarkings;
import edu.ksu.cis.util.graph.visualization.operators.MarkupOperator;
import edu.ksu.cis.util.graph.visualization.operators.NewColorLegend;
import edu.ksu.cis.util.graph.visualization.operators.TempEdgeCreate;
import edu.ksu.cis.util.graph.visualization.operators.TempEdgeFlush;
/*!
 * \file VisualizationController.java
 * \author Jeffrey M. Barber
 */
public class VisualizationController
{
	private ArrayList	_Operators;
	private ArrayList	_CurrentMarkings;
	private int			_Cursor;
	private Graph		_OriginalGraph;
	private Graph		_Graph;
	private int			_Lockdown;
	private int			_TransactionCount;
	private int			_time2next;
	private int			_time2prev;
	private CodePage[]	_codepages;
	private int			_CurrentPage	= -1;
	private String		_Header;
	private ArrayList	_TempEdges;
	private ArrayList	_ColorLegend;
	private boolean _DoZoom2Fit;
	
	/*! query if the vis agent should zoom2fit on this frame
	 * \return answer
	 */
	public boolean doZoom2Fit()
	{
		boolean ret = _DoZoom2Fit;
		_DoZoom2Fit = false;
		return ret;
	}
	/*! construct
	 * \param[in] g the graph to start with
	 */
	public VisualizationController(Graph g)
	{
		_OriginalGraph = g;
		_Graph = g;
		_Operators = new ArrayList();
		_Cursor = -1;
		_Lockdown = -1;
		_TransactionCount = 0;
		_CurrentMarkings = new ArrayList();
		_codepages = new CodePage[100];
		_codepages[0] = CodePage.getMoralizationCode();
		_codepages[0].setIndex(0);
		_codepages[1] = CodePage.getRemoveDirectionalityCode();
		_codepages[1].setIndex(1);
		_codepages[2] = CodePage.getMaxCardinalitySearch();
		_codepages[2].setIndex(2);
		_codepages[3] = CodePage.getFillIn();
		_codepages[3].setIndex(3);
		_codepages[4] = CodePage.getBuildCliqueTree_A();
		_codepages[4].setIndex(4);
		_codepages[5] = CodePage.getBuildCliqueTree_B();
		_codepages[5].setIndex(5);
		_codepages[6] = CodePage.getLSMessagePassing();
		_codepages[6].setIndex(6);
		_codepages[7] = CodePage.getAISStageOne();
		_codepages[7].setIndex(7);
		_codepages[8] = CodePage.getAISStageTwo();
		_codepages[8].setIndex(8);
		_codepages[9] = CodePage.getAISMain();
		_codepages[9].setIndex(9);
		_Header = "";
		_TempEdges = new ArrayList();
		_ColorLegend = new ArrayList();
	}
	/*! get the color legend
	 * \return the color legend
	 */
	public ArrayList getColorLegend()
	{
		return _ColorLegend;
	}
	/*! get the temporary edges
	 * \return the temporary edges
	 */
	public ArrayList getTempEdges()
	{
		return _TempEdges;
	}
	/*! get the header (annotation)
	 * \return the header (annotation)
	 */
	public String getHeader()
	{
		return _Header;
	}
	/*! get the current codepage
	 * \return the current codepage
	 */
	public String[] getCode()
	{
		if (_CurrentPage < 0) return null;
		return _codepages[_CurrentPage].getCode();
	}
	/*! get the active line for the current codepage
	 * \return the active line for the current codepage
	 */
	public int getActiveLine()
	{
		if (_CurrentPage < 0) return -1;
		return _codepages[_CurrentPage].getActive();
	}
	/*! get the markings on the wall
	 * \return the markings on the wall
	 */
	public ArrayList getMarkings()
	{
		return _CurrentMarkings;
	}
	/*! push and apply an operator
	 * \param[in] o the operator
	 */
	public void pushAndApplyOperator(Object o)
	{
		_Operators.add(o);
		applyNextFrameP();
	}
	/*! begin a vis transaction
	 */
	public void beginTransaction()
	{
		if (_TransactionCount == 0)
		{
			_Lockdown = _Cursor;
		}
		_TransactionCount++;
	}
	/*! commit a vis transaction
	 */
	public void commitTransaction()
	{
		if (_TransactionCount >= 1) _TransactionCount--;
		if (_TransactionCount == 0)
		{
			while (_Cursor > _Lockdown)
				applyPreviousFrame();
			_Lockdown = -1000;
		}
	}
	/*! skip forward until next header
	 */
	public void SkipForwardUntilNextHeader()
	{
		String Head = _Header;
		while (_Cursor + 1 < _Operators.size())
		{
			applyNextFrame();
			if (Head != _Header) return;
		}
	}
	/*! skip backward until prev header
	 */
	public void SkipBackwardUntilPrevHeader()
	{
		String Head = _Header;
		while (_Cursor >= 0)
		{
			applyPreviousFrame();
			if (Head != _Header) return;
		}
	}
	/*! apply the next frame
	 * \return true if it should go again
	 */
	private boolean applyNextFrameP()
	{
		if (_Cursor + 1 < _Operators.size())
		{
			_time2prev = _time2next;
			_Cursor++;
			Object obj = _Operators.get(_Cursor);
			if (obj instanceof GraphOperator)
			{
				GraphOperator o = (GraphOperator) obj;
				_time2next = o.getTime();
				_Graph = o.apply(_Graph);
			}
			else if (obj instanceof MarkupOperator)
			{
				MarkupOperator MO = (MarkupOperator) obj;
				_time2next = 50;
				MO.apply(_CurrentMarkings);
			}
			else if (obj instanceof CodePageOperator)
			{
				CodePageOperator cpo = (CodePageOperator) obj;
				if (_CurrentPage < 0)
					cpo.apply(null);
				else
					cpo.apply(_codepages[_CurrentPage]);
				_CurrentPage = cpo.getCodePage();
				_time2next = 25;
				if (!(cpo instanceof CodePageSelectLine))
				{
					return true;
				}
			}
			else if (obj instanceof Annotation)
			{
				Annotation AN = (Annotation) obj;
				_Header = AN.apply(_Header);
				_time2next = 25;
				return true;
			}
			else if (obj instanceof FlushMarkings)
			{
				FlushMarkings FM = (FlushMarkings) obj;
				_CurrentMarkings = FM.apply(_CurrentMarkings);
				_time2next = 25;
			}
			else if (obj instanceof TempEdgeCreate)
			{
				TempEdgeCreate TEC = (TempEdgeCreate) obj;
				_TempEdges.add(TEC.edgeCreate);
				_time2next = 25;
			}
			else if (obj instanceof TempEdgeFlush)
			{
				TempEdgeFlush TEF = (TempEdgeFlush) obj;
				TEF.setOld(_TempEdges);
				_TempEdges = new ArrayList();
				return true;
			}
			else if (obj instanceof NewColorLegend)
			{
				NewColorLegend NCL = (NewColorLegend) obj;
				NCL.setOld(_ColorLegend);
				_ColorLegend = new ArrayList();
				return true;
			}
			else if (obj instanceof ColorLegendMap)
			{
				_ColorLegend.add(obj);
				return true;
			}
			else if(obj instanceof CallForLayoutPolyTree)
			{
				LayPolyTree.apply( _Graph );
				_DoZoom2Fit = true;
			}
		}
		return false;
	}
	/*! apply the next operator
	 */
	public void applyNextFrame()
	{
		while (applyNextFrameP())
		{}
	}
	/*! apply the previous frame
	 * \return true if it should go again
	 */
	public boolean applyPreviousFrameP()
	{
		if (_Cursor >= 0)
		{
			_time2next = _time2prev;
			Object obj = _Operators.get(_Cursor);
			_Cursor--;
			if (obj instanceof GraphOperator)
			{
				GraphOperator o = (GraphOperator) obj;
				_time2prev = o.getTime();
				_Graph = o.applyInverse(_Graph);
			}
			else if (obj instanceof MarkupOperator)
			{
				MarkupOperator MO = (MarkupOperator) obj;
				MO.applyInverse(_CurrentMarkings);
				_time2prev = 50;
			}
			else if (obj instanceof CodePageOperator)
			{
				CodePageOperator cpo = (CodePageOperator) obj;
				if (_CurrentPage < 0)
					cpo.applyInverse(null);
				else
					cpo.applyInverse(_codepages[_CurrentPage]);
				_CurrentPage = cpo.getCodePage();
				_time2prev = 25;
				if (!(cpo instanceof CodePageSelectLine))
				{
					return true;
				}
			}
			else if (obj instanceof Annotation)
			{
				Annotation AN = (Annotation) obj;
				_Header = AN.applyInverse();
				_time2prev = 25;
				return true;
			}
			else if (obj instanceof FlushMarkings)
			{
				FlushMarkings FM = (FlushMarkings) obj;
				_CurrentMarkings = FM.applyInverse();
				_time2next = 25;
				return true;
			}
			else if (obj instanceof TempEdgeCreate)
			{
				TempEdgeCreate TEC = (TempEdgeCreate) obj;
				_TempEdges.remove(TEC.edgeCreate);
				_time2next = 25;
			}
			else if (obj instanceof TempEdgeFlush)
			{
				TempEdgeFlush TEF = (TempEdgeFlush) obj;
				_TempEdges = TEF.getOld();
				return true;
			}
			else if (obj instanceof NewColorLegend)
			{
				NewColorLegend NCL = (NewColorLegend) obj;
				_ColorLegend = NCL.getOld();
				return true;
			}
			else if (obj instanceof ColorLegendMap)
			{
				_ColorLegend.remove(obj);
				return true;
			}
		}
		return false;
	}
	/*! apply the previous frame
	 */
	public void applyPreviousFrame()
	{
		while (applyPreviousFrameP())
		{}
	}
	/*! is the animation done
	 * \return
	 */
	public boolean isStopped()
	{
		return (_Cursor < 0) || (_Operators.size() <= _Cursor);
	}
	/*! get the current graph frame
	 * \return
	 */
	public Graph getFrame()
	{
		return _Graph;
	}
	/*! get the time 2 next operation
	 * \return
	 */
	public int getTime2Next()
	{
		return _time2next;
	}
	/*! get ideal time to previous operation
	 * \return
	 */
	public int getTime2Prev()
	{
		return _time2prev;
	}
}