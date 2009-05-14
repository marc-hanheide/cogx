/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

package cast.ui.inspectable;

import java.util.ArrayList;

import cast.cdl.guitypes.*;
import cast.cdl.ui.*;

public class DrawBuffer
{
	public RGBImage m_image;
	public ArrayList<Text2D> m_text2Ds;
	public ArrayList<Point2D> m_point2Ds;
	public ArrayList<Line2D> m_line2Ds;
	public ArrayList<Rect2D> m_rect2Ds;
	public ArrayList<Polygon2D> m_poly2Ds;

	public ArrayList<Text3D> m_text3Ds;
	public ArrayList<Point3D> m_point3Ds;
	public ArrayList<Line3D> m_line3Ds;
	public ArrayList<Box3D> m_box3Ds;
	public ArrayList<Frame3D> m_frame3Ds;

	public ArrayList<String> m_texts;

	public DrawBuffer()
	{
        m_image = new RGBImage(0,0,new byte[0],0);
        m_text2Ds = new ArrayList<Text2D>();
		m_point2Ds = new ArrayList<Point2D>();
		m_line2Ds = new ArrayList<Line2D>();
		m_rect2Ds = new ArrayList<Rect2D>();
		m_poly2Ds = new ArrayList<Polygon2D>();

		m_text3Ds = new ArrayList<Text3D>();
		m_point3Ds = new ArrayList<Point3D>();
		m_line3Ds = new ArrayList<Line3D>();
		m_box3Ds = new ArrayList<Box3D>();
		m_frame3Ds = new ArrayList<Frame3D>();

		m_texts = new ArrayList<String>();
	}

	public void clear()
	{
        m_image = new RGBImage(0,0,new byte[0],0);

        m_text2Ds.clear();
		m_point2Ds.clear();
		m_line2Ds.clear();
		m_rect2Ds.clear();
		m_poly2Ds.clear();

		m_text3Ds.clear();
		m_point3Ds.clear();
		m_line3Ds.clear();
		m_box3Ds.clear();
		m_frame3Ds.clear();

		m_texts.clear();
	}
}
