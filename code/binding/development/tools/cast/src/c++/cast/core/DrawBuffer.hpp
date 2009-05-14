/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Michael Zillich, Nick Hawes
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
 
 
#ifndef DRAW_BUFFER_H
#define DRAW_BUFFER_H

#include <list>
#include <string>
#include <cast/cdl/guitypes.hh>

namespace cast {

  class DrawBuffer
  {
  public:
    cdl::guitypes::RGBImage m_image;
    std::list<cdl::guitypes::Text2D> m_text2Ds;
    std::list<cdl::guitypes::Point2D> m_point2Ds;
    std::list<cdl::guitypes::Line2D> m_line2Ds;
    std::list<cdl::guitypes::Rect2D> m_rect2Ds;
    std::list<cdl::guitypes::Polygon2D> m_poly2Ds;

    std::list<cdl::guitypes::Text3D> m_text3Ds;
    std::list<cdl::guitypes::Point3D> m_point3Ds;
    std::list<cdl::guitypes::Line3D> m_line3Ds;
    std::list<cdl::guitypes::Box3D> m_box3Ds;
    std::list<cdl::guitypes::Frame3D> m_frame3Ds;

    std::list<std::string> m_texts;

    void clear() {
      m_image.m_rgbBuffer.length(0);
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
  };


} //namespace cast
#endif
