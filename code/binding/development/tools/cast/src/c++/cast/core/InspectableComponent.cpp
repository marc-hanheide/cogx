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
 
 #include "InspectableComponent.hpp"


namespace cast {

using namespace std;
using namespace cdl;
using namespace cdl::guitypes;


InspectableComponent::InspectableComponent(const string &_id)
: FrameworkProcess(_id)
{
  m_flushConnector = 0;
}

InspectableComponent::~InspectableComponent()
{
}

void InspectableComponent::setPushConnector(const string &_connectionID,
					    PushConnectorOut<DrawBatch> *_out)
{
	m_flushConnector = _out;
}

void InspectableComponent::clearGraphics()
{
	m_buffer.clear();
}

/**
 * Indicate to teh GUI that we wish to have ourselves redrawn NOW.
 * Note that depending on what the GUI has set as currently visible
 * component the redraw might or might not actually happen
 */
void InspectableComponent::redrawGraphicsNow()
{
	if(m_flushConnector != 0)
	{
		DrawBatch *batch = new DrawBatch();
		// Just send an empty batch to "wake up" GUI. The GUI will then use its
		// normal pull connection to do the actual redraw.
		// Downside: two communiction steps
		// Upside: If the GUI is not currently displaying this component (so that
		// nothing should actually be drawn) we don't waste bandwidth sending lots
		// of drawing primitives into nirvana.
		m_flushConnector->push(
			new FrameworkLocalData<DrawBatch>(getProcessIdentifier(), batch));
	}
}

/**
 * We can process the following types of query:
 * "GetConfig" to tell the GUI who we are.
 * "Redraw2D", "Redraw3D" and "RedrawText", whenever we should redraw 
 * ourselves into the GUI.
 */
void InspectableComponent::receivePullQuery(const FrameworkQuery &_query,
					    FrameworkLocalData<DrawBatch> *&_pData) {

  //lockProcess();

  DrawBatch *batch = new DrawBatch();

  // This query will typically be called once by the GUI
  // Just send back our ID.
  if(_query.getQuery() == "GetConfig")
  {
    batch->m_compID = CORBA::string_dup(getProcessIdentifier().c_str());
  }
  // this query is called whenever a 2D redraw is necessary
  else if(_query.getQuery() == "Redraw2D")
  {
    clearGraphics();
    redrawGraphics2D();
    copyDrawBufferToBatch(batch);
  }
  // this query is called whenever a 3D redraw is necessary
  else if(_query.getQuery() == "Redraw3D")
  {
    clearGraphics();
    redrawGraphics3D();
    copyDrawBufferToBatch(batch);
  }
  // this query is called whenever a 2D redraw is necessary
  else if(_query.getQuery() == "RedrawText")
  {
    clearGraphics();
    redrawGraphicsText();
    copyDrawBufferToBatch(batch);
  }
  _pData = new FrameworkLocalData<DrawBatch>(getProcessIdentifier(), batch);

  //unlockProcess();
}

/**
 * Copy the draw buffer into a draw batch.
 * This is necessary because IDL types can only have arrays of FIXED size.
 */
void InspectableComponent::copyDrawBufferToBatch(DrawBatch *batch)
{
	int i = 0;

	batch->m_image.m_width = m_buffer.m_image.m_width;
	batch->m_image.m_height = m_buffer.m_image.m_height;
	batch->m_image.m_rgbBuffer = m_buffer.m_image.m_rgbBuffer;
	batch->m_image.m_flags = m_buffer.m_image.m_flags;

	i = 0;
	batch->m_text2Ds.length(m_buffer.m_text2Ds.size());
	for(list<Text2D>::iterator it = m_buffer.m_text2Ds.begin();
		it != m_buffer.m_text2Ds.end(); it++, i++)
		batch->m_text2Ds[i] = *it;

	i = 0;
	batch->m_point2Ds.length(m_buffer.m_point2Ds.size());
	for(list<Point2D>::iterator it = m_buffer.m_point2Ds.begin();
		it != m_buffer.m_point2Ds.end(); it++, i++)
		batch->m_point2Ds[i] = *it;

	i = 0;
	batch->m_line2Ds.length(m_buffer.m_line2Ds.size());
	for(list<Line2D>::iterator it = m_buffer.m_line2Ds.begin();
		it != m_buffer.m_line2Ds.end(); it++, i++)
		batch->m_line2Ds[i] = *it;

	i = 0;
	batch->m_rect2Ds.length(m_buffer.m_rect2Ds.size());
	for(list<Rect2D>::iterator it = m_buffer.m_rect2Ds.begin();
		it != m_buffer.m_rect2Ds.end(); it++, i++)
		batch->m_rect2Ds[i] = *it;

	i = 0;
	batch->m_poly2Ds.length(m_buffer.m_poly2Ds.size());
	for(list<Polygon2D>::iterator it = m_buffer.m_poly2Ds.begin();
		it != m_buffer.m_poly2Ds.end(); it++, i++)
		batch->m_poly2Ds[i] = *it;

	i = 0;
	batch->m_text3Ds.length(m_buffer.m_text3Ds.size());
	for(list<Text3D>::iterator it = m_buffer.m_text3Ds.begin();
		it != m_buffer.m_text3Ds.end(); it++, i++)
		batch->m_text3Ds[i] = *it;

	i = 0;
	batch->m_point3Ds.length(m_buffer.m_point3Ds.size());
	for(list<Point3D>::iterator it = m_buffer.m_point3Ds.begin();
		it != m_buffer.m_point3Ds.end(); it++, i++)
		batch->m_point3Ds[i] = *it;

	i = 0;
	batch->m_line3Ds.length(m_buffer.m_line3Ds.size());
	for(list<Line3D>::iterator it = m_buffer.m_line3Ds.begin();
		it != m_buffer.m_line3Ds.end(); it++, i++)
		batch->m_line3Ds[i] = *it;

	i = 0;
	batch->m_box3Ds.length(m_buffer.m_box3Ds.size());
	for(list<Box3D>::iterator it = m_buffer.m_box3Ds.begin();
		it != m_buffer.m_box3Ds.end(); it++, i++)
		batch->m_box3Ds[i] = *it;

	i = 0;
	batch->m_frame3Ds.length(m_buffer.m_frame3Ds.size());
	for(list<Frame3D>::iterator it = m_buffer.m_frame3Ds.begin();
		it != m_buffer.m_frame3Ds.end(); it++, i++)
		batch->m_frame3Ds[i] = *it;

	i = 0;
	batch->m_texts.length(m_buffer.m_texts.size());
	for(list<string>::iterator it = m_buffer.m_texts.begin();
		it != m_buffer.m_texts.end(); it++, i++)
		batch->m_texts[i] = CORBA::string_dup(it->c_str());
}

void InspectableComponent::drawRGBImage(int width, int height,
	const char *rgbBuffer, int flags)
{
  unsigned int size = 3*width*height;  // we always have RGB
	m_buffer.m_image.m_width = width;
	m_buffer.m_image.m_height = height;
	m_buffer.m_image.m_rgbBuffer.length(size);
	for(unsigned int i = 0; i < size; i++)
	  m_buffer.m_image.m_rgbBuffer[i] = rgbBuffer[i];
	m_buffer.m_image.m_flags = flags;
}

void InspectableComponent::drawRGBImage(int width, int height,
	const vector<char> &rgbBuffer, int flags) throw(CASTException)
{
  if(rgbBuffer.size() != static_cast<unsigned int>(3*width*height)) {
    throw CASTException(__HERE__,
			"image size %d pixels does not match dimensions %d x %d",
			rgbBuffer.size(), width, height);
  }

  m_buffer.m_image.m_width = width;
  m_buffer.m_image.m_height = height;
  m_buffer.m_image.m_rgbBuffer.length(rgbBuffer.size());

  for(unsigned int i = 0; i < rgbBuffer.size(); i++) {
    m_buffer.m_image.m_rgbBuffer[i] = rgbBuffer[i];
  }

  m_buffer.m_image.m_flags = flags;
}

void InspectableComponent::drawText2D(double x, double y, const string &text,
		int red, int green, int blue, int flags)
{
	Text2D naughtyCorba;
	naughtyCorba.m_x = x;
	naughtyCorba.m_y = y;
	naughtyCorba.m_text = CORBA::string_dup(text.c_str());
	naughtyCorba.m_color.m_r = red;
	naughtyCorba.m_color.m_g = green;
	naughtyCorba.m_color.m_b = blue;
	naughtyCorba.m_flags = flags;
	m_buffer.m_text2Ds.push_back(naughtyCorba);
}

void InspectableComponent::drawPoint2D(double x, double y,
		int red, int green, int blue, int flags)
{
	Point2D naughtyCorba;
	naughtyCorba.m_x = x;
	naughtyCorba.m_y = y;
	naughtyCorba.m_color.m_r = red;
	naughtyCorba.m_color.m_g = green;
	naughtyCorba.m_color.m_b = blue;
	naughtyCorba.m_flags = flags;
	m_buffer.m_point2Ds.push_back(naughtyCorba);
}

void InspectableComponent::drawLine2D(double x1, double y1, double x2, double y2,
		int red, int green, int blue, int flags)
{
	Line2D naughtyCorba;
	naughtyCorba.m_x1 = x1;
	naughtyCorba.m_y1 = y1;
	naughtyCorba.m_x2 = x2;
	naughtyCorba.m_y2 = y2;
	naughtyCorba.m_color.m_r = red;
	naughtyCorba.m_color.m_g = green;
	naughtyCorba.m_color.m_b = blue;
	naughtyCorba.m_flags = flags;
	m_buffer.m_line2Ds.push_back(naughtyCorba);
}

void InspectableComponent::drawRect2D(double xmin, double ymin,
		double xmax, double ymax,int red, int green, int blue, int flags)
{
	Rect2D naughtyCorba;
	naughtyCorba.m_xmin = xmin;
	naughtyCorba.m_ymin = ymin;
	naughtyCorba.m_xmax = xmax;
	naughtyCorba.m_ymax = ymax;
	naughtyCorba.m_color.m_r = red;
	naughtyCorba.m_color.m_g = green;
	naughtyCorba.m_color.m_b = blue;
	naughtyCorba.m_flags = flags;
	m_buffer.m_rect2Ds.push_back(naughtyCorba);
}

void InspectableComponent::drawPolygon2D(const vector<double> &x, const vector<double> &y,
		int red, int green, int blue, int flags)
{
	int n = min(x.size(), y.size());
	Polygon2D naughtyCorba;
	naughtyCorba.m_x.length(n);
	naughtyCorba.m_y.length(n);
	for(int i = 0; i < n; i++)
	{
		naughtyCorba.m_x[i] = x[i];
		naughtyCorba.m_y[i] = y[i];
	}
	naughtyCorba.m_color.m_r = red;
	naughtyCorba.m_color.m_g = green;
	naughtyCorba.m_color.m_b = blue;
	naughtyCorba.m_flags = flags;
	m_buffer.m_poly2Ds.push_back(naughtyCorba);
}

void InspectableComponent::drawText3D(double x, double y, double z,
		const string &text, int red, int green, int blue, int flags)
{
	Text3D naughtyCorba;
	naughtyCorba.m_x = x;
	naughtyCorba.m_y = y;
	naughtyCorba.m_z = z;
	naughtyCorba.m_text = CORBA::string_dup(text.c_str());
	naughtyCorba.m_color.m_r = red;
	naughtyCorba.m_color.m_g = green;
	naughtyCorba.m_color.m_b = blue;
	naughtyCorba.m_flags = flags;
	m_buffer.m_text3Ds.push_back(naughtyCorba);
}

void InspectableComponent::drawPoint3D(double x, double y, double z,
		int red, int green, int blue, int flags)
{
	Point3D naughtyCorba;
	naughtyCorba.m_x = x;
	naughtyCorba.m_y = y;
	naughtyCorba.m_z = z;
	naughtyCorba.m_color.m_r = red;
	naughtyCorba.m_color.m_g = green;
	naughtyCorba.m_color.m_b = blue;
	naughtyCorba.m_flags = flags;
	m_buffer.m_point3Ds.push_back(naughtyCorba);
}

void InspectableComponent::drawLine3D(double x1, double y1, double z1,
		double x2, double y2, double z2, int red, int green, int blue, int flags)
{
	Line3D naughtyCorba;
	naughtyCorba.m_x1 = x1;
	naughtyCorba.m_y1 = y1;
	naughtyCorba.m_z1 = z1;
	naughtyCorba.m_x2 = x2;
	naughtyCorba.m_y2 = y2;
	naughtyCorba.m_z2 = z2;
	naughtyCorba.m_color.m_r = red;
	naughtyCorba.m_color.m_g = green;
	naughtyCorba.m_color.m_b = blue;
	naughtyCorba.m_flags = flags;
	m_buffer.m_line3Ds.push_back(naughtyCorba);
}

void InspectableComponent::drawBox3D(double cx, double cy, double cz,
    double sx, double sy, double sz, int red, int green, int blue, int flags)
{
	Box3D naughtyCorba;
	naughtyCorba.m_cx = cx;
	naughtyCorba.m_cy = cy;
	naughtyCorba.m_cz = cz;
	naughtyCorba.m_sx = sx;
	naughtyCorba.m_sy = sy;
	naughtyCorba.m_sz = sz;
	naughtyCorba.m_color.m_r = red;
	naughtyCorba.m_color.m_g = green;
	naughtyCorba.m_color.m_b = blue;
	naughtyCorba.m_flags = flags;
	m_buffer.m_box3Ds.push_back(naughtyCorba);
}

void InspectableComponent::drawFrame3D(double px, double py, double pz,
		double rx, double ry, double rz, int red, int green, int blue, int flags)
{
	Frame3D naughtyCorba;
	naughtyCorba.m_px = px;
	naughtyCorba.m_py = py;
	naughtyCorba.m_pz = pz;
	naughtyCorba.m_rx = rx;
	naughtyCorba.m_ry = ry;
	naughtyCorba.m_rz = rz;
	naughtyCorba.m_color.m_r = red;
	naughtyCorba.m_color.m_g = green;
	naughtyCorba.m_color.m_b = blue;
	naughtyCorba.m_flags = flags;
	m_buffer.m_frame3Ds.push_back(naughtyCorba);
}

void InspectableComponent::printText(const string &text)
{
	m_buffer.m_texts.push_back(text);
}

/**
 * Output text printf-style.
 * @param format   (in) printf-style format string
 * Note: text length is limited to 1024 (including terminating '\0')
 */
void InspectableComponent::printText(const char *format, ...)
{
	static char what[1024];
	va_list arg_list;
	va_start(arg_list, format);
	vsnprintf(what, 1024, format, arg_list);
	va_end(arg_list);
	printText(string(what));
}

} //namespace cast
