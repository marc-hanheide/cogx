//
// = LIBRARY
//
// = FILENAME
//    X11DispLocalGridMap.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = DESCRIPTION
//
// = COPYRIGHT
//    Copyright (c) 2005 Patric Jensfelt (XDisplayLocalGridMap.hh in Cure)
//                  2009 Alper Aydemir and Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef X11DispLocalGridMap_hh
#define X11DispLocalGridMap_hh

#include "Transformation/Pose3D.hh"
#include "Utils/CureDebug.hh"
#include "Navigation/FrontierFinder.hh"
#include "Navigation/NavGraph.hh"
#include "Navigation/NavGraphNode.hh"

#ifndef DEPEND
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <list>
#endif

namespace Cure {

/**
 * Class that that provides a simple way to display a LocalGridMap
 *
 * NOTE that even though the source for this class is in the toolbox
 * you will have to include the library explicitly (it will not come
 * with libtoolbox). The reason for this is to avoid dependences on
 * X11 stuff in the toolbox.
 * 
 * @author Patric Jensfelt 
 * @see
 */
template<class MAPDATA>
class X11DispLocalGridMap {
public:

	/**
	 * Create a new display window of size width x height pixels where
	 * the center of the window corresponds to position xc,yc in the
	 * world
	 *
	 * @param lm reference to LocalGridMap to display
	 * @param magnification need to be >=1, if 2 each cell in lm results
	 * in 2x2 pixels in the display
	 */
	X11DispLocalGridMap(LocalGridMap<MAPDATA> &lm, int magnification = 1);

	~X11DispLocalGridMap();

	/**
	 * Set the robot pose
	 * @param x x-coordinate of the robot [m]
	 * @param y y-coordinate of the robot [m]
	 * @param a orientation of the robot [rad]
	 */
	void setRobotPose(double x, double y, double a);

	/**
	 * Set the robot pose
	 * @rp robot pose
	 */
	void setRobotPose(const Pose3D &rp);

	/**
	 * The function that does the actual displaying. Without extra
	 * arguments it just displays the LocalMap but you can supply extra
	 * stuff to display like a NavGraph or the FronttierPoints.
	 *
	 * @param fPts list of frontier points
	 */
	void updateDisplay(Cure::Pose3D *robPose = 0, NavGraph *graph = 0, std::list<
			FrontierPt> *fPts = 0, int m_samplesize = 0, int* samples = 0,
			std::vector<int> tpoints = 0, std::vector<std::vector<int> > ViewConePts =
					0, int highestVCindex = 0, std::vector<Cure::Pose3D> plan = 0,
			std::vector<int> planindex = 0);
	void updatePlaneDisplay(Pose3D *robPose = 0);

	void updateCoverageDisplay();

private:
	LocalGridMap<MAPDATA> &m_Map;

	// X11 stuff
	Display *disp;
	Window win;
	GC gcBlack, gcWhite, gcGrey;
	GC gcGreen, gcBlue, gcRed, gcYellow, gcMagenta;
	XSizeHints hint;
	int screen;
	unsigned long foreg, backg;
	Pixmap pixmap;
	XGCValues values;
	XColor color, exact;
	GC grayScale[255];
	/** Pose of the robot */
	Cure::Pose3D m_Xr;

	/** Points in polygon that describe the robot */
	XPoint m_RobPts[50];

	int m_Magnification;
}; // class X11DispLocalGridMap


template<class MAPDATA>
X11DispLocalGridMap<MAPDATA>::X11DispLocalGridMap(LocalGridMap<MAPDATA> &lm,
		int magnification) :
	m_Map(lm) {
	if ((disp = XOpenDisplay("")) == NULL) {
		CureCERR(10) << "\nCan't open display\nEXIT!!!";
		exit(1);
	}

	m_Magnification = magnification;

	if (m_Magnification < 1) {
		std::cerr << "WARNING: X11DispLocalGridMap only accepts magnification>=1\n";
		m_Magnification = 1;
	}

	screen = DefaultScreen(disp);
	backg = WhitePixel(disp, screen);
	foreg = BlackPixel(disp, screen);
	hint.x = 200;
	hint.y = 300;
	hint.width = (2 * lm.getSize() + 1) * m_Magnification;
	hint.height = (2 * lm.getSize() + 1) * m_Magnification;
	hint.flags = PPosition | PSize;
	win = XCreateSimpleWindow(disp, DefaultRootWindow(disp), hint.x, hint.y,
			hint.width, hint.height, 5, foreg, backg);
	pixmap = XCreatePixmap(disp, win, hint.width, hint.height, XDefaultDepth(
			disp, screen));
	gcBlack = XCreateGC(disp, pixmap, 0, 0);
	gcWhite = XCreateGC(disp, pixmap, 0, 0);

	unsigned long pixel;
	if (XAllocNamedColor(disp, DefaultColormap(disp, screen), "blue", &color,
			&exact))
		pixel = color.pixel;
	else
		pixel = XBlackPixel(disp, screen);
	values.foreground = pixel;
	values.line_width = 1 * m_Magnification;
	values.fill_style = FillSolid;
	values.arc_mode = ArcPieSlice;
	unsigned long valuemask = 0;
	valuemask |= GCForeground | GCLineWidth | GCFillStyle | GCArcMode;
	gcBlue = XCreateGC(disp, win, valuemask, &values);

	if (XAllocNamedColor(disp, DefaultColormap(disp, screen), "grey", &color,
			&exact))
		pixel = color.pixel;
	else
		pixel = XBlackPixel(disp, screen);
	values.foreground = pixel;
	values.line_width = 1 * m_Magnification;
	values.fill_style = FillSolid;
	values.arc_mode = ArcPieSlice;
	valuemask = GCForeground | GCLineWidth | GCFillStyle | GCArcMode;
	gcGrey = XCreateGC(disp, win, valuemask, &values);

	if (XAllocNamedColor(disp, DefaultColormap(disp, screen), "green", &color,
			&exact))
		pixel = color.pixel;
	else
		pixel = XBlackPixel(disp, screen);
	values.foreground = pixel;
	values.line_width = 1 * m_Magnification;
	values.fill_style = FillSolid;
	values.arc_mode = ArcPieSlice;
	valuemask = GCForeground | GCLineWidth | GCFillStyle | GCArcMode;
	gcGreen = XCreateGC(disp, win, valuemask, &values);

	if (XAllocNamedColor(disp, DefaultColormap(disp, screen), "red", &color,
			&exact))
		pixel = color.pixel;
	else
		pixel = XBlackPixel(disp, screen);
	values.foreground = pixel;
	values.line_width = 1 * m_Magnification;
	values.fill_style = FillSolid;
	values.arc_mode = ArcPieSlice;
	valuemask = GCForeground | GCLineWidth | GCFillStyle | GCArcMode;
	gcRed = XCreateGC(disp, win, valuemask, &values);

	if (XAllocNamedColor(disp, DefaultColormap(disp, screen), "yellow", &color,
			&exact))
		pixel = color.pixel;
	else
		pixel = XBlackPixel(disp, screen);
	values.foreground = pixel;
	values.line_width = 1 * m_Magnification;
	values.fill_style = FillSolid;
	values.arc_mode = ArcPieSlice;
	valuemask = GCForeground | GCLineWidth | GCFillStyle | GCArcMode;
	gcYellow = XCreateGC(disp, win, valuemask, &values);

	if (XAllocNamedColor(disp, DefaultColormap(disp, screen), "magenta", &color,
			&exact))
		pixel = color.pixel;
	else
		pixel = XBlackPixel(disp, screen);
	values.foreground = pixel;
	values.line_width = 1 * m_Magnification;
	values.fill_style = FillSolid;
	values.arc_mode = ArcPieSlice;
	valuemask = GCForeground | GCLineWidth | GCFillStyle | GCArcMode;
	gcMagenta = XCreateGC(disp, win, valuemask, &values);

	XColor temp;
	unsigned short int coeff = (unsigned short int) (65535 / 255);
	for (unsigned short i = 0; i < 255; i++) {
		temp.red = coeff * i;
		temp.blue = coeff * i;
		temp.green = coeff * i;
		if (XAllocColor(disp, DefaultColormap(disp, screen), &temp)) {
			values.foreground = temp.pixel;
			values.line_width = 1 * m_Magnification;
			values.fill_style = FillSolid;
			values.arc_mode = ArcPieSlice;
			valuemask = GCForeground | GCLineWidth | GCFillStyle | GCArcMode;
			grayScale[254 - i] = XCreateGC(disp, win, valuemask, &values);
		}

	}

	XSetBackground(disp, gcBlack, backg);
	XSetForeground(disp, gcBlack, foreg);
	XSetBackground(disp, gcWhite, foreg);
	XSetForeground(disp, gcWhite, backg);
	XMapRaised(disp, win);
	XFlush(disp);

}

template<class MAPDATA>
X11DispLocalGridMap<MAPDATA>::~X11DispLocalGridMap() {
	XFreeGC(disp, gcBlack);
	XFreeGC(disp, gcWhite);
	XFreeGC(disp, gcBlue);
	XFreeGC(disp, gcGrey);
	XFreeGC(disp, gcGreen);
	XFreeGC(disp, gcRed);
	XFreeGC(disp, gcYellow);
	XFreeGC(disp, gcMagenta);
	XFreePixmap(disp, pixmap);
	XDestroyWindow(disp, win);
	XCloseDisplay(disp);
}

/*
 template <class MAPDATA> void
 X11DispLocalGridMap<MAPDATA>::setScreenCenter(double xc, double yc)
 { m_Xc = xc; m_Yc = yc; }
 
 template <class MAPDATA> void
 X11DispLocalGridMap<MAPDATA>::getScreenCenter(double &xc, double &yc) const
 { xc = m_Xc; yc = m_Yc; }
 */

template<class MAPDATA>
void X11DispLocalGridMap<MAPDATA>::setRobotPose(double x, double y, double a) {
	Cure::Pose3D p;
	p.Position.X[0] = x;
	p.Position.X[1] = y;
	p.setTheta(a);

	setRobotPose(p);
}

template<class MAPDATA>
void X11DispLocalGridMap<MAPDATA>::setRobotPose(const Pose3D &p) {
	m_Xr = p;

	double aStep = 2.0 * M_PI / (50 - 1);
	for (int i = 0; i < 50; i++) {
		int xi, yi;
		m_Map.worldCoords2Index(m_Xr.Position.X[0] + 0.2 * cos(aStep * i),
				m_Xr.Position.X[1] + 0.2 * sin(aStep * i), xi, yi);
		m_RobPts[i].x = (xi + m_Map.getSize()) * m_Magnification;
		m_RobPts[i].y = (m_Map.getSize() - yi) * m_Magnification;
	}
}

template<class MAPDATA>
void X11DispLocalGridMap<MAPDATA>::updateCoverageDisplay() {
	//printf("Updating coverage display");
	XFillRectangle(disp, pixmap, gcWhite, 0, 0, hint.width, hint.height);
	XPoint gridPt;

	if (m_Magnification == 1) {

		for (int y = -m_Map.getSize(); y <= m_Map.getSize(); y++) {
			for (int x = -m_Map.getSize(); x <= m_Map.getSize(); x++) {
				gridPt.x = x + m_Map.getSize();
				gridPt.y = 2 * m_Map.getSize() - (y + m_Map.getSize());
				if (m_Map(x, y) == '0') {
					XDrawPoints(disp, pixmap, gcGrey, &gridPt, 1, CoordModeOrigin);
				} else if (m_Map(x, y) == '1') {
					XDrawPoints(disp, pixmap, gcBlack, &gridPt, 1, CoordModeOrigin);
				} else if (m_Map(x, y) == '2') {
					XDrawPoints(disp, pixmap, gcYellow, &gridPt, 1, CoordModeOrigin);
				}
			}
		}

	} else {

		for (int y = -m_Map.getSize(); y <= m_Map.getSize(); y++) {
			for (int x = -m_Map.getSize(); x <= m_Map.getSize(); x++) {

				for (int mx = 0; mx < m_Magnification; mx++) {
					for (int my = 0; my < m_Magnification; my++) {

						gridPt.x = (x + m_Map.getSize()) * m_Magnification + mx;
						gridPt.y = (2 * m_Map.getSize() - (y + m_Map.getSize()))
								* m_Magnification - my;

						if (m_Map(x, y) == 0) {
							XDrawPoints(disp, pixmap, gcWhite, &gridPt, 1, CoordModeOrigin);
						} else if (m_Map(x, y) == 1) {
							XDrawPoints(disp, pixmap, gcBlack, &gridPt, 1, CoordModeOrigin);
						} else if (m_Map(x, y) == 2) {
							XDrawPoints(disp, pixmap, gcGrey, &gridPt, 1, CoordModeOrigin);
						}
					}
				}
			}
		}

	}

	XCopyArea(disp, pixmap, win, gcBlack, 0, 0, hint.width, hint.height, 0, 0);
	XFlush(disp);
}

template<class MAPDATA>
void X11DispLocalGridMap<MAPDATA>::updatePlaneDisplay(Pose3D *robPose) {
	//printf("Updating coverage display");
	XFillRectangle(disp, pixmap, gcWhite, 0, 0, hint.width, hint.height);
	XPoint gridPt;

	if (m_Magnification == 1) {

		for (int y = -m_Map.getSize(); y <= m_Map.getSize(); y++) {
			for (int x = -m_Map.getSize(); x <= m_Map.getSize(); x++) {
				gridPt.x = x + m_Map.getSize();
				gridPt.y = 2 * m_Map.getSize() - (y + m_Map.getSize());
				if (m_Map(x, y) == 0) {
					XDrawPoints(disp, pixmap, gcWhite, &gridPt, 1, CoordModeOrigin);
				} else if (m_Map(x, y) == 1) {
					XDrawPoints(disp, pixmap, gcBlack, &gridPt, 1, CoordModeOrigin);
				} else if (m_Map(x, y) == 2) {
					XDrawPoints(disp, pixmap, gcGrey, &gridPt, 1, CoordModeOrigin);
				} else if (m_Map(x, y) == 3) {
					XDrawPoints(disp, pixmap, gcMagenta, &gridPt, 1, CoordModeOrigin);
				}
			}

		}
	} else {

		for (int y = -m_Map.getSize(); y <= m_Map.getSize(); y++) {
			for (int x = -m_Map.getSize(); x <= m_Map.getSize(); x++) {

				for (int mx = 0; mx < m_Magnification; mx++) {
					for (int my = 0; my < m_Magnification; my++) {

						gridPt.x = (x + m_Map.getSize()) * m_Magnification + mx;
						gridPt.y = (2 * m_Map.getSize() - (y + m_Map.getSize()))
								* m_Magnification - my;

						if (m_Map(x, y) == 0) {
							XDrawPoints(disp, pixmap, gcWhite, &gridPt, 1, CoordModeOrigin);
						} else if (m_Map(x, y) == 1) {
							XDrawPoints(disp, pixmap, gcBlack, &gridPt, 1, CoordModeOrigin);
						} else if (m_Map(x, y) == 2) {
							XDrawPoints(disp, pixmap, gcGrey, &gridPt, 1, CoordModeOrigin);
						} else if (m_Map(x, y) == 3) {
							XDrawPoints(disp, pixmap, gcMagenta, &gridPt, 1, CoordModeOrigin);
						}
					}
				}
			}
		}

	}

	if (robPose) {
		setRobotPose(*robPose);
		XDrawPoints(disp, pixmap, gcBlack, m_RobPts, 50, CoordModeOrigin);
	}

	XCopyArea(disp, pixmap, win, gcBlack, 0, 0, hint.width, hint.height, 0, 0);
	XFlush(disp);
}

template<class MAPDATA>
void X11DispLocalGridMap<MAPDATA>::updateDisplay(Pose3D *robPose,
		NavGraph *graph, std::list<FrontierPt> *fPts, int m_samplesize,
		int* samples, std::vector<int> tpoints,
		std::vector<std::vector<int> > ViewConePts, int highestVCindex,
		std::vector<Cure::Pose3D> plan, std::vector<int> planindex) {

	//printf("Updating display\n");
	// clear the pixmap
	XFillRectangle(disp, pixmap, gcWhite, 0, 0, hint.width, hint.height);
	XPoint gridPt;

	int msize = m_Map.getSize() * m_Magnification;

	if (m_Magnification == 1) {

		for (int y = -m_Map.getSize(); y <= m_Map.getSize(); y++) {
			for (int x = -m_Map.getSize(); x <= m_Map.getSize(); x++) {
				gridPt.x = x + m_Map.getSize();
				gridPt.y = 2 * m_Map.getSize() - (y + m_Map.getSize());
				if (m_Map(x, y) == '1') {
					XDrawPoints(disp, pixmap, gcBlack, &gridPt, 1, CoordModeOrigin);
				} //else if (0 <= m_Map(x,y) &&  m_Map(x,y) < 255) {
				//XDrawPoints(disp, pixmap, grayScale[int(m_Map(x,y))], &gridPt, 1, CoordModeOrigin);
				else if (m_Map(x, y) == '0') {
					XDrawPoints(disp, pixmap, gcWhite, &gridPt, 1, CoordModeOrigin);
				} else {
					XDrawPoints(disp, pixmap, gcMagenta, &gridPt, 1, CoordModeOrigin);
				}
			}
		}

		// Display samples
		for (int i = 0; i < m_samplesize; i++) {
			gridPt.x = samples[2 * i] * m_Magnification + msize;
			gridPt.y = 2 * msize - (samples[2 * i + 1] * m_Magnification + msize);
			XDrawPoints(disp, pixmap, gcGreen, &gridPt, 1, CoordModeOrigin);
		}

	} else {

		for (int y = -m_Map.getSize(); y <= m_Map.getSize(); y++) {
			for (int x = -m_Map.getSize(); x <= m_Map.getSize(); x++) {
				for (int mx = 0; mx < m_Magnification; mx++) {
					for (int my = 0; my < m_Magnification; my++) {

						gridPt.x = (x + m_Map.getSize()) * m_Magnification + mx;
						gridPt.y = (2 * m_Map.getSize() - (y + m_Map.getSize()))
								* m_Magnification - my;
						if (m_Map(x, y) == 1) {
							XDrawPoints(disp, pixmap, gcBlack, &gridPt, 1, CoordModeOrigin);
						} else if (0 < m_Map(x, y) && m_Map(x, y) < 255) {
							XDrawPoints(disp, pixmap, grayScale[int(m_Map(x, y))], &gridPt,
									1, CoordModeOrigin);
						} else if (m_Map(x, y) == 0) {
							XDrawPoints(disp, pixmap, gcWhite, &gridPt, 1, CoordModeOrigin);
						} else if (m_Map(x, y) == 256) {
							XDrawPoints(disp, pixmap, gcMagenta, &gridPt, 1, CoordModeOrigin);
						} else {
							printf("%d\n", m_Map(x, y));
							XDrawPoints(disp, pixmap, gcYellow, &gridPt, 1, CoordModeOrigin);
						}
					}
				}
			}
		}

		// Display samples
		for (int i = 0; i < m_samplesize; i++) {
			for (int mx = 0; mx < m_Magnification; mx++) {
				for (int my = 0; my < m_Magnification; my++) {

					gridPt.x = samples[2 * i] * m_Magnification + mx + msize;
					gridPt.y = 2 * msize - (samples[2 * i + 1] * m_Magnification + my
							+ msize);
					XDrawPoints(disp, pixmap, gcGreen, &gridPt, 1, CoordModeOrigin);
				}
			}
		}

	}

	//Display triangle
	//printf("Displaying triangle with size %i", (int)tpoints.size());
	/*for (int i=0; i< int(tpoints.size()/2); i++){
	 gridPt.x = tpoints.at(2*i) + msize;
	 gridPt.y = 2 * msize - (tpoints.at(2*i +1) + msize);
	 XDrawPoints(disp, pixmap, gcGreen, &gridPt, 1, CoordModeOrigin);
	 }*/

	//printf("Displaying %i cones. \n", (int)ViewConePts.size());

	if (graph) {
		int r = int(0.4 / m_Map.getCellSize() + 0.5) * m_Magnification;

		for (std::list<NavGraphEdge>::iterator ei = graph-> m_Edges.begin(); ei
				!= graph->m_Edges.end(); ei++) {
			NavGraphNode *n1 = ei->getNode1();
			NavGraphNode *n2 = ei->getNode2();
			int i1, i2, j1, j2;
			if (n1 != 0 && n2 != 0 && m_Map.worldCoords2Index(n1->getX(), n1->getY(),
					i1, j1) == 0 && m_Map.worldCoords2Index(n2->getX(), n2->getY(), i2,
					j2) == 0) {
				i1 = i1 * m_Magnification + msize;
				j1 = msize - j1 * m_Magnification;
				i2 = i2 * m_Magnification + msize;
				j2 = msize - j2 * m_Magnification;
				XDrawLine(disp, pixmap, gcBlack, i1, j1, i2, j2);
			}
		}

		for (std::list<NavGraphNode*>::iterator ni = graph->m_Nodes.begin(); ni
				!= graph->m_Nodes.end(); ni++) {
			int i, j;
			if (m_Map.worldCoords2Index((*ni)->getX(), (*ni)->getY(), i, j) == 0) {
				i = i * m_Magnification + msize;
				j = msize - j * m_Magnification;
				if ((*ni)->getType() == Cure::NavGraphNode::NODETYPE_GATEWAY) {
					XFillArc(disp, pixmap, gcRed, i - r / 2, j - r / 2, r, r, 0, 64 * 360);
				} else {
					XFillArc(disp, pixmap, gcGreen, i - r / 2, j - r / 2, r, r, 0, 64
							* 360);
				}
			}

		}
	}

	if (fPts) {
		int r = int(0.8 / m_Map.getCellSize() + 0.5) * m_Magnification;
		for (std::list<FrontierPt>::iterator fi = fPts-> begin(); fi != fPts->end(); fi++) {
			int i, j;
			if (m_Map.worldCoords2Index(fi->getX(), fi->getY(), i, j) == 0) {
				i = i * m_Magnification + msize;
				j = msize - j * m_Magnification;

				double halfLen = 0.5 * fi->m_Width / m_Map.getCellSize()
						* m_Magnification;

				GC *gc = &gcBlue;
				if (fi->m_State == FrontierPt::FRONTIER_STATUS_CURRENT) {
					gc = &gcYellow;
				} else if (fi->m_State == FrontierPt::FRONTIER_STATUS_UNREACHABLE) {
					gc = &gcRed;
				} else if (fi->m_State == FrontierPt::FRONTIER_STATUS_PATHBLOCKED) {
					gc = &gcMagenta;
				} else if (fi->m_State == FrontierPt::FRONTIER_STATUS_GATEWAYBLOCKED) {
					gc = &gcGreen;
				}

				XFillArc(disp, pixmap, *gc, i - r / 2, j - r / 2, r, r, int(64 * 180
						* (fi->getTheta() - M_PI_2) / M_PI), 64 * 360 / 2);
				XDrawLine(disp, pixmap, *gc, i + int(cos(fi->getTheta() - M_PI_2)
						* halfLen), j - int(sin(fi->getTheta() - M_PI_2) * halfLen), i
						+ int(cos(fi->getTheta() + M_PI_2) * halfLen), j - int(sin(
						fi->getTheta() + M_PI_2) * halfLen));
			}
		}
	}

	if (robPose) {
		setRobotPose(*robPose);
		XDrawPoints(disp, pixmap, gcBlack, m_RobPts, 50, CoordModeOrigin);
	}
	/*for (unsigned int i = 0; i < ViewConePts.size(); i++) {
	 //printf("Displaying triangle with size %i\n", (int)ViewConePts[i].size());
	 for (unsigned int j=0; j< (unsigned int)(ViewConePts[i].size()/2); j++) {
	 gridPt.x = ViewConePts[i][2*j]*1 + msize;
	 gridPt.y = 2 * msize - (ViewConePts[i][2*j + 1]*1 + msize);
	 XDrawPoints(disp, pixmap, gcYellow, &gridPt, 1, CoordModeOrigin);
	 }
	 }*/

	int x, y;
	/*if ( highestVCindex != -1) {
	 
	 for (unsigned int i = 0; i < ViewConePts.size(); i++) {
	 //printf("Displaying triangle with size %i\n", (int)ViewConePts[i].size());
	 for (unsigned int j=0; j< (unsigned int)(ViewConePts[i].size()/2); j++) {
	 gridPt.x = ViewConePts[i][2*j]*m_Magnification + msize;
	 gridPt.y = 2 * msize - (ViewConePts[i][2*j + 1]*m_Magnification + msize);
	 if (i == (unsigned int)highestVCindex) {
	 XDrawPoints(disp, pixmap, gcYellow, &gridPt, 1, CoordModeOrigin);
	 }
	 }
	 }
	 
	 for (unsigned int i = 0; i < ViewConePts.size(); i++) {
	 gridPt.x = ViewConePts[i][0]*m_Magnification + msize;
	 gridPt.y = 2 * msize - (ViewConePts[i][1]*m_Magnification + msize);
	 x = gridPt.x-3;
	 y = gridPt.y-3;
	 if (i == (unsigned int)highestVCindex) {
	 XDrawRectangle(disp, pixmap, gcMagenta, x, y,10,10);
	 } else {
	 XDrawRectangle(disp, pixmap, gcBlue, x, y,5,5);
	 }
	 }
	 }*/
	GC gcs[] = { gcGreen, gcBlue, gcRed, gcYellow };

	for (unsigned int i = 0; i < planindex.size(); i++) {
		gridPt.x = ViewConePts[planindex[i]][0] * m_Magnification + msize;
		gridPt.y = 2 * msize - (ViewConePts[planindex[i]][1] * m_Magnification
				+ msize);
		x = gridPt.x - 3;
		y = gridPt.y - 3;
		XDrawRectangle(disp, pixmap, gcBlack, x, y, 10, 10);
		for (int j = 0; j < int(ViewConePts[planindex[i]].size() / 2); j++) {
			gridPt.x = ViewConePts[planindex[i]][2 * j] * m_Magnification + msize;
			gridPt.y = 2 * msize - (ViewConePts[planindex[i]][2 * j + 1]
					* m_Magnification + msize);
			XDrawPoints(disp, pixmap, gcs[i % 4], &gridPt, 1, CoordModeOrigin);
		}
	}

	// blit bitmap onto screen
	XCopyArea(disp, pixmap, win, gcBlack, 0, 0, hint.width, hint.height, 0, 0);

	XFlush(disp);
}

} // namespace Cure

#endif // X11DispLocalGridMap_hh
