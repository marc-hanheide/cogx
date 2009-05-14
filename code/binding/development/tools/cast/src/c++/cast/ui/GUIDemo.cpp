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
 
#include <cstdlib>
#include <cmath>
#include "GUIDemo.hpp"

using namespace std;

extern "C" {
  FrameworkProcess* newComponent(const string &_id) {
    return new cast::GUIDemo(_id);
  }
}

namespace cast {

const double GUIDemo::min2D[2] = {0., 0.,};
const double GUIDemo::max2D[2] = {400., 300.};

// maximum velocity of a 2D drawing primitive (actually sqrt(2)..)
const double GUIDemo::velMax2D = 5.0;

// maximum velocity of a 3D drawing primitive (actually sqrt(2)..)
const double GUIDemo::velMax3D = 0.02;


GUIDemo::GUIDemo(const string &_id)
  : //InspectableComponent(_id),
    //CASTProcessingComponent(_id),
  WorkingMemoryAttachedComponent(_id),  
  UnmanagedProcess(_id)
{

	srand(13);

	// set up 2D motions
	for(int i = 0; i < numPrimitives2D; i++)
	{
		for(int j = X; j <= Y; j++)
		{
			// all primitives start out in the center
			pos2D[i][j] = (min2D[j] + max2D[j])/2.;
			// with random velocities
			vel2D[i][j] = velMax2D*2*((double)rand()/(double)RAND_MAX - 0.5);
		}
	}

	// set up 3D motions
	for(int i = 0; i < numPrimitives3D; i++)
	{
		for(int j = X; j <= Z; j++)
		{
			// all primitives start out in the center of the unit cube
			pos3D[i][j] = 0.5;
			// with random velocities
			vel3D[i][j] = velMax3D*2*((double)rand()/(double)RAND_MAX - 0.5);
		}
	}
	
	// create our image to display
	createImage(imgWidth, imgHeight, rgbImage);
}

void GUIDemo::configure(map<string,string> & _config)
{
  UnmanagedProcess::configure(_config);
}

void GUIDemo::runComponent()
{

  println("running");
	// HACK otherwise GUI crashes!
	sleepProcess(1000);

	while(true)
	{
	  redrawGraphicsNow();

		// update 2D positions
		for(int i = 0; i < numPrimitives2D; i++)
		{
			for(int j = X; j <= Y; j++)
			{
				// advance one step
				pos2D[i][j] += vel2D[i][j];
				// if hit a wall, reverse direction
				if(pos2D[i][j] < min2D[j] || pos2D[i][j] > max2D[j])
					vel2D[i][j] = -vel2D[i][j];
			}
		}
		
		// update 3D postions
		for(int i = 0; i < numPrimitives3D; i++)
		{
			for(int j = 0; j <= Z; j++)
			{
				// advance one step
				pos3D[i][j] += vel3D[i][j];
				// if hit a wall, reverse direction
				// walls are sides of the unit cube
				if(pos3D[i][j] < 0. || pos3D[i][j] > 1.)
					vel3D[i][j] = -vel3D[i][j];
			}
		}
		
		sleepProcess(stepTime);
	}
}

void GUIDemo::redrawGraphics2D()
{
	const double l = 100.; // length of line, size of rectangle
	vector<double> x, y;
	
	// draw image
	drawRGBImage(imgWidth, imgHeight, rgbImage,  cdl::guitypes::NONE);
	
	// draw moving thingies
	drawPoint2D(pos2D[0][X], pos2D[0][Y],  255, 255, 0,  cdl::guitypes::FAT);
	drawLine2D(pos2D[1][X], pos2D[1][Y],  pos2D[1][X] + l, pos2D[1][Y] + l,
		0, 255, 0,  cdl::guitypes::FAT);
	drawRect2D(pos2D[2][X], pos2D[2][Y],  pos2D[2][X] + l, pos2D[2][Y] + l,
		255, 0, 0,  cdl::guitypes::NONE);
	createPolyX(5, x);
	createPolyY(5, y);
	drawPolygon2D(x, y,  255, 0, 255, cdl::guitypes::FILLED);
}

void GUIDemo::redrawGraphics3D()
{
	const double l = 0.1; // length of line

	// draw ground plane
	drawLine3D(0., 0., 0.,  1., 0., 0.,  128, 128, 0,  cdl::guitypes::NONE);
	drawLine3D(1., 0., 0.,  1., 1., 0.,  128, 128, 0,  cdl::guitypes::NONE);
	drawLine3D(1., 1., 0.,  0., 1., 0.,  128, 128, 0,  cdl::guitypes::NONE);
	drawLine3D(0., 1., 0.,  0., 0., 0.,  128, 128, 0,  cdl::guitypes::NONE);
	// and z-axis
	drawLine3D(0., 0., 0.,  0., 0., 1.,  128, 128, 0,  cdl::guitypes::NONE);

	// draw moving thingies
	drawPoint3D(pos3D[0][X], pos3D[0][Y], pos3D[0][Z],
			255, 255, 0,  cdl::guitypes::FAT);
	drawLine3D(pos3D[1][X], pos3D[1][Y], pos3D[1][Z],
			pos3D[1][X] + l, pos3D[1][Y] + l, pos3D[1][Z] + l,
			0, 255, 0,  cdl::guitypes::FAT);
}

void GUIDemo::redrawGraphicsText()
{
	printText("status:\t\t\trunning\n");
	printText("\n");
	printText("--- drawing primitives: -----------------------------------\n");	
	printText("number 2D:\t\t\t%d\n", numPrimitives2D);
	printText("range2D :\t\t\t(%.0f %.0f) to (%.0f %.0f)\n", min2D[X], min2D[Y],
		max2D[X], max2D[Y]);
	printText("number 3D:\t\t\t%d\n", numPrimitives3D);
	printText("range3D :\t\t\t(%.0f %.0f %.0f) to (%.0f %.0f %.0f)\n", 0., 0., 0., 1., 1., 1.);
	printText("\n");
	printText("--- positions 2D: -----------------------------------------\n");
	for(int i = 0; i < numPrimitives2D; i++)
		printText("position %d:\t\t(%.0f %.0f)\n", i, pos2D[i][X], pos2D[i][Y]);
	printText("\n");
	printText("--- positions 3D: -----------------------------------------\n");
	for(int i = 0; i < numPrimitives3D; i++)
		printText("position %d:\t\t(%.3f %.3f %.3f)\n", i, pos3D[i][X], pos3D[i][Y], pos3D[i][Z]);
}

/**
 * Create a black/white checkerboard image of size w*h.
 */
void GUIDemo::createImage(int w, int h, vector<char> &rgb)
{
	int cnt = 0;
	rgb.resize(3*w*h);
	for(int i = 0; i < h; i++)
		for(int j = 0; j < w; j++)
			if((i/40 + j/40)%2 == 0)
			{
				rgb[cnt] = 255;
				cnt++;
				rgb[cnt] = 255;
				cnt++;
				rgb[cnt] = 255;
				cnt++;
			}
			else
			{
				rgb[cnt] = 0;
				cnt++;
				rgb[cnt] = 0;
				cnt++;
				rgb[cnt] = 0;
				cnt++;
			}
}

/**
 * Create the x co-ordinate values for an n-sided polygon, already adding
 * the position offset for animation.
 */
void GUIDemo::createPolyX(int n, vector<double> &x)
{
	const double rad = 50.;
	x.resize(n);
	for(int i = 0; i < n; i++)
		x[i] = pos2D[3][X] + rad*cos(i*2*M_PI/n);
}

/**
 * Create the y co-ordinate values for an n-sided polygon, already adding
 * the position offset for animation.
 */
void GUIDemo::createPolyY(int n, vector<double> &y)
{
	const double rad = 50.;
	y.resize(n);
	for(int i = 0; i < n; i++)
		y[i] = pos2D[3][Y] + rad*sin(i*2*M_PI/n);
}

} //namespace cast
