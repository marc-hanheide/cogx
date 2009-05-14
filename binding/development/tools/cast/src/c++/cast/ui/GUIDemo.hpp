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

#ifndef GUI_DEMO_H
#define GUI_DEMO_H

#include <cast/core/CASTCore.hpp>
#include <cast/architecture/UnmanagedProcess.hpp>


namespace cast {

  class GUIDemo : public UnmanagedProcess
  {
  private:
    static const int X = 0;
    static const int Y = 1;
    static const int Z = 2;

    // number of moving drawing primitives
    static const int numPrimitives2D = 4;
    static const int numPrimitives3D = 2;

    // maximum velocity of a 2D drawing primitive (actually sqrt(2)..)
    static const double velMax2D;

    // maximum velocity of a 3D drawing primitive (actually sqrt(2)..)
    static const double velMax3D;

    // time to wait between steps [ms]
    // 50 ms corresponds to a frame rate of 20 Hz
    static const int stepTime = 50;

    // size of the field where drawing primitives move around
    static const double min2D[2];
    static const double max2D[2];

    // positions of 2D drawing primitives
    double pos2D[numPrimitives2D][2];
    // velocities of 2D drawing primitives
    double vel2D[numPrimitives2D][2];

    // positions of 3D drawing primitives
    double pos3D[numPrimitives3D][3];
    // velocities of 3D drawing primitives
    double vel3D[numPrimitives3D][3];

    static const int imgWidth = 320;
    static const int imgHeight = 240;
    std::vector<char> rgbImage;

    void createImage(int w, int h, std::vector<char> &rgb);
    void createPolyX(int n, std::vector<double> &x);
    void createPolyY(int n, std::vector<double> &y);

  public:
    GUIDemo(const std::string &_id);

  protected:
    virtual void configure(std::map<std::string,std::string> & _config);
    virtual void runComponent();
    virtual void redrawGraphics2D();
    virtual void redrawGraphics3D();
    virtual void redrawGraphicsText();
  };

} //namespace cast

#endif

