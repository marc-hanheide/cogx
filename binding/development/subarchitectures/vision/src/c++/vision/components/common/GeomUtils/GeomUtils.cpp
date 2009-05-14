/** @file GeomUtils.cpp
 *  @brief Useful data structures and functions for geometrical manipulation.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include "GeomUtils.hpp"
#include <iostream>

using namespace std;

void Geom::convert2Points(CvRect rect, std::vector<Vector2D> &pts)
{
    pts.push_back( Vector2D(rect.x, rect.y) );
    pts.push_back( Vector2D(rect.x, rect.y+rect.height) );
    pts.push_back( Vector2D(rect.x+rect.width, rect.y+rect.height) );
    pts.push_back( Vector2D(rect.x+rect.width, rect.y) );
}

void Geom::show(CvRect rect)
{
    cout << "Geom::show rect: " << rect.x << ", " << rect.y
	 << ", " << rect.width << ", " << rect.height << endl;
}


void Geom::scale(CvRect &rect, float scaleFactor)
{
    float center_x = (float)rect.x + ((float)rect.width/2.0);
    float center_y = (float)rect.y + ((float)rect.height/2.0);
    float new_w = (float)rect.width * scaleFactor;
    float new_h = (float)rect.height * scaleFactor;
    rect.x = (int)(center_x - new_w/2.0);
    rect.y = (int)(center_y - new_h/2.0);
    rect.width = (int)new_w;
    rect.height = (int)new_h;
}
