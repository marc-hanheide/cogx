/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) <year>  <name of author>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef V4R_LINE3DBASE_H
#define V4R_LINE3DBASE_H

#include <opencv/cv.h>
#include <cstdio>
#include <v4r/cvextensions/operator_cv.h>
#include <v4r/geometry/linebase.h>
#include <v4r/cvextensions/print_cv.h>

namespace V4R
{

#ifndef V4R_LINESEGMENT3D
#define V4R_LINESEGMENT3D
template <typename T>
class LineSegment3D
{
protected:
    cv::Point3_<T> p1_;
    cv::Point3_<T> p2_;
public:
    LineSegment3D() {};
    LineSegment3D ( const LineSegment3D &ls ) : p1_ ( ls.p1 ), p2_ ( ls.p2 ) {};
    template <typename T2>
    LineSegment3D ( const cv::Point3_<T2> &pt1, const cv::Point3_<T2> &pt2 )
    {
        p1_.x = pt1.x, p1_.y = pt1.y, p1_.z = pt1.z;
        p2_.x = pt2.x, p2_.y = pt2.y, p2_.z = pt2.z;
    };
    template <typename T2>
    LineSegment3D ( const cv::Vec<T2,3> &pt1, const cv::Vec<T2,3> &pt2 )
    {
        p1_.x = pt1[0], p1_.y = pt1[1], p1_.z = pt1[2];
        p2_.x = pt2[0], p2_.y = pt2[1], p2_.z = pt2[2];
    };
    LineSegment3D ( T x1, T y1, T z1, T x2, T y2, T z2 )
    {
        p1_.x = x1, p1_.y = y1, p1_.z = z1;
        p2_.x = x2, p2_.y = y2, p2_.z = z2;
    };
    void clear()
    {
        p1_.x = 0, p1_.y = 0, p1_.z = 0;
        p2_.x = 0, p2_.y = 0, p2_.z = 0;
    }
    /// @return p1_.x
    T &x1()
    {
        return p1_.x;
    }
    /// @return p1_.x
    const T &x1() const
    {
        return p1_.x;
    }
    /// @return p1_.y
    T &y1()
    {
        return p1_.y;
    }
    /// @return p1_.y
    const T &y1() const
    {
        return p1_.y;
    }
    /// @return p1_.z
    T &z1()
    {
        return p1_.z;
    }
    /// @return p1_.z
    const T &z1() const
    {
        return p1_.z;
    }
    /// @return p2_.x
    T &x2()
    {
        return p2_.x;
    }
    /// @return p2_.x
    const T &x2() const
    {
        return p2_.x;
    }
    /// @return p2_.y
    T &y2()
    {
        return p2_.y;
    }
    /// @return p2_.y
    const T &y2() const
    {
        return p2_.y;
    }
    /// @return p2_.z
    T &z2()
    {
        return p2_.z;
    }
    /// @return p2_.z
    const T &z2() const
    {
        return p2_.z;
    }
    /// @return p1_ startpoint
    cv::Point3_<T> &p1()
    {
        return p1_;
    }
    /// @return p1_ startpoint
    const cv::Point3_<T> &p1() const
    {
        return p1_;
    }
    /// @return p1_ startpoint
    cv::Vec<T,3> &v1()
    {
        return ( cv::Vec<T,3> & ) p1_;
    }
    /// @return p1_ startpoint
    const cv::Vec<T,3> &v1() const
    {
        return ( cv::Vec<T,3> & ) p1_;
    }
    /// @return p2_ endpoint
    cv::Point3_<T> &p2()
    {
        return ( cv::Point3_<T> & ) p2_;
    }
    /// @return p2_ endpoint
    const cv::Point3_<T> &p2() const
    {
        return ( cv::Point3_<T> & ) p2_;
    }
    /// @return p2_ endpoint
    cv::Vec<T,3> &v2()
    {
        return ( cv::Vec<T,3> & ) p2_;
    }
    /// @return p2_ endpoint
    const cv::Vec<T,3> &v2() const
    {
        return ( cv::Vec<T,3> & ) p2_;
    }
    /// @return direction vectior
    cv::Vec<T,3> direction() const
    {
        return cv::Vec<T,3> ( p2_.x - p1_.x, p2_.y - p1_.y, p2_.z - p1_.z );
    }
    /// @return direction vectior
    cv::Vec<T,3> v() const
    {
        return direction();
    }
    /// @return squared length
    double length2() const
    {
        cv::Vec<T,3> v = direction();
        return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
    }
    /// @return length
    double length() const
    {
        return sqrt ( length2() );
    }
    /// @return unit vector
    cv::Vec<T,3> unit() const
    {
        cv::Vec<T,3> v = direction()/length();
        return v;
    }
    /// @return distance to a line
    double distanceToLine ( const cv::Point3_<T> &p0 ) const
    {
        // http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
        cv::Vec<T,3> va = p0 - p1_, vb = p0 - p2_, vc = p2_ - p1_;
        double a = cv::norm(va.cross(vb));
        double b = cv::norm(vc);
        return a/b;
    }
    /// @return returns the instersection of two 3D lines or estimates the most likly one
    template <typename T2>
    cv::Point3_<T> intersection(const LineSegment3D<T2> &lb)
    {
        return intersection(p1(), p2(), lb.p1(), lb.p2());
    };

    /// @return returns the instersection of two 3D lines or estimates the most likly one
    static const cv::Point3_<T> intersection(const cv::Point3_<T> A1, const cv::Point3_<T> A2, const cv::Point3_<T> B1, const cv::Point3_<T> B2)
    {
        using namespace cv;
        struct plane {
            Vec<T,3> n;
            Point3_<T> p;
        };
	/// Plane between lines
        Vec<T,3> d1 = A2-A1, d2 = B2-B1;
        Vec<T,3> b = B1 - A1;
        Vec<T,3> n1 = d1.cross(b), n2 = d2.cross(b);
        plane pl;
        pl.n = (n2 + n1) * (1.0 / norm(n2 + n1)); /// plane normal
        pl.p = A1;

        /// Translation to the plane
        Mat_<T> Tr = Mat_<T>::eye(4,4);
        Tr(0,3) = -pl.p.x, Tr(1,3) = -pl.p.y, Tr(2,3) = -pl.p.z;
        Mat_<T> invTr = Mat_<T>::eye(4,4);
        invTr(0,3) = pl.p.x, invTr(1,3) = pl.p.y, invTr(2,3) = pl.p.z;


        /// Roation to the plane
        cv::Vec<T, 3> u(pl.n[0], pl.n[1], pl.n[2]);
        double du = sqrt(pl.n[0]*pl.n[0]+pl.n[1]*pl.n[1]);
        double ct = u[0]/du, st = u[1]/du;
        Mat_<T> Rxz = Mat_<T>::eye(4,4);
        Rxz(0,0) =  ct, Rxz(0,1) =  st;
        Rxz(1,0) = -st, Rxz(1,1) =  ct;
        Mat_<T> invRxz = Rxz.t();

        /// Roation to the plane
        double cb = u[2], sb = du;
        Mat_<T> Rxz2z = Mat_<T>::eye(4,4);
        Rxz2z(0,0) =  cb, Rxz2z(0,2) = -sb;
        Rxz2z(2,0) =  sb, Rxz2z(2,2) =  cb;
        Mat_<T> invRxz2z = Rxz2z.t();

	/// Combining rotations and translations
        Mat_<T> R = Rxz2z * Rxz * Tr;
        Mat_<T> invR =  invTr * invRxz * invRxz2z;
	
	/// Points on plane
        cv::Vec<T,3> a1 = R * A1;
        cv::Vec<T,3> a2 = R * A2;
        cv::Vec<T,3> b1 = R * B1;
        cv::Vec<T,3> b2 = R * B2;
        a1[2] = a2[2] = 1;  /// homogeneous entry
        cv::Vec<T,3> l1 = a1.cross( a2);  /// line on plane
        l1 = l1 * (1.0/sqrt(l1[0]*l1[0]+l1[1]*l1[1])); /// normalize line
        b1[2] = b2[2] = 1;  /// homogeneous entry
        cv::Vec<T,3> l2 = b1.cross( b2);  /// line on plane
        l2 = l2 * (1.0/sqrt(l2[0]*l2[0]+l2[1]*l2[1])); /// normalize line
        cv::Point3_<T> p3D = l1.cross(l2); /// Intersection on plane
        p3D.x/=p3D.z, p3D.y/=p3D.z;
        p3D.z = 0;
	
	/// Rotation back  into world space
        cv::Point3_<T> pi = invR * p3D;

        return pi;
    }
    /**
     * produces a human readable string
     * @return string p1 = [ %-8.3f, %-8.3f, %-8.3f]; p2 = [ %-8.3f, %-8.3f, %-8.3f];"
     **/
    std::string human_readable() const
    {
        char pText[0xFF];
        sprintf ( pText, "p1 = [ %-8.3f, %-8.3f, %-8.3f]; p2 = [ %-8.3f, %-8.3f, %-8.3f];",
                  x1(), y1(), z1(), x2(), y2(), z2() );
        return pText;
    }

private:
};
typedef LineSegment3D<float> LineSegment3Df;
typedef LineSegment3D<double> LineSegment3Dd;
#endif //V4R_LINESEGMENT3D

};
#endif // V4R_LINE3DBASE_H
