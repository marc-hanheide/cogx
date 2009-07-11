//
// = FILENAME
//    ObjGridLineRayTracer.hh
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 1997 Patric Jensfelt
//                  1999 Patric Jensfelt
//                  2005 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef Cure_ObjGridLineRayTracer_hh
#define Cure_ObjGridLineRayTracer_hh

#include "Navigation/LocalGridMap.hh"
#include "Utils/HelpFunctions.hh"
#include "SensorData/SICKScan.hh"
#include "Transformation/Pose3D.hh"
#include <math.h>
#include <algorithm>

namespace Cure
  {

  /**
   * Class that helps you update a gridmap along a ray
   *
   * @author Patric Jensfelt
   * @see
   */
  using namespace Cure;

  template <class MAPDATA>
  class ObjGridLineRayTracer
    {
    public:
      /**
       * Constructor
       *
       * @param m reference to a LocalGridMap where we want to perform ray
       * tracing
       */
      ObjGridLineRayTracer(XLocalGridMap<MAPDATA> &m);

      /**
       * Initiate the starting point for the line
       * 
       * @param x x-coordinate for start of the ray
       * @param y y-coordinate for start of the ray
       * @return 0 if inside map, else error code
       */
      int setStart(double x, double y, double angle,bool isWorldCoord = true);


      /**
       * Use this function to access the current cell data in the ray
       * tracing process.
       *
       * @return reference to cell data in the underlying LocalGridMap
       * or error cell LocalGridMap:getMapdataError() if you are outside
       * for example. You check for thsi by comparing the address of the
       * cell
       */
      MAPDATA& data();

      /**
       * Get the coordinates (in cells) of the current ray trace position
       * @return 0 if inside map, else error code
       */
      int getGridCoords(int& x, int& y) const;

      /**
       * Get the global coordinates (in mm) of the current cursor position
       * @return 0 if inside map, otherwise error code
       */
      int getWorldCoords(double& x, double& y) const;

      /**
       * @return the distance from the start point to the current
       * ray trace position in m
       */
      double getDistToStart() const;

      /**
       * Use this function to step to the next cell in th eray trace process
       * @eturns 0 if inside map, otherwise error code
       */
      int stepRay();

      /**
       * @retur true if ray is outside
       */
      int isOutside() const;

      /**
       * Add the data from a laser scan to the map
       * 
       * @param scan the scan data
       * @param sp laser scanner sensor pose at time of the scan
       * @param maxRange the max range to use when updating the map [m]
       */
      void addScan(SICKScan &scan, const Pose3D &sp, double maxRange);

      /**
       * Accumulate the data from a laser scan to the map
       * 
       * @param scan the scan data
       * @param sp laser scanner sensor pose at time of the scan
       * @param maxRange the max range to use when updating the map [m]
       */
      double IntegrateProb(int sampleX,int sampleY, double maxRange, double StartAngle, double EndAngle,
                           double** pdf, bool** coveragemap,Cure::XLocalGridMap<double>* m_lgm =0);

      bool isInsideTriangle(int* triangle, int Px, int Py);
      int max(int a, int b);
      int min(int a, int b);

    private:

      /// A reference to the map object
      XLocalGridMap<MAPDATA> &m_Map;

      /// The size of the map object
      long m_Size;

      /// The size of the cells in the map
      double m_CellSize;

      /// Map coordinates as floating point values
      float m_Xf, m_Yf;

      /// Map coordinates as integer values
      int m_Xi, m_Yi;

      /// Map coordinates of the start point as floating point values
      float m_Xs, m_Ys;

      /// The direction of the ray [rad]
      double m_Angle;

      /// The direction of the ray as k in y=kx+m or x=km+y
      double m_K;

      /// Pointer to the ray tracing function best suited for the current
      /// direction
      int (ObjGridLineRayTracer<MAPDATA>::* drawLineFcn)(void);

      /// Draw a line between -45 and +45 degs or 135 and 225 degs
      int drawLineXp(void);

      /// Draw a line between 135 and 225 degs
      int drawLineXn(void);

      /// Draw a line between 45 and 135 degs
      int drawLineYp(void);

      /// Draw a line between -135 and -45 degs
      int drawLineYn(void);

    }
  ; // class ObjGridLineRayTracer //

  template <class MAPDATA>
  ObjGridLineRayTracer<MAPDATA>::ObjGridLineRayTracer(XLocalGridMap<MAPDATA> &m)
      :m_Map(m)
  {
    m_Size = m_Map.getSize();
    m_CellSize = m_Map.getCellSize();
  }

  template <class MAPDATA>
  int
  ObjGridLineRayTracer<MAPDATA>::setStart(double x, double y, double angle, bool isWorldCoord)
  {
    // Set the map coordinates
    if ( isWorldCoord){
    m_Xf = (x - m_Map.getCentXW()) / m_CellSize;
    m_Yf = (y - m_Map.getCentYW()) / m_CellSize;
    m_Xi = (m_Xf<0 ? int(m_Xf-0.5) : int(m_Xf+0.5));
    m_Yi = (m_Yf<0 ? int(m_Yf-0.5) : int(m_Yf+0.5));
    }
    else
    {
    	m_Xi = x; m_Yi= y;
    	m_Xf = (float)x; m_Yf = (float)y;
    }
    	
    m_Xs = m_Xf;
    m_Ys = m_Yf;	
    	
    m_Angle = Cure::HelpFunctions::mod2PI(angle);

    if ((0<=m_Angle) && (m_Angle<M_PI/4.0))
      {
        drawLineFcn = &(ObjGridLineRayTracer::drawLineXp);
        m_K = tan(m_Angle);
      }
    else if ((M_PI/4.0<=m_Angle) && (m_Angle<3.0*M_PI/4.0))
      {
        drawLineFcn = &(ObjGridLineRayTracer::drawLineYp);
        m_K = 1/tan(m_Angle);
      }
    else if ((3.0*M_PI/4.0<=m_Angle) && (m_Angle<5.0*M_PI/4.0))
      {
        drawLineFcn = &(ObjGridLineRayTracer::drawLineXn);
        m_K = tan(m_Angle);
      }
    else if ((5.0*M_PI/4.0<=m_Angle) && (m_Angle<7.0*M_PI/4.0))
      {
        drawLineFcn = &(ObjGridLineRayTracer::drawLineYn);
        m_K = 1/tan(m_Angle);
      }
    else
      {
        drawLineFcn = &(ObjGridLineRayTracer::drawLineXp);
        m_K = tan(m_Angle);
      }

    return 0;
  }

  template <class MAPDATA>
  MAPDATA&
  ObjGridLineRayTracer<MAPDATA>::data()
  {
    return m_Map(m_Xi, m_Yi);
  }

  template <class MAPDATA>
  int
  ObjGridLineRayTracer<MAPDATA>::getGridCoords(int& x, int& y) const
    {
      x = m_Xi;
      y = m_Yi;

      // Don't move outside the map
      if ((m_Xi < -m_Size) || (m_Size < m_Xi) ||
          (m_Yi < -m_Size) || (m_Size < m_Yi))
        {
          return 1;
        }
      else
        {
          return 0;
        }
    }

  template <class MAPDATA>
  int
  ObjGridLineRayTracer<MAPDATA>::getWorldCoords(double& x, double& y) const
    {
      x = long(m_Map.getCentXW() + m_Xf * m_CellSize);
      y = long(m_Map.getCentYW() + m_Yf * m_CellSize);

      return 0;
    }

  template <class MAPDATA>
  double
  ObjGridLineRayTracer<MAPDATA>::getDistToStart() const
    {
      return hypot(m_Yf-m_Ys, m_Xf-m_Xs) * m_CellSize;
    }

  template <class MAPDATA>
  int
  ObjGridLineRayTracer<MAPDATA>::isOutside() const
    {
      if ((m_Xi < -m_Size) || (m_Size < m_Xi) ||
          (m_Yi < -m_Size) || (m_Size < m_Yi))
        return true;
      else
        return false;
    }

  template <class MAPDATA>
  int
  ObjGridLineRayTracer<MAPDATA>::stepRay()
  {
    return (this->*drawLineFcn)();
  }

  template <class MAPDATA>
  int
  ObjGridLineRayTracer<MAPDATA>::drawLineXp()
  {
    m_Xi++;
    m_Xf++;
    m_Yf += m_K;
    m_Yi = (0<m_Yf ? int(m_Yf+0.5) : int(m_Yf-0.5));

    // Don't move outside the map
    if ((m_Xi < -m_Size) || (m_Size < m_Xi) ||
        (m_Yi < -m_Size) || (m_Size < m_Yi))
      return false;
    else
      return true;
  }

  template <class MAPDATA>
  int
  ObjGridLineRayTracer<MAPDATA>::drawLineXn()
  {
    m_Xi--;
    m_Xf--;
    m_Yf -= m_K;
    m_Yi = (0<m_Yf ? int(m_Yf+0.5) : int(m_Yf-0.5));

    // Don't move outside the map
    if ((m_Xi < -m_Size) || (m_Size < m_Xi) ||
        (m_Yi < -m_Size) || (m_Size < m_Yi))
      return false;
    else
      return true;
  }

  template <class MAPDATA>
  int
  ObjGridLineRayTracer<MAPDATA>::drawLineYp()
  {
    m_Yi++;
    m_Yf++;
    m_Xf += m_K;
    m_Xi = (0<m_Xf ? int(m_Xf+0.5) : int(m_Xf-0.5));

    // Don't move outside the map
    if ((m_Xi < -m_Size) || (m_Size < m_Xi) ||
        (m_Yi < -m_Size) || (m_Size < m_Yi))
      return false;
    else
      return true;
  }

  template <class MAPDATA>
  int
  ObjGridLineRayTracer<MAPDATA>::drawLineYn()
  {
    m_Yi--;
    m_Yf--;
    m_Xf -= m_K;
    m_Xi = (0<m_Xf ? int(m_Xf+0.5) : int(m_Xf-0.5));

    // Don't move outside the map
    if ((m_Xi < -m_Size) || (m_Size < m_Xi) ||
        (m_Yi < -m_Size) || (m_Size < m_Yi))
      return false;
    else
      return true;
  }

  template <class MAPDATA>
  void ObjGridLineRayTracer<MAPDATA>::addScan(Cure::SICKScan &scan,
      const Cure::Pose3D &sp,
      double maxRange)
  {
    for (int i = 0; i < scan.getNPts(); i++)
      {
        setStart(sp.getX(),
                 sp.getY(),
                 sp.getTheta() +
                 scan.getStartAngle() + scan.getAngleStep() * i);

        while (!isOutside() &&
               getDistToStart() < scan.getRange(i) &&
               getDistToStart() < maxRange)
          {
            if (data() != 60)
              data() = 0; // free space
            stepRay();
          }
        if (!isOutside() && getDistToStart() < maxRange)
          {
            data() = 1; //obstacle
          }
      }
  }


  template <class MAPDATA>
  double ObjGridLineRayTracer<MAPDATA>::IntegrateProb(int sampleX,int sampleY,
      double maxRange,
      double StartAngle,
      double EndAngle,
      double** pdf,
      bool** coveragemap,
      Cure::XLocalGridMap<double>* m_lgm)
  {
    double rad = M_PI/180;
    double acc = 0;
    printf("in AccumulateProb\n");
    int Ax,Ay,Bx,By,Cx,Cy;
    
    Ax = sampleX;
    Ay = sampleY;
    
    setStart(sampleX,
             sampleY,
             StartAngle,false);
    while (!isOutside() && getDistToStart() < maxRange)
      {
        stepRay();
      }
    Bx = m_Xi;
    By = m_Yi;

    setStart(sampleX,
             sampleY,
             EndAngle,false);
    while (!isOutside() && getDistToStart() < maxRange)
      {
        stepRay();
      }
    Cx = m_Xi;
    Cy = m_Yi;
    printf("Got triangle coordinates: A:(%i,%i),B:(%i,%i),C:(%i,%i) \n", Ax,Ay,Bx,By,Cx,Cy);
    int triangle[6] = {Ax, Ay, Bx, By, Cx, Cy};
    int maxy, maxx, miny, minx;
    maxy = max(max(Ay,By),Cy);
    maxx = max(max(Ax,Bx),Cy);
    miny = min(min(Ay,By),Cy);
    minx = min(min(Ax,Bx),Cx);
    printf("Rectangle coordinates: Min: (%i,%i), Max:(%i,%i)\n",minx,miny,maxx, maxy);
    // Integrate over triangle corresponding to view cone
    for (int x=minx; x < maxx ; x++) // rectangle bounding triangle
      {
        for (int y=miny; y < maxy; y++)
          {
          	(*m_lgm)(x, y) = 200;
          }
      }
      printf("Integrated probabilities");
    return acc;
  }

  template <class MAPDATA>
  bool ObjGridLineRayTracer<MAPDATA>::isInsideTriangle(int* triangle, int Px, int Py)
  {

    return true;
  }
  template <class MAPDATA>
  int ObjGridLineRayTracer<MAPDATA>::max(int a, int b)
  {
    return ((a > b) ? (a) : (b) );
  }
  template <class MAPDATA>
  int ObjGridLineRayTracer<MAPDATA>::min (int a, int b)
  {
    return ((a < b) ? (a) : (b) );
  }
} // namespace Cure

#endif // ISR_GridLineRayTracer_h
