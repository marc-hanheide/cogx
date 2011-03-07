//
// = Filename
//    HSSGridMap2D
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = END<CODE>
//
// = COPYRIGHT
//    Copyright (c) 1997 Patric Jensfelt (ISR)
//                  2005 Patric Jensfelt (Cure)
//                  2010 Patric Jensfelt (HSS)
//
/* ---------------------------------------------------------------------- */

#ifndef HSS_GridMap2D_hh
#define HSS_GridMap2D_hh

#include "HSSGridContainer2D.hh"
#include "HSSGridMapFunctor.hh"
#include "HSSutils.hh"

#include "Eigen/Core"

#ifndef DEPEND
#include <string.h>  // memcpy,memmove
#include <iostream>
#include <cstdlib>   // abs
#include <cmath>     // hypot
#endif

namespace HSS {

/**
 *  The GridMap2D is a class that defines a grid map structure.  The
 *  map is robot centered, well it doesn't have to be a robot but it
 *  is centered around something. You can ask the map to move and it
 *  will do so. Moving causes the information that was found in the
 *  part that scrolls "out of sight" to be lost.  The GridMap2D is
 *  derived from GridContainer2D but defines its own access
 *  functions. 
 *
 *  The size of the GridMap2D is defined as the number of cells from
 *  the center, i.e., where the robot is to the end of the map along
 *  one of the axis. All values as in meters. Using the function
 *  moveCenterTo(x,y) you can tell the map to center around another
 *  point.
 *
 *  // Creates a grid map with 100 cells from center to the end of the map
 *  // with 0.02 by 0.02 m large cells. Each new cell is given the value 0
 *  HSS::CharGridMapFunctor f;
 *  HSS::GridMap2D<char> lm(10, 0.020, f);
 *  
 *  // Let cell (10,4) i.e. a cell 0.16 m to the right and 0.08 m up be free
 *  lm(8,4) = f.getFreeValue();
 * 
 *  // Let cell (0,0) i.e. the the center be occupied
 *  lm(0,0) = f.getOccupiedValue();
 * 
 *  // Display the map on standard output
 *  std::cout << "Before moving\n";
 *  lm.print(std::cout);
 *  
 *  // Move center of the map to to (0.16,0.08) in world coordinates [m]
 *  lm.moveCenterTo(0.16,0.08);
 *  
 *  // Display the map on standard output
 *  std::cout << "After moving\n";
 *  lm.print(std::cout);
 *
 * @author Patric Jensfelt
 * @see GridContainer2D
 */
template <class MAPDATA>
class GridMap2D : public GridContainer2D<MAPDATA> 
{
public:
  /**
   * Constructs a grid map. The size specified is the number of cells
   * form the center to the side (quadratic map). The cells are
   * assumed to be quadratic with side cellSize m. The position of the
   * center position is specified in m, if you don't give a position
   * it will be set to the origin.
   *
   * @param f functor that defines the unknown, free and occupied 
   *          state for the map
   * @param size number of cells from center to side
   * @param cellSize size of the side fo a cell
   * @param xc x-coordinate in world coordinates of the center of the map
   * @param yc y-coordinate in world coordinates of the center of the map
   */
  GridMap2D(GridMapFunctor<MAPDATA> &f,
            int size, double cellSize,
            double xc = 0, double yc = 0);

  /**
   * Copy constructor
   */
  GridMap2D(GridMap2D<MAPDATA> &lgm);
       
  /**
   * Destructor
   */
  virtual ~GridMap2D();

  /**
   * Operator used to set this map equal another map
   */
  void operator=(GridMap2D<MAPDATA> &lgm);

  /** 
   * @param x coord (in cell indices from center where (0,0) is center)
   * @param y coord (in cell indices from center where (0,0) is center)
   * @return the value at position (x,y) to be edited if desired
   */
  const MAPDATA& operator() (int x, int y) const;
  MAPDATA& operator () (int x, int y);
 
  /**
   * @param index index in grid map
   * @returns the value at position index. Position 0 is the position 
   * in the lower left corner
   */
  const MAPDATA& operator [] (long index)const;
  MAPDATA& operator [] (long index);

  /**
   * @return the size of the map, defined as the number of cells
   * from the center of the map to the end of the map.
   */
  int getSize() const { return m_Size; }

  /** @return the size of each cell in m */
  double getCellSize() const { return m_CellSize; }

  /** @return the number of cells in the map */
  long getNumCells() const { return m_NumCells; }

  /**
   * Use this function to transform from indices to world coordinates
   *
   * @param i index in x-direction
   * @param j index in y-direction
   * @param xW world x-coordinate corresponding to the point
   * @param yW world y-coordinate corresponding to the point
   *
   * @return 0 if inside map, else not 0
   */
  int index2WorldCoords(int i, int j, double &xW, double &yW) const
  { 
    xW = m_XCentW + i * m_CellSize; 
    yW = m_YCentW + j * m_CellSize; 
    
    if (i >= -m_Size && i <= m_Size && j >= -m_Size && j <= m_Size) {
      return 0;
    } else {
      return 1;
    }
  }

  /**
   * Use this function to transform from indices to world coordinates
   *
   * @param xW world x-coordinate corresponding to the point
   * @param yW world y-coordinate corresponding to the point
   * @param i index in x-direction
   * @param j index in y-direction
   *
   * @return 0 if inside map, else not 0
   */
  int worldCoords2Index(double xW, double yW, int &i, int &j) const
  { 
    i = (int((xW - m_XCentW)/(0.5*m_CellSize)) + (xW >= m_XCentW ? 1: -1)) / 2;
    j = (int((yW - m_YCentW)/(0.5*m_CellSize)) + (yW >= m_YCentW ? 1: -1)) / 2;

    if (i >= -m_Size && i <= m_Size && j >= -m_Size && j <= m_Size) {
      return 0;
    } else {
      return 1;
    }
  }

  /** 
   * @return the value that is given to a cell which status is completely
   * unknown 
   */
  MAPDATA getUnknownValue() const { return m_Functor.getUnknownValue(); }

  /** @return the x-coordinate of the center position in world coordinates */
  double getCentXW() const { return m_XCentW; }

  /** @return the y-coordinate of the center position in world coordinates */
  double getCentYW() const { return m_YCentW; }

  /**
   * Set all values within the circle to a certain value.
   *
   * @param xC x-coordinate of the center of the circle
   * @param yC y-coordinate of the center of the circle
   * @param r radius of the circle
   * @param value the value to assign to the cells inside the circle
   */
  void setValueInsideCircle(double xC, double yC, double r, MAPDATA value);
  
  /**
   * Use this function to check if a circular region is free from
   * obstacles.
   *
   * @param xW coordinate in world coordinates for center of region [m]
   * @param yW coordinate in world coordinates for center of region [m]
   * @param rad radius of region to check [m]
   *
   * @return true if the circular region is free from obstacles.
   */
  virtual bool isCircleObstacleFree(double xW, double yW, double rad);

  /**
   * Use this function to check if a rectangular region is free from
   * obstacles. The region is defined by a center line connecting
   * (x1W,y1W) and (x2W,y2W) being width wide.
   *
   * @param x1W coordinate in world coordinates for start of region [m]
   * @param y1W coordinate in world coordinates for start of region [m]
   * @param x2W coordinate in world coordinates for end of region [m]
   * @param y2W coordinate in world coordinates for end of region [m]
   * @param width of the rectangular region to check [m]
   *
   * @return true if the circular region is free from obstacles.
   */
  virtual bool isRectangleObstacleFree(double x1W, double y1W, 
                                       double x2W, double y2W, double width);

  /** 
   * Print the local map on the specified ostream 
   * 
   * @param os stream to print on
   */
  virtual void print(std::ostream& os);

  /** 
   * Move the center of the map to a particaluar position in world
   * coord, this means that the map must be updated so that the robot
   * is in the middle still. Information that was further behind the
   * robo than the distance the robot moved will no longer be in the
   * map, new areas that come into view will be given
   * m_Functor.getUnknownvalue(). If we move more than cellSize * size all
   * cells will be m_Functor.getUnknownvalue().
   */
  int moveCenterTo(double x, double y, bool update = true);
  int moveCenterTo(const Eigen::Vector3d &p, bool update = true)
  { return moveCenterTo(p[0], p[1], update); }

  /** 
   * Clear the whole grid map.
   */
  virtual void clearMap();

private:

  /**
   * Called by cnstructors to do common setup stuff
   */
  void constructorInit();

private:

  /// Functor to give values for unknown, free and occupied
  GridMapFunctor<MAPDATA> &m_Functor;

  /** 
   * Number of cells in the local map from center to the side. The
   * size is defined fr the part of the localmap that is "visible" to
   * the user.
   */
  int m_Size;          

  /** The side of each cell in m */
  double m_CellSize;    
  
  /** Length of the side of the local map */
  int m_SideLength;  

  /** Number of cells in the map (i.e. the active part of the whole map) */
  long m_NumCells;

  /** 
   * X-coordinate of the center of the grid map in the underlying
   * GridContainer2D [cells].
   */
  int m_XCentC;   

  /** 
   * X-coordinate of the center of the grid map in the underlying
   * GridContainer2D [cells].
   */
  int m_YCentC;   

  /** X-coordinate of the center in the world frame [m] */
  double m_XCentW;

  /** Y-coordinate of the center in the world frame [m] */
  double m_YCentW;

  /** 
   * Lookup table from coordinates in the inner map
   * to indices in the big map.
   */
  GridContainer2D<long> *m_Lookup; 

  /** 
   * Vector that contains m_Functor.getUnknownValue() and is used when
   * new areas are to be set totally unknown. This is done by memcpy
   * to save time.
   */
  MAPDATA *m_ClearVector;

  /** The number of bytes on one row in the map */
  int m_BytesInMap;

  // ==================================================
  // = PRIVATE PROPERTY ACCESSORS
  // ==================================================

}; // class GridMap2D

template <class MAPDATA>
GridMap2D<MAPDATA>::GridMap2D(GridMapFunctor<MAPDATA> &f,
                              int size, double cellSize, 
                              double xc, double yc)
  :GridContainer2D<MAPDATA>(2*size +1,2*size +1),
   m_Functor(f),
   m_Size(size),
   m_CellSize(cellSize),
   m_SideLength(2*size+1),
   m_NumCells((2*size+1)*(2*size+1)),
   m_XCentW(xc),
   m_YCentW(yc)
{
  constructorInit();
  clearMap();
}

template <class MAPDATA>
GridMap2D<MAPDATA>::GridMap2D(GridMap2D<MAPDATA> &lgm)
  :GridContainer2D<MAPDATA>(2*lgm.m_Size+1,2*lgm.m_Size+1),
   m_Functor(lgm.m_Functor),
   m_Size(lgm.m_Size),
   m_CellSize(lgm.m_CellSize),
   m_SideLength(lgm.m_SideLength),
   m_NumCells(lgm.m_NumCells),
   m_XCentW(lgm.m_XCentW),
   m_YCentW(lgm.m_YCentW)
{
  constructorInit();
  (*this) = lgm;
}

template <class MAPDATA>
inline void GridMap2D<MAPDATA>::constructorInit()
{
  m_Lookup = new GridContainer2D<long>(m_SideLength, m_SideLength);

  int i = 0;
  for (int y = -m_Size; y <= m_Size; y++) {
    for (int x = -m_Size; x <= m_Size; x++) {
      (*m_Lookup)[i++] = m_Lookup->getIndex(x,y);
    }
  }

  // Initial position of the origin in cells
  m_XCentC = m_Size;
  m_YCentC = m_Size;

  // Create a vector used for clearing the map
  m_ClearVector = new MAPDATA[m_NumCells];
  m_BytesInMap = m_NumCells * sizeof(MAPDATA);
  for (int ii = 0; ii < m_NumCells; ii++) {
    m_ClearVector[ii] = m_Functor.getUnknownValue();
  }
}

template <class MAPDATA>
GridMap2D<MAPDATA>::~GridMap2D()
{
  delete m_Lookup;
  delete [] m_ClearVector;
}

template <class MAPDATA>
inline void GridMap2D<MAPDATA>::operator=(GridMap2D<MAPDATA> &lgm)
{
  if (getSize() == lgm.getSize() &&
      m_Functor.getUnknownValue() == lgm.m_Functor.getUnknownValue()) {

    m_XCentC = lgm.m_XCentC;
    m_YCentC = lgm.m_YCentC;

    m_XCentW = lgm.m_XCentW;
    m_YCentW = lgm.m_YCentW;

    m_CellSize = lgm.getCellSize();

    for (int i = -m_Size; i <= m_Size; i++) {
      for (int j = -m_Size; j <= m_Size; j++) {
        (*this)(i,j) = lgm(i,j);
      }
    }

    return;
  }

  if (getSize() != lgm.getSize()) {
    std::cerr << "Cannot copy matrix if not of same size\n";
  } else {
    std::cerr << "Cannot copy matrix if unknown value not same\n";
  }
}

template <class MAPDATA>
inline MAPDATA&  GridMap2D<MAPDATA>::operator() (int x, int y)
{
  if ((x < -m_Size) || (x > m_Size)) {
    std::cerr << "GridMap2D::operator ()  x-index out of bounds x=" 
              << x << std::endl;
    return GridContainer2D<MAPDATA>::m_MapdataError;
  }

  if ((y < -m_Size) || (y > m_Size)) {
    std::cerr << "GridMap2D::operator ()  y-index out of bounds y=" 
              << y << std::endl;
    return GridContainer2D<MAPDATA>::m_MapdataError;
  }
  
  return GridContainer2D<MAPDATA>::operator()(m_XCentC + x, m_YCentC + y);
}

template <class MAPDATA>
inline const MAPDATA&  GridMap2D<MAPDATA>::operator() (int x, int y) const
{
  if ((x < -m_Size) || (x > m_Size)) {
    std::cerr << "GridMap2D::operator ()  x-index out of bounds x=" 
              << x << std::endl;
    return GridContainer2D<MAPDATA>::m_MapdataError;
  }

  if ((y < -m_Size) || (y > m_Size)) {
    std::cerr << "GridMap2D::operator ()  y-index out of bounds y=" 
              << y << std::endl;
    return GridContainer2D<MAPDATA>::m_MapdataError;
  }
  
  return GridContainer2D<MAPDATA>::operator()(m_XCentC + x, m_YCentC + y);
}

template <class MAPDATA>
inline MAPDATA& GridMap2D<MAPDATA>::operator[] (long index)
{
  if ((index < 0) || (m_NumCells < index)) {
    std::cerr << "GridMap2D::operator []  index out of bounds:" 
              << index << std::endl;
    return GridContainer2D<MAPDATA>::m_MapdataError;
  }
  return GridContainer2D<MAPDATA>::operator[]((*m_Lookup)[index] + 
                                      m_YCentC * GridContainer2D<MAPDATA>::m_XSize + m_XCentC);
}

template <class MAPDATA>
inline const MAPDATA& GridMap2D<MAPDATA>::operator[] (long index)const 
{
  if ((index < 0) || (m_NumCells < index)) {
    std::cerr << "GridMap2D::operator []  index out of bounds:" 
              << index << std::endl;
    return GridContainer2D<MAPDATA>::m_MapdataError;
  }
  return GridContainer2D<MAPDATA>::operator[]((*m_Lookup)[index] + 
                                      m_YCentC * GridContainer2D<MAPDATA>::m_XSize + m_XCentC);
}

template <class MAPDATA>
inline bool GridMap2D<MAPDATA>::isCircleObstacleFree(double xW,
                                                        double yW,
                                                        double rad)
{
  int xiC, yiC;
  if (worldCoords2Index(xW, yW, xiC, yiC) != 0) {
    std::cerr << "GridMap2D Querying area outside the map (xW="
                 << xW << ", yW=" << yW << ")\n";
    return true;
  }

  double w = rad / m_CellSize;
  int wi = int(w + 0.5);

  for (int x = xiC-wi; x <= xiC+wi; x++) {
    for (int y = yiC-wi; y <= yiC+wi; y++) {
      if (x >= -m_Size && x <= m_Size && y >= -m_Size && y <= m_Size) {
        if (hypot(x-xiC,y-yiC) < w) {
          if ((*this)(x,y) == '1') return false;
        }
      }
    }
  }

  return true;
}

template <class MAPDATA>
inline bool GridMap2D<MAPDATA>::isRectangleObstacleFree(double x1W, 
                                                           double y1W, 
                                                           double x2W, 
                                                           double y2W, 
                                                           double width)
{
  double xC = 0.5 * (x1W + x2W);
  double yC = 0.5 * (y1W + y2W);

  int xiC, yiC;
  if (worldCoords2Index(xC, yC, xiC, yiC) != 0) {
    std::cerr << "GridMap2D Querying area outside the map\n";
    return true;
  }

  // Currently the implementation is highly inefficient as it goes
  // though all cells in a rectacle with the sides parallel to the
  // coordinate axis containing the desired rectangle and checks if
  // the cells are inside or not

  double len = hypot(y2W - y1W, x2W - x1W);
  double dir = atan2(y2W - y1W, x2W - x1W);
  double cosDir = cos(dir);
  double sinDir = sin(dir);

  double halfWidth = 0.5 * width;

  // Calculate the number of cells
  // cos(dir+pi/2) = -sin(dir)
  // sin(dir+pi/2) =  cos(dir)
  int wx = int((fabs((x2W-x1W) * cosDir) + fabs(halfWidth * sinDir)) / 
               m_CellSize + 1);
  int wy = int((fabs((y2W-y1W) * sinDir) + fabs(halfWidth * cosDir)) / 
               m_CellSize + 1);

  double xW, yW;

  for (int x = xiC-wx; x <= xiC+wx; x++) {
    for (int y = yiC-wy; y <= yiC+wy; y++) {
      if (index2WorldCoords(x, y, xW, yW) == 0) {
        
        // Check the distance along the line
        // cos(dir+pi/2) = -sin(dir)
        // sin(dir+pi/2) = cos(dir)
        double d = HSS::distPt2Line(xW,yW,
                                    x1W, y1W,
                                    -sinDir, cosDir);
        if (d >= 0 && d <= len) {
          if (fabs(HSS::distPt2LinePts(x1W, y1W, x2W, y2W, xW, yW)) 
            < halfWidth) {
            if ((*this)(x,y) == '1') return false;
          }
        }
      }
    }
  }

  return true;
}


template <class MAPDATA>
inline void GridMap2D<MAPDATA>::print(std::ostream& os)
{
  for (int y = m_Size; y >= -m_Size; y--) {
    for (int x = -m_Size; x <= m_Size; x++)
      os << (*this)(x,y) << " ";
    os << std::endl;
  }
}

template <class MAPDATA>
inline void GridMap2D<MAPDATA>::clearMap()
{
  std::cerr << "GridMap2D<MAPDATA>::clearMap\n";
  memcpy(&((*this)[0]), m_ClearVector, m_BytesInMap);
}
 

template <class MAPDATA>
int GridMap2D<MAPDATA>::moveCenterTo(double xpos, double ypos, bool update)
{
  int dx = (int) ((xpos - m_XCentW) / m_CellSize);
  int dy = (int) ((ypos - m_YCentW) / m_CellSize);
  
  if (update) {
    // Check if the whole map can be cleared, i.e. the robot has moved very far
    if (( dx <= -m_SideLength ) || ( dx >= m_SideLength ) || 
	( dy <= -m_SideLength ) || ( dy >= m_SideLength )) {

      clearMap();

    } else {

      size_t bytesToMove = sizeof(MAPDATA) * (m_SideLength - abs(dx));
      size_t bytesPerRow = sizeof(MAPDATA) * m_SideLength;
      
      // Move the values from the cells that will not be replaced with
      // unknown values to their new position relative to the center
      if (dy > 0) { 
        if (dx > 0) {
          for(int j = -m_Size + dy; j <= m_Size; j++) {
            memmove(&(*this)(-m_Size, j - dy),
                    &(*this)(-m_Size + dx, j),
                    bytesToMove);
          }
        } else {
          for(int j = -m_Size + dy; j <= m_Size; j++) {
            memmove(&(*this)(-m_Size- dx, j - dy),
                    &(*this)(-m_Size, j),
                    bytesToMove);
          }
        }
      } else {
        if (dx > 0) {
          for(int j = m_Size + dy; j >= -m_Size; j--) {
            memmove(&(*this)(-m_Size, j - dy),
                    &(*this)(-m_Size + dx, j),
                    bytesToMove);
          }
        } else {
          for(int j = m_Size + dy; j >= -m_Size; j--) {
            memmove(&(*this)(-m_Size - dx, j - dy),
                    &(*this)(-m_Size, j),
                    bytesToMove);
          }
        }
      }
      
      // Initialize the "new" parts of the map that has not been seen
      // before
      if (dx > 0) { 
        for (int k = 0; k < dx; k++) {
          for ( int j = -m_Size; j <=m_Size; j++) {
            (*this)(m_Size - k, j) = m_Functor.getUnknownValue();
          }
        }
      } else { 
        for (int k = dx+1; k <= 0; k++) {
          for ( int j = -m_Size; j <=m_Size; j++) {
            (*this)(-m_Size - k, j) = m_Functor.getUnknownValue();
          }
        }
      }
      if (dy > 0) { 
        for (int j = 0; j < dy; j++) {
          memcpy(&(*this)(-m_Size, m_Size - j), m_ClearVector, 
                 bytesPerRow);
        }
      } else {
        for (int j = dy+1; j <= 0; j++) {
          memcpy(&(*this)(-m_Size, -m_Size - j), m_ClearVector, 
                 bytesPerRow);
        }
      }
    }
  }
      
  m_XCentW += dx * m_CellSize;
  m_YCentW += dy * m_CellSize;

  //std::cout << "Map moved to: " << m_XCentW << " , " << m_YCentW <<std::endl;
  
  return true;
}

template <class MAPDATA>
void GridMap2D<MAPDATA>::setValueInsideCircle(double xC, double yC, 
                                              double r, MAPDATA value)
{
  int xi, yi;
  worldCoords2Index(xC, yC, xi, yi);

  int w = int(r / m_CellSize + 0.5);

  for (int i = -w; i <= w; i++) {
    for (int j = -w; j <= w; j++) {
      if (abs(xi+i) <= m_Size && abs(yi+j) <= m_Size) {
        if (hypot(i,j) < w) {
          (*this)(xi+i, yi+j) = value;
        }
      }
    }
  }
}

}; // namespace HSS

#endif // HSS_GridMap2D_hh
