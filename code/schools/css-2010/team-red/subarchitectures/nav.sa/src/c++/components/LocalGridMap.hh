//
// = FILENAME
//    LocalGridMap
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = END<CODE>
//
// = COPYRIGHT
//    Copyright (c) 1997 Patric Jensfelt
//                  2005 Patric Jensfelt
//
/* ---------------------------------------------------------------------- */

#ifndef Cure_LocalGridMap_hh
#define Cure_LocalGridMap_hh

#include "Navigation/GridContainer.hh"
#include "Utils/HelpFunctions.hh"
#include "Utils/CureDebug.hh"

#ifndef DEPEND
#include <string.h>  // memcpy,memmove
#include <iostream>
#include <cstdlib>   // abs
#endif

namespace Cure {

/**
 *  The LocalGridMap is a class that defines a local map structure. 
 *  The map is robot centered, well it doesn't have to be a robot
 *  but it is centered around something. You can ask the map to move 
 *  and it will do so. By doing so you loose the information that 
 *  was found in the part that scrolls "out of sight".
 *  The LocalGridMap is derived from GridMap but defines its own 
 *  accessfunctions. This is so since the internal representation of the 
 *  LocalGridMap is such that the information that is stored is much larger
 *  than the infomation that can be accessed from outside. Instead
 *  of having to copy a lot of data every time the map moves, the origin
 *  of the local map is moved in a larger map. Only when the local map 
 *  hits the boundaries of the larger map, it is necessary to copy data.
 *
 *  The size of the LocalGridMap is defined as the number of cells from the
 *  ce
 * nter, i.e., where the robot is to the end of the map along one 
 *  of the axis. The size of each cell has to be specified in mm
 *  The position of the origin of the local map in robot coordinates are
 *  used for specifying where the local map is. By using the function 
 *  move_to(x,y) you can tell the map to center around another point in 
 *  the world. The position in robot coordinates are specified in mm.
 *
 *  When creating an object from the class LocalGridMap you can choose
 *  one of two possible implementations.
 *  MAP9 - In this implementation the actual size of the map that is stored
 *         is nine times the size of the map that is visible to the user.
 *         This means that the values in the map does not have to be copied
 *         so often when the robot moves. On the other hand it requires
 *         more memory.
 *  MAP1 - In this implementation the actual size and the visible size of 
 *         the map is the same. Saves memory but requires more copying of 
 *         data.
 * 
 *  // Creates a local map with 100 cells from center to the end of the map
 *  // with 0.02 by 0.02 m large cells. Each new cell is given the value 0
 *  Cure::LocalGridMap<float> lm(10, 0.020, 0.0f, 
 *                               Cure::LocalGridMap<float>::MAP1);  
 *  
 *  // Let cell (10,4) i.e. a cell 0.16 m to the right and 0.08 m up be 1.0
 *  lm(8,4) = 1.0;
 * 
 *  // Let cell (0,0) i.e. the the center be 2.0
 *  lm(0,0) = 2.0;
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
 * @see GridMapContainer
 */
template <class MAPDATA>
class LocalGridMap : public GridContainer<MAPDATA> 
{
public:
  enum MapType {
    MAP1 = 0,
    MAP9,  
  };

public:
  /**
   * Initalizes a local map, the size specified here is the number of
   * cells form the center, where the robot is, to the side (quadratic
   * map).  The cells are assumed to be quadratic with side cellSize m
   * The position of the robot is specified in m, if you don't give a
   * position it will be set to the origin.  The unknownvalue is the
   * value that is to be assign to new cells that comes into the map
   * when the robot moves and it is also the value that is used to
   * initialize the map. The maptype specifies what implementation is
   * used for handling the data.
   *
   * @param size number of cells from center to side
   * @param cellSize size of the side fo a cell
   * @param unknownvalue the value to give a new cell that comes into the 
   *                     map when the robot moves
   * @param maptype type of map to be used, either only as much memory
   * as is required (MAP1) or storing a 3x3 larger map than needed to
   * avoid to have to copy data (MAP9). Data must only be copied when
   * the robot is closer than size cells from the side in the larger
   * map.
   * @param xc x-coordinate in world coordinates of the center of the map
   * @param yc y-coordinate in world coordinates of the center of the map
   */
  LocalGridMap(int size, double cellSize, MAPDATA unknownvalue,
               int maptype = MAP1, double xc = 0, double yc = 0);


  /**
   * Copy constructor
   */
  LocalGridMap(LocalGridMap<MAPDATA> &lgm);
       
  /**
   * Destructor
   */
  virtual ~LocalGridMap();

  /**
   * Operator used to set this map equal another map
   */
  void operator=(LocalGridMap<MAPDATA> &lgm);

  /** 
   * @param x coord (in cell indices from center (0,0) is center)
   * @param y coord (in cell indices from center (0,0) is center)
   * @return the value at position (x,y) to be edited if desired
   */
  MAPDATA& operator () (int x, int y);
 
  /**
   * @param index index in grid map
   * @returns the value at position index. Position 0 is the position 
   * in the lower left corner
   */
  MAPDATA& operator [] (long index);

  /**
   * @return the size of the map, defined as the number of cells cells
   * from the center of the map to the end of the map. The size is
   * defined for the "visible" part of the map, i.e. not the entire
   * size of a MAP9 type map.
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
  int index2WorldCoords(int i, int j, double &xW, double &yW) 
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
  int worldCoords2Index(double xW, double yW, int &i, int &j) 
  { 
    i = (int((xW - m_XCentW)/(m_CellSize/2.0)) + (xW >= m_XCentW ? 1: -1)) / 2;
    j = (int((yW - m_YCentW)/(m_CellSize/2.0)) + (yW >= m_YCentW ? 1: -1)) / 2;

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
  MAPDATA getUnknownValue() const { return m_UnknownValue; }

  /** @return the x-coordinate of the center position in world coordinates */
  double getCentXW() { return m_XCentW; }

  /** @return the y-coordinate of the center position in world coordinates */
  double getCentYW() { return m_YCentW; }

  /**
   * Set all values within the circle to a certain value.
   *
   * @param xC x-coordinate of the center of the circle
   * @param yC y-coordinate of the center of the circle
   * @param rad radius of the circle
   * @param value the value to assign to the cells inside the circle
   */
  void setValueInsideCircle(double xC, double yC, double rad, MAPDATA value);
  
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
   * unknownvalue(). If we move more than cellSize * size all
   * cells will be unknownvalue().
   */
  int moveCenterTo(double x, double y);

  /** 
   * Used to clear the whole local map. USE WITH CAUTION, IT IS NOT
   * CLEAR IT IS WORKS AS IT SHOULD!!!!!!!!! TEST IT!!!  
   */
  virtual void clearMap();

private:

  /**
   * Called by cnstructors to do common setup stuff
   */
  void constructorInit();

private:

  /** The type of implementation of concerning the size of the map
   * that is actually stored (see Description).*/
  int m_Maptype;

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
   * X-coordinate of the center position of the underlying GridContainer
   * map. This is a constant for a given map [cells] 
   */
  int m_XCent0C;

  /** 
   * Y-coordinate of the center position of the underlying GridContainer
   * map. This is a constant for a given map [cells] 
   */
  int m_YCent0C;

  /** 
   * X-coordinate of the center of the local map in the underlying
   * GridContainer [cells]. This will change with calls to
   * moveCenterTo for MAP9 type maps but not for MAP1 type maps.
   */
  int m_XCentC;   

  /** 
   * X-coordinate of the center of the local map in the underlying
   * GridContainer [cells]. This will change with calls to
   * moveCenterTo for MAP9 type maps but not for MAP1 type maps.
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
  GridContainer<long> *m_Lookup; 

  /** 
   * The value to be used for initializing a new cell in the map, e.g. 0.5
   * in the case of the standard occupanygrid for a cell which status is 
   * completely unknown 
   */
  MAPDATA m_UnknownValue;

  /** 
   * Vector that contains m_UnknownValues and is used when new areas are
   * to be set totally unknown. This is done by memcpy to save time. 
   */
  MAPDATA *m_ClearVector;

  /** The number of bytes on one row in the map */
  int m_BytesPerRow;

  // ==================================================
  // = PRIVATE PROPERTY ACCESSORS
  // ==================================================

  /** Move the local map to the center of the whole map */
  int moveBackMap9ToCenter();

}; // class LocalGridMap

template <class MAPDATA>
LocalGridMap<MAPDATA>::LocalGridMap(int size, double cellSize, 
				    MAPDATA unknownvalue,
				    int maptype,
				    double xc, double yc)
  :GridContainer<MAPDATA>(((maptype == MAP9) ? (2*size+1)*3 : (2*size +1)), 
                          ((maptype == MAP9) ? (2*size+1)*3 : (2*size +1))),
   m_Maptype(maptype),
   m_Size(size),
   m_CellSize(cellSize),
   m_SideLength(2*size+1),
   m_NumCells((2*size+1)*(2*size+1)),
   m_XCentW(xc),
   m_YCentW(yc),
   m_UnknownValue(unknownvalue)
{
  CureCERR(30) << "Constructor called\n";
  constructorInit();
  clearMap();
}

template <class MAPDATA>
LocalGridMap<MAPDATA>::LocalGridMap(LocalGridMap<MAPDATA> &lgm)
  :GridContainer<MAPDATA>(((lgm.m_Maptype == MAP9) ? 
                           (2*lgm.m_Size+1)*3 : (2*lgm.m_Size+1)),
                          ((lgm.m_Maptype==MAP9) ? 
                           (2*lgm.m_Size+1)*3 : (2*lgm.m_Size+1))),
   m_Maptype(lgm.m_Maptype),
   m_Size(lgm.m_Size),
   m_CellSize(lgm.m_CellSize),
   m_SideLength(lgm.m_SideLength),
   m_NumCells(lgm.m_NumCells),
   m_XCentW(lgm.m_XCentW),
   m_YCentW(lgm.m_YCentW),
   m_UnknownValue(lgm.m_UnknownValue)
{
  CureCERR(30) << "Copy constructor called\n";
  constructorInit();
  (*this) = lgm;
}

template <class MAPDATA>
inline void LocalGridMap<MAPDATA>::constructorInit()
{
  m_Lookup = new GridContainer<long>(m_SideLength, m_SideLength);

  int i = 0;
  for (int y = -m_Size; y <= m_Size; y++) {
    for (int x = -m_Size; x <= m_Size; x++) {
      (*m_Lookup)[i++] = m_Lookup->getIndex(x,y);
    }
  }

  // Initial position of the origin
  if (m_Maptype == MAP9) {
      m_XCent0C = 3 * m_Size + 1;
      m_YCent0C = 3 * m_Size + 1;
  } else {
    m_XCent0C = m_Size;
    m_YCent0C = m_Size;
  }

  m_XCentC = m_XCent0C;
  m_YCentC = m_YCent0C;

  // Create a vector used for clearing the map
  if (m_Maptype == MAP9) {
    m_ClearVector = new MAPDATA[m_SideLength];
    m_BytesPerRow = m_SideLength * sizeof(MAPDATA);
    for (int ii = 0; ii < m_SideLength; ii++) {
      m_ClearVector[ii] = m_UnknownValue;
    }
  } else {
    m_ClearVector = new MAPDATA[m_NumCells];
    m_BytesPerRow = m_NumCells * sizeof(MAPDATA);
    for (int ii = 0; ii < m_NumCells; ii++) {
      m_ClearVector[ii] = m_UnknownValue;
    }
  }
}

template <class MAPDATA>
LocalGridMap<MAPDATA>::~LocalGridMap()
{
  delete m_Lookup;
  delete m_ClearVector;
}

template <class MAPDATA>
inline void LocalGridMap<MAPDATA>::operator=(LocalGridMap<MAPDATA> &lgm)
{
  if (getSize() == lgm.getSize() &&
      getUnknownValue() == lgm.getUnknownValue()) {

    m_XCent0C = lgm.m_XCent0C;
    m_YCent0C = lgm.m_YCent0C;

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
inline MAPDATA&  LocalGridMap<MAPDATA>::operator() (int x, int y)
{
  if ((x < -m_Size) || (x > m_Size)) {
    //std::cerr << "LocalGridMap::operator ()  x-index out of bounds x=" 
      //        << x << std::endl;
    return GridContainer<MAPDATA>::m_MapdataError;
  }

  if ((y < -m_Size) || (y > m_Size)) {
    //std::cerr << "LocalGridMap::operator ()  y-index out of bounds y=" 
      //        << y << std::endl;
    return GridContainer<MAPDATA>::m_MapdataError;
  }
  
  return GridContainer<MAPDATA>::operator()(m_XCentC + x, m_YCentC + y);
}

template <class MAPDATA>
inline MAPDATA& LocalGridMap<MAPDATA>::operator[] (long index)
{
  if ((index < 0) || (m_NumCells < index)) {
    //std::cerr << "LocalGridMap::operator []  index out of bounds:" 
      //        << index << std::endl;
    return GridContainer<MAPDATA>::m_MapdataError;
  }
  return GridContainer<MAPDATA>::operator[]((*m_Lookup)[index] + 
                                      m_YCentC * GridContainer<MAPDATA>::m_XSize + m_XCentC);
}

template <class MAPDATA>
inline bool LocalGridMap<MAPDATA>::isCircleObstacleFree(double xW,
                                                        double yW,
                                                        double rad)
{
  int xiC, yiC;
  if (worldCoords2Index(xW, yW, xiC, yiC) != 0) {
    CureCERR(30) << "Querying area outside the map (xW="
                 << xW << ", yW=" << yW << ")\n";
    return true;
  }

  double w = rad / m_CellSize;
  int wi = int(w + 0.5);

  for (int x = xiC-wi; x <= xiC+wi; x++) {
    for (int y = yiC-wi; y <= yiC+wi; y++) {
      if (x >= -m_Size && x <= m_Size && y >= -m_Size && y <= m_Size) {
        if (hypot(x-xiC,y-yiC) < w) {
          if ((*this)(x,y) == 1) return false;
        }
      }
    }
  }

  return true;
}

template <class MAPDATA>
inline bool LocalGridMap<MAPDATA>::isRectangleObstacleFree(double x1W, 
                                                           double y1W, 
                                                           double x2W, 
                                                           double y2W, 
                                                           double width)
{
  double xC = 0.5 * (x1W + x2W);
  double yC = 0.5 * (y1W + y2W);

  int xiC, yiC;
  if (worldCoords2Index(xC, yC, xiC, yiC) != 0) {
    CureCERR(30) << "Querying area outside the map\n";
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
        double d = HelpFunctions::distPt2Line(xW,yW,
                                              x1W, y1W,
                                              -sinDir, cosDir);
        if (d >= 0 && d <= len) {
          if (fabs(HelpFunctions::distPt2LinePts(x1W, y1W, x2W, y2W, xW, yW)) 
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
inline void LocalGridMap<MAPDATA>::print(std::ostream& os)
{
  for (int y = m_Size; y >= -m_Size; y--) {
    for (int x = -m_Size; x <= m_Size; x++)
      os << (*this)(x,y) << " ";
    os << std::endl;
  }
}

template <class MAPDATA>
inline void LocalGridMap<MAPDATA>::clearMap()
{
  if (m_Maptype == MAP9) {
    for (int ii = 0; ii < m_SideLength; ii++)
      memcpy(&(*this)[ii*m_SideLength], m_ClearVector, m_BytesPerRow);
  } else {
    memcpy(&(*this)[0], m_ClearVector, m_BytesPerRow);
  }
}
 

template <class MAPDATA>
int LocalGridMap<MAPDATA>::moveCenterTo(double xpos, double ypos)
{
  int dx = (int) ((xpos - m_XCentW) / m_CellSize);
  int dy = (int) ((ypos - m_YCentW) / m_CellSize);
  
  // Check if the whole map can be cleared, i.e. the robot has moved very far
  if (( dx <= -m_SideLength ) || ( m_SideLength <= dx ) || 
      ( dy <= -m_SideLength ) || ( m_SideLength <= dy )) {
    m_XCentC = m_XCent0C;
    m_YCentC = m_YCent0C;
    clearMap();
  } else {
    long tmp_x = m_XCentC + dx;
    long tmp_y = m_YCentC + dy;
    
    if (m_Maptype == MAP9) {
      if ((tmp_x < m_Size) || ((5*m_Size + 3) <= tmp_x) || 
          (tmp_y < m_Size) || ((5*m_Size + 3) <= tmp_y)) {
        moveBackMap9ToCenter();
        moveCenterTo(xpos, ypos);
      } else {
        m_XCentC = tmp_x;
        m_YCentC = tmp_y;

        if (dx > 0) { 
          for (int k = 0; k < dx; k++) {
            for ( int j = -m_Size; j <=m_Size; j++) {
              (*this)(m_Size - k, j) = m_UnknownValue;
            }
          }
        } else { 
          for (int k = dx+1; k <= 0; k++) {
            for ( int j = -m_Size; j <=m_Size; j++) {
              (*this)(-m_Size - k, j) = m_UnknownValue;
            }
          }
        }

        if (dy > 0) { 
          for (int j = 0; j < dy; j++) {
            memcpy(&(*this)(-m_Size, m_Size - j), m_ClearVector, 
                   m_BytesPerRow);
          }
        } else { 
          for (int j = dy+1; j <= 0; j++) {
            memcpy(&(*this)(-m_Size, -m_Size - j), m_ClearVector, 
                   m_BytesPerRow);
          }
        }
      }

    } else { // maptype is now MAP1

      size_t row_length = sizeof(MAPDATA) * (m_SideLength - dx);
      size_t m_BytesPerRowrow = sizeof(MAPDATA) * m_SideLength;

      // Move the values from the cells that will not be replaced with
      // unknown values to their new position relative to the center
      if (dy > 0) { 
        if (dx > 0) {
          for(int j = -m_Size + dy; j <= m_Size; j++) {
            memmove(&(*this)(-m_Size, j - dy),
                    &(*this)(-m_Size + dx, j),
			  row_length);
          }
        } else {
          for(int j = -m_Size + dy; j <= m_Size; j++) {
            memmove(&(*this)(-m_Size- dx, j - dy),
                    &(*this)(-m_Size, j),
                    row_length);
          }
        }
      } else {
        if (dx > 0) {
          for(int j = m_Size + dy; j >= -m_Size; j--) {
		  memmove(&(*this)(-m_Size, j - dy),
			  &(*this)(-m_Size + dx, j),
			  row_length);
          }
        } else {
          for(int j = m_Size + dy; j >= -m_Size; j--) {
            memmove(&(*this)(-m_Size - dx, j - dy),
                    &(*this)(-m_Size, j),
                    row_length);
          }
        }
      }

      // Initialize the "new" parts of the map that has not been seen
      // before
      if (dx > 0) { 
        for (int k = 0; k < dx; k++) {
          for ( int j = -m_Size; j <=m_Size; j++) {
            (*this)(m_Size - k, j) = m_UnknownValue;
          }
        }
      } else { 
        for (int k = dx+1; k <= 0; k++) {
          for ( int j = -m_Size; j <=m_Size; j++) {
		  (*this)(-m_Size - k, j) = m_UnknownValue;
          }
        }
      }
      if (dy > 0) { 
        for (int j = 0; j < dy; j++) {
          memcpy(&(*this)(-m_Size, m_Size - j), m_ClearVector, 
		       m_BytesPerRowrow);
        }
      } else {
        for (int j = dy+1; j <= 0; j++) {
          memcpy(&(*this)(-m_Size, -m_Size - j), m_ClearVector, 
                 m_BytesPerRowrow);
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
int LocalGridMap<MAPDATA>::moveBackMap9ToCenter()
{
  if (m_Maptype != MAP9) return true;

  // specifies how many index positions the origin of the local map
  // has to move to be in the center of the map
  if (m_YCentC < m_YCent0C) { // Robot in lower half of the map
    for (int y = m_Size; y >= -m_Size; y--) {
      memmove(&GridContainer<MAPDATA>::operator[]((m_XCent0C - m_Size) +
                                            (m_YCent0C + y) * GridContainer<MAPDATA>::m_XSize),
              &(*this)(-m_Size,y), m_BytesPerRow);		
    }
  } else { // Robot in the upper half of the map
    for (int y = -m_Size; y <= m_Size; y++) {
      memmove(&GridContainer<MAPDATA>::operator[]((m_XCent0C - m_Size) +
                                            (m_YCent0C + y) * GridContainer<MAPDATA>::m_XSize),
	      &(*this)(-m_Size,y), m_BytesPerRow);		
    }
  }

  m_XCentC = m_XCent0C;
  m_YCentC = m_YCent0C;

  return true;
}
template <class MAPDATA>
void LocalGridMap<MAPDATA>::setValueInsideCircle(double xC, double yC, 
                                                 double rad, MAPDATA value)
{
  int xi, yi;
  worldCoords2Index(xC, yC, xi, yi);

  int w = int(rad / m_CellSize + 0.5);

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

}; // namespace Cure

#endif // Cure_LocalGridMap_hh
