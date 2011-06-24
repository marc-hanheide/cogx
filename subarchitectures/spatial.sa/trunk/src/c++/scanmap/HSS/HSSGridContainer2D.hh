//
// = FILENAME
//    HSSGridContainer2D.hh
//
// = FUNCTION
//    Defines a matrix like structure used for 
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 1997 Patric Jensfelt (ISR)
//                  2005 Patric Jensfelt (Cure)
//                  2010 Patric Jensfelt (HSS)
//
/* ---------------------------------------------------------------------- */

#ifndef HSS_GridContainer2D_hh
#define HSS_GridContainer2D_hh

#ifndef DEPEND
#include <iostream>
#endif // DEPEND //

namespace HSS {

/**
 * The GridContainer2D class is a base class for the diffenernt kinds of grid
 * maps. It defines a matrix structure that can be be used to hold the
 * data in the map. The GridContainer2D is a template class, meaning that the
 * type of data you want to store in the map is something you can
 * choose yourself.  The GridContainer2D provides a few very simple access
 * function, map(x,y) which will give you the value in position (x,y)
 * and map[index] which will give you the value on position with index
 * index. It is up to the user of the class to handle the mapping from
 * the coordinates of the map to the world coordinates.
 *
 * HSS::GridContainer2D<int> m(20,10);// Defines a 20x10 map
 * m(0,0) = 4;                 // Let the lower left corner in the matrix be 4
 * m(9,19) = 8;                // Let the upper right corner be 8
 * m.print(cout);              // Display the matrix on standard output 
 *
 * @author Patric Jensfelt
 * @see LocalGridContainer2D
 */
template <class MAPDATA>
class GridContainer2D
{
public:
  /**
   * Constructor
   *
   * @param xSize number of cells in the x-direction (columns)
   * @param ySize number of cells in the y-direction (rows)
   */
  GridContainer2D(int xSize, int ySize);

  /**
   * Destructor
   */
  virtual ~GridContainer2D();

  /**
   * Access content of cell (x,y)
   *
   * @param x column number
   * @param y row number
   * @return the value of the map at position (x,y)
   * where row 0 is the first row and 0 is the first column
   */
  virtual MAPDATA& operator () (int x, int y);
  virtual const MAPDATA& operator () (int x, int y) const;

  /**
   * Access content of cell with a certain index = x + y * getXSize()
   * 
   * @return the value at position index. 
   * Position 0 is the position in the lower left corner
   */
  virtual MAPDATA& operator [] (long index);
  virtual const MAPDATA& operator [] (long index) const;

  /** @return the index corresponding to a cell */
  long getIndex(int x, int y) const { return x + y * m_XSize; }

  /** @return number of cells in the x-direction (columns) */
  int getXSize() const { return m_XSize; }

  /** @return number of cells in the y-direction (rows) */
  int getYSize() const { return m_YSize; }

  /** @return the maximum index in the GridContainer2D, i.e., the number of
      elements */
  long getMaxIndex() const { return m_MaxIndex; }
 
  /** 
   * Prints the map on specified ostream
   *
   * @param os stream to print on
   */
  virtual void print(std::ostream & os);

  /**
   * Compare the results from a the operator() and operator[]
   * functions with this reference to see if you got an error when
   * accessing the map data (out of bounds for example)
   *
   * @return a reference to an error object
   */
  MAPDATA& getErrorRef() { return m_MapdataError; }

  /**
   * Clear the map      
   */
 virtual void clearMap();

protected:
  /** object that is returned when trying to access object outsude the grid
   */
  MAPDATA m_MapdataError;

  /** the grid data */
  MAPDATA *m_Map;

  /** The number of cells in the x-direction (columns) */
  int m_XSize;

  /** The number of cells in the y-direction (rows) */
  int m_YSize;

  /** The number of cells in the map*/
  long m_MaxIndex;
}; // class GridContainer2D //

template<class MAPDATA>
GridContainer2D<MAPDATA>::GridContainer2D(int xSize, int ySize)
  :m_XSize(xSize),
   m_YSize(ySize)
{
  m_MaxIndex = m_XSize * m_YSize;

  // Add one more for junk that you don't want in the GridContainer2D 
  // but have to put somewhere
  m_Map = new MAPDATA[m_MaxIndex + 1];
  for (int ii = 0; ii <= m_MaxIndex; ii++)
    m_Map[ii] = MAPDATA();
  
  // Used as return when there is something wrong
  m_MapdataError = MAPDATA();
}

template<class MAPDATA>
GridContainer2D<MAPDATA>::~GridContainer2D()
{
  delete []m_Map;
}

template<class MAPDATA>
inline MAPDATA& GridContainer2D<MAPDATA>::operator()(int x, int y)
{
  // These lines can be removed if you are damn sure that you don't try
  // to access data outside the map.
  if ((x < 0) || (x >= m_XSize)) {
    std::cerr << "GridContainer2D<MAPDATA>::operator ()  x-index out of bound x=" 
              << x << std::endl;
    return m_MapdataError;
  }

  if ((y < 0) || (y >= m_YSize)) {
    std::cerr << "GridContainer2D<MAPDATA>::operator ()  y-index out of bound y=" 
              << y << std::endl;
    return m_MapdataError;
  }

  return m_Map[x + y * m_XSize];
}

template<class MAPDATA>
inline const MAPDATA& GridContainer2D<MAPDATA>::operator()(int x, int y) const
{
  // These lines can be removed if you are damn sure that you don't try
  // to access data outside the map.
  if ((x < 0) || (x >= m_XSize)) {
    std::cerr << "GridContainer2D<MAPDATA>::operator ()  x-index out of bound x=" 
              << x << std::endl;
    return m_MapdataError;
  }

  if ((y < 0) || (y >= m_YSize)) {
    std::cerr << "GridContainer2D<MAPDATA>::operator ()  y-index out of bound y=" 
              << y << std::endl;
    return m_MapdataError;
  }

  return m_Map[x + y * m_XSize];
}

template<class MAPDATA>
inline MAPDATA& GridContainer2D<MAPDATA>::operator[](long index)
{
  if ((index < 0) || (index >= m_MaxIndex)) {
    std::cerr << "GridContainer2D<MAPDATA>::operator []  index out of bounds:" 
              << index << std::endl;
    return m_MapdataError;
  }

  return m_Map[index];
}

template<class MAPDATA>
inline const MAPDATA& GridContainer2D<MAPDATA>::operator[](long index) const
{
  if ((index < 0) || (index >= m_MaxIndex)) {
    std::cerr << "GridContainer2D<MAPDATA>::operator []  index out of bounds:" 
              << index << std::endl;
    return m_MapdataError;
  }

  return m_Map[index];
}

template<class MAPDATA>
inline void GridContainer2D<MAPDATA>::clearMap()
{
  // Remove all cells, i.e. even the last one for junk
  for (int i = 0; i <= m_MaxIndex; i++)
    m_Map[i] = MAPDATA();
}

template<class MAPDATA>
void GridContainer2D<MAPDATA>::print(std::ostream &os)
{
  for (int y = (m_YSize-1); y >= 0; y--) {
    for (int x = 0; x < m_XSize; x++) {
      os << (*this)[x + y * m_XSize] << " ";
    }
    os << std::endl;
  }
}

}; // namespace HSS

#endif // HSS_GridContainer2D_hh

