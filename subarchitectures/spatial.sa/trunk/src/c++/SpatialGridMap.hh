#ifndef SPATIALGRIDMAP_H
#define SPATIALGRIDMAP_H

#include "FunctorTypes.hh"
#include "ConeSlicer.hh"
#include <cv.h>
#include <cstdlib> 
#include <vector>
#include <iostream>
#include <exception> 
#include <algorithm> //max, min, sort
#include <cmath> //sin, cos
#include <utility> //pair
#include <limits> //double max
#include <sstream> //stringstream
#include <fstream> //ofstream, ifstream
namespace SpatialGridMap { //temporary namespace for now

#define BLOXEL_SIZE_EPS (1e-8)

using namespace std;

typedef pair<int, int> Range;

/**
 Bloxels are used to represent data in the GridMap,
 they are organized on the z-axis as columns in the x/y map grid.
 Each bloxel contains some data, as well as the z-coordinate
 of its celing, this also being the implicit bottom of the next Bloxel.
 */
template<class MapData>
struct Bloxel {
  MapData data;
  double celing;

  Bloxel() {
  }
  ;
  Bloxel(MapData data, double celing) :
    data(data), celing(celing) {
  }
  ;
};

// This exception is thrown when an access outside the grid is performed
class gridMapOutOfBounds: public exception {
private:
  string desc;
public:
  gridMapOutOfBounds(int x, int y) {
    stringstream ss;
    ss << "Tried to access GridMap column (" << x << "," << y
        << ") out of bounds";
    //     ss << " MapSize: " << xGrid << " , " << yGrid << ",";
    desc = ss.str();
  }
  ~gridMapOutOfBounds() throw () {
  }
  virtual const char* what() const throw () {
    return desc.c_str();
  }
};

/**
 Class GridMap 
 GridMap stores a 3D map in the format of a discrete x/y Grid, where each grid cell
 stores a column of bloxels each bloxel representing the MapData for a specific zInterval.
 The z interval is not discrete but defined by floating point doubles.

 Operations on the map are performed by queries, that take a set of parameters and a template parameter
 functor that defines the operation on each bloxel. See BloxelFunctors.hh and GridDataFunctors.hh for examples.

 The template datatype MapData has a few requirements. It needs an operator=(MapData &) and merge(Iterator, int)
 to check for equality and merge bloxels that are considered equal. MapData also needs stream operators >> and << 
 defined for input/output serialization. An example of a compliant MapData can be found in GridMapData.hh
 */
template<class MapData>
class GridMap {
  friend class ConeSlicer;
public:
  /**
   Default constructor for GridMap
   @param xGrid number of cells on the x-axis
   @param yGrid number of cells on the y-axis
   @param cellSize real size of cell, cells are quadratic
   @param minBloxel minimum size of bloxel
   @param zMin z-coordinate of floor, lower height-bound in the map
   @param zMax z-coordinate of ceeling, highest hight-bound in the map
   @param mapCenterX x-coordinate of map center
   @param mapCenterY y-coordinate of map center
   @param mapRotator angle of rotation
   @param defaultValue default value of columns
   */
  GridMap(int xGrid, int yGrid, double cellSize, double minBloxel, double zMin,
      double zMax, double mapCenterX, double mapCenterY, double mapRotation,
      MapData defaultValue);

  /**
   Constructor with input stream
   @param stream an input stream that feeds information with the following specification:
   (All values should be separated by space or newline)
   First line: Values for: xGrid, yGrid, cellSize, minBloxel, zMin, zMax, mapCenterX, mapCenterY, mapRotation
   Second line: <Default value as defined by the specification for that MapData type>
   Then follows xGrid*yGrid lines where line i*xGrid+j represents x=i, y=j in the grid,
   having the folowing format: Size <Celing Value> <Celing Value> ... 
   That is an integer defining column size, followed by Size number of Celing Value pairs,
   Value once again conforming to the MapData specification
   */
  GridMap(std::istream& stream);
  ~GridMap(); // Destructor
  GridMap(const GridMap<MapData> & copy); // Copy constructor
  GridMap & operator=(const GridMap<MapData> & copy); //Assigment

  //Getters
  pair<int, int> getMapSize() const;
  pair<double, double> getCentW();
  pair<double, double> getZBounds();
  double getCellSize();
  double getMinBloxelHeight();
  double getRotation();
  MapData getDefaultValue();

  //Setters
  void setCentW(std::pair<double, double> cent);
  void setRotation(double mapRotation);
  void setDefaultValue(MapData defaultValue);

  /**
   Set all cells in the map to their default value
   Map size and other map specifications remain unchanged
   */
  void clearMap();

  // Return an entire column, using grid coordinates
  vector<Bloxel<MapData> > & operator()(int xPos, int yPos);
  const vector<Bloxel<MapData> > & operator()(int xPos, int yPos) const;

  // Return a value at a point in world coordinates
  const MapData & valueAtPoint(double xPos, double yPos, double zPos) const;
  // Return a value at a point in world coordinates
  const MapData & valueAtPoint(int i, int j, double zPos) const;

  /**
   Read and write to stream
   (You may use << and >> operators instead of these methods)
   For format spec see stream constructor
   */
  void writeStream(std::ostream& stream) const;
  void readStream(std::istream& stream);
  void writeFile(string filename) const;
  void readFile(string filename);

  // Transforming between world and grid coordinates
  pair<int, int> worldToGridCoords(double x, double y) const;
  pair<double, double> gridToWorldCoords(int x, int y) const;

  // Query dirty status (for visualization and the like)
  /*
   Check if grid is dirty, using grid cordinates
   */
  bool isDirty(int x, int y) const;
  // Clear dirty grid
  void clearDirty();

  //
  // Region queries and modifiers
  // 

  /**
   Apply functor to interval for single column, mostly used for testing
   Same as columnQuery but with grid coordinates
   */
  template<class BloxelFunctor>
  void boxSubColumnModifier(int i, int j, double zPos, double zSize,
      BloxelFunctor & functor, bool autoMerge = true);

  /**
   Apply functor to every bloxel in the entire map
   */
  template<class BloxelFunctor>
  void universalQuery(BloxelFunctor &functor, bool autoMerge = true);
  /**
   Apply functor to every bloxel in the entire map
   */
  template<class BloxelFunctor>
  void universalModifier(BloxelFunctor &functor, bool autoMerge = true);

  /**
   Axis aligned box, mostly used for testing
   Define box with min/max (using grid coords for x/y) and apply functor
   */
  template<class BloxelFunctor>
  void alignedBoxQuery(int xMin, int xMax, int yMin, int yMax, double zMin,
      double zMax, BloxelFunctor & query, bool autoMerge = true);
  // Deprecated, use query method for everything
  template<class BloxelFunctor>
  void alignedBoxModifier(int xMin, int xMax, int yMin, int yMax, double zMin,
      double zMax, BloxelFunctor & modifier, bool autoMerge = true);

  /** 
   Perform a query on a single column
   @param {x,y}Pos world coordinates of column
   @param zPos center position on the z axis
   @param zSize size on the z axis
   @param functor functor called for each bloxel
   */
  template<class BloxelFunctor>
  void columnQuery(double xPos, double yPos, double zPos, double zSize,
      BloxelFunctor & functor, bool autoMerge = true);

  /** 
   Box, aligned on the z-axis. But rotated on the x/y axis
   @param {x,y,z}Pos center position of box 
   @param {x,y,z}Size size of box on all axis
   @param clockwise rotation angle of rotation for box, in radians
   @param functor functor called for each bloxel
   */
  template<class BloxelFunctor>
  void boxQuery(double xPos, double yPos, double zPos, double xSize,
      double ySize, double zSize, double rotation, BloxelFunctor & functor);
  // Deprecated, use query method for everything
  template<class BloxelFunctor>
  void boxModifier(double xPos, double yPos, double zPos, double xSize,
      double ySize, double zSize, double rotation, BloxelFunctor & functor);

  /**
   Lasermodifier, modify map with a set of laser scans. Requires two functors, one to modify the points detected in the scan and one to modify the free space in between.
   @param {x,y,z}Pos position of camera (top of cone)
   @param n the number of laser scans
   @param startAngle pan angle where the scan starts
   @param angleStep angle incrementation on each step 
   @param maxRange how long the laser scan can be trusted
   @param scans a pointer to a memory block of n doubles, representing the range on each scan
   @param clearModifier functor to be used for modification of the free space in between laser scans
   @param pointModifier functor to be used for modification on each bloxel that corresponds to a scan hit
   */
  template<class PointFunctor, class ClearFunctor>
  void laserQuery(double xPos, double yPos, double zPos, int n,
      double startAngle, double angleStep, double maxRange, double *scans,
      PointFunctor & pointQuery, ClearFunctor & clearQuery);
  // Deprecated, use query method for everything
  template<class PointFunctor, class ClearFunctor>
  void laserModifier(double xPos, double yPos, double zPos, int n,
      double startAngle, double angleStep, double maxRange, double *scans,
      PointFunctor & pointModifier, ClearFunctor & clearModifier);

  /**
   Cone region, tilted cone for queries and modification. Cone can be thought of as an inclined pyramid with its top at the viewpoint of the camera.
   @param {x,y,z}Pos position of camera (top of cone)
   @param panAngle angle of panning, in what x/y direction the cone is looking
   angle is defined as looking at the corresponding position on the unit circle  
   @param tiltAngle tiltangle on the z-axis, how much the cone is looking down 
   0 is straight ahead, pi/2 straight up -pi/2 straight down
   @param {vert,horiz}SizeAnge defines the size of the cone by the angle
   @param maxRange maximum range (depth) of cone
   @param {width,height}InRays determines number of rays for obstacle raytracing
   @param pointFunctor functor used for query / modification of each individual bloxel on a border to an obstacle
   @param clearFunctor functor used for query / modification of each individual bloxel in un-obsctructed space
   @param obstacle a functor returning a boolean representing if bloxel is an obstacle
   */
  template<class ObstacleFunctor, class PointFunctor, class ClearFunctor>
  void coneQuery(double xPos, double yPos, double zPos, double panAngle,
      double tiltAngle, double horizSizeAngle, double vertSizeAngle,
      double maxRange, int widthInRays, int heightInRays,
      ObstacleFunctor & obstacle, PointFunctor & pointQuery,
      ClearFunctor & clearQuery, double minDistance = 0.0);

  // Deprecated, use query method for everything
  template<class ObstacleFunctor, class PointFunctor, class ClearFunctor>
  void coneModifier(double xPos, double yPos, double zPos, double panAngle,
      double tiltAngle, double horizSizeAngle, double vertSizeAngle,
      double maxRange, int widthInRays, int heightInRays,
      ObstacleFunctor & obstacle, PointFunctor & pointModifier,
      ClearFunctor & clearModifier, double minDistance = 0.0);

  // Merging, called after modification
  void tryMergeColumn(int i, int j);

  //
  // End of public interface
  //

private:

  // Data
  int xGrid;
  int yGrid;
  double cellSize;
  double minBloxel;
  double mapZMin;
  double mapZMax;
  double mapCenterX;
  double mapCenterY;
  double mapRotation;
  MapData defaultValue;
  vector<Bloxel<MapData> > ** grid;
  bool * dirtyMap;

  // Common constructor / destructor code
  void deconstruct();
  void construct(const GridMap<MapData> & copy);
  void construct(std::istream& stream);
  void constructGrid();

  // Helper methods
  double minAcceptedBloxel();

  // Collumn access with out-of-bounds exceptions
  // This should be used for all access to grid
  vector<Bloxel<MapData> > & getCol(int x, int y);
  const vector<Bloxel<MapData> > & getCol(int x, int y) const;

  // Helper to calculate floor of a bloxel
  inline double getFloor(int xi, int yi, int zi) {
    return (zi == 0 ? mapZMin : getCol(xi, yi)[zi - 1].celing);
  }

  // Helper that calls the correct query method, read or write
  // This is the general entry point for column access for query-functions
  template<class BloxelFunctor>
  inline void colQuery(int i, int j, BloxelFunctor & functor, double zMin,
      double zMax, bool autoMerge = true);

  // Helper functions for modification of columns
  template<class BloxelFunctor>
  inline bool modifySubcolumn(int i, int j, BloxelFunctor &modifier,
      double zMin, double zMax, bool autoMerge);
  // Helper function modifies bloxels before query call
  Range carveHelper(FunctorType type, vector<Bloxel<MapData> > & column,
      double zMin, double zMax);

  // Helper function for queries
  template<class BloxelFunctor>
  inline void iteratorHelper(int xPos, int yPos, BloxelFunctor & functor,
      double zMin, double zMax);

  // Helper for box rotation
  pair<double, double> boxRotHelper(double xCent, double yCent, double x,
      double y, double rot) const;

  // Helper for occlusion when cone building
  template<class ObstacleFunctor>
  inline double rayCollisionDistance(cv::Mat_<double> origin,
      cv::Mat_<double> direction, double maxDistance,
      const ObstacleFunctor &obstacle) const;

}; // End of SpatialGridMap

//Constructor
template<class MapData>
GridMap<MapData>::GridMap(int xGrid, int yGrid, double cellSize,
    double minBloxel, double zMin, double zMax, double mapCenterX,
    double mapCenterY, double mapRotation, MapData defaultValue) :
  xGrid(xGrid), yGrid(yGrid), cellSize(cellSize), minBloxel(minBloxel),
      mapZMin(zMin), mapZMax(zMax), mapCenterX(mapCenterX), mapCenterY(
          mapCenterY), mapRotation(mapRotation), defaultValue(defaultValue) {

  constructGrid();
  //init bloxels with default value
  for (int i = 0; i < xGrid; i++) {
    for (int j = 0; j < yGrid; j++) {
      grid[i][j].push_back(Bloxel<MapData> (defaultValue, mapZMax));
    }
  }
  clearDirty(); // Init dirty map to clear
}
//Stream constructor
template<class MapData>
GridMap<MapData>::GridMap(std::istream& stream) {
  construct(stream);
}

//Destructor
template<class MapData>
GridMap<MapData>::~GridMap() {
  deconstruct();
}

//Copy constructor
template<class MapData>
GridMap<MapData>::GridMap(const GridMap<MapData> & copy) {
  construct(copy);
}

//Assignment operator
template<class MapData>
GridMap<MapData> & GridMap<MapData>::operator=(const GridMap<MapData> & copy) {
  deconstruct();
  construct(copy);

  return *(this);
}

// Common deconstruction operations
template<class MapData>
void GridMap<MapData>::deconstruct() {
  for (int i = 0; i < xGrid; i++) {
    delete[] grid[i];
  }
  free(grid);
  free(dirtyMap);
}

// Common grid constructor, call after variables are initiated
template<class MapData>
void GridMap<MapData>::constructGrid() {
  size_t gridSizeX = sizeof(vector<Bloxel<MapData> > *) * xGrid;
  size_t gridSizeY = sizeof(vector<Bloxel<MapData> > ) * yGrid;

  grid = (vector<Bloxel<MapData> > **) malloc(gridSizeX);
  for (int i = 0; i < xGrid; i++) {
    grid[i] = new vector<Bloxel<MapData> > [gridSizeY];
  }

  dirtyMap = (bool *) malloc(sizeof(bool) * xGrid * yGrid);
}

// Common copy operations
template<class MapData>
void GridMap<MapData>::construct(const GridMap<MapData> & copy) {
  xGrid = copy.xGrid;
  yGrid = copy.yGrid;
  cellSize = copy.cellSize;
  minBloxel = copy.minBloxel;
  mapZMin = copy.mapZMin;
  mapZMax = copy.mapZMax;
  mapCenterX = copy.mapCenterX;
  mapCenterY = copy.mapCenterY;
  mapRotation = copy.mapRotation;
  defaultValue = copy.defaultValue;

  constructGrid();
  //init bloxels with default value
  for (int i = 0; i < xGrid; i++) {
    for (int j = 0; j < yGrid; j++) {
      grid[i][j] = copy.grid[i][j];
      dirtyMap[i * yGrid + j] = copy.dirtyMap[i * yGrid + j];
    }
  }
}

// Common stream construction
template<class MapData>
void GridMap<MapData>::construct(std::istream& stream) {
  stream >> xGrid >> yGrid;
  stream >> cellSize >> minBloxel;
  stream >> mapZMin >> mapZMax;
  stream >> mapCenterX >> mapCenterY;
  stream >> mapRotation;
  stream >> defaultValue;

  constructGrid();
  // Dirty map is not saved, it is more of a runtime thingy
  clearMap();
  int bloxels;
  for (int i = 0; i < xGrid; i++) {
    for (int j = 0; j < yGrid; j++) {
      stream >> bloxels;
      for (int k = 0; k < bloxels; k++) {
        Bloxel<MapData> b;
        stream >> b.celing >> b.data;
        grid[i][j].push_back(b);
      }
    }
  }
}

// String serialization
template<class MapData>
void GridMap<MapData>::readStream(std::istream& stream) {
  deconstruct();
  construct(stream);
}

template<class MapData>
void GridMap<MapData>::writeStream(std::ostream& stream) const {
  // Set precision to full
  stream.precision(30);

  stream << xGrid << " " << yGrid << " ";
  stream << cellSize << " " << minBloxel << " ";
  stream << mapZMin << " " << mapZMax << " ";
  stream << mapCenterX << " " << mapCenterY << " ";
  stream << mapRotation << endl;
  stream << defaultValue << endl;
  for (int i = 0; i < xGrid; i++) {
    for (int j = 0; j < yGrid; j++) {
      stream << grid[i][j].size() << " ";
      for (int k = 0; k < grid[i][j].size(); k++) {
        stream << grid[i][j][k].celing << " " << grid[i][j][k].data << " ";
      }
      stream << endl;
    }
  }
}

// Reading and writing to file
template<class MapData>
void GridMap<MapData>::writeFile(string filename) const {
  ofstream file(filename.c_str(), ios::out);
  writeStream(file);
}
template<class MapData>
void GridMap<MapData>::readFile(string filename) {
  ifstream file(filename.c_str());
  readStream(file);
}

//Getters
template<class MapData>
pair<int, int> GridMap<MapData>::getMapSize() const {
  pair<int, int> mapsize;
  mapsize.first = xGrid;
  mapsize.second = yGrid;
  return mapsize;
}
template<class MapData>
pair<double, double> GridMap<MapData>::getCentW() {
  pair<double, double> mapCenterW;
  mapCenterW.first = mapCenterX;
  mapCenterW.second = mapCenterY;

  return mapCenterW;
}
template<class MapData>
pair<double, double> GridMap<MapData>::getZBounds() {
  pair<double, double> ret;
  ret.first = mapZMin;
  ret.second = mapZMax;

  return ret;
}
template<class MapData>
double GridMap<MapData>::getCellSize() {
  return cellSize;
}
template<class MapData>
double GridMap<MapData>::getMinBloxelHeight() {
  return minBloxel;
}
template<class MapData>
double GridMap<MapData>::getRotation() {
  return mapRotation;
}
template<class MapData>
MapData GridMap<MapData>::getDefaultValue() {
  return defaultValue;
}

//Setters
template<class MapData>
void GridMap<MapData>::setCentW(std::pair<double, double> cent) {
  mapCenterX = cent.first;
  mapCenterY = cent.second;
}
template<class MapData>
void GridMap<MapData>::setRotation(double rotation) {
  mapRotation = rotation;
}
template<class MapData>
void GridMap<MapData>::setDefaultValue(MapData value) {
  defaultValue = value;
}

template<class MapData>
void GridMap<MapData>::clearMap() {
  for (int i = 0; i < xGrid; i++) {
    for (int j = 0; j < yGrid; j++) {
      grid[i][j].clear();
      grid[i][j].push_back(Bloxel<MapData> (defaultValue, mapZMax));
    }
  }
}

//Column retrieval
template<class MapData>
vector<Bloxel<MapData> > & GridMap<MapData>::operator()(int xPos, int yPos) {
  return getCol(xPos, yPos);
}

template<class MapData>
const vector<Bloxel<MapData> > & GridMap<MapData>::operator()(int xPos,
    int yPos) const {
  return getCol(xPos, yPos);
}

// Point retrieval
template<class MapData>
const MapData & GridMap<MapData>::valueAtPoint(double xPos, double yPos,
    double zPos) const {
  if (zPos < mapZMin || zPos > mapZMax) {
    cerr << "GridMap ERROR: zPos is out of bounds" << endl;
    return defaultValue;
  }
  std::pair<int, int> coords = worldToGridCoords(xPos, yPos);
  int x = coords.first, y = coords.second;
  int i = 0;
  for (; i < grid[x][y].size(); i++) {
    if (grid[x][y][i].celing > zPos) {
      break;
    }
  }
  cout << "i == " << i << endl;
  return grid[x][y][i].data;
}

//
// Position transformations
//
// Transforming between world and grid coordinates
template<class MapData>
pair<int, int> GridMap<MapData>::worldToGridCoords(double x, double y) const {
  //Pos relative center position
  x -= mapCenterX;
  y -= mapCenterY;

  //Reverse rotate around origin
  double xr = cos(-mapRotation) * x - sin(-mapRotation) * y;
  double yr = sin(-mapRotation) * x + cos(-mapRotation) * y;

  //Cordinates relative corner
  x = xr + ((xGrid * cellSize) / 2);
  y = yr + ((yGrid * cellSize) / 2);

  //Round to nearest grid pos
  //int xg = (x + (cellSize / 4)) / cellSize;
  //int yg = (y + (cellSize / 4)) / cellSize;
  int xg = x / cellSize;
  int yg = y / cellSize;

  return pair<int, int> (xg, yg);
}

template<class MapData>
pair<double, double> GridMap<MapData>::gridToWorldCoords(int x, int y) const {
  //Coordinates relative center
  double xc = x * cellSize + (cellSize / 2) - (xGrid * cellSize) / 2;
  double yc = y * cellSize + (cellSize / 2) - (yGrid * cellSize) / 2;

  //Reverse rotate around origin
  double xw = cos(mapRotation) * xc - sin(mapRotation) * yc;
  double yw = sin(mapRotation) * xc + cos(mapRotation) * yc;

  //Add in map center position
  xw += mapCenterX;
  yw += mapCenterY;

  return pair<double, double> (xw, yw);
}

template<class MapData>
bool GridMap<MapData>::isDirty(int x, int y) const {
  // Input validation ?

  return dirtyMap[x * yGrid + y];
}

template<class MapData>
void GridMap<MapData>::clearDirty() {
  memset(dirtyMap, 0, sizeof(bool) * xGrid * yGrid);
}

//
// Helper functions
//

// Return minBloxel with a delta value for comparison safety 
template<class MapData>
double GridMap<MapData>::minAcceptedBloxel() {
  return this->minBloxel - BLOXEL_SIZE_EPS;
}

// Collumn access with input validation
template<class MapData>
vector<Bloxel<MapData> > & GridMap<MapData>::getCol(int x, int y) {
  if (x < 0 || y < 0 || x >= xGrid || y >= yGrid) {
    gridMapOutOfBounds g(x, y);
    throw g;
  }
  return grid[x][y];
}
template<class MapData>
const vector<Bloxel<MapData> > & GridMap<MapData>::getCol(int x, int y) const {
  if (x < 0 || y < 0 || x >= xGrid || y >= yGrid) {
    gridMapOutOfBounds g(x, y);
    throw g;
  }
  return grid[x][y];
}

// Helper that calls the correct query method, read or write
// This is the general entry point for column access for query-functions
template<class MapData>
template<class BloxelFunctor>
inline void GridMap<MapData>::colQuery(int i, int j, BloxelFunctor & functor,
    double zMin, double zMax, bool autoMerge) {
  // This is a good place for central input validation
  if (zMax - zMin < minAcceptedBloxel() || zMax < this->mapZMin || zMin
      > this->mapZMax) {
    //                cerr << "GridMap ERROR: Column query has invalid z-range COL: " << i << "," << j
    //                    << " RANGE: " << zMin << "-" << zMax << endl;
    return;
  }

  if (functor.type() == Read) {
    iteratorHelper(i, j, functor, zMin, zMax);
  } else {
    modifySubcolumn(i, j, functor, zMin, zMax, autoMerge);
    dirtyMap[i * yGrid + j] = true;
  }
}

// Main helper for column modification
template<class MapData>
template<class BloxelFunctor>
inline bool GridMap<MapData>::modifySubcolumn(int i, int j,
    BloxelFunctor &modifier, double zMin, double zMax, bool autoMerge) {

  Range indices = carveHelper((FunctorType) modifier.type(), getCol(i, j),
      zMin, zMax);

  if (indices.second < indices.first)
    return false; // Null modification in this column

  const vector<Bloxel<MapData> >&column = getCol(i, j);

  for (int zi = indices.first; zi <= indices.second; zi++) {
    //Calc zPos
    double floor = getFloor(i, j, zi);
    double ceiling = column[zi].celing;

    double size = ceiling - floor;
    double zMiddle = floor + size * 0.5;

    //REMOVEME: sanity check
    if (ceiling - floor < minAcceptedBloxel()) {
      cerr << "Oh noes! (" << i << "," << j << ") is [" << floor << ","
          << ceiling << "]!\n";
    }
    //Update bloxel
    pair<double, double> coords = gridToWorldCoords(i, j);
    modifier(getCol(i, j)[zi].data, size, coords.first, coords.second, zMiddle);
  }

  if (autoMerge)
    tryMergeColumn(i, j);

  return true;
}

// Generic helper function for modifier functions, takes column in grid and an z-interval
// Carves out new bloxels in this interval and returns index range for modification
template<class MapData>
Range GridMap<MapData>::carveHelper(FunctorType type,
    vector<Bloxel<MapData> > & column, double zMin, double zMax) {
  double floor = mapZMin;

  if (zMax - zMin < minAcceptedBloxel()) {
    return Range(0, -1);
  }

  int i;
  for (i = 0; i < (int) column.size(); i++) {
    // At zMin: Insert new bloxel boundary, or merge with existing
    double ceiling = column[i].celing;
    if (zMin < ceiling) {
      if (zMin < floor + minAcceptedBloxel()) {
        // Within margin of the bottom of [i]; merge with [i]
        break;
      } else if (zMin > ceiling - minAcceptedBloxel()) {
        // Within margin of the top of [i]; merge with [i+1]
        floor = column[i].celing;
        i++;
        break;
      } else {
        //Cut [i] and insert a new bloxel after it
        column[i].celing = zMin;
        column.insert(column.begin() + i + 1, Bloxel<MapData> (column[i].data,
            ceiling));
        floor = zMin;
        i++;
        break;
      }
    }
    floor = column[i].celing;
  }
  if (zMax - floor < minAcceptedBloxel()) {
    return Range(0, -1);
  }

  if (type == Replace) {
    // Delete all bloxels until the first one that is not covered by [zMin,zMax]

    int maxAffected = column.size() - 1;
    int maxDeleted = column.size() - 1;
    bool createNewBloxel = false;
    for (int j = i; j < (int) column.size(); j++) {
      if (zMax < column[j].celing - minAcceptedBloxel()) {
        maxAffected = j;
        maxDeleted = j - 1;
        createNewBloxel = true;
        break;
      } else if (zMax < column[j].celing + minAcceptedBloxel()) {
        maxAffected = j;
        maxDeleted = j;
        break;
      }
    }
    if (createNewBloxel) {
      // We need to add a new bloxel
      Bloxel<MapData> t(column[i].data, zMax);
      column.insert(column.begin() + maxAffected, t);
    }
    if (maxDeleted > i) {
      column.erase(column.begin() + i + 1, column.begin() + maxDeleted + 1);
    }
    if (i > 0) {
      column[i - 1].celing = zMin; //redundant?
    }
    column[i].celing = zMax;
    return Range(i, i);
  } else {
    // At zMax: Insert new bloxel boundary, or merge with existing
    unsigned int j = i;
    int maxAffected = column.size() - 1;
    for (; j < column.size(); j++) {
      if (zMax < column[j].celing - minAcceptedBloxel()) {
        //Split bloxel and insert new in the middle
        Bloxel<MapData> t1(column[j].data, zMax); //Clone old bloxel and put below
        column.insert(column.begin() + j, t1); //Insert
        maxAffected = j;
        break;
      } else if (zMax < column[j].celing + minAcceptedBloxel()) {
        //Within margin of ceiling of [j]; merge with[j]
        maxAffected = j;
        break;
      }
    }
    int insertedBloxels = 0;
    if (type == Split) {
      // Split all affected bloxels into many smaller ones
      double floor = (i == 0 ? this->mapZMin : column[i - 1].celing);

      for (int k = i; k <= (maxAffected + insertedBloxels);) {
        // for every bloxel, insert as many bloxels as possible before it
        double celing = column[k].celing;
        // Calculate how many bloxels this can be splitted into, and avg size
        int newBloxels = (celing - floor) / minAcceptedBloxel() - 1;
        if (newBloxels < 0)
          newBloxels = 0;
        double uniformSize = (celing - floor) / (newBloxels + 1);

        if (newBloxels > 0) {
          // Build tmp vector for efficiency
          vector<Bloxel<MapData> > tmp;
          double nextCel = floor + uniformSize;
          for (int m = 0; m < newBloxels; m++) {
            tmp.push_back(Bloxel<MapData> (column[k].data, nextCel));
            nextCel += uniformSize;
          }

          //Insert tmp vector before current one
          column.insert(column.begin() + k, tmp.begin(), tmp.end());
        }
        insertedBloxels += newBloxels;
        k = k + newBloxels + 1;
        floor = celing;
      }
    }
    if (column[maxAffected].celing - floor < minAcceptedBloxel()) {
      return Range(0, -1);
    }
    return Range(i, maxAffected + insertedBloxels);
  }
}

// Merging of similar bloxels, done after modification
template<class MapData>
void GridMap<MapData>::tryMergeColumn(int i, int j) {
  vector<Bloxel<MapData> >&column = getCol(i, j);

  for (typename vector<Bloxel<MapData> >::iterator it = column.begin(); it
      != column.end();) {

    int n = 1;
    // Collect interval of bloxels that are equal to start bloxel
    while ((it + n) != column.end() && it->data == (it + n)->data) {
      n++;
    }

    // If more than one bloxel in it, merge and delete
    if (n > 1) {
      // Set last bloxel to new value
      (it + (n - 1))->data.merge(it, n);
      // Erase all bloxels underneath, the last bloxel now covers everything
      it = column.erase(it, it + (n - 2 + 1));
    }
    it++;
  }
}

// Generic helper function for query function, takes column and z-interval
// iterates through all bloxels in interval and applies functor
// this calculates a new position and size if needed, without altering the bloxel itself
template<class MapData>
template<class BloxelFunctor>
inline void GridMap<MapData>::iteratorHelper(int xPos, int yPos,
    BloxelFunctor & functor, double zMin, double zMax) {
  vector<Bloxel<MapData> > & column = getCol(xPos, yPos);
  double floor = mapZMin;
  unsigned int i;
  // Find all bloxel in interval and run functor
  for (i = 0; i < column.size(); i++) {
    if (floor >= zMax)
      break;

    if (zMin < column[i].celing) {
      // Calulate size of bloxel covered by query
      double bMax = min(zMax, column[i].celing);
      double bMin = max(zMin, floor);
      double size = bMax - bMin;
      double zMid = (bMax + bMin) / 2;

      // Run functor
      pair<double, double> coords = gridToWorldCoords(xPos, yPos);
      functor(column[i].data, size, coords.first, coords.second, zMid);
    }
    floor = column[i].celing;
  }
}

//
// Region queries and modifiers
//

template<class MapData>
template<class BloxelFunctor>
void GridMap<MapData>::boxSubColumnModifier(int i, int j, double zPos,
    double zSize, BloxelFunctor & functor, bool autoMerge) {
  colQuery(i, j, functor, zPos - zSize / 2, zPos + zSize / 2, autoMerge);
}

template<class MapData>
template<class BloxelFunctor>
void GridMap<MapData>::alignedBoxQuery(int xMin, int xMax, int yMin, int yMax,
    double zMin, double zMax, BloxelFunctor & query, bool autoMerge) {
  for (int i = xMin; i <= xMax; i++) {
    for (int j = yMin; j <= yMax; j++) {
      //Iteratorhelper does all the work
      colQuery(i, j, query, zMin, zMax, autoMerge);
    }
  }
}

template<class MapData>
template<class BloxelFunctor>
void GridMap<MapData>::universalQuery(BloxelFunctor &query, bool autoMerge) {
  for (int i = 0; i < xGrid; i++) {
    for (int j = 0; j < xGrid; j++) {
      //Iteratorhelper does all the work
      colQuery(i, j, query, mapZMin, mapZMax, autoMerge);
    }
  }
}

template<class MapData>
template<class BloxelFunctor>
void GridMap<MapData>::universalModifier(BloxelFunctor &query, bool autoMerge) {
  universalQuery(query, autoMerge);
}

template<class MapData>
template<class BloxelFunctor>
void GridMap<MapData>::alignedBoxModifier(int xMin, int xMax, int yMin,
    int yMax, double zMin, double zMax, BloxelFunctor & modifier,
    bool autoMerge) {
  alignedBoxQuery(xMin, xMax, yMin, yMax, zMin, zMax, modifier, autoMerge);
}

template<class MapData>
template<class BloxelFunctor>
void GridMap<MapData>::columnQuery(double xPos, double yPos, double zPos,
    double zSize, BloxelFunctor & functor, bool autoMerge) {
  pair<int, int> coords = worldToGridCoords(xPos, yPos);
  colQuery(coords.first, coords.second, functor, zPos - zSize / 2, zPos + zSize
      / 2, autoMerge);
}

// Takes a point in world coordinates, translate to grid coordinates and rotate around point
// Returns position in grid without converting to int to keep precision up
template<class MapData>
pair<double, double> GridMap<MapData>::boxRotHelper(double xCent, double yCent,
    double x, double y, double rot) const {
  //Pos relative center position
  x -= mapCenterX;
  y -= mapCenterY;
  xCent -= mapCenterX;
  yCent -= mapCenterY;

  //Rotate around origin
  double xr = cos(-mapRotation) * x - sin(-mapRotation) * y;
  double yr = sin(-mapRotation) * x + cos(-mapRotation) * y;
  double xc = cos(-mapRotation) * xCent - sin(-mapRotation) * yCent;
  double yc = sin(-mapRotation) * xCent + cos(-mapRotation) * yCent;

  //Pos relative box center
  xr -= xc;
  yr -= yc;

  //Rotate around box
  x = cos(rot) * xr - sin(rot) * yr;
  y = sin(rot) * xr + cos(rot) * yr;

  //Add box again
  x += xc;
  y += yc;

  //Cordinates relative corner
  x = x + ((xGrid * cellSize) / 2);
  y = y + ((yGrid * cellSize) / 2);

  return pair<double, double> (x, y);
}

template<class MapData>
template<class BloxelFunctor>
void GridMap<MapData>::boxQuery(double xPos, double yPos, double zPos,
    double xSize, double ySize, double zSize, double rotation,
    BloxelFunctor & functor) {
  //Validate input
  if (xSize < this->cellSize) {
    cerr
        << "GridMap warning: Box is to thin to work, setting xSize to cellSize"
        << endl;
    xSize = this->cellSize;
  }
  if (ySize < this->cellSize) {
    cerr
        << "GridMap warning: Box is to thin to work, setting ySize to cellSize"
        << endl;
    ySize = this->cellSize;
  }

  //Transform all corners to relative float position, and rotate
  pair<double, double> corners[4];
  corners[0] = boxRotHelper(xPos, yPos, xPos + xSize / 2, yPos + ySize / 2,
      rotation);
  corners[1] = boxRotHelper(xPos, yPos, xPos + xSize / 2, yPos - ySize / 2,
      rotation);
  corners[2] = boxRotHelper(xPos, yPos, xPos - xSize / 2, yPos - ySize / 2,
      rotation);
  corners[3] = boxRotHelper(xPos, yPos, xPos - xSize / 2, yPos + ySize / 2,
      rotation);

  //Find xmin/xmax
  double xMin = corners[0].first;
  for (int i = 1; i < 4; i++) {
    xMin = min(xMin, corners[i].first);
  }
  double xMax = corners[0].first;
  for (int i = 1; i < 4; i++) {
    xMax = max(xMax, corners[i].first);
  }

  //Round to nearest grid pos, away from partially covered boxes
  int xMinI = xMin / cellSize + cellSize / 2.0 - 1e-8; //Round down to cover box
  int xMaxI = xMax / cellSize - cellSize / 2.0;

  double intersect[4];

  //Iterate on x-axis
  for (int i = xMinI; i <= xMaxI; i++) {
    //Relative float x-position
    double relX = ((double) i) * cellSize + cellSize / 2.0;
    //Make sure this is not out of bounds
    relX = max(xMin, relX);
    relX = min(xMax, relX);

    //Find y coordinate of all intersections between a 
    //vertical y-line and the bounding lines of the box

    double x1 = relX, x2 = relX, y1 = 0, y2 = 1; //vertical line
    bool first = true;
    for (int j = 0; j < 4; j++) {
      double x3 = corners[j].first, y3 = corners[j].second;
      double x4 = corners[(j + 1) % 4].first, y4 = corners[(j + 1) % 4].second;
      // Vertical line, No intersection
      if (fabs(x3 - x4) < 1e-8) {
        // Set values to +- Max
        intersect[j] = first ? DBL_MAX : -DBL_MAX;
        first = false;
      } else {
        intersect[j] = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4
            - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
      }
    }

    sort(intersect, intersect + 4);

    //Round to nearest integer, away from partially covered boxes
    int minBound = (intersect[1] / cellSize + cellSize / 2 - 1e-8); //Round down to cover box
    int maxBound = (intersect[2] / cellSize - cellSize / 2);

    // Iterate between the inner intersections
    for (int j = minBound; j <= maxBound; j++) {
      //pair<double, double> coords = gridToWorldCoords(i, j);
      colQuery(i, j, functor, zPos - zSize / 2, zPos + zSize / 2);
    }
  }
}

template<class MapData>
template<class BloxelFunctor>
void GridMap<MapData>::boxModifier(double xPos, double yPos, double zPos,
    double xSize, double ySize, double zSize, double rotation,
    BloxelFunctor & functor) {
  boxQuery(xPos, yPos, zPos, xSize, ySize, zSize, rotation, functor);
}

template<class MapData>
template<class PointFunctor, class ClearFunctor>
void GridMap<MapData>::laserQuery(double xPos, double yPos, double zPos, int n,
    double startAngle, double angleStep, double maxRange, double *scans,
    PointFunctor & pointQuery, ClearFunctor & clearQuery) {
}
;

template<class MapData>
template<class PointFunctor, class ClearFunctor>
void GridMap<MapData>::laserModifier(double xPos, double yPos, double zPos,
    int n, double startAngle, double angleStep, double maxRange, double *scans,
    PointFunctor & pointModifier, ClearFunctor & clearModifier) {
}
;

// Helper for cone modification
template<class MapData>
template<class ObstacleFunctor>
inline double GridMap<MapData>::rayCollisionDistance(cv::Mat_<double> origin,
    cv::Mat_<double> direction, double maxDistance,
    const ObstacleFunctor &obstacle) const {
  pair<int, int> originIndices = worldToGridCoords(origin(0, 0), origin(1, 0));
  pair<double, double> originCellMiddle = gridToWorldCoords(
      originIndices.first, originIndices.second);

  double currentX = origin(0, 0);
  double currentY = origin(1, 0);
  double currentZ = origin(2, 0);

  double i = 0, j = 0;

  double currentDepth = 0.0;

  int stepX = direction(0, 0) > 0 ? 1 : -1;
  int stepY = direction(1, 0) > 0 ? 1 : -1;
  bool upwards = direction(2, 0) > 0 ? true : false;

  double depthPerX = 1 / direction(0, 0); //distance along ray to increase
  //x coordinate by 1. Could be INF, shouldn't matter
  double depthPerY = 1 / direction(1, 0); //distance along ray to increase
  //y coordinate by 1.

  bool noYStep = false;
  bool noXStep = false;
  if (fabs(depthPerY) > 1000.0)
    noYStep = true;
  if (fabs(depthPerX) > 1000.0)
    noXStep = true;

  if (noYStep && noXStep)
    return maxDistance;
  // Step one cell at a time in the x/y plane, whichever is the closest
  // along the ray
  while (currentDepth < maxDistance) {
    int currentBloxelID;
    // Check bloxel we're entering in the next column
    if (i + originIndices.first >= 0 && i + originIndices.first < xGrid && j
        + originIndices.second >= 0 && j + originIndices.second < yGrid) {
      const vector<Bloxel<MapData> > &column =

      getCol(i + originIndices.first, j + originIndices.second);

      for (currentBloxelID = 0; currentBloxelID < (int) column.size(); currentBloxelID++) {
        if (column[currentBloxelID].celing > currentZ) {
          if (obstacle(column[currentBloxelID].data)) {
            return currentDepth;
          }
          break;
        }
      }

    }
    //What's the x and y values of the next respective transitions?
    double nextX = originCellMiddle.first + i * cellSize + 0.5 * cellSize
        * stepX;
    double nextY = originCellMiddle.second + j * cellSize + 0.5 * cellSize
        * stepY;
    double nextZ;
    //double nextDepth;

    //How far along the line is that?
    double depthStepX = (nextX - currentX) * depthPerX;
    double depthStepY = (nextY - currentY) * depthPerY;
    double depthStep;

    double nexti = i, nextj = j;
    if (!noXStep && (noYStep || depthStepX < depthStepY)) {
      // Step X
      depthStep = depthStepX;
      nextY = currentY + depthStep * direction(1, 0);
      nextZ = currentZ + depthStep * direction(2, 0);
      //nextDepth = currentDepth + depthStep;

      nexti += stepX;
    } else {
      // Step Y
      depthStep = depthStepY;
      nextX = currentX + depthStep * direction(0, 0);
      nextZ = currentZ + depthStep * direction(2, 0);
      //nextDepth = currentDepth + depthStep;

      nextj += stepY;
    }

    if (i + originIndices.first >= 0 && i + originIndices.first < xGrid && j
        + originIndices.second >= 0 && j + originIndices.second < yGrid) {
      // Check bloxels passed through in current cell
      const vector<Bloxel<MapData> > &column = getCol(i + originIndices.first,
          j + originIndices.second);

      if (upwards) {
        for (int id = currentBloxelID; id < (int) column.size() - 1; id++) {
          if (column[id].celing > currentZ && column[id].celing < nextZ
              && obstacle(column[id + 1].data)) {
            return currentDepth + depthStep * (column[id].celing - currentZ)
                / (nextZ - currentZ);
          }
          if (column[id].celing > nextZ)
            break;
        }
      } else {
        for (int id = currentBloxelID; id >= 0; id--) {
          if (column[id].celing < currentZ && column[id].celing > nextZ
              && obstacle(column[id].data)) {
            return currentDepth + depthStep * (column[id].celing - currentZ)
                / (nextZ - currentZ);
          }
          if (column[id].celing < nextZ)
            break;
        }
      }
    }

    currentX = nextX;
    currentZ = nextZ;
    currentY = nextY;
    currentDepth += depthStep;
    i = nexti;
    j = nextj;
  }

  if (currentDepth > maxDistance)
    currentDepth = maxDistance;

  return currentDepth;
}

template<class MapData>
template<class ObstacleFunctor, class PointFunctor, class ClearFunctor>
void GridMap<MapData>::coneQuery(double xPos, double yPos, double zPos,
    double panAngle, double tiltAngle, double horizSizeAngle,
    double vertSizeAngle, double maxRange, int widthInRays, int heightInRays,
    ObstacleFunctor & obstacle, PointFunctor & pointQuery,
    ClearFunctor & clearQuery, double minDistance) {

  //will be treated as perfectly vertical.
  ConeSlicer slicer(xPos, yPos, zPos, panAngle, tiltAngle, horizSizeAngle,
      vertSizeAngle, maxRange, widthInRays, heightInRays, obstacle, *this,
      minDistance);
  pair<double, double> lowerCorner = slicer.getBoundsLower();
  pair<double, double> upperCorner = slicer.getBoundsUpper();
  pair<int, int> lowerCornerIndices = worldToGridCoords(lowerCorner.first,
      lowerCorner.second);
  pair<int, int> upperCornerIndices = worldToGridCoords(upperCorner.first,
      upperCorner.second);
  pair<double, double> lowerCornerCenter = gridToWorldCoords(
      lowerCornerIndices.first, lowerCornerIndices.second);

  double x = lowerCornerCenter.first;
  for (int i = lowerCornerIndices.first; i <= upperCornerIndices.first; i++, x
      += cellSize) {
    double y = lowerCornerCenter.second;
    for (int j = lowerCornerIndices.second; j <= upperCornerIndices.second; j++, y
        += cellSize) {
      if (i < 0 || i >= xGrid || j < 0 || j >= yGrid)
        continue;

      // if point is inside cone contour, calculate zMin and zMax
      // zMax is the minimum z-value of all possible celing planes
      // in the same way zMin is maximum value of all floor planes
      if (slicer.startColumn(x, y)) {
        // Keep track of upper and lower bounds on the z-level
        double zUpper = slicer.getCurrentZ();
        double zLower = slicer.getCurrentZ();

        while (!slicer.isFinished()) {
          // Whether this chunk falls inside the clipping range of
          // the current ray
          bool wasInsideClip;
          // Lower bound of current update
          double nextZ;

          // Trace next ray, get occlusion and next lower z-bound
          slicer.getNextBoundary(wasInsideClip, nextZ);
          // We "save up" intervals until we hit something that is occuluded or cone outside
          // Then we modify our saved up interval
          if (!wasInsideClip || slicer.isFinished()) {
            if (wasInsideClip) {
              // If end is hit, but interval is still included, add it aswell
              zLower = nextZ;
            }
            if (zUpper > zLower + minAcceptedBloxel()) {
              colQuery(i, j, clearQuery, zLower, zUpper);
            }
            zUpper = nextZ;
          }
          zLower = nextZ;
        }
        /* BACKUP old code with bottom rounding errors
         // Whether this chunk falls inside the clipping range of
         // the current ray
         bool wasInsideClip = false; 

         // Lower bound of current update
         double nextZ;

         slicer.getNextBoundary(wasInsideClip, nextZ);

         if (wasInsideClip) {
         iteratorHelper(column, nextZ, currentZ, clearQuery,
         x, y);
         }

         currentZ = nextZ;*/
      }
    }
  }
}
;

template<class MapData>
template<class ObstacleFunctor, class PointFunctor, class ClearFunctor>
void GridMap<MapData>::coneModifier(double xPos, double yPos, double zPos,
    double panAngle, double tiltAngle, double horizSizeAngle,
    double vertSizeAngle, double maxRange, int widthInRays, int heightInRays,
    ObstacleFunctor & obstacle, PointFunctor & pointModifier,
    ClearFunctor & clearModifier, double minDistance) {
  coneQuery(xPos, yPos, zPos, panAngle, tiltAngle, horizSizeAngle,
      vertSizeAngle, maxRange, widthInRays, heightInRays, obstacle,
      pointModifier, clearModifier, minDistance);
}
;

template<class MapData>
inline std::ostream& operator <<(std::ostream& stream,
    const GridMap<MapData>& map) {
  map.writeStream(stream);
  return stream;
}

template<class MapData>
inline std::istream& operator >>(std::istream& stream, GridMap<MapData>& map) {
  map.readStream(stream);
  return stream;
}

}
; // End of namespace

#endif
