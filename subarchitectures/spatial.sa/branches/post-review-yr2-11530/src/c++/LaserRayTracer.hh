#ifndef LASERRAYTRACER_H
#define LASERRAYTRACER_H

#include "SpatialGridMap.hh"
#include "GridDataFunctors.hh"
#include <vector>
#include <cmath>
namespace SpatialGridMap{
  
  /* For laser ray tracing operations on BloxelMap. The reason this is a class is 
     one day we may have tilting laser scanners offering more functionality. For now
   * we only have the 2D case where a laser scanner has a fixed height. Alper Aydemir 24-Jun-2010
  */
  
  template <class MapData>
  class LaserRayTracer {
    public:
      LaserRayTracer(GridMap<MapData>* map, double maxlen);
     
      // laserpose = [x y z theta]
      //template <class ObstacleFunctor>
      //void addScanStationarySensor(const Laser::Scan2d &castScan, const ObstacleFunctor &obstacle,double* LaserPose);
      
//       template <class CheckObstacleFunctor, class MakeEmptyFunctor, class MakeObstacleFunctor> 
      //void addScanStationarySensor(const Laser::Scan2d &castScan, vector<double> LaserPose, CheckObstacleFunctor &checkobstacle, MakeEmptyFunctor &makeempty, MakeObstacleFunctor &makeobstacle);
      void addScanStationarySensor(const Laser::Scan2d &castScan, vector<double> LaserPose, GDIsObstacle &checkobstacle, GDMakeFree &makeempty, GDMakeObstacle &makeobstacle);
    private:
      void setRayStart(double x, double y, double theta);
      int (LaserRayTracer<MapData>::* drawLineFcn)(void);
      int stepRay();
      int isOutside() const;
      double getDistToStart() const;
      /// Draw a line between -45 and +45 degs or 135 and 225 degs
      int drawLineXp(void);

      /// Draw a line between 135 and 225 degs
      int drawLineXn(void);

      /// Draw a line between 45 and 135 degs
      int drawLineYp(void);

      /// Draw a line between -135 and -45 degs
      int drawLineYn(void);

      double maxRayLenght,secondaryStep,cellSize;
      double m_Xf,m_Yf,m_Xs,m_Ys;
      int m_Xi,m_Yi;
      pair <double,double> mapCenterW;
      pair <int,int> mapSize;
      SpatialGridMap::GridMap<MapData>* m_map;
  };

   template <class MapData>
     LaserRayTracer<MapData>::LaserRayTracer(GridMap<MapData>* map, double maxlen)
     : maxRayLenght(maxlen), m_map(map){
       mapCenterW = m_map->getCentW();
       cellSize = m_map->getCellSize();
       mapSize = m_map->getMapSize();
     }
 

template <class MapData> int
  LaserRayTracer<MapData>::isOutside() const
  {
    if ((m_Xi < 0 || (mapSize.first < m_Xi) ||
	(m_Yi < 0 || (mapSize.second < m_Yi))))
      return true;
    else
      return false;
  }


template <class MapData> double
  LaserRayTracer<MapData>::getDistToStart() const
  {
      return hypot(m_Yf-m_Ys, m_Xf-m_Xs) * cellSize;
  }

  template <class MapData>
    void LaserRayTracer<MapData>::setRayStart(double x, double y, double theta){
   
      double TWOPI = 6.28318530717959;

      double tmp = theta;

        // Make sure that it is larger than or equal to 0
        while (tmp < 0)
	      tmp += TWOPI;

	  // Make sure that it is less than 2*pi
	  while (tmp > TWOPI)
	        tmp -= TWOPI;

      theta = tmp;
      //printf("theta is %f \n",theta);
      m_Xf = (x - mapCenterW.first); /// cellSize;
      m_Yf = (y - mapCenterW.second); /// cellSize;
      
      m_Xf = m_Xf + ((mapSize.first * cellSize)/2.0);
      m_Yf = m_Yf + ((mapSize.second * cellSize)/2.0);
      m_Xf = m_Xf / cellSize;
      m_Yf = m_Yf / cellSize;

      m_Xi = (m_Xf<0 ? int(m_Xf-0.5) : int(m_Xf+0.5));
      m_Yi = (m_Yf<0 ? int(m_Yf-0.5) : int(m_Yf+0.5));

       m_Xs = m_Xf;
       m_Ys = m_Yf;

      if ((0<=theta) && (theta<M_PI/4.0)) {
	drawLineFcn = &(LaserRayTracer::drawLineXp);
	secondaryStep = tan(theta);
      } else if ((M_PI/4.0<=theta) && (theta<3.0*M_PI/4.0)) {
	drawLineFcn = &(LaserRayTracer::drawLineYp);
	secondaryStep = 1/tan(theta);
      } else if ((3.0*M_PI/4.0<=theta) && (theta<5.0*M_PI/4.0)) {
	drawLineFcn = &(LaserRayTracer::drawLineXn);
	secondaryStep = tan(theta);
      } else if ((5.0*M_PI/4.0<=theta) && (theta<7.0*M_PI/4.0)) {
	drawLineFcn = &(LaserRayTracer::drawLineYn);
	secondaryStep = 1/tan(theta);
      } else {
	drawLineFcn = &(LaserRayTracer::drawLineXp);
	secondaryStep = tan(theta);
      }
    
    }

  template <class MapData> int
    LaserRayTracer<MapData>::stepRay()
    {
      //printf("stepping ray \n");
      return (this->*drawLineFcn)();
    }

  template <class MapData> int
    LaserRayTracer<MapData>::drawLineXp()
  {
    m_Xi++;
    m_Xf++;
    m_Yf += secondaryStep;
    m_Yi = (0<m_Yf ? int(m_Yf+0.5) : int(m_Yf-0.5));

    // Don't move outside the map
    if ((m_Xi < -mapSize.first) || (mapSize.first < m_Xi) ||
	(m_Yi < -mapSize.second) || (mapSize.second < m_Yi))
      return true;
    else
      return false;

  }

  template <class MapData> int
    LaserRayTracer<MapData>::drawLineXn()
    {
      m_Xi--;
      m_Xf--;
      m_Yf -= secondaryStep;
      m_Yi = (0<m_Yf ? int(m_Yf+0.5) : int(m_Yf-0.5));
      // Don't move outside the map
      if ((m_Xi < -mapSize.first) || (mapSize.first < m_Xi) ||
	  (m_Yi < -mapSize.second) || (mapSize.second < m_Yi))
	return true;
      else
	return false;

    
    }

  template <class MapData> int
    LaserRayTracer<MapData>::drawLineYp()
    {
      m_Yi++;
      m_Yf++;
      m_Xf += secondaryStep;
      m_Xi = (0<m_Xf ? int(m_Xf+0.5) : int(m_Xf-0.5));
      // Don't move outside the map
      if ((m_Xi < -mapSize.first) || (mapSize.first < m_Xi) ||
	  (m_Yi < -mapSize.second) || (mapSize.second < m_Yi))
	return true;
      else
	return false;

    }

  template <class MapData> int
    LaserRayTracer<MapData>::drawLineYn()
    {
      m_Yi--;
      m_Yf--;
      m_Xf -= secondaryStep;
      m_Xi = (0<m_Xf ? int(m_Xf+0.5) : int(m_Xf-0.5));

      // Don't move outside the map
      if ((m_Xi < -mapSize.first) || (mapSize.first < m_Xi) ||
	  (m_Yi < -mapSize.second) || (mapSize.second < m_Yi))
	return true;
      else
	return false;
    }

  template <class MapData>
//    template <class CheckObstacleFunctor, class MakeEmptyFunctor, class MakeObstacleFunctor>
  //void LaserRayTracer<MapData>::addScanStationarySensor(const Laser::Scan2d &castScan, vector<double> LaserPose, CheckObstacleFunctor &checkobstacle, MakeEmptyFunctor &makeempty, MakeObstacleFunctor &makeobstacle){
  void LaserRayTracer<MapData>::addScanStationarySensor(const Laser::Scan2d &castScan, vector<double> LaserPose, GDIsObstacle &checkobstacle, GDMakeFree &makeempty, GDMakeObstacle &makeobstacle){
    if (LaserPose.size() != 4){
      printf("Not adding scan, laser pose is incomplete! (should be [x,y,z,theta] \n");
      return;
    }
    //printf("LaserPose: %f, %f, %f, %f \n",LaserPose[0],LaserPose[1],LaserPose[2],LaserPose[3]);
    //printf("minimum bloxel height %f \n",m_map->getMinBloxelHeight());
    for (unsigned int i=0; i < castScan.ranges.size(); i++){
     // printf("setting ray start %d i\n",i);
      setRayStart(LaserPose[0],
	  	  LaserPose[1],
		  LaserPose[3] + castScan.startAngle + castScan.angleStep * i);
      while (!isOutside() &&
	  getDistToStart() < castScan.ranges[i] &&
	  getDistToStart() < maxRayLenght) {

	//TODO: Check if the bloxel at height=z is occupied, if so call EmptierFunctor
	//data() = '0';

	typedef vector<Bloxel<MapData> > MapDataColumn;
	MapDataColumn &column = (*m_map)(m_Xi, m_Yi);
//	for(MapDataColumn::iterator it = column.begin(); it != column.end(); it++){
	for(unsigned int i = 0; i < column.size(); i++){

	  //	  printf("grid cell (%d, %d) has %d bloxels, celing: %f data: %d \n",m_Xi,m_Yi,(*m_map)(m_Xi,m_Yi).size(),it->celing, int(it->data)); 

	  if (LaserPose[2] < column[i].celing){
	    //this is the bloxel that our point is in
	    if(checkobstacle(column[i].data)){
	      //if this is an obstacle carve an empty box in it of minbloxel size
	      //printf("obstacle!, delete it %d, %d \n",m_Xi,m_Yi);
	      m_map->boxSubColumnModifier(m_Xi,m_Yi,LaserPose[2],m_map->getMinBloxelHeight(),makeempty);
	    }
	    break;
	  }
	}
	stepRay();
      }
      if (!isOutside() && getDistToStart() < maxRayLenght) {
	//TODO: Check if the bloxel at height=z is free, if so call ObstacleFunctor
	//data() = '1';
	//for(typename vector<Bloxel<MapData> >::iterator it = (*m_map)(m_Xi,m_Yi).begin(); it != (*m_map)(m_Xi,m_Yi).end(); it++){
	
	//printf("column size: %d \n",(*m_map)(m_Xi,m_Yi).size());
	unsigned int colsize = (*m_map)(m_Xi,m_Yi).size();
	for (unsigned int i = 0; i< colsize; i++){
	 // printf("i : %d \n",i);
	//  printf("column size: %d \n",(*m_map)(m_Xi,m_Yi).size());
	if (LaserPose[2] < (*m_map)(m_Xi,m_Yi)[i].celing){
	    //this is the bloxel that our point is in
	    if(!checkobstacle((*m_map)(m_Xi,m_Yi)[i].data)){
	  //    printf("free space!, add obstacle %d, %d \n",m_Xi,m_Yi);
	      //if this is an obstacle carve an empty box in it of minbloxel size
	      m_map->boxSubColumnModifier(m_Xi,m_Yi,LaserPose[2],m_map->getMinBloxelHeight(),makeobstacle);
	    }
	  }
	  break;

	}

      }

    
    }

  }

};
#endif

