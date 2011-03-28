#ifndef PB_VISUALIZATION_H
#define PB_VISUALIZATION_H

#include <peekabot.hh>
#include <peekabot/Types.hh>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include "SpatialGridMap.hh"
#include "GridMapData.hh"
#include "GridMapWrapper.hh"
#include <vector>
#include <iostream>
#include <sstream>
#include <Navigation/LocalGridMap.hh>
#include "Math/BinaryMatrix.hh"
#include <Navigation/LocalGridMap.hh>

class VisualPB_Bloxel{
  public:
    VisualPB_Bloxel(std::string PB_Hostname, int PB_Port,int PB_xSize,int PB_ySize, double PB_CellSize, double PB_scale, bool replaceMap = false);
    ~VisualPB_Bloxel();
  private:
    // Do not use
    VisualPB_Bloxel(const VisualPB_Bloxel &);
    VisualPB_Bloxel & operator=(const VisualPB_Bloxel &);
  public:
    bool connectPeekabot();
    template<class MapData>
      void DisplayMap(const SpatialGridMap::GridMap<MapData> &map, std::string name = "boxmap" );
    template<class MapData>
      void DisplayPCMap(const SpatialGridMap::GridMap<MapData> &map,double cellsize);

    void Display2DCureMap(const Cure::LocalGridMap<unsigned char>* curemap,std::string name = "og2d", bool drawNew = false);
    void Display2DBinaryMap(const Cure::BinaryMatrix &binmap,const Cure::LocalGridMap<unsigned char>* curemap, std::string name = "og2d_binarymatrix");
    void Add3DPointCloud(std::vector< Cure::Pose3D > point, bool ishidden = true, std::string name = "extrapoints");
    
    template<class MapData>
    void AddPDF (SpatialGridMap::GridMap<MapData> &map, bool drawNew = false);
    
    double scale,CellSize;
    int xSize,ySize;
  private:
    
    peekabot::GroupProxy m_2DOccGridProxy;


    bool isReplaceMap;
    peekabot::PeekabotClient client;
    std::string host;
    int port;
    peekabot::GroupProxy pb_map;
    char * obsCount;

    //peekabot::ObjectProxy root;

    peekabot::GroupProxy pdf;
    int maxLevels;
    int lastCutLevel;
    std::vector <peekabot::GroupProxy> pdflevels;
    std::vector <double> thresholds;
    SpatialGridMap::GridMap<SpatialGridMap::GridMapWrapper<char> > pdfCache;
    //peekabot::OccupancyGrid3DProxy OGProxy;
};

void VisualPB_Bloxel::Display2DCureMap(const Cure::LocalGridMap<unsigned char>* curemap, std::string name, bool drawNew){
  peekabot::OccupancySet2D cells;
  peekabot::OccupancyGrid2DProxy og2d;
  peekabot::GroupProxy local2DOccGridProxy = m_2DOccGridProxy;

  if (drawNew){
	  peekabot::GroupProxy temp2DOccGridProxy;
	  temp2DOccGridProxy.add(client, "combined_placemap2D", peekabot::AUTO_ENUMERATE_ON_CONFLICT);
  local2DOccGridProxy = temp2DOccGridProxy;
  }

  og2d.add(local2DOccGridProxy,name,curemap->getCellSize(), peekabot::REPLACE_ON_CONFLICT);
  og2d.translate(0,0,-0.10,peekabot::LOCAL_COORDINATES);
  og2d.set_unoccupied_color(0.5,0.5,0.5);
  og2d.set_occupied_color(0,0,0);

  for (int x = -curemap->getSize(); x < curemap->getSize(); x++){
    for (int y = -curemap->getSize(); y< curemap->getSize(); y++){
      double xw,yw;
      curemap->index2WorldCoords(x,y,xw,yw);
     if ((*curemap)(x,y) == '1'){
	cells.set_cell(xw,yw,1);
      }
      else if ((*curemap)(x,y) == '0'){
	cells.set_cell(xw,yw,0);
      }
      else if ((*curemap)(x,y) == '0'){
    		cells.set_cell(xw,yw,0.5);
      }
    }
  }
  og2d.set_cells(cells);
  client.sync();
}
void VisualPB_Bloxel::Display2DBinaryMap(const Cure::BinaryMatrix &binmap, const Cure::LocalGridMap<unsigned char>* curemap, std::string name){
  peekabot::OccupancyGrid2DProxy og2d;
 peekabot::OccupancySet2D cells;
 og2d.add(client, name , curemap->getCellSize(),0.3,0.3,0,1,0,0,peekabot::REPLACE_ON_CONFLICT); 
 //og2d.translate(0,0,-0.005);
 
 for (int x = -curemap->getSize(); x < curemap->getSize(); x++){
   for (int y = -curemap->getSize(); y< curemap->getSize(); y++){
     double xw,yw;
     curemap->index2WorldCoords(x,y,xw,yw);
     if ((binmap)(x+ curemap->getSize(),y+curemap->getSize())){
	cells.set_cell(xw,yw,1);
      }
      else{
	cells.set_cell(xw,yw,0);
      }
    }
  }
   og2d.set_cells(cells);
    client.sync();
}


VisualPB_Bloxel::VisualPB_Bloxel(std::string PB_Hostname, int PB_Port,int PB_xSize
    ,int PB_ySize, double PB_CellSize, double PB_scale, bool replaceMap) 
    : pdfCache(PB_xSize, PB_ySize, PB_CellSize, 1e-8, -1e10, 1e10, 0, 0, 0, -1){
      // TODO map may need better bounds
  host = PB_Hostname;
  port = PB_Port;
  xSize = PB_xSize;
  ySize = PB_ySize;
  CellSize = PB_CellSize;
  scale = PB_scale;
  scale = 1;
  isReplaceMap = replaceMap;
  lastCutLevel = -1;
  obsCount = (char*) malloc(sizeof(char)*xSize*ySize);
  for(int i=0; i<xSize*ySize; i++){
     obsCount[i] = 0;
  }
}

VisualPB_Bloxel::~VisualPB_Bloxel(){
   free(obsCount);
}

bool VisualPB_Bloxel::connectPeekabot(){
  try{
    printf("Trying to connect to Peekabot on host %s and port %d \n", host.c_str(), port);
    client.connect(host, port);
    printf("Connection to Peekabot established! \n");
    // Assign root
    pb_map.add(client,"map",peekabot::REPLACE_ON_CONFLICT);
    
    pdf.add(client, "pdf", peekabot::AUTO_ENUMERATE_ON_CONFLICT);
    m_2DOccGridProxy.add(client, "combined_placemap2D", peekabot::AUTO_ENUMERATE_ON_CONFLICT);
    
    // Add 15 levels for pdf-visualization
    maxLevels = 15;
    thresholds.push_back(1.0);
    for(int i=0; i<maxLevels; i++){
      thresholds.push_back(thresholds.back()/2);
      
      char buf[256];
      sprintf(buf, "distribution_%4.5f_%4.5f", thresholds[i],thresholds[i+1]);
      std::string s(buf);
      std::replace(s.begin(),s.end(),'.',',');
      strcpy(buf,s.c_str());
      
      peekabot::GroupProxy level;
      level.add(pdf, buf, peekabot::REPLACE_ON_CONFLICT);
      pdflevels.push_back(level);
    }
    
    // Add GridMap for pdf-tracking
    
    //  OGProxy.add(client, "root.og3d", 0.1, peekabot::REPLACE_ON_CONFLICT);
    /* peekabot::CubeProxy cube;
       cube.add(pb_map, std::string("cube"), peekabot::AUTO_ENUMERATE_ON_CONFLICT);
       cube.set_position(-1,-1, 0);
       cube.set_scale(scale,scale,scale);*/
    
  }
  catch(std::exception& e){
    printf("Caught exception when connecting to peekabot (%s). \n", e.what());
    client.disconnect();
    //nah: wtf! isn't this why you have a return value?!
    //exit(-1);
    return false;
  }
  return true;
}

/*
template <class MapData>
void VisualPB_Bloxel::DisplayPCMap(const SpatialGridMap::GridMap<MapData> &map, double cellsize){
  std::cout << "Displaying Map.." << std::endl;
  try{
     peekabot::OccupancyGrid3DProxy ogproxy;
    ogproxy.add(client, "root.og3d", cellsize, peekabot::REPLACE_ON_CONFLICT);
     typedef SpatialGridMap::Bloxel<MapData> MapBloxel;
     peekabot::OccupancySet3D cells;
    for (int x  = 0 ; x < xSize; x++){
      for (int y = 0; y < ySize; y++){
	std::pair<double,double> coord = map.gridToWorldCoords(x, y);
	double bloxel_floor = 0;
	// for each bloxel sitting on this 2D cell
	for(typename std::vector<MapBloxel>::const_iterator it = map(x,y).begin(); it != map(x,y).end(); it++){
	  //set_position moves the center of cube to a defined position and adds a 1mx1mx1mx cube
	  // the cube's center point z coordinate is
	  // bloxel_floor + (ceiling - bloxel_floor)/2) = (b+c)/2
	  for (double z = bloxel_floor + cellsize/2; z < it->celing; z = z + cellsize){
	    //std::cout << "z, floor, celing, cellsize: " << z << "," << bloxel_floor << "," << it->celing << "," << cellsize << std::endl;
	  if(it->data.occupancy == SpatialGridMap::OCCUPIED){
	    cells.set_cell(coord.first,coord.second, z,1);
	  }
	  }
	  //next bloxel's floor is this one's ceiling
	  bloxel_floor = it->celing;
	}
      }
    }
    std::cout << "synching.." << std::endl;
    ogproxy.set_cells(cells);
    client.sync();

  }
  catch(std::exception &e){
    printf("Caught exception %s: \n", e.what());
    client.disconnect();
    exit(-1);
  }
}
*/

void VisualPB_Bloxel::Add3DPointCloud(std::vector< Cure::Pose3D > point, bool ishidden, std::string name){
  try{
    peekabot::OccupancyGrid3DProxy ogproxy;
    ogproxy.add(client, name, 0.05, 0.05, peekabot::REPLACE_ON_CONFLICT);
    peekabot::OccupancySet3D cells;
    for (unsigned int i = 0; i < point.size(); i++){
      cells.set_cell(point[i].getX(),point[i].getY(), point[i].getZ(),1);
    }

    ogproxy.set_cells(cells);
    if(ishidden)
      ogproxy.hide();
    client.sync();
  }

  catch(std::exception &e){
    printf("Caught exception %s: \n", e.what());
    client.disconnect();
    exit(-1);
  }
}

int fallsWhere(std::vector < std::pair<double,double> > thresholdvalues, double value){

  for (unsigned int i = 0; i < thresholdvalues.size(); i++){
    if (value >= thresholdvalues[i].first && value <= thresholdvalues[i].second)
      return i;
  }
  return -1;
}

template<class MapData>
void VisualPB_Bloxel::AddPDF (SpatialGridMap::GridMap<MapData> &map, bool drawNew){
  try{
	  std::vector <peekabot::GroupProxy> localpdflevels = pdflevels;
	  if (drawNew){
	  peekabot::GroupProxy pdftemp;
	  std::vector <peekabot::GroupProxy> pdflevelstemp;

	  pdftemp.add(client, "pdf", peekabot::AUTO_ENUMERATE_ON_CONFLICT);
	     // Add 15 levels for pdf-visualization
	     maxLevels = 15;
	     thresholds.push_back(1.0);
	     for(int i=0; i<maxLevels; i++){
	       thresholds.push_back(thresholds.back()/2);

	       char buf[256];
	       sprintf(buf, "distribution_%4.5f_%4.5f", thresholds[i],thresholds[i+1]);
	       std::string s(buf);
	       std::replace(s.begin(),s.end(),'.',',');
	       strcpy(buf,s.c_str());
	       peekabot::GroupProxy level;
	       level.add(pdftemp, buf, peekabot::REPLACE_ON_CONFLICT);
	       pdflevelstemp.push_back(level);
	     }
	     localpdflevels = pdflevelstemp;
	  }
    std::cout << "Displaying PDF" << std::endl;
    std::cout << "Map Size: " << map.getMapSize().first << " " << map.getMapSize().second << endl;
    typedef SpatialGridMap::Bloxel<MapData> MapBloxel;
    typedef SpatialGridMap::Bloxel<SpatialGridMap::GridMapWrapper<char> > VisData;

    // Count bloxels in each layer
    int counters[20];
    memset(counters, 0, sizeof(int)*20);
    for (int x  = 0 ; x < map.getMapSize().first; x++){
       for (int y = 0; y < map.getMapSize().second; y++){
          for(typename std::vector<MapBloxel>::const_iterator it = map(x,y).begin(); it != map(x,y).end();it++){
             double val = it->data.pdf;
             if(val <= thresholds[0]){
                for(int i=1; i<maxLevels+1; i++){
                   if(val > thresholds[i]){
                      counters[i-1]++;
                      break;
                   }
                }
             }
          }
       }
    }
    int maxBloxels = 9000;
    // Add layers until above maxbloxels
    int count = 0;
    int cutLevel = -1;
    for(int i=0; i<maxLevels; i++){
      count += counters[i];
      if(count <= maxBloxels)
	cutLevel++;
    }
    if(cutLevel < 0){
      cerr << "Over " << maxBloxels << " (" << counters[0] << ") bloxels in first layer, no layers will be visualized";
      return;
    }

    // Check if level has changed, if not we can ignore some operations
    bool levelChanged = false;
    if(cutLevel != lastCutLevel){
      levelChanged = true;
    }
    lastCutLevel = cutLevel;
    stringstream nameBuilder;
    double colorval = 0;
    pair<double,double> bounds = map.getZBounds();
    for (int x  = 0 ; x < map.getMapSize().first; x++){
      for (int y = 0; y < map.getMapSize().second; y++){
	if(levelChanged || map.isDirty(x,y)){ // If column or level has been changed
	  std::pair<double,double> coord = map.gridToWorldCoords(x, y);

	  // first we build a potential visualization data
	  vector<VisData> col;
	  double bloxel_floor = bounds.first;
	  col.push_back(VisData(-1, bloxel_floor));
	  for(typename std::vector<MapBloxel>::const_iterator it = map(x,y).begin(); it != map(x,y).end();it++){

	    // find out what layer this bloxel should be in, if any
	    int level = -1;
	    if(it->data.pdf <= thresholds[0]){
	      for(int i=1; i<=cutLevel+1; i++){
		if(it->data.pdf > thresholds[i]){
		  level = i-1;
                         break;
                      }
                   }
                }
                col.push_back(VisData(level, it->celing));

	     }
	    // compare column with cache from previous edit
	     bool unchanged = true;
	     vector<VisData> & cache = pdfCache(x,y);
             if(col.size() != cache.size()){
                unchanged = false;
             } else {
                for(unsigned int i=0; i<col.size(); i++){
                   if(col[i].data.data != cache[i].data.data || col[i].celing != cache[i].celing){
                      unchanged = false;
                      break;
                   }
                }
             }

             // if changed send column to peekabot
             if(!unchanged){
                // THIS IS A HACK!
                // Redo everything we did last time, but then remove the object
                // Can probably be improved a lot
                // (Having a column proxy for each level did not work that well however, was very slow)
                int number = 0;
                for(std::vector<VisData>::const_iterator it = cache.begin(); it != cache.end(); it++){
                   // Add all bloxels that have an associated layer
                   if(it->data.data >= 0){
                      nameBuilder.str(""); //reset name
                      nameBuilder << x << "_" << y << "_" << number; //add coords and count for unique name

                      peekabot::CubeProxy cube;
                      cube.add(localpdflevels[it->data.data],nameBuilder.str().c_str(),peekabot::REPLACE_ON_CONFLICT);
                      cube.remove();

                      number++;
                   }
                   bloxel_floor = it->celing;
                }

                //save cache for next iteration
                cache = col;

                // Add the new values for this colum
                number = 0;
                for(std::vector<VisData>::const_iterator it = col.begin(); it != col.end(); it++){
                   // Add all bloxels that have an associated layer
                   if(it->data.data >= 0){
                      nameBuilder.str(""); //reset name
                      nameBuilder << x << "_" << y << "_" << number; //add coords and count for unique name

                      peekabot::CubeProxy cube;
                      cube.add(localpdflevels[it->data.data],nameBuilder.str().c_str(),peekabot::REPLACE_ON_CONFLICT);
                      cube.set_position(coord.first,coord.second, (bloxel_floor+it->celing)/2);
                      cube.set_scale(CellSize,CellSize,(it->celing-bloxel_floor));
                      colorval = (double)(it->data.data) / maxLevels;
                      cube.set_color(0+ colorval, 0, 1- colorval, true);

                      number++;
                   }
                   bloxel_floor = it->celing;
                }
             }
          }
       }
    }

    std::cout << "syncing" << std::endl;
    client.sync();
  }

  catch(std::exception &e){
     printf("Caught exception %s: \n", e.what());
     client.disconnect();
     exit(-1);
  }

}



template <class MapData>
void VisualPB_Bloxel::DisplayMap(const SpatialGridMap::GridMap<MapData> &map, std::string name){
  typedef SpatialGridMap::Bloxel<MapData> MapBloxel;
  printf("displaying map... %f\n",CellSize);

  stringstream nameBuilder;
  std::pair<int,int> pairs = map.getMapSize();
  try{
     for (int x  = 0 ; x < pairs.first; x++){
        for (int y = 0; y < pairs.second; y++){
           //if(map.isDirty(x,y)){

              // Remove all old values
              // THIS IS A HACK!
              // Remove all objects that exist in column, rather slow
              // Can probably be improved a lot
              // (Having a column proxy for each level did not work that well however, was very slow)
              for(int i=0; i < obsCount[x*ySize + y]; i++){
                 peekabot::CubeProxy cube;
                 nameBuilder.str(""); //reset name
                 nameBuilder << x << "_" << y << "_" << i; //add coordinates and count for unique name
                 cube.add(pb_map, nameBuilder.str().c_str(), peekabot::REPLACE_ON_CONFLICT);
                 cube.remove();
              }

              std::pair<double,double> coord = map.gridToWorldCoords(x, y);
              double bloxel_floor = 0;

              // for each bloxel sitting on this 2D cell
              int number = 0;
              for(typename std::vector<MapBloxel>::const_iterator it = map(x,y).begin(); it != map(x,y).end(); it++, number++){

                 if(it->data.occupancy == SpatialGridMap::OCCUPIED){
                    peekabot::CubeProxy cube;
                    nameBuilder.str(""); //reset name
                    nameBuilder << x << "_" << y << "_" << number; //add coordinates and count for unique name

                    cube.add(pb_map, nameBuilder.str().c_str(), peekabot::REPLACE_ON_CONFLICT);
                    //	  std::cout << buf << ": " << bloxel_floor << ", " << it->celing << std::endl;
                    //set_position moves the center of cube to a defined position and adds a 1mx1mx1mx cube
                    // the cube's center point z coordinate is
                    // bloxel_floor + (ceiling - bloxel_floor)/2) = (b+c)/2
                    cube.set_position(coord.first,coord.second, (bloxel_floor+it->celing)/2);

                    //then we set the scale..
                    cube.set_scale(CellSize,CellSize,(it->celing-bloxel_floor));
                 }
                 //next bloxel's floor is this one's ceiling
                 bloxel_floor = it->celing;
              }
              obsCount[x*ySize + y] = number;
           }
        }
     //}
     client.sync();
     client.flush();
  }
  catch(std::exception &e){
    printf("Caught exception %s: \n", e.what());
    client.disconnect();
    exit(-1);
  }

}
#endif
