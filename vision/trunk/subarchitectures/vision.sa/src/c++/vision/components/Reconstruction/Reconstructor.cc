/**
 * @author Kai Zhou
 * @date April 2009
 */

#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <VideoUtils.h>
#include "Reconstructor.h"
#include "System.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::Reconstructor();
  }
}

namespace cast
{

using namespace std;
using namespace VisionData;

void Reconstructor::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
 
  // first let the base classes configure themselves
  configureVideoCommunication(_config);

  if((it = _config.find("--camid")) != _config.end())
  {
    istringstream istr(it->second);
    istr >> camId;
  }
}

void Reconstructor::start()
{
  startVideoCommunication(*this);
}

void Reconstructor::runComponent()
{
	System s;
	while(isRunning())
	{
	//////////////////////////////////////////////////////////////////////////
		Video::Image image;
		getImage(camId, image);
		IplImage *iplImage = convertImageToIpl(image);
		s.Run(iplImage);

		if (s.para_A!=0.0 || s.para_B!=0.0 || s.para_C!=0.0 || s.para_D!=0.0)
		{
			CurrentObjList.clear();

			for(unsigned int i=0; i<s.WMcenter.size(); i++)
			{
				ObjPara OP;
				OP.c = s.WMcenter.at(i);
				OP.s = s.WMsize.at(i);
				OP.r = s.WMradius.at(i);
				OP.id = "";
				CurrentObjList.push_back(OP);
			}

			if (PreviousObjList.empty())
			{
				for(unsigned int i=0; i<CurrentObjList.size(); i++)
				{
					CurrentObjList.at(i).id = newDataID();
					SOIPtr obj = createObj(CurrentObjList.at(i).c, CurrentObjList.at(i).s, CurrentObjList.at(i).r);
					addToWorkingMemory(CurrentObjList.at(i).id, obj);
				}
				PreviousObjList = CurrentObjList;
			}
			else
			{
				std::vector <int> newObjList; //store the serial number of new objects in CurrentObjList
				for(unsigned int i=0; i<CurrentObjList.size(); i++)
				{
					bool flag = false;
					for(unsigned int j=0; j<PreviousObjList.size(); j++)
					{
						if(Compare2SOI(CurrentObjList.at(i), PreviousObjList.at(j)))// if these two objects were the same one
						{
							flag = true;
							SOIPtr obj = createObj(CurrentObjList.at(i).c, CurrentObjList.at(i).s, CurrentObjList.at(i).r);
							overwriteWorkingMemory(PreviousObjList.at(j).id, obj);
							break;
						}
					}
					if (!flag) //there might be some new objects
						newObjList.push_back(i);
				}
				if(!newObjList.empty())
				{
					for(unsigned int i=0; i<newObjList.size(); i++)// add all new objects
					{
						CurrentObjList.at(newObjList.at(i)).id = newDataID();
						SOIPtr obj = createObj(CurrentObjList.at(newObjList.at(i)).c, CurrentObjList.at(newObjList.at(i)).s, CurrentObjList.at(newObjList.at(i)).r);
						addToWorkingMemory(CurrentObjList.at(newObjList.at(i)).id, obj);
						PreviousObjList.push_back(CurrentObjList.at(newObjList.at(i)));//update PreviousObjList
					}
				}
				if (PreviousObjList.size()!=CurrentObjList.size()) //need to delete the disappeared objects
				{
					std::vector <int> disappearedObjList; //store the serial number of disappeared objects in PreviousObjList
					for(unsigned int i=0; i<PreviousObjList.size(); i++)
					{
						bool flag = false;
						for(unsigned int j=0; j<CurrentObjList.size(); j++)
						{
							if(Compare2SOI(CurrentObjList.at(j), PreviousObjList.at(i)))// if these two objects were the same
							{
								flag = true;
								break;
							}
						}
						if (!flag) //this is a disappeared object
							disappearedObjList.push_back(i);
					}
					if(!disappearedObjList.empty())
					{
						for(unsigned int i=0; i<disappearedObjList.size(); i++)// delete all new objects
						{
							deleteFromWorkingMemory(PreviousObjList.at(disappearedObjList.at(i)).id);
							PreviousObjList.erase(PreviousObjList.begin()+disappearedObjList.at(i));
						}

					}

				}
			
			}
		}
		//cout<<"number of SOIs = "<<PreviousObjList.size()<<endl;
		cvReleaseImage(&iplImage);
		//////////////////////////////////////////////////////////////////////////
		// wait a bit so we don't hog the CPU
		sleepComponent(100);
	}
}

SOIPtr Reconstructor::createObj(Vector<3> center, Vector<3> size, double radius)
{
	VisionData::SOIPtr obs = new VisionData::SOI;
	obs->boundingBox.pos.x = obs->boundingSphere.pos.x = center[0];
	obs->boundingBox.pos.y = obs->boundingSphere.pos.y = center[1];
	obs->boundingBox.pos.z = obs->boundingSphere.pos.z = center[2];
	obs->boundingBox.size.x = size[0];
	obs->boundingBox.size.y = size[1];
	obs->boundingBox.size.z = size[2];
	obs->boundingSphere.rad = radius;
	
	return obs;
}

bool Reconstructor::Compare2SOI(ObjPara obj1, ObjPara obj2)
{
	if (sqrt((obj1.c[0]-obj2.c[0])*(obj1.c[0]-obj2.c[0])+(obj1.c[1]-obj2.c[1])*(obj1.c[1]-obj2.c[1])+(obj1.c[2]-obj2.c[2])*(obj1.c[2]-obj2.c[2]))<0.25*obj1.r && obj1.r/obj2.r>0.5 && obj1.r/obj2.r<2)
		return true; //the same object
	else	
		return false; //not the same one
}

}

