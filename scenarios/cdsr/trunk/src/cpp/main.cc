/* ============================================================
 *
 * This file is based on an example from the RSB project
 *
 * Example file Copyright (C) 2011 Jan Moringen <jmoringe@techfak.uni-bielefeld.de>
 *
 * This program is free software; you can redistribute it
 * and/or modify it under the terms of the GNU General
 * Public License as published by the Free Software Foundation;
 * either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ============================================================ */

#include <iostream>

#include <boost/thread.hpp>

#include <rsb/Factory.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#include <float.h>
#include <string.h>

#define PI 3.1415926535897932

// See ../CMakeLists.txt for the generation of this file.
// The generated file can be found in build/src/cpp
#include <cdsr_messages.pb.h>

using namespace std;

using namespace boost;

using namespace rsb;
using namespace rsb::converter;

// The generated protocol buffer class is in this namespace.
using namespace cdsr::rsb;

static boost::shared_ptr<SensedObjects> sobjects;
static boost::shared_ptr<Room> sroom;


void processRoom(boost::shared_ptr<Room> room) {
    sroom = room;
	cout << "received Room " << room->id() << endl;
	cout << "Category = " << room->category() << endl;
}

void processSensedObjects(boost::shared_ptr<SensedObjects> sensed_objects) {
  sobjects = sensed_objects;
	cout << "received " << sensed_objects->object_size() << " Sensed Objects "<< endl;
	/*for (int i = 0; i < sensed_objects->object_size(); i++) {
		const SensedObject& next_object = sensed_objects->object(i);

		cout << "Object " << i << " ID = " << next_object.id() << endl;
		cout << "Type = " << next_object.type() << endl;

		for (int j = 0; j < next_object.side_size(); j++) {
			cout << "[" << next_object.side(j).start().x() << ", "
					<< next_object.side(j).start().y() << "] to ["
					<< next_object.side(j).end().x() << ", "
					<< next_object.side(j).end().y() << "]" << endl;
		}

		cout << "----------------" << endl;
	}*/
}

double distanceBetween2Points(double p1x, double p1y, double p2x, double p2y) {
	double xd = p1x - p2x;
	double yd = p1y - p2y;
	return sqrt((xd*xd) + (yd*yd));	
}


double dotProduct(double v1_x, double v1_y, double v2_x, double v2_y) {
	return ((v1_x*v2_x)+(v1_y*v2_y));
}

double distanceFromOrigin(double x, double y) {
	return sqrt((x*x) + (y*y));
}

double toDegrees(double radians) {
	return (360/(2*PI))*radians;
}


double angleBetween(double v1_x, double v1_y, double v2_x, double v2_y) {
	
	if( ((v1_x == 0) && (v1_y == 0)) || ((v2_x == 0) && (v2_y == 0))){
		return 0;
	}
	
	double degrees;
	double rads;
	double tmp;
	double dprod = dotProduct(v1_x, v1_y, v2_x, v2_y);
	double distA = distanceFromOrigin(v1_x, v1_y);
	double distB = distanceFromOrigin(v2_x, v2_y);
    
	tmp = dprod/(distA*distB);
	if (tmp >= 1.0) {
		return 0;
	} 
	if (tmp <= -1.0) {
		return 180;
	}
	rads = acos(tmp);
	
	if(isnan(rads)) {
		cout << "Error: ::angleBetween: rads is not a number" << endl;
		cout << "Vec1: x = " << v1_x << " y = "<< v1_y <<endl;
		cout << "Vec2: x = " << v2_x << " y = "<< v2_y <<endl;
		cout << "tmp = " << tmp << endl;
		for(double x = 1.0; x >= -1; x = x - 0.1) {
			cout << "x = " << x << endl;
			cout << "acos(x) = " << acos(x) << endl;
			cout << "toDegrees(acos(x)) = " << toDegrees(acos(x)) << endl;
		}		
		exit(EXIT_FAILURE);
	}
	
	degrees = toDegrees(rads);
	
	if(isnan(degrees)) {
		cout << "Error: myVector::angleBetween: isnan(degrees)" << endl;
		exit(EXIT_FAILURE);
	}
	return degrees;
}

double getObjectXCenter(const SensedObject& obj) {
    double max_x = -DBL_MAX;
    double min_x = DBL_MAX;
    
    for(int side = 0; side < obj.side_size(); side++) {
        
        if(obj.side(side).start().x() > max_x) {
            max_x = obj.side(side).start().x();
        } 
        
        if(obj.side(side).start().x() < min_x) {
            min_x = obj.side(side).start().x();
        }
        
        if(obj.side(side).end().x() > max_x) {
            max_x = obj.side(side).end().x();
        } 
        
        if(obj.side(side).end().x() < min_x) {
            min_x = obj.side(side).end().x();
        }
    }
    
    return min_x + ((max_x - min_x)/2);
}

double getObjectYCenter(const SensedObject& obj) {
    double max_y = -DBL_MAX;
    double min_y = DBL_MAX;
    
    for(int side = 0; side < obj.side_size(); side++) {
        
        if(obj.side(side).start().y() > max_y) {
            max_y = obj.side(side).start().y();
        } 
        
        if(obj.side(side).start().y() < min_y) {
            min_y = obj.side(side).start().y();
        }
        
        if(obj.side(side).end().y() > max_y) {
            max_y = obj.side(side).end().y();
        } 
        
        if(obj.side(side).end().y() < min_y) {
            min_y = obj.side(side).end().y();
        }
    }
    return min_y + ((max_y - min_y)/2);
}



double computeMaxDistance() {
    double max_distance = 0;
    /*
     * compute the max distance between room vertices
     */
    //get the extent of the room
    double max_x = -DBL_MAX; 
    double min_x = DBL_MAX;
    double max_y = -DBL_MAX; 
    double min_y = DBL_MAX;
    
    double room_min_x = DBL_MAX;
    double room_max_x = -DBL_MAX;
    double room_min_y = DBL_MAX;
    double room_max_y = -DBL_MAX;
    
    for (int i = 0; i < sroom->wall_size(); i++) {
		const Line& next_wall = sroom->wall(i);
        if(next_wall.start().x() > max_x) {
            max_x = next_wall.start().x();
        } else if(next_wall.start().x() < min_x) {
            min_x = next_wall.start().x();
        }
        
        if(next_wall.end().x() > max_x) {
            max_x = next_wall.end().x();
        } else if(next_wall.end().x() < min_x) {
            min_x = next_wall.end().x();
        }
        
        if(next_wall.start().y() > max_y) {
            max_y = next_wall.start().y();
        } else if(next_wall.start().y() < min_y) {
            min_y = next_wall.start().y();
        }
        
        if(next_wall.end().y() > max_y) {
            max_y = next_wall.end().y();
        } else if(next_wall.end().y() < min_y) {
            min_y = next_wall.end().y();
        }
	}
    room_min_x = min_x;
    room_max_x = max_x;
    room_min_y = min_y;
    room_max_y = max_y;
    //calculate the hypotheuse of the triangle of the room width and height
    double room_width = (room_max_x - room_min_x);
    double room_height = (room_max_y - room_min_y);
    
    max_distance = sqrt(((room_width * room_width) + (room_height * room_height)));

    /*
     * compute the max distance between any two objects in the room
     */
    
    int i, j;
    for (i = 0; i < sobjects->object_size(); i++) {
        const SensedObject& landmark = sobjects->object(i);
        double lx = getObjectXCenter(landmark);
        double ly = getObjectYCenter(landmark);        
        for (j = 0; j < sobjects->object_size(); j++) {
            if(j != i) {
                const SensedObject& target = sobjects->object(j);
                double tx = getObjectXCenter(target);
                double ty = getObjectYCenter(target);        
                double distbetween = distanceBetween2Points(lx, ly, tx, ty);
                if(distbetween > max_distance) {
                    max_distance = distbetween;
                }
            }
        }

    }
    
    return max_distance;
}


double projSpatRel(const SensedObject& landmark, const SensedObject& target, double max_distance,  double vDir_x, double vDir_y) {
    
    double l_cent_x = getObjectXCenter(landmark);
    double l_cent_y = getObjectYCenter(landmark);

    double t_cent_x = getObjectXCenter(target);
    double t_cent_y = getObjectYCenter(target);


    //compute the angular deviation model
    //subtract the landmarks position from the trajector
    //equivalent to moving the landmark and the trajector towards the origin
    
    double t_cent_x2 = t_cent_x - l_cent_x;
    double t_cent_y2 = t_cent_y - l_cent_y;

    //get the angle between the trajector and the direction vector
    //i.e. compute the dot product 
    double angle = angleBetween(t_cent_x2, t_cent_y2, vDir_x, vDir_y);

    //compute a normalised distance between the landmark and the trajector
    //we normalise the distance by the extent of the room
    //double max_distance =  computeMaxDistance(l_cent_x, l_cent_y);
    double distbetween = distanceBetween2Points(l_cent_x, l_cent_y, t_cent_x, t_cent_y);
    double distfactor = 1 - (distbetween/max_distance); 

    if(distfactor < 0) {
        cout << "***************************************************" << endl;
        cout << "***************************************************" << endl;
        cout << "***************************************************" << endl;
        cout << "***************************************************" << endl;
        cout << "***************************************************" << endl;
        cout << "***************************************************" << endl;
        cout << "landmark : " << landmark.id() << endl;
        cout << "target : " << target.id() << endl;
        cout << "distFactor : " << distfactor << endl;
        cout << "distbetween " << distbetween << endl;
        cout << "max_distance (new) " << max_distance << endl;
        cout << "***************************************************" << endl;
        cout << "***************************************************" << endl;
        cout << "***************************************************" << endl;
        cout << "***************************************************" << endl;
        cout << "***************************************************" << endl;
        cout << "***************************************************" << endl;
    }
    
    if(angle > 90) {
        return 0;
    } else {
        double strength = (1-(angle/90.0)) * distfactor;
        if(strength < 0) {
            cout << "***************************************************" << endl;
            cout << "***************************************************" << endl;
            cout << "***************************************************" << endl;
            cout << "***************************************************" << endl;
            cout << "***************************************************" << endl;
            cout << "***************************************************" << endl;
            cout << "strength (neg) : " << strength << endl;
            cout << "***************************************************" << endl;
            cout << "***************************************************" << endl;
            cout << "***************************************************" << endl;
            cout << "***************************************************" << endl;
            cout << "***************************************************" << endl;
            cout << "***************************************************" << endl;            
        }
        return (1-(angle/90.0)) * distfactor;
    }    
}

int main() {
	// Register specific template instantiations of the
	// ProtocolBufferConverter for each message sent or received
	shared_ptr<ProtocolBufferConverter<Room> > room_converter(
			new ProtocolBufferConverter<Room> ());
	stringConverterRepository()->registerConverter(room_converter);

	shared_ptr<ProtocolBufferConverter<SensedObjects> >
			sensed_objects_converter(
					new ProtocolBufferConverter<SensedObjects> ());
	stringConverterRepository()->registerConverter(sensed_objects_converter);

	shared_ptr<ProtocolBufferConverter<SpatialRelations> >
			spatial_relations_converter(new ProtocolBufferConverter<
					SpatialRelations> ());
	stringConverterRepository()->registerConverter(spatial_relations_converter);

	// create the listeners

	ListenerPtr room_listener = Factory::getInstance().createListener(Scope("/cdsr/room"));
	room_listener->addHandler(HandlerPtr(new DataFunctionHandler<Room> (&processRoom)));
	ListenerPtr sensed_objects_listener = Factory::getInstance().createListener(Scope("/cdsr/sensedobjects"));
	sensed_objects_listener->addHandler(HandlerPtr(new DataFunctionHandler<SensedObjects> (&processSensedObjects)));

	// wait for the room and sensed objects to be received

	// sleep is a temporary hack
	boost::this_thread::sleep(boost::posix_time::seconds(5));

  	Informer<SpatialRelations>::Ptr informer =
    Factory::getInstance().createInformer<SpatialRelations> (Scope(
                                                                   "/cdsr/spatialrelations"));    
	Informer<SpatialRelations>::DataPtr data(new SpatialRelations());
  
    double max_distance =  computeMaxDistance();
    
    int i, j;
    for (i = 0; i < sobjects->object_size(); i++) {
      const SensedObject& landmark = sobjects->object(i);
      for (j = 0; j < sobjects->object_size(); j++) {
          if(j != i) {
              const SensedObject& target = sobjects->object(j);
              const string separator = "_";

              SpatialRelation* relation1 = data->add_relation();
              relation1->set_id(landmark.id() + separator + target.id() + separator + "xOneYZero");
              relation1->set_type("xOneYZero");
              relation1->set_start_id(landmark.id());
              relation1->set_end_id(target.id());
              relation1->set_strength(projSpatRel(landmark, target, max_distance, 1.0, 0.0));

              SpatialRelation* relation2 = data->add_relation();
              relation2->set_id(landmark.id() + separator + target.id() + separator + "xOneYOne");
              relation2->set_type("xOneYOne");
              relation2->set_start_id(landmark.id());
              relation2->set_end_id(target.id());
              relation2->set_strength(projSpatRel(landmark, target, max_distance, 1.0, 1.0));

              SpatialRelation* relation3 = data->add_relation();
              relation3->set_id(landmark.id() + separator + target.id() + separator + "xZeroYOne");
              relation3->set_type("xZeroYOne");
              relation3->set_start_id(landmark.id());
              relation3->set_end_id(target.id());
              relation3->set_strength(projSpatRel(landmark, target, max_distance, 0.0, 1.0));

              SpatialRelation* relation4 = data->add_relation();
              relation4->set_id(landmark.id() + separator + target.id() + separator + "xNegOneYOne");
              relation4->set_type("xNegOneYOne");
              relation4->set_start_id(landmark.id());
              relation4->set_end_id(target.id());
              relation4->set_strength(projSpatRel(landmark, target, max_distance, -1.0, 1.0));

              SpatialRelation* relation5 = data->add_relation();
              relation5->set_id(landmark.id() + separator + target.id() + separator + "xNegOneYZero");
              relation5->set_type("xNegOneYZero");
              relation5->set_start_id(landmark.id());
              relation5->set_end_id(target.id());
              relation5->set_strength(projSpatRel(landmark, target, max_distance, -1.0, 0.0));

              SpatialRelation* relation6 = data->add_relation();
              relation6->set_id(landmark.id() + separator + target.id() + separator + "xNegOneYNegOne");
              relation6->set_type("xNegOneYNegOne");
              relation6->set_start_id(landmark.id());
              relation6->set_end_id(target.id());
              relation6->set_strength(projSpatRel(landmark, target, max_distance, -1.0, -1.0));

              SpatialRelation* relation7 = data->add_relation();
              relation7->set_id(landmark.id() + separator + target.id() + separator + "xZeroYNegOne");
              relation7->set_type("xZeroYNegOne");
              relation7->set_start_id(landmark.id());
              relation7->set_end_id(target.id());
              relation7->set_strength(projSpatRel(landmark, target, max_distance, 0.0, -1.0));

              SpatialRelation* relation8 = data->add_relation();
              relation8->set_id(landmark.id() + separator + target.id() + separator + "xOneYNegOne");
              relation8->set_type("xOneYNegOne");
              relation8->set_start_id(landmark.id());
              relation8->set_end_id(target.id());
              relation8->set_strength(projSpatRel(landmark, target, max_distance, 1.0, -1.0));
              
          }
      }
  }
    
    cout << "publishing spatial relations" << endl;
          
	informer->publish(data);

	return EXIT_SUCCESS;
}

/*  cout << "----------------" << endl;
 cout << "sobjects->object_size()" << sobjects->object_size() << endl;
 cout << "----------------" << endl;
 int i,j;
 for (i = 0; i < sobjects->object_size(); i++) {
 const SensedObject& next_object = sobjects->object(i);
 
 cout << "Object " << i << " ID = " << next_object.id() << endl;
 cout << "Type = " << next_object.type() << endl;
 
 for (j = 0; j < next_object.side_size(); j++) {
 cout << "[" << next_object.side(j).start().x() << ", "
 << next_object.side(j).start().y() << "] to ["
 << next_object.side(j).end().x() << ", "
 << next_object.side(j).end().y() << "]" << endl;
 }
 cout << "----------------" << endl;
 }*/


/*  cout << "landmark: " << landmark.id() << " target: " << target.id() << " xOneYOne: " << projSpatRel(landmark, target, 1.0, 1.0) << endl;              
 cout << "landmark: " << landmark.id() << " target: " << target.id() << " xZeroYOne: " << projSpatRel(landmark, target, 0.0, 1.0) << endl;              
 cout << "landmark: " << landmark.id() << " target: " << target.id() << " xNegOneYOne: " << projSpatRel(landmark, target, -1.0, 1.0) << endl;              
 cout << "landmark: " << landmark.id() << " target: " << target.id() << " xNegOneYZero: " << projSpatRel(landmark, target, -1.0, 0.0) << endl;              
 cout << "landmark: " << landmark.id() << " target: " << target.id() << " xNegOneYNegOne: " << projSpatRel(landmark, target, -1.0, -1.0) << endl;              
 cout << "landmark: " << landmark.id() << " target: " << target.id() << " xZeroYNegOne: " << projSpatRel(landmark, target, 0.0, -1.0) << endl;              
 cout << "landmark: " << landmark.id() << " target: " << target.id() << " xOneYNegOne: " << projSpatRel(landmark, target, 1.0, -1.0) << endl;              */

//    const SensedObject& target = sobjects->object(15);
//    float xZeroYPlus =  projSpatRel(landmark, target, 0.0, 1.0);
//    cout << "xZeroYPlus: " << xZeroYPlus << endl;

