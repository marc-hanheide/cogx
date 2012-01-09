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

void processRoom(boost::shared_ptr<Room> room) {
	cout << "received Room " << room->id() << endl;
	cout << "Category = " << room->category() << endl;

	for (int i = 0; i < room->wall_size(); i++) {
		const Line& next_wall = room->wall(i);
		cout << "[" << next_wall.start().x() << ", " << next_wall.start().y()
				<< "] to [" << next_wall.end().x() << ", "
				<< next_wall.end().y() << "]" << endl;
		cout << "----------------" << endl;
	}

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


double projSpatRel(const SensedObject& landmark, const SensedObject& target, double vDir_x, double vDir_y) {
    
    //compute the centroid of the landmark and target    
    double max_x = 0; 
    double min_x = DBL_MAX;
    double max_y = 0; 
    double min_y = DBL_MAX;
    
    for(int l_side = 0; l_side < landmark.side_size(); l_side++) {
        
        if(landmark.side(l_side).start().x() > max_x) {
            max_x = landmark.side(l_side).start().x();
        } else if(landmark.side(l_side).start().x() < min_x) {
            min_x = landmark.side(l_side).start().x();
        }

        if(landmark.side(l_side).end().x() > max_x) {
            max_x = landmark.side(l_side).end().x();
        } else if(landmark.side(l_side).end().x() < min_x) {
            min_x = landmark.side(l_side).end().x();
        }

        
        if(landmark.side(l_side).start().y() > max_y) {
            max_y = landmark.side(l_side).start().y();
        } else if(landmark.side(l_side).start().y() < min_y) {
            min_y = landmark.side(l_side).start().y();
        }
        
        if(landmark.side(l_side).end().y() > max_y) {
            max_y = landmark.side(l_side).end().y();
        } else if(landmark.side(l_side).end().y() < min_y) {
            min_y = landmark.side(l_side).end().y();
        }

    }
    double l_cent_x = min_x + ((max_x - min_x)/2);
    double l_cent_y = min_y + ((max_y - min_y)/2);
    
    cout << "landmark position: " << l_cent_x << " ," << l_cent_y << endl;
    
    max_x = 0; 
    min_x = DBL_MAX;
    max_y = 0; 
    min_y = DBL_MAX;
    
    for(int l_side = 0; l_side < target.side_size(); l_side++) {
        if(target.side(l_side).start().x() > max_x) {
            max_x = target.side(l_side).start().x();
        } else if(target.side(l_side).start().x() < min_x) {
            min_x = target.side(l_side).start().x();
        }

        if(target.side(l_side).end().x() > max_x) {
            max_x = target.side(l_side).end().x();
        } else if(target.side(l_side).end().x() < min_x) {
            min_x = target.side(l_side).end().x();
        }
             
        if(target.side(l_side).start().y() > max_y) {
            max_y = target.side(l_side).start().y();
        } else if(target.side(l_side).start().y() < min_y) {
            min_y = target.side(l_side).start().y();
        }
        
        if(target.side(l_side).end().y() > max_y) {
            max_y = target.side(l_side).end().y();
        } else if(target.side(l_side).end().y() < min_y) {
            min_y = target.side(l_side).end().y();
        }

    }
    double t_cent_x = min_x + ((max_x - min_x)/2);
    double t_cent_y = min_y + ((max_y - min_y)/2);

    cout << "target position: " << t_cent_x << " ," << t_cent_y << endl; 
    
    //subtract the landmarks position from the trajector
    //equivalent to moving the landmark and the trajector towards the origin
    
    t_cent_x = t_cent_x - l_cent_x;
    t_cent_y = t_cent_y - l_cent_y;

    //get the angle between the trajector and the direction vector
    //i.e. compute the dot product 
    
    double angle = angleBetween(t_cent_x, t_cent_y, vDir_x, vDir_y);
    
    //cout << " angle: " << angle << endl;
    
    if(angle > 90) {
        return 0;
    } else {
        return (1-(angle/90.0));
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
  
    int i, j;
    for (i = 0; i < sobjects->object_size(); i++) {
      const SensedObject& landmark = sobjects->object(i);
      for (j = 0; j < sobjects->object_size(); j++) {
          if(j != i) {
              const SensedObject& target = sobjects->object(j);
              
              SpatialRelation* relation1 = data->add_relation();
              relation1->set_id(landmark.id() + ":" + target.id() + ":xOneYZero");
              relation1->set_type("xOneYZero");
              relation1->set_start_id(landmark.id());
              relation1->set_end_id(target.id());
              relation1->set_strength(projSpatRel(landmark, target, 1.0, 0.0));

              SpatialRelation* relation2 = data->add_relation();
              relation2->set_id(landmark.id() + ":" + target.id() + ":xOneYOne");
              relation2->set_type("xOneYOne");
              relation2->set_start_id(landmark.id());
              relation2->set_end_id(target.id());
              relation2->set_strength(projSpatRel(landmark, target, 1.0, 1.0));

              SpatialRelation* relation3 = data->add_relation();
              relation3->set_id(landmark.id() + ":" + target.id() + ":xZeroYOne");
              relation3->set_type("xZeroYOne");
              relation3->set_start_id(landmark.id());
              relation3->set_end_id(target.id());
              relation3->set_strength(projSpatRel(landmark, target, 0.0, 1.0));

              SpatialRelation* relation4 = data->add_relation();
              relation4->set_id(landmark.id() + ":" + target.id() + ":xMinOneYOne");
              relation4->set_type("xMinOneYOne");
              relation4->set_start_id(landmark.id());
              relation4->set_end_id(target.id());
              relation4->set_strength(projSpatRel(landmark, target, -1.0, 1.0));

              SpatialRelation* relation5 = data->add_relation();
              relation5->set_id(landmark.id() + ":" + target.id() + ":xMinOneYZero");
              relation5->set_type("xMinOneYZero");
              relation5->set_start_id(landmark.id());
              relation5->set_end_id(target.id());
              relation5->set_strength(projSpatRel(landmark, target, -1.0, 0.0));

              SpatialRelation* relation6 = data->add_relation();
              relation6->set_id(landmark.id() + ":" + target.id() + ":xMinOneYMinOne");
              relation6->set_type("xMinOneYMinOne");
              relation6->set_start_id(landmark.id());
              relation6->set_end_id(target.id());
              relation6->set_strength(projSpatRel(landmark, target, -1.0, -1.0));

              SpatialRelation* relation7 = data->add_relation();
              relation7->set_id(landmark.id() + ":" + target.id() + ":xZeroYMinOne");
              relation7->set_type("xZeroYMinOne");
              relation7->set_start_id(landmark.id());
              relation7->set_end_id(target.id());
              relation7->set_strength(projSpatRel(landmark, target, 0.0, -1.0));

              SpatialRelation* relation8 = data->add_relation();
              relation8->set_id(landmark.id() + ":" + target.id() + ":xOneYMinOne");
              relation8->set_type("xOneYMinOne");
              relation8->set_start_id(landmark.id());
              relation8->set_end_id(target.id());
              relation8->set_strength(projSpatRel(landmark, target, 1.0, -1.0));
              
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
 cout << "landmark: " << landmark.id() << " target: " << target.id() << " xMinOneYOne: " << projSpatRel(landmark, target, -1.0, 1.0) << endl;              
 cout << "landmark: " << landmark.id() << " target: " << target.id() << " xMinOneYZero: " << projSpatRel(landmark, target, -1.0, 0.0) << endl;              
 cout << "landmark: " << landmark.id() << " target: " << target.id() << " xMinOneYMinOne: " << projSpatRel(landmark, target, -1.0, -1.0) << endl;              
 cout << "landmark: " << landmark.id() << " target: " << target.id() << " xZeroYMinOne: " << projSpatRel(landmark, target, 0.0, -1.0) << endl;              
 cout << "landmark: " << landmark.id() << " target: " << target.id() << " xOneYMinOne: " << projSpatRel(landmark, target, 1.0, -1.0) << endl;              */

//    const SensedObject& target = sobjects->object(15);
//    float xZeroYPlus =  projSpatRel(landmark, target, 0.0, 1.0);
//    cout << "xZeroYPlus: " << xZeroYPlus << endl;

