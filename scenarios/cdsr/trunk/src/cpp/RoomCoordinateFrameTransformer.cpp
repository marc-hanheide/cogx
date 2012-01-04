#include <CGAL/Cartesian.h>
#include <CGAL/bounding_box.h>
#include <CGAL/linear_least_squares_fitting_2.h>

#include <iostream>

#include <boost/thread.hpp>

#include <rsc/threading/SynchronizedQueue.h>

#include <rsb/QueuePushHandler.h>
#include <rsb/Factory.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>


// See ../CMakeLists.txt for the generation of this file.
// The generated file can be found in build/src/cpp
#include <cdsr_messages.pb.h>


#include <list>
#include <iostream>


//from http://www.cgal.org/Manual/latest/doc_html/cgal_manual/Principal_component_analysis/Chapter_main.html#Subsection_68.2.1
typedef double              FT;
typedef CGAL::Cartesian<FT> K;
typedef K::Point_2          Point_2;
typedef K::Point_3          Point_3;
typedef K::Line_2            Line_2;


using namespace std;

using namespace boost;

using namespace rsb;
using namespace rsb::converter;

using namespace rsb::patterns;

// The generated protocol buffer class is in this namespace.
using namespace cdsr::rsb;


class RoomCallback: public rsb::patterns::Server::Callback<RoomWithObjects, RoomWithObjects> {
public:
  boost::shared_ptr<RoomWithObjects> call(const string& _methodName, 
										  boost::shared_ptr<RoomWithObjects> room) {
    cout << "received Room " << room->room().id() << endl;
    cout << "Category = " << room->room().category() << endl;
    
    
    std::list<Point_2> points;
    
    for (int i = 0; i < room->room().wall_size(); i++) {
      const Line& next_wall = room->room().wall(i);
      cout << "[" << next_wall.start().x() << ", " << next_wall.start().y()
      << "] to [" << next_wall.end().x() << ", "
      << next_wall.end().y() << "]" << endl;
      cout << "----------------" << endl;
      
      points.push_back(Point_2(next_wall.start().x(), next_wall.start().y()));
      // points.push_back(Point_2(next_wall.end().x(), next_wall.end().y()));
    }
    
    // add an axis-aligned bounding box of 2D points as an extra object

    K::Iso_rectangle_2 c2 = CGAL::bounding_box(points.begin(), points.end());
    // std::cout << c2 << std::endl;

	::cdsr::rsb::SensedObject* newObject = room->add_object();
	newObject->set_id("fake object");
	newObject->set_type("bounding box");

	
	::cdsr::rsb::Line* newSide = newObject->add_side();
	newSide->mutable_start()->set_x(c2.xmin());
	newSide->mutable_start()->set_y(c2.ymin());
	newSide->mutable_end()->set_x(c2.xmin());
	newSide->mutable_end()->set_y(c2.ymax());

	newSide = newObject->add_side();
	newSide->mutable_start()->set_x(c2.xmin());
	newSide->mutable_start()->set_y(c2.ymax());
	newSide->mutable_end()->set_x(c2.xmax());
	newSide->mutable_end()->set_y(c2.ymax());
	
	newSide = newObject->add_side();
	newSide->mutable_start()->set_x(c2.xmax());
	newSide->mutable_start()->set_y(c2.ymax());
	newSide->mutable_end()->set_x(c2.xmax());
	newSide->mutable_end()->set_y(c2.ymin());
	
	newSide = newObject->add_side();
	newSide->mutable_start()->set_x(c2.xmax());
	newSide->mutable_start()->set_y(c2.ymin());
	newSide->mutable_end()->set_x(c2.xmin());
	newSide->mutable_end()->set_y(c2.ymin());
	
	
	
	//add a 2D regression line to the room as an extra wall
	
	//fit line to 2D points
   Line_2 line;
   linear_least_squares_fitting_2(points.begin(),points.end(),line,CGAL::Dimension_tag<0>());

	::cdsr::rsb::Line* newWall = room->mutable_room()->add_wall();	
	newWall->mutable_start()->set_x(-1);
	newWall->mutable_start()->set_y(line.y_at_x(newWall->start().x()));
	newWall->mutable_end()->set_x(10);
	newWall->mutable_end()->set_y(line.y_at_x(newWall->end().x()));
    return room;
  }
};


int main() {
  
  
	shared_ptr<ProtocolBufferConverter<RoomWithObjects> > room_converter(new ProtocolBufferConverter<RoomWithObjects> ());
	stringConverterRepository()->registerConverter(room_converter);
  
  Factory& factory = Factory::getInstance();
  ServerPtr server = factory.createServer(Scope("/cdsr/rooms"));
  
  server->registerMethod("standardiseCoordinateFrame",
                         Server::CallbackPtr(new RoomCallback()));

  boost::this_thread::sleep(boost::posix_time::seconds(1000));

	return EXIT_SUCCESS;
}
