#include <gmpxx.h>
#include <mpfr.h>


#include <CGAL/Cartesian.h>
// #include <CGAL/bounding_box.h>


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

typedef double              FT;
typedef CGAL::Cartesian<FT> K;
typedef K::Point_2          Point_2;
typedef K::Point_3          Point_3;

using namespace std;

using namespace boost;

using namespace rsb;
using namespace rsb::converter;

// The generated protocol buffer class is in this namespace.
using namespace cdsr::rsb;

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


int main() {

  // axis-aligned bounding box of 2D points
  std::list<Point_2> points_2;
  points_2.push_back(Point_2(1.0, 0.0));
  points_2.push_back(Point_2(2.0, 2.0));
  points_2.push_back(Point_2(3.0, 5.0));

	// Register specific template instantiations of the
	// ProtocolBufferConverter for each message sent or received
	shared_ptr<ProtocolBufferConverter<Room> > room_converter(
			new ProtocolBufferConverter<Room> ());
	stringConverterRepository()->registerConverter(room_converter);


	// create the listeners

	ListenerPtr room_listener = Factory::getInstance().createListener(Scope(
			"/cdsr/room"));

	
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<Room> > > queue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<Room> >);

  room_listener->addHandler(HandlerPtr(new QueuePushHandler<Room> (queue)));
  
  boost::shared_ptr<Room> data = queue->pop();

  processRoom(data);  
  
	return EXIT_SUCCESS;
}
