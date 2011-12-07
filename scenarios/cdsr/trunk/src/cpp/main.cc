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

// See ../CMakeLists.txt for the generation of this file.
// The generated file can be found in build/src/cpp
#include <cdsr_messages.pb.h>

using namespace std;

using namespace boost;

using namespace rsb;
using namespace rsb::converter;

// The generated protocol buffer class is in this namespace.
using namespace cdsr::rsb;

void processRoom(boost::shared_ptr<Room> room) {
	cout << "received Room " << room->id() << endl;
	cout << "Context = " << room->context() << endl;

	for (int i = 0; i < room->wall_size(); i++) {
		const Line& next_wall = room->wall(i);
		cout << "[" << next_wall.start().x() << ", " << next_wall.start().y()
				<< "] to [" << next_wall.end().x() << ", "
				<< next_wall.end().y() << "]" << endl;
		cout << "----------------" << endl;
	}

}

void processSensedObjects(boost::shared_ptr<SensedObjects> sensed_objects) {
	cout << "received " << sensed_objects->object_size() << " Sensed Objects "
			<< endl;
	for (int i = 0; i < sensed_objects->object_size(); i++) {
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

	ListenerPtr room_listener = Factory::getInstance().createListener(Scope(
			"/cdsr/room"));
	room_listener->addHandler(HandlerPtr(new DataFunctionHandler<Room> (
			&processRoom)));
	ListenerPtr sensed_objects_listener =
			Factory::getInstance().createListener(Scope("/cdsr/sensedobjects"));
	sensed_objects_listener->addHandler(HandlerPtr(new DataFunctionHandler<
			SensedObjects> (&processSensedObjects)));

	// wait for the room and sensed objects to be received

	// sleep is a temporary hack
	boost::this_thread::sleep(boost::posix_time::seconds(5));

	// TODO John add code here

	// then compute the spatial relations

	// TODO John add code here

	// then send the spatial relations back

	cout << "sending spatial relations " << endl;

	Informer<SpatialRelations>::Ptr informer =
			Factory::getInstance().createInformer<SpatialRelations> (Scope(
					"/cdsr/spatialrelations"));

	Informer<SpatialRelations>::DataPtr data(new SpatialRelations());

	// TODO John add code here

	SpatialRelation* example_relation = data->add_relation();
	example_relation->set_id("sr1");
	example_relation->set_type("above");
	example_relation->set_start_id("object1");
	example_relation->set_end_id("object2");
	example_relation->set_strength(10.0);

	informer->publish(data);

	return EXIT_SUCCESS;
}
