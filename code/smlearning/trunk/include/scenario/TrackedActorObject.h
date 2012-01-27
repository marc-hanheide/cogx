/** @file TrackedActorObject.h
 * 
 * 
 * @author	Manuel Noll (DFKI)
 *
 * @version 1.0T
 *
 * 2011      Manuel Noll
 
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

 
*/

#ifndef TRACKEDACTOROBJECT_H
#define TRACKEDACTOROBJECT_H

#include <scenario/ActorObject.h>
#include <scenario/TrackerThread.h>

namespace smlearning {

/** \brief The class for an real world object that is tracked by the blort tracker.
*
* The class is derived from the ActorObject class and permits to track a real world object via the blort tracker
*/

class TrackedActorObject : public ActorObject
{
	public:
		/** Returns Actor pose
		 * @return				current Actor pose
		*/
		virtual Mat34 getPose() const;
	
		/** Sets a new Actor pose (only kinematic Actors)
		 * @param pose			target Actor pose
		*/
		virtual void setPose(const Mat34& pose);
	private:
		TrackerThread* _trackerThread;
		
};

}; // namespace smlearning 

#endif


