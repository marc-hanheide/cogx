#include "FakeBinderSA.hpp"
#include "FakeBinderData.hpp"

using namespace std;

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new FakeBinderSA();
  }
}

void FakeBinderSA::runComponent() {
    println("FakeBinderSA: running");
    autogen::FakeBinderData::FakeBinderActualStateProviderPtr s = new autogen::FakeBinderData::FakeBinderActualStateProvider();

    s->objects = "R2D2 - robot User - human R2D2 - planning_agent living_room kitchen bedroom - room coffee cake - movable kitchen_door free_space - door";
    s->state = "(connects kitchen_door living_room kitchen) (connects kitchen_door kitchen living_room) (doorstate kitchen_door : open)	(connects free_space bedroom kitchen) (connects free_space kitchen bedroom) (doorstate free_space : open) (pos cake : kitchen) (pos coffee  : bedroom) (pos r2d2 : living_room) (KVAL (User) (pos cake))";
    string id = newDataID();
    addToWorkingMemory(id,s);
}

