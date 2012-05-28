#ifndef PlaceholderExplorer_hpp
#define PlaceholderExplorer_hpp

#include "SpatialData.hpp"
#include <FrontierInterface.hpp>
#include <cast/architecture/ManagedComponent.hpp>
#include <string>

using namespace std;

namespace navsa {

class PlaceholderExplorer: public cast::ManagedComponent {
public:
	PlaceholderExplorer();
	virtual ~PlaceholderExplorer() {
	}
	;

	virtual void runComponent();
	virtual void start();
protected:
	virtual void configure(const map<string, string>& config);
private:
	int m_status;
	float m_x, m_y, m_theta;
	bool m_hasPosition;
	void poseChange(const cast::cdl::WorkingMemoryChange &objID);
	void navCommandResponse(const cast::cdl::WorkingMemoryChange &objID);
	void goToPlace(int placeId);
	int findClosestPlaceholderInNodeGraph(int curPlaceId);
};
}
;

#endif
