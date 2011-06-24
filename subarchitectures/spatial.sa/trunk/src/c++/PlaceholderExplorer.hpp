#ifndef PlaceholderExplorer_hpp
#define PlaceholderExplorer_hpp

#include "SpatialData.hpp"
#include <FrontierInterface.hpp>
#include <cast/architecture/ManagedComponent.hpp>
#include <string>

using namespace std;

namespace navsa {

  class PlaceholderExplorer : public cast::ManagedComponent
  {
    private:
      class PlaceholderCompare
      {
        private:
          FrontierInterface::PlaceInterfacePrx proxy;
          double robotX, robotY;
          double getCost(const SpatialData::PlacePtr& placeholder) const;
        public:
          bool operator()(const SpatialData::PlacePtr& lhs, const SpatialData::PlacePtr& rhs) const;
          PlaceholderCompare(FrontierInterface::PlaceInterfacePrx _proxy, double x, double y);
      };

    public:
      PlaceholderExplorer();
      virtual ~PlaceholderExplorer() {};

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

      SpatialData::PlacePtr getPlaceholder();
  };
};

#endif
