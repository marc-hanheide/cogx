#include <motivation/components/BindingMotiveGenerator/BindingMotiveGenerator.hpp>


/**
 * An example of how to use the binding motive generator. This is not
 * necessarily just for he explorer, or all the explorer requires.
 */
class ExplorerBindingMotiveGenerator : public BindingMotiveGenerator {
public:
  ExplorerBindingMotiveGenerator(const std::string &_id);
  /**
   * Start the motive generator.
   */
  virtual void start();


protected:

  /**
   * Provide details of the agent for the planning process/
   */
  virtual std::string domainName() const {
    return "explorer";
  }

  /**
   * Provide details of the agent for the planning process/
   */
  virtual std::string domainFile() const {
    return "subarchitectures/planning.sa/src/python/mapsim/domains/demo_explorer/mapl_files/domain.mapl";
  }



}; 
