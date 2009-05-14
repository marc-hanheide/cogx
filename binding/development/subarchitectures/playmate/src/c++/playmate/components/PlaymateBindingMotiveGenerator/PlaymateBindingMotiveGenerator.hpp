#include <motivation/components/BindingMotiveGenerator/BindingMotiveGenerator.hpp>


/**
 * An example of how to use the binding motive generator. This is not
 * necessarily just for he playmate, or all the playmate requires.
 */
class PlaymateBindingMotiveGenerator : public BindingMotiveGenerator {
public:
  PlaymateBindingMotiveGenerator(const std::string &_id);
  /**
   * Start the motive generator.
   */
  virtual void start();

protected:

  /**
   * Provide details of the agent for the planning process/
   */
  virtual std::string domainName() const {
    return "playmate";
  }

  /**
   * Provide details of the agent for the planning process/
   */
  virtual std::string domainFile() const {
    return "subarchitectures/planning.sa/src/python/mapsim/domains/playmate/mapl_files/domain.mapl";
  }

}; 
