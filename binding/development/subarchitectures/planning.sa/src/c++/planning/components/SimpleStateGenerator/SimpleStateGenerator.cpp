/**
 * Simple State Generator...
 *
 * @author Mohan
 * @date October 19 2007
 */

#include <iostream>
#include <cstdlib>

#include "SimpleStateGenerator.hpp"


/**
 * The function called to create a new instance of our component...
 */
extern "C" {
  FrameworkProcess* newComponent( const std::string& _id ) {
    return new SimpleStateGenerator(_id);
  }
}



/**
 * Constructor, simple because something has to exist because it can
 * be used -- remember to pass on the id to the super class...
 */
SimpleStateGenerator::SimpleStateGenerator( const std::string _id ):
  WorkingMemoryAttachedComponent(_id),
  PlanningStateGenerator(_id) {

}



/**
 * Destructor, where we clean up the mess we have created in this
 * class...
 */
SimpleStateGenerator::~SimpleStateGenerator() {
  cout << "SimpleStateGenerator::~SimpleStateGenerator()" << endl;
}



/**
 * Just a function to get the process running...
 */
void SimpleStateGenerator::start() {
  PlanningStateGenerator::start();
  // Register the plan state mappings that are essential for running
  // this process..
  FactTranslator* ft = new FactTranslator();
  registerPlanStateMapping<Fact>( ft );

  ObjectDeclarationTranslator* odt = new ObjectDeclarationTranslator();
  registerPlanStateMapping<ObjectDeclaration>( odt );
}



/**
 * Default runComponent function for the CASTComponent that does
 * nothing here...
 */
void SimpleStateGenerator::runComponent() {
  
}

