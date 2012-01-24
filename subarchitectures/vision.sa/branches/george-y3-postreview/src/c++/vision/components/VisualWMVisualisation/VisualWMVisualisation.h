/**
 * @author Michael Zillich
 * @date February 2009
 *
 * Visualises interesing content of the visual working memory
 */

#ifndef VISUAL_WM_VISUALISATION_H
#define VISUAL_WM_VISUALISATION_H

#include <map>
#include <string>
#include <ManagedComponent.hpp>
#include <VisionData.hpp>

namespace cast
{

class VisualWMVisualisation : public ManagedComponent
{
private:
  typedef std::map<std::string, VisionData::VisualObjectPtr> ObjectMap;

  /**
   * our mirror of the visual working memories content
   */
  std::map<std::string, VisionData::VisualObjectPtr> objects;
  int windowWidth;
  int windowHeight;

  /**
   * callback function called whenever a new object appears
   */
  void newVisualObject(const cdl::WorkingMemoryChange & _wmc);
  /**
   * callback function called whenever an object was overwritten
   */
  void changedVisualObject(const cdl::WorkingMemoryChange & _wmc);
  /**
   * callback function called whenever an object was deleted
   */
  void deletedVisualObject(const cdl::WorkingMemoryChange & _wmc);

  void drawZAxis(char axis);
  void drawCoordFrame();
  void drawWorldGrid();
  void drawVisualObject(const VisionData::VisualObjectPtr obj);
  void drawVisualObjects();
  void initGL();
  void updateDisplay();

protected:
  /**
   * called by the framework to configure our component
   */
  virtual void configure(const std::map<std::string,std::string> & _config);
  /**
   * called by the framework after configuration, before run loop
   */
  virtual void start();
  /**
   * called by the framework to start run loop
   */
  virtual void runComponent();

public:
  VisualWMVisualisation();
  virtual ~VisualWMVisualisation();
  void resizeGL(int width, int height);
  void redrawGL();
  void mousePressed(int button, int state, int x, int y);
  void mouseMoved(int x, int y);
};

}

#endif


