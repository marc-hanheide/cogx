/**
 * @file Mouse.cpp
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Functions for mouse handling.
 */

#include "StereoDetector.h"

namespace cast
{
	
// Gobal variables: Can't solve this in another way
bool mouseEvent = false;				///< Flag for mouse events
int mouseX = 0, mouseY = 0;			///< Coordinates from mouse event
int mouseSide = 0;							///< Left / right side of stereo

/**
 * @brief Mouse handler for the two stereo images.
 * @param event Mouse event.
 * @param x x-coordinate
 * @param y y-coordinate
 * @param flags 
 * @param param 
 */
void LeftMouseHandler(int event, int x, int y, int flags, void* param)
{
	switch(event){
		case CV_EVENT_LBUTTONUP:
			mouseEvent = true;
			mouseX = x;
			mouseY = y;
			mouseSide = 0;
			break;
	}
}

/**
 * @brief Mouse handler for the two stereo images.
 * @param event Mouse event.
 * @param x x-coordinate
 * @param y y-coordinate
 * @param flags 
 * @param param 
 */
void RightMouseHandler(int event, int x, int y, int flags, void* param)
{
	switch(event){
		case CV_EVENT_LBUTTONUP:
			mouseEvent = true;
			mouseX = x;
			mouseY = y;
			mouseSide = 1;
			break;
	}
}

/**
 * @brief Processes the mouse event.
 * Stores found ID of Gestalt in showID. If more than one Gestalt found,
 * take with next pressed button the next Gestalt and so on.
 */
void StereoDetector::MouseEvent()
{
  static int prev_picked = -1, prev_mouseSide = 0;
  static int prev_x = INT_MAX, prev_y = INT_MAX;

  if(mouseX != prev_x || mouseY != prev_y || mouseSide != prev_mouseSide)
    prev_picked = -1;

  showID = score->PickGestaltAt(mouseSide, showType, mouseX, mouseY, prev_picked, false);
  if(showID != -1)
  {
    prev_picked = showID;
    prev_mouseSide = mouseSide;
    prev_x = mouseX;
    prev_y = mouseY;
  }
  else // check again in small surrounding area
  {
    bool found = false;
    int a_end = min((int) ((score->GetMonoCore(mouseSide))->IW())-1, mouseX+1);
    int b_end = min((int) ((score->GetMonoCore(mouseSide))->IH())-1, mouseY+1);
    for(int a = max(0, mouseX-1); a <= a_end && !found; a++)
      for(int b = max(0, mouseY-1); b <= b_end && !found; b++)
      {
        showID = score->PickGestaltAt(mouseSide, showType, a, b, prev_picked, false);
        if(showID != -1)
        {
          found = true;
          prev_picked = showID;
          prev_mouseSide = mouseSide;
          prev_x = mouseX;
          prev_y = mouseY;
        }
      }
  }

  if (showID < 0) return;
  debug("show feature with id=%u at position (%u/%u).", showID, mouseX, mouseY);
  ShowImages(true);
}

}