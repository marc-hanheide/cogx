/**
 * @file Mouse.cpp
 * @author Andreas Richtsfeld
 * @date 2010
 * @version 0.1
 * @brief Functions for mouse handling.
 */

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


}