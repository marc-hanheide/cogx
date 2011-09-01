/**
* @author Markus Bader
* @date
* @version 0.1
* @brief
*/


#include <stdio.h>

#include <v4r/TomGine/shm/tgShape.h>

using namespace TomGine;
int main(int argc, char *argv[])
{ 
    std::vector<ShapeAppearance>  tgAppearances;
    tgAppearances.push_back(ShapeAppearance(1, 1, 0, 0, 0.5));
    tgAppearances.push_back(ShapeAppearance(2, 1, 0, 1, 0.5));
    
    std::vector<ShapeEntry>  tgShapes;
    tgShapes.push_back(ShapeCylinder(tgAppearances[0], 0.1f, 0.2f, 64, 2, true));
    tgShapes.back().position(0.2, 0, 0);
    tgShapes.push_back(ShapeBox(tgAppearances[0], 0.1, 0.1, 0.1));
    tgShapes.push_back(ShapeSphere(tgAppearances[1],  0.05, 3));
    
    std::vector<ShapeLine>  tgLines;
    tgLines.push_back(ShapeLine(tgAppearances[0], 0, 0, 0, 1, 1, 1));
    tgLines.push_back(ShapeLine(tgAppearances[1], 0.1, 0.1, 0.1, -0.2, 0.2, 0.2));
    
    ShmTGShapes shapes("TG/Shapes");
    ShmTGAppearance appearances("TG/Appearance");
    ShmTGLines shmLines("TG/Lines");
    
    shapes.set(tgShapes);
    shmLines.set(tgLines);
    appearances.set(tgAppearances);
    
    std::cout << "Use the programm: \"shmadmin -c\" to clear the shared memory if needed!" << std::endl; 
    
    return 0;
}



