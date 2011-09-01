/**
* @file main.cpp
* @author Thomas Mörwald
* @date October 2009
* @version 0.1
* @brief Main file for standalone version of TomGine rendering engine.
*/

#include <stdio.h>
#include <csignal>

#include <v4r/TomGine/tgEngine.h>
#include <v4r/TomGine/tgRenderModel.h>
#include <v4r/TomGine/tgShapeCreator.h>
#include <v4r/TomGine/tgFont.h>

#include <v4r/TomGine/shm/tgShape.h>

#include <time.h>

using namespace TomGine;
using namespace std;

typedef vector<vec3> PointList;

void drawLines()
{
    ShmTGLines shmLines("TG/Lines");
    ShmTGAppearance shmAppearance("TG/Appearance");
    std::vector<ShapeLine>  tgLines;
    std::vector<ShapeAppearance>  tgAppearances;
    shmLines.copyTo(tgLines);
    shmAppearance.copyTo(tgAppearances);
    std::vector<ShapeAppearance>::iterator app;
    std::vector<ShapeLine>::iterator line;

    glDisable(GL_LIGHTING);
    glEnable(GL_POINT_SMOOTH);
    glDisable(GL_TEXTURE_2D);
    glBegin(GL_LINES);
    for (app = tgAppearances.begin(); app != tgAppearances.end(); app++) {
        glColor4f(app->r(), app->g(), app->b(), app->a());
        for (line = tgLines.begin(); line != tgLines.end(); line++) {
            if (line->appearance() != app->id()) continue;
            glVertex3f(line->x1(),line->y1(), line->z1());
            glVertex3f(line->x2(),line->y2(), line->z2());
        }
    }
    glEnd( );
}

void drawShapes()
{

    tgShapeCreator shape_creator;
    ShmTGShapes shmShapes("TG/Shapes");
    ShmTGAppearance shmAppearance("TG/Appearance");
    std::vector<ShapeEntry>  tgShapes;
    std::vector<ShapeAppearance>  tgAppearances;
    shmShapes.copyTo(tgShapes);
    shmAppearance.copyTo(tgAppearances);
    std::vector<ShapeAppearance>::iterator app;
    std::vector<ShapeEntry>::iterator shape;
    for (app = tgAppearances.begin(); app != tgAppearances.end(); app++) {
        tgRenderModel tgRModel;
        tgRModel.m_material.Color(app->r(), app->g(), app->b(), app->a());
        for (shape = tgShapes.begin(); shape != tgShapes.end(); shape++) {
            if (shape->appearance() != app->id()) continue;
            switch (shape->type()) {
            case ShapeEntry::BOX:
            {
                ShapeBox *s = (ShapeBox *) &(*shape);
                shape_creator.CreateBox(tgRModel, s->dx(), s->dy(), s->dz());
            }
            break;
            case ShapeEntry::SPHERE:
            {
                ShapeSphere *s = (ShapeSphere *) &(*shape);
                shape_creator.CreateSphere(tgRModel, s->radius(), s->subdevisions(), s->mode());
            }
            break;
            case ShapeEntry::CYLINDER:
            {
                ShapeCylinder *s = (ShapeCylinder *) &(*shape);
                shape_creator.CreateCylinder(tgRModel, s->radius(), s->height(), s->slices(), s->stacks(), s->closed());
            }
            break;
            }
        }
        tgRModel.DrawFaces();
    }

}


volatile bool gQuit = false;
void quit(int){
  gQuit = true;
}

int main(int argc, char *argv[])
{
    (void) signal( SIGTERM, quit );
    (void) signal( SIGINT, quit );
    
    unsigned width = 800;
    unsigned height = 600;
    char avg_time[128];
    tgEngine render(width,height, 1.0f, 0.1f, "TomGine Render Engine", true);

    printf("\n Demo TomGine\n\n");

    printf(" TomGine control\n");
    printf(" -------------------------------------------\n");
    printf(" [Left Mouse Button] Rotate\n");
    printf(" [Right Mouse Button] Move\n");
    printf(" [Scroll Wheel] Zoom\n");
    printf(" [w] Switch to wireframe mode\n");
    printf(" [f] Switch to flat/smooth shading\n");
    printf(" \n\n");

    srand(time(NULL));
    float fTime;
    tgTimer timer;

    printf("[TomGine/main.cpp] Warning HARDCODED font file\n");


    ak::ShmUInt32 cycleTime("TG/CycleTime");
    cycleTime.set(5000);


    // Rendering loop
    while (gQuit == false) {


        render.Activate3D();
        glEnable(GL_LIGHTING);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


        drawShapes();
        drawLines();
	
	
	
        glDisable(GL_BLEND);

        render.Update();
        render.ProcessEvents();
        usleep(cycleTime.get());   // not to overload GPU
	
    }

    return 0;
}



