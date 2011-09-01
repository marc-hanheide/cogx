 /**
 * @file main_tomgine_ip.cpp
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief TomGine image processing demo.
 */
 
#include <stdio.h>

#include <v4r/TomGine/tgEngine.h>
#include <v4r/TomGine/tgFont.h>
#include <v4r/TomGine/tgImageProcessor.h>

#include <time.h>

using namespace TomGine;
using namespace std;


int main(int argc, char *argv[])
{
	unsigned width = 512;
	unsigned height = 256;
	tgEngine render(width, height, 1.0f, 0.1f, "TomGine Image Processing", true);
	
	printf("\n TomGine Image Processing\n\n");
	printf(" [Escape] Quit demo\n");
	printf(" \n");
	
	srand(time(NULL));
	
	std::string path_smiley = std::string(TOMGINE_DIR) + 	"/example/Resources/smiley.jpg";

	tgImageProcessor ip( width, height );
	
	std::vector<float> data;
	unsigned s = 512*512*4;
	float ds = 1.0f/s;
	double avg = 0.0;
	for(unsigned i=0; i<s; i++){
		float val = float(rand())/RAND_MAX;
		data.push_back(float(val));

	}
	tgTimer timer;

	timer.Update();
	for(unsigned i=0; i<s; i++){
		avg += double(data[i])*ds;
	}
	printf("time: %f\n", timer.Update());

	float avg_gpu = ip.average(data);

	printf("time_gpu: %f\n", timer.Update());


	printf("avg: %f\n", avg);
	printf("avg_gpu: %f\n", avg_gpu);


	tgTexture2D tex_smiley, tex;
	tex_smiley.Load(path_smiley.c_str());
	
	long dut = 1000000;

	glClearColor(.2,.2,.2,0);
	
	// Rendering loop
	bool run = true;
	while(run){
		glDisable(GL_DEPTH_TEST);

		ip.render(tex_smiley);
		g_font->Print("original", 20, 10, 10);
		if(!(run = run && render.ProcessEvents())) break;
		render.Update();
		usleep(dut);

		ip.flipUpsideDown(tex_smiley, tex);
		g_font->Print("flip", 20, 10, 10);
		if(!(run = run && render.ProcessEvents())) break;
		render.Update();
		usleep(dut);

		ip.gauss(tex,tex);
		g_font->Print("gauss", 20, 10, 10);
		if(!(run = run && render.ProcessEvents())) break;
		render.Update();
		usleep(dut);

		ip.sobel(tex,tex);
		g_font->Print("sobel", 20, 10, 10);
		if(!(run = run && render.ProcessEvents())) break;
		render.Update();
		usleep(dut);

		ip.thinning(tex,tex);
		g_font->Print("thinning", 20, 10, 10);
		if(!(run = run && render.ProcessEvents())) break;
		render.Update();
		usleep(dut);

		ip.spreading(tex,tex);
		g_font->Print("spreading", 20, 10, 10);
		if(!(run = run && render.ProcessEvents())) break;
		render.Update();
		usleep(dut);

	}
	
	printf("... done.\n");

	return 0;
}



