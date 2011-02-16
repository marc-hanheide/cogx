
#include <libplayerc++/playerc++.h>
#include <iostream>
using namespace PlayerCc;

PlayerClient robot("localhost");
Position2dProxy pp(&robot, 1);

void gotoPos(double x, double y, double theta) {
  std::cout << "goto (" << x << ", " << y <<")" << std::endl;
  pp.GoTo(x, y, theta);
  double dist=1e10;
  while (dist>0.5) {
    robot.Read();
    double dx=pp.GetXPos()-x;
    double dy=pp.GetYPos()-y;
    dist=sqrt(dx*dx + dy*dy);
    std::cout << "  wait to reach goal (dist = " << dist<< ")\r" <<std::flush;
    usleep(100000);
  }
  std::cout << std::endl;
}

void watch() {
  robot.Read();
  double lastX=pp.GetXPos();
  double lastY=pp.GetYPos();
  double dist=0.0;

  while(true) {
    robot.Read();
    double cx=pp.GetXPos();
    double cy=pp.GetYPos();
    if (cx != lastX || cy != lastY) {
      double dx=cx-lastX;
      double dy=cy-lastY;
      dist+=sqrt(dx*dx + dy*dy);
      lastX=cx;
      lastY=cy;
      std::cout << "  pos = (" 
		<< cx <<", " << cy << ") dist = "
		<< dist <<"               "
		<<"\r" <<std::flush;
      usleep(20000);
    }
    
  }
}

int main(int argc, char** argv)
{
  // we throw exceptions on creation if we fail
  try
    {
      std::cout << robot << std::endl;

      if (argc > 1) {
	pp.SetMotorEnable (true);
	for (int i=1; i<argc-1; i+=2) {
	  double x=atof(argv[i]);
	  double y=atof(argv[i+1]);
	  gotoPos(x, y, 0.0);
	}
      } else {
	watch();
      }


    }

  catch (PlayerCc::PlayerError e)
    {
      std::cerr << e << std::endl;
      return -1;
    }
}
