
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



int main(int argc, char** argv)
{
  // we throw exceptions on creation if we fail
  try
    {
      std::cout << robot << std::endl;
      std::cout << "argc = " << argc << std::endl;

      pp.SetMotorEnable (true);
      for (int i=1; i<argc-1; i+=2) {
	double x=atof(argv[i]);
	double y=atof(argv[i+1]);
	gotoPos(x, y, 0.0);
      }

    }

  catch (PlayerCc::PlayerError e)
    {
      std::cerr << e << std::endl;
      return -1;
    }
}
