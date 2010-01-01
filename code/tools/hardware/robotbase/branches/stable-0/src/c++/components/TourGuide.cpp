
#include <libplayerc++/playerc++.h>
#include <iostream>
using namespace PlayerCc;

PlayerClient robot("localhost");
Position2dProxy pp(&robot, 1);

void gotoPos(double x, double y, double theta) {
  std::cout << "goto (" << x << ", " << y <<")" << std::endl;
  pp.GoTo(x, y, theta);
  double dist=1e10;
  std::cout << "wait to reach goal" << std::endl;
  while (dist>0.5) {
    robot.Read();
    double dx=pp.GetXPos()-x;
    double dy=pp.GetYPos()-y;
    dist=sqrt(dx*dx + dy*dy);
    std::cout << "dist = " << dist<< std::endl;
    usleep(100000);
  }
}



int main(int argc, char** argv)
{
  // we throw exceptions on creation if we fail
  try
    {
      std::cout << robot << std::endl;

      pp.SetMotorEnable (true);

      gotoPos(1.0, 0.0, 0.0);
      gotoPos(0.0, 0.0, 0.0);

    }

  catch (PlayerCc::PlayerError e)
    {
      std::cerr << e << std::endl;
      return -1;
    }
}
