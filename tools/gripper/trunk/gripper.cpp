#include <libplayerc++/playerc++.h>
#include <iostream>

using namespace PlayerCc;
using namespace std;

static void PrintKeyUsage()
{
	printf("q .. quit\n"
         "o .. open gripper\n"
				 "c .. close gripper\n"
				 "i .. info on current gripper position\n");
}

int main(int argc, char **argv)
{
  // We throw exceptions on creation if we fail
  try
  {
		static double GRIPPER_OPEN_POS = 0.05;
		static double GRIPPER_CLOSE_POS = 0.00;

    string host = PlayerCc::PLAYER_HOSTNAME;
    int port = PlayerCc::PLAYER_PORTNUM;
    int iface_id = 0;

    printf("usage: %s [iface [host port]]\n", argv[0]);
    printf("  Connect to arm interface and open/close gripper.\n"
           "  iface .. interface number, default 0\n"
           "  host .. host on which player is running, default localhost\n"
           "  port .. player port, default 6665\n\n");

    if(argc >= 2)
      iface_id = atoi(argv[1]);
    if(argc >= 3)
      host = argv[2];
    if(argc >= 4)
      port = atoi(argv[3]);

    printf("connecting to host:port:iface : %s:%d:%d\n", host.c_str(), port, iface_id);

    // Create a player client object, using the variables assigned by the
    // call to parse_args()
    PlayerClient robot(host, port);

    // Subscribe to the arm proxy
    ActArrayProxy ap(&robot, iface_id);

    // Print out some stuff
    std::cout << robot << std::endl;

		PrintKeyUsage();
		int c = 0;
		do
    {
      // This blocks until new data comes
      robot.Read();

			c = getchar();
			switch(c)
			{
				case 'o':
					// left and right finger
					ap.MoveTo(5, GRIPPER_OPEN_POS);
					ap.MoveTo(6, GRIPPER_OPEN_POS);
					break;
				case 'c':
					// left and right finger
					ap.MoveTo(5, GRIPPER_CLOSE_POS);
					ap.MoveTo(6, GRIPPER_CLOSE_POS);
					break;
				case 'i':
				{
          player_actarray_actuator_t data_l = ap.GetActuatorData(5);
    		  player_actarray_actuator_t data_r = ap.GetActuatorData(6);
					printf("pos (vel) left/right: %f (%f) / %f (%f)\n", data_l.position,
              data_l.speed, data_r.position, data_r.speed);
					break;
				}
				default:
					PrintKeyUsage();
					break;
			}
    } while(c != 'q');
  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << "Error:" << e << std::endl;
    return -1;
  }
}
