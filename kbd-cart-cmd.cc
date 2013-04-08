/*
 Copyright (c) 2009 Technische Universitaet Muenchen, Informatik Lehrstuhl IX.
 Author: Ingo Kresse <kresse at in.tum.de>

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <termios.h> // stdin raw read
#include <string.h>
#include <stdio.h>
#include <signal.h> // SIGINT signal handling
#include <math.h>
#include <pthread.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <getopt.h>
#include <unistd.h>
#include <yarp/os/all.h>
using namespace yarp::os;

#include "vector.h"
#include "vector.c"

using namespace std;

string   gPort("kbd-cart");
string   gConn("");

// standard player command line options
void print_usage(int argc, char** argv)
{
  using namespace std;
  cerr << "USAGE:  " << *argv << " [options]" << endl << endl;
  cerr << "Where [options] can be:" << endl;
  cerr << "  -p <port base name>    : port base name for the yarp ports (default: \"kbd-cart\""<< endl;
  cerr << "  -c <robot_prefix>      : connect to the left or right arm. Example: \"/lwr/right\"" << endl;
  cerr << "  -?             : display this help"  << endl;
}

int parse_args(int argc, char** argv)
{
  // set the flags
  const char* optflags = "p:c:";
  int ch;

  // use getopt to parse the flags
  while((ch = getopt(argc, argv, optflags)) != -1 )
  {
    switch(ch)
    {
      // case values must match long_options
      case 'p': // port
          gPort = string(optarg);
          break;
      case 'c': //connect
          gConn = string(optarg);
          break;
      case '?': // help
      default:  // unknown
        print_usage(argc, argv);
	::exit(-1);
    }
  }
  return 0;
}

#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72

#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Z 0x7a
//#define KEYCODE_Z 0x79  // map 'Y' on the german keyboard

#define KEYCODE_X 0x78
#define KEYCODE_C 0x63

#define KEYCODE_G 0x67
#define KEYCODE_H 0x68
#define KEYCODE_J 0x6a
#define KEYCODE_B 0x62
#define KEYCODE_N 0x6e
#define KEYCODE_M 0x6d

#define KEYCODE_P 0x70

float xi=0,yi=0,zi=0,axi=0,ayi=0,azi=0;
float incrLin=0.01;
float incrAng=5.0*M_PI/180.0;
bool want_pose=true;
bool dirty=false;

struct termios cooked;
int kfd = 0;

pthread_mutex_t *controllerMutex;

void* keyboard_handler(void* arg)
{
	char c;
	struct termios raw;

	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("");
	puts("keyboard limb control");
	puts("---------------------------");
	puts("      moving:               rotating:");
	puts("");
	puts("  | X    Y    Z          | X    Y    Z");
	puts("--+-------------       --+-------------");
	puts("+ | a    s    d        + | g    h    j");
	puts("- | z    x    c        - | b    n    m");
	puts("");
	puts("q/w e/r: increase/decrease increments angles, displacements");
	puts("p: toggle updating current arm pose from robot");
	puts("");
	puts("anything else : stop");
	puts("---------------------------");

	while(true)
	{
		// get the next event from the keyboard
		if(read(kfd, &c, 1) < 0)
		{
			perror("read():");
			::exit(-1);
		}

		pthread_mutex_lock(controllerMutex);
		xi=0.0; yi=0.0; zi=0.0; axi=0.0; ayi=0.0; azi=0.0;
		switch(c)
		{
			case KEYCODE_Q:
				incrAng*=2;
				printf("increments: %.2fdeg (ang)\n",incrAng/M_PI*180);
				break;
			case KEYCODE_W:
				incrAng/=2;
				printf("decrements: %.2fdeg (ang)\n",incrAng/M_PI*180);
				break;
			case KEYCODE_E:
				incrLin*=2;
				printf("increments: %.4fm (lin)\n",incrLin);
				break;
			case KEYCODE_R:
				incrLin/=2;
				printf("decrements: %.4fm (lin)\n",incrLin);
				break;
			case KEYCODE_P:
				want_pose = !want_pose;
				printf("pose update %s\n", (want_pose) ? "on" : "off");
				break;
			case KEYCODE_A:
				xi+=incrLin;
				break;
			case KEYCODE_S:
				yi+=incrLin;
				break;
			case KEYCODE_D:
				zi+=incrLin;
				break;
			case KEYCODE_Z:
				xi-=incrLin;
				break;
			case KEYCODE_X:
				yi-=incrLin;
				break;
			case KEYCODE_C:
				zi-=incrLin;
				break;

			case KEYCODE_G:
				axi+=incrAng;
				break;
			case KEYCODE_H:
				ayi+=incrAng;
				break;
			case KEYCODE_J:
				azi+=incrAng;
				break;

			case KEYCODE_B:
				axi-=incrAng;
				break;
			case KEYCODE_N:
				ayi-=incrAng;
				break;
			case KEYCODE_M:
				azi-=incrAng;
				break;

			default:
				;
				//printf("#define KEYCODE_? 0x%x\n", c);
		}
		if(c != KEYCODE_Q && c != KEYCODE_W) {
			dirty=true;
		}
		pthread_mutex_unlock(controllerMutex);
	}

	

	return(0);
}

//! Ctrl-C handler
void catchsignal(int signo)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	::exit(0);
}


int main(int argc, char** argv)
{

	// install Ctrl-C handler
	struct sigaction act;
	act.sa_handler = catchsignal;
	act.sa_flags = 0;
	if ((sigemptyset(&act.sa_mask) == -1) ||
	    (sigaction(SIGINT, &act, NULL) == -1)) {
		printf("Failed to set SIGINT to handle Ctrl-C, oh well\n");
	}
	
	parse_args(argc, argv);

    Network yarp;
    BufferedPort<Bottle> portIn;
    BufferedPort<Bottle> portOut;

    string portInbaseS=gPort.c_str();
    portInbaseS="/"+portInbaseS+"/in";
    portIn.open(portInbaseS.c_str());

    string portOutbaseS=gPort.c_str();
    portOutbaseS="/"+portOutbaseS+"/out";
    portOut.open(portOutbaseS.c_str());

    
    if (gConn != string("")) {
        string prefix0=gConn.c_str();
        string objectS=prefix0+"/ofeeder/object";
        yarp::os::Network::connect(portOutbaseS.c_str(),objectS.c_str());
        string poseS=prefix0+"/vectorField/pose";
        yarp::os::Network::connect(poseS.c_str(),portInbaseS.c_str());
    }

	controllerMutex = new pthread_mutex_t;
	pthread_t dummy;
    int dummyargs;
	pthread_create(&dummy, NULL, &keyboard_handler, (void*)&dummyargs);

    ias_homo r;
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            r.m[i][j] = (i==j) ? 1.0 : 0.0;
    bool has_pose = false;

    double pos[3]={0.0,0.0,0.0};
    bool init = true;
	while(true)
	{
        Bottle *bottlein=portIn.read(false);
        if (bottlein!=NULL && (want_pose || !has_pose)) {
            if (bottlein->size()==16) {
                for (int i=0;i<16;i++) {
                    int y=i/4;
                    int x=i-y*4;
                    if ((x<3) && (y<3)) {
                        r.m[x][y]=bottlein->get(i).asDouble();
                    }
                    if ((x==3) && (y<3)) {
                        pos[y]=bottlein->get(i).asDouble();
                    }
                }
                init=false;
                has_pose=true;
            }
        }
		if(dirty && !init) {
			pthread_mutex_lock(controllerMutex);

			//ias_vector x,y,z;
			//ias_homo r;
            /*
			r.m[0][1] = y.x = ori.px;
			r.m[1][1] = y.y = ori.py;
			r.m[2][1] = y.z = ori.pz;

			r.m[0][2] = z.x = app.px;
			r.m[1][2] = z.y = app.py;
			r.m[2][2] = z.z = app.pz;
            */
			//x = ias_vector_cross(y,z);

			//r.m[0][0] = x.x;
			//r.m[1][0] = x.y;
			//r.m[2][0] = x.z;

			ias_quatern rot_now = ias_quatern_from_homo(r);

			ias_quatern rx = ias_quatern_from_axis(1,0,0,axi);
			ias_quatern ry = ias_quatern_from_axis(0,1,0,ayi);
			ias_quatern rz = ias_quatern_from_axis(0,0,1,azi);

			ias_quatern rot_new = ias_quatern_mul(rot_now, ias_quatern_mul(rx, ias_quatern_mul(ry, rz)));
            r = ias_homo_from_quatern(rot_new);

			pos[0] += xi;
			pos[1] += yi;
			pos[2] += zi;

            Bottle& bottleout = portOut.prepare();
            bottleout.clear();

            bottleout.addString("set");
            bottleout.addString("goal");
            Bottle& bottlelist=bottleout.addList();
            for (int i=0;i<4;i++) {
                for (int j=0;j<4;j++) {
                    if ((i<3) && (j<3)) {
                        bottlelist.addDouble(r.m[j][i]);
                    }
                    if ((i<3) && (j==3)) {
                        bottlelist.addDouble(pos[i]);
                    }
                    if ((i==3) && (j<3)) {
                        bottlelist.addDouble(0);
                    }
                    if ((i==3) && (j==3)){
                        bottlelist.addDouble(1);
                    }
                }
            }
            bottlelist.addDouble(0.1);
            portOut.write();

			pthread_mutex_unlock(controllerMutex);
		}
		dirty = false; // we've handled the changes

        struct timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = 50*1000;
        nanosleep(&ts, NULL);
        
	}
	
	// restore terminal settings
	catchsignal(0);

	return(0);
}

