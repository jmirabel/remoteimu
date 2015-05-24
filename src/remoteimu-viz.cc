// Copyright (c) 2015, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of remoteimu.
// remoteimu is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// remoteimu is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// remoteimu. If not, see <http://www.gnu.org/licenses/>.

/* Sample UDP client */

#include <netinet/in.h>
#include <iostream>
#include <sstream>

#include <state-observation/dynamical-system/imu-magnetometer-dynamical-system.hpp>
#include <state-observation/dynamical-system/dynamical-system-simulator.hpp>
#include <state-observation/observer/extended-kalman-filter.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>
#include <state-observation/tools/definitions.hpp>

#include <gepetto/viewer/corba/client.hh>

#include <remoteimu/mouse.hh>

using namespace remoteimu;

void transRotToCORBA (const stateObservation::Vector& tr, CORBA::Float (&c)[7])
{
  for (int i=0;i<3;i++) c[i] = (float)tr[i];
  Eigen::Quaternion<double> q (
    stateObservation::kine::rotationVectorToAngleAxis(stateObservation::Vector3(tr.segment<3>(stateObservation::kine::ori)))
    );
  c[3]=(float)q.w();
  c[4]=(float)q.x();
  c[5]=(float)q.y();
  c[6]=(float)q.z();
}

class EventSender : public MouseEventSender
{
  public:
    EventSender (graphics::corbaServer::Client& c)
      : viz (c)
    {}

    void mouseEvent (const MouseEventSender::Event e) {
      CORBA::Float boxPosition[7];
      for (int i=0;i<3;i++) boxPosition[i  ] = e.pos[i];
      for (int i=0;i<4;i++) boxPosition[i+3] = e.ori[i];
      viz.gui()->applyConfiguration ("example/IMU", boxPosition);
      viz.gui()->refresh ();
    }

  private:
    graphics::corbaServer::Client& viz;
};

graphics::corbaServer::Client::WindowID initVisualizer (graphics::corbaServer::Client& c)
{
  graphics::corbaServer::Client::WindowID wid;
  try {
    wid = c.gui()->getWindowID ("remoteimu");
    return wid;
  } catch (gepetto::Error& e) {
    wid = c.gui()->createWindow ("remoteimu");
  }
  c.gui()->createScene ("example");
  c.gui()->addSceneToWindow ("example", wid);
  float black[4] = {1.,1.,1.,1.};
  c.gui()->addBox ("example/IMU", 0.07f, 0.12f, 0.005f, black);
  c.gui()->addLandmark ("example", 0.05f);
  c.gui()->addLandmark ("example/IMU", 0.05f);
  return wid;
}

int main(int argc, char** argv)
{
  int port = 6000;
  std::string host = "0.0.0.0";
  std::string iiop = "corbaloc:rir:/NameService";
  switch (argc) {
    default:
    case 4:
      iiop = std::string (argv[3]);
    case 3:
      port = atoi(argv[2]);
    case 2:
      host = std::string (argv[1]);
      break;
    case 1: // argv[0] contains the executable name.
      std::cout << "Usage: " << argv[0] << " [host [port [iiop_address]]]" << std::endl;
      break;
    case 0: // This should not happen
      break;
  }
  std::cout << "Listening on " << host << ":" << port << std::endl;
  std::cout << "IIOP address: " << iiop << std::endl;
  UDPServer* server (new UDPServer (host, port));
  Mouse mouse (server, 60);

  // Build a client, for visualization
  graphics::corbaServer::Client viz (0, NULL);
  viz.connect(iiop.c_str());
  /* graphics::corbaServer::Client::WindowID wid = */ initVisualizer (viz);
  CORBA::Float boxPosition[7];
  boxPosition[0]=0; boxPosition[1]=0; boxPosition[2]=0;
  boxPosition[3]=1; boxPosition[4]=0; boxPosition[5]=0; boxPosition[6]=0;
  viz.gui()->applyConfiguration ("example/IMU", boxPosition);
  EventSender es (viz);
  mouse.setMouseEventSender (&es);

  mouse.handleEvents (true);
}
