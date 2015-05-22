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

#ifndef REMOTEIMU_MOUSE_HH
#define REMOTEIMU_MOUSE_HH

#include <map>

#include <state-observation/dynamical-system/imu-magnetometer-dynamical-system.hpp>
#include <state-observation/observer/extended-kalman-filter.hpp>

#include <remoteimu/udpserver.hh>

namespace remoteimu {
  class MouseEventSender
  {
    public:
      enum EventType {
        Orientation,
        Position,
        LinVelocity,
        AngVelocity
      };
      struct Event {
        EventType type;
        double pos[3], ori[3], linAlg[3], linVel[3];
      };
      virtual void mouseEvent (const Event e) = 0;
  };

  class Mouse
  {
    public:
      typedef unsigned short int SensorID;
      SensorID ACCELEROMETER;
      SensorID GYROMETER    ;
      SensorID MAGNETOMETER ;
      SensorID LIN_ACC      ;
      SensorID GRAVITY      ;

      /// Constructor
      /// \param server from which the IMU data are received.
      /// \param maxRate number of emitted event.
      Mouse (UDPServer* server, int maxRate = 125);

      ~Mouse();

      void setMouseEventSender (MouseEventSender* me) {
        me_ = me;
      }

      /// Receive IMU datas and emit events.
      /// \param loop if set to true, datas will be read until 
      void handleEvents (bool loop = true);

      /// Thread-safe
      void stopEventHandler () {
        loop_ = false;
      }

    private:
      void initFilter ();

      MouseEventSender* me_;
      UDPServer* server_;
      int maxRate_;

      bool loop_;

      const unsigned stateSize;
      const unsigned measurementSize;
      const unsigned inputSize;

      stateObservation::ExtendedKalmanFilter filter_;
      stateObservation::IMUMagnetometerDynamicalSystem imuFunctor_;
      stateObservation::Vector dx_;

      struct SensorDataParser {
        typedef Eigen::VectorXd Vector;
        typedef std::pair <SensorID, int> SensorDataLength;
        typedef std::pair <SensorID, Vector> SensorData;

        double time_;
        std::map <SensorID, Vector> sensorDatas_;
        std::map <SensorID, int> sensorDatasLength_;

        void parse (int len, const char* sensorString);
        bool has (const SensorID sid) const {
          return sensorDatas_.find (sid) != sensorDatas_.end ();
        }
      } sensorParser_;
  };
} // namespace remoteimu

#endif // REMOTEIMU_MOUSE_HH
