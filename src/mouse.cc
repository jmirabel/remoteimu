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

#include "remoteimu/mouse.hh"

#include <iostream>

namespace remoteimu {
  Mouse::Mouse (UDPServer* server, int maxRate) :
    ACCELEROMETER (3), GYROMETER (4), MAGNETOMETER (5),
    LIN_ACC (82), GRAVITY (83),
    server_ (server), maxRate_ (maxRate), nbMeasToInit (100),
    q_j0 (1, 0, 0, 0), q_m0_inv (1, 0, 0, 0), loop_ (true),
    stateSize (18), measurementSize (9), inputSize(0),
    filter_ (stateSize, measurementSize, inputSize, false, false)
  {
    sensorParser_.sensorDatasLength_ [ACCELEROMETER] = 3;
    sensorParser_.sensorDatasLength_ [GYROMETER] = 3;
    sensorParser_.sensorDatasLength_ [MAGNETOMETER] = 3;
    sensorParser_.sensorDatasLength_ [LIN_ACC] = 3;
    sensorParser_.sensorDatasLength_ [GRAVITY] = 3;

    initFilter ();
  }

  void Mouse::initFilter ()
  {
    ///initalization of the functor
    imuFunctor_ = stateObservation::IMUMagnetometerDynamicalSystem ();
    filter_.setFunctor(&imuFunctor_);

    ///the initalization of the estimation of the initial state
    stateObservation::Vector xh0 = stateObservation::Vector::Zero (stateSize);
    filter_.setState(xh0,0);

    ///computation and initialization of the covariance matrix of the initial state
    stateObservation::Matrix p (stateSize, stateSize);
    p.setIdentity (); p *= 1e-3;
    filter_.setStateCovariance(p);

    ///The covariance matrix of the process noise and the measurement noise
    /// for the extended Kalman filter
    stateObservation::Matrix r(measurementSize, measurementSize);
    stateObservation::Matrix q(stateSize, stateSize);
    r.setIdentity ();
    r.diagonal().head<3>()     = 1e-1*stateObservation::Vector3::Ones ();
    r.diagonal().segment<3>(3) = 1e-5*stateObservation::Vector3::Ones ();
    r.diagonal().tail<3>()     = 1e-1*stateObservation::Vector3::Ones ();
    q.setIdentity (); q *= 1e-2;
    q.diagonal().segment<3>(stateObservation::kine::linAcc) = 1e-0*stateObservation::Vector3::Ones ();
    q.diagonal().segment<3>(stateObservation::kine::angAcc) = 1e-0*stateObservation::Vector3::Ones ();
    filter_.setR(r);
    filter_.setQ(q);

    ///set the derivation step for the finite difference method
    dx_ = filter_.stateVectorConstant(1)*1e-4;
  }

  void Mouse::clearUDPBuffer ()
  {
    char recvline[1000];
    while (server_->timed_recv (recvline, 999, 5) >= 0) {
      // std::cout << "Skip " << recvline << std::endl;
    }
  }

  Mouse::~Mouse ()
  {
    if (server_) delete server_;
  }

  void Mouse::handleEvents (bool loop)
  {
    clearUDPBuffer ();

    loop_ = loop;
    char recvline[1000];
    int len, k=0;
    double lastTime = -1, dt, lastEvent = -1;
    stateObservation::Vector y (measurementSize);y.setZero();

    do {
      len = server_->timed_recv (recvline, 999, 5);
      if (len < 0) {
        if (loop_) continue;
        else break;
      }
      recvline[len] = '\0';

      sensorParser_.sensorDatas_.clear ();
      sensorParser_.parse (len, recvline);

      dt = sensorParser_.time_ - lastTime;
      /// If UDP packet is late, forget it
      if (dt <= 0) continue;
      if (lastTime < 0) {
        lastTime = sensorParser_.time_;
        continue;
      }

      if (!(sensorParser_.has (ACCELEROMETER) && sensorParser_.has (GYROMETER)
          && sensorParser_.has (MAGNETOMETER))) continue;
      y.head <3> ()     = sensorParser_.sensorDatas_ [ACCELEROMETER];
      y.segment <3> (3) = sensorParser_.sensorDatas_ [GYROMETER];
      y.tail <3> ()     = sensorParser_.sensorDatas_ [MAGNETOMETER];

      k++;
      imuFunctor_.setSamplingPeriod(dt);
      filter_.setMeasurement (y, k);
      lastTime = sensorParser_.time_;

      stateObservation::Matrix a = filter_.getAMatrixFD(dx_);
      stateObservation::Matrix c = filter_.getCMatrixFD(dx_);
      filter_.setA(a);
      filter_.setC(c);

      ///get the estimation and give it to the array
      stateObservation::Vector xhk=filter_.getEstimatedState(k);

      ///regulate the part of orientation vector in the state vector
      xhk.segment(stateObservation::kine::ori,3)=stateObservation::kine::regulateOrientationVector
        (xhk.segment(stateObservation::kine::ori,3));

      // Reset position and velocity to 0 from time to time.
      xhk.segment <3> (stateObservation::kine::linVel).setZero ();
      xhk.segment <3> (stateObservation::kine::pos).setZero ();

      // give the new value of the state to the kalman filter.
      // This step is usually unnecessary, unless we modify the
      // value of the state esimation which is the case here.
      filter_.setState(xhk,k);

      if (k == nbMeasToInit) {
        using namespace stateObservation;
        q_m0_inv = Eigen::Quaternion<double> (
            kine::rotationVectorToAngleAxis(
              Vector3(xhk.segment<3>(kine::ori)))
            ).inverse ();
      }

      // Emit an event if necessary
      if (k >= nbMeasToInit &&
          (lastEvent < 0 || (sensorParser_.time_ - lastEvent)*maxRate_ > 1)) {
        using namespace stateObservation;
        MouseEventSender::Event e;
        e.type = MouseEventSender::Orientation;
        for (int i=0;i<3;i++) e.pos[i] = xhk [stateObservation::kine::pos + i];
        Eigen::Quaternion<double> q =
          Eigen::Quaternion<double> ( kine::rotationVectorToAngleAxis(
                Vector3(xhk.segment<3>(kine::ori))))
          * q_m0_inv * q_j0;
        e.ori[0]=q.w(); e.ori[1]=q.x(); e.ori[2]=q.y(); e.ori[3]=q.z();
        me_->mouseEvent (e);
        lastEvent = sensorParser_.time_;
      }
    } while (loop_);
  }

  void Mouse::SensorDataParser::parse (const int len, const char* sString)
  {
    char *pEnd = const_cast<char*>(sString);
    time_ = strtod (pEnd, &pEnd);

    while (pEnd - sString <= len) {
      pEnd++;
      SensorID sensorId = (SensorID) strtol (pEnd, &pEnd, 10);
      int N = 3;

      std::map <SensorID, int>::const_iterator itLength
       = sensorDatasLength_.find (sensorId);

      // TODO This sensor is not known...
      // Pop 3 values - this is a guess.
      if (itLength != sensorDatasLength_.end ())
        N = itLength->second;
      Vector values (N);

      for (int i = 0; i < N; i++) {
        pEnd++;
        values[i] = strtod (pEnd, &pEnd);
      }

      //
      // TODO Here, we could filter raw datas maybe.
      //
      // if (sensorId == 3) {
        // y.segment<3> (3*shift) = K*y.segment<3> (3*shift) + bias;
        // acc_n1 = acc_n;
        // acc_n = y.segment<3> (3*shift);
        // ahat_n2 = ahat_n1;
        // ahat_n1 = ahat_n;
        // double alpha1 = exp (-dt/tau1), alpha2 = exp (-dt/tau2);
        // ahat_n = tau1*tau2*(1-alpha1)*(1-alpha2) * gain * acc_n1 + (alpha1+alpha2) * ahat_n1 - alpha1*alpha2*ahat_n2;
        // y.segment <3> (3*shift) = ahat_n;
        // std::cout << dt << std::endl;
        // std::cout << alpha1 << std::endl;
        // std::cout << alpha2 << std::endl;
        // std::cout << acc_n << std::endl;
        // std::cout << ahat_n << std::endl;
      // }

      sensorDatas_ [sensorId] = values;
    }
  }

  void Mouse::setInitialQuat (const Eigen::Quaternion<double> q)
  {
    q_j0 = q;
  }
} // namespace remoteimu
