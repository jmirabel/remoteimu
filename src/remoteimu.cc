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

#include <remoteimu/udpserver.hh>


using namespace remoteimu;

void transRotToCORBA (const stateObservation::Vector& tr, CORBA::Float (&c)[7])
{
  // for (int i=0;i<3;i++) c[i] = (float)tr[i];
  Eigen::Quaternion<double> q (
    stateObservation::kine::rotationVectorToAngleAxis(stateObservation::Vector3(tr.segment<3>(stateObservation::kine::ori)))
    );
  c[3]=(float)q.w();
  c[4]=(float)q.x();
  c[5]=(float)q.y();
  c[6]=(float)q.z();
}

void transRotToSE3 (const stateObservation::Vector& tr, se3::SE3& pos)
{
  se3::SE3::Vector3& trans = pos.translation ();
  se3::SE3::Matrix3& rot   = pos.rotation ();
  for (int i=0;i<3;i++) trans[i] = (float)tr[i];
  stateObservation::Matrix3 rotSO =
    stateObservation::kine::rotationVectorToAngleAxis(stateObservation::Vector3(tr.segment<3>(stateObservation::kine::ori)))
    .toRotationMatrix();
  for (int i=0;i<3;i++) for (int j=0;j<3;j++) rot(i,j) = (float)rotSO(i,j);
}

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
  UDPServer server (host, port);
  char recvline[1000];
  int len,k = 0;

  std::string comma;
  int sensorId;

  ///Sizes of the states for the state, the measurement, and the input vector
  const unsigned stateSize=18;
  const unsigned measurementSize=9;
  // const unsigned measurementSize=12;
  const unsigned inputSize=0;

  /// The measurements and time of measurements.
  double lastMeasTime = -1, time, dt;
  stateObservation::Vector y (measurementSize);y.setZero();

  ///initialization of the extended Kalman filter
  stateObservation::ExtendedKalmanFilter filter(stateSize, measurementSize, inputSize, false);

  ///initalization of the functor
  stateObservation::IMUMagnetometerDynamicalSystem imuFunctor;
  // imuFunctor.setSamplingPeriod(dt);
  filter.setFunctor(& imuFunctor);

  ///the initalization of the estimation of the initial state
  stateObservation::Vector xh0 = stateObservation::Vector::Zero (stateSize);
  filter.setState(xh0,0);

  ///computation and initialization of the covariance matrix of the initial state
  stateObservation::Matrix p (stateSize, stateSize);
  p.setIdentity (); p *= 1e-3;
  filter.setStateCovariance(p);

  ///set initial input
  // filter.setInput(u[y.getFirstIndex()-1],y.getFirstIndex()-1);

  ///The covariance matrix of the process noise and the measurement noise
  /// for the extended Kalman filter
  stateObservation::Matrix r(measurementSize, measurementSize);
  stateObservation::Matrix q(stateSize, stateSize);
  r.setIdentity ();
  r.diagonal().head<3>()     = 1e-1*stateObservation::Vector3::Ones ();
  r.diagonal().segment<3>(3) = 1e-5*stateObservation::Vector3::Ones ();
  // r.diagonal().segment<3>(6) = 1e-2*stateObservation::Vector3::Ones ();
  r.diagonal().tail<3>()     = 1e-1*stateObservation::Vector3::Ones ();
  q.setIdentity (); q *= 1e-2;
  q.diagonal().segment<3>(stateObservation::kine::linAcc) = 1e-0*stateObservation::Vector3::Ones ();
  q.diagonal().segment<3>(stateObservation::kine::angAcc) = 1e-0*stateObservation::Vector3::Ones ();
  filter.setR(r);
  filter.setQ(q);
  // Calibration
  Eigen::Matrix3d K;
  K << 0.976138  , -0.0208134 , -0.0043994,
      -0.0163318 ,  1.00101   ,  0.0107982,
       0.00238409, -0.00441258,  1.02669  ;
  K.transposeInPlace ();
  Eigen::Vector3d bias;
  bias << 0.0886857, 0.0924424, -0.181003;

  ///set the derivation step for the finite difference method
  stateObservation::Vector dx=filter.stateVectorConstant(1)*1e-4;

  // Build a client, for visualization
  graphics::corbaServer::Client viz (0, NULL);
  viz.connect(iiop.c_str());
  /* graphics::corbaServer::Client::WindowID wid = */ initVisualizer (viz);
  CORBA::Float boxPosition[7];
  boxPosition[0]=0; boxPosition[1]=0; boxPosition[2]=0;
  boxPosition[3]=1; boxPosition[4]=0; boxPosition[5]=0; boxPosition[6]=0;
  transRotToCORBA (xh0, boxPosition);
  viz.gui()->applyConfiguration ("example/IMU", boxPosition);

  // Acceleration filtering
  // stateObservation::Vector3 acc_n, acc_n1;
  // stateObservation::Vector3 ahat_n, ahat_n1, ahat_n2;
  // const double tau1 = 2, tau2 = 0.5, gain = 1;
  // acc_n.setZero(); acc_n1.setZero ();
  // ahat_n.setZero(); ahat_n1.setZero (); ahat_n2.setZero ();

  while ((len = server.recv(recvline, 999)) >= 0)
  {
    recvline[len] = '\0';
    // std::cout << recvline << std::endl;
    char *pEnd = recvline;
    std::stringstream ss (recvline);
    // ss >> time;
    time = strtod (pEnd, &pEnd);
    dt = time - lastMeasTime;
    /// If UDP packet is late, forget it
    if (dt <= 0) continue;
    if (lastMeasTime < 0) {
      lastMeasTime = time;
      continue;
    }
    int updated = 0;
    while (pEnd - recvline <= len) {
      pEnd++;
      sensorId = strtol (pEnd, &pEnd, 10);
      int shift;
      switch (sensorId) {
        case 3: // Accelerometer
          /*
          for (int j = 0; j < 3; j++) {
            pEnd++;
            strtod (pEnd, &pEnd);
          }
          continue; // */
          updated += 1;
          shift = 0;
          break;
        case 4: // Gyrometer
          updated += 2;
          shift = 1;
          break;
        case 5: // Magnetometer
          updated += 4;
          shift = 2;
          break;
        case 82: // Linear acceleration
          updated += 8;
          shift = 0;
          break;
        case 83: // Gravity sensor
          updated += 16;
          shift = 3;
          break;
        default:
          continue;
      }
      for (int j = 0; j < 3; j++) {
        // ss >> comma >> y[3*shift+j];
        pEnd++;
        y[3*shift+j] = strtod (pEnd, &pEnd);
        // std::cout << 3*shift+j << ": "<< y[3*shift+j] << std::endl;
      }
      if (sensorId == 3) {
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
      }
    }
    // Select Accelerometer or Linear acceleration
    if (updated != 4+2+1) continue;
    // if (updated != 16+8+4+2) continue;
    // std::cout << time;
    for (int i = 2; i < 3; i++) {
    // for (int i = 0; i < 4; i++) {
    // for (int i = 0; i < 4; i++) {
      // std::cout << ", " << i;
      // for (int j = 0; j < 3; j++) std::cout << ", " << y[3*i+j];
    }
    // std::cout << std::endl;
    k++;
    imuFunctor.setSamplingPeriod(dt);
    filter.setMeasurement (y, k);
    lastMeasTime = time;

    stateObservation::Matrix a = filter.getAMatrixFD(dx);
    stateObservation::Matrix c = filter.getCMatrixFD(dx);
    filter.setA(a);
    filter.setC(c);

    ///get the estimation and give it to the array
    stateObservation::Vector xhk=filter.getEstimatedState(k);

    ///regulate the part of orientation vector in the state vector
    xhk.segment(stateObservation::kine::ori,3)=stateObservation::kine::regulateOrientationVector
      (xhk.segment(stateObservation::kine::ori,3));

    // Reset position and velocity to 0 from time to time.
    if (k % 5 == 0) {
      xhk.segment <3> (stateObservation::kine::linVel).setZero ();
      std::cout << "==================== Reset ===========================" << std::endl;
    }
    if (k % 100 == 0) {
      xhk.segment <3> (stateObservation::kine::pos).setZero ();
      std::cout << "==================== Reset ===========================" << std::endl;
    }

    ///give the new value of the state to the kalman filter.
    ///This step is usually unnecessary, unless we modify the
    ///value of the state esimation which is the case here.
    filter.setState(xhk,k);

    transRotToCORBA (xhk, boxPosition);
    viz.gui()->applyConfiguration ("example/IMU", boxPosition);
    viz.gui()->refresh ();
    // std::cout << time << ": " << xhk[0] << "," << xhk[1] << "," << xhk[2] << ","
                              // << xhk[3] << "," << xhk[4] << "," << xhk[5] << "," << std::endl;
  }
}
