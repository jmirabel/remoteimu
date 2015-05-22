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

#include <netinet/in.h>
#include <iostream>
#include <sstream>

#include <state-observation/tools/definitions.hpp>

#include <Eigen/Core>

#include <remoteimu/udpserver.hh>

using namespace remoteimu;

typedef Eigen::Matrix<double, Eigen::Dynamic, 3> MeasurementVector;
typedef Eigen::Matrix<double, Eigen::Dynamic, 4> ExtentedMeasurementVector;
typedef Eigen::Matrix<double, 4, 3> CalibrationMatrix;

void getMeasurements (Eigen::Ref<MeasurementVector> m, UDPServer& server)
{
  char recvline[1000];
  int len,k = 0;
  int N = m.rows();

  std::string comma;
  int sensorId;

  while (k < N && (len = server.recv(recvline, 999)) >= 0)
  {
    recvline[len] = '\0';
    std::cout << recvline << std::endl;
    char *pEnd = recvline;
    strtod (pEnd, &pEnd);
    bool done = false;
    while (!done || (pEnd - recvline <= len)) {
      pEnd++;
      sensorId = strtol (pEnd, &pEnd, 10);
      switch (sensorId) {
        case 3: // Accelerometer
          for (int j = 0; j < 3; j++) {
            pEnd++;
            m(k,j) = strtod (pEnd, &pEnd);
          }
          done = true;
          break;
        default:
          for (int j = 0; j < 3; j++) {
            pEnd++;
            strtod (pEnd, &pEnd);
          }
          continue;
      }
    }
    if (!done) continue;
    k++;
  }
}

void clearServerInput (UDPServer& server)
{
  char recvline[1000];
  while (server.timed_recv (recvline, 999, 5) >= 0) {
    std::cout << "Skip " << recvline << std::endl;
  }
}

void getReady (std::string instruction, UDPServer& server)
{
  std::cout << instruction << std::endl;
  std::cout << "Press any key when ready...";
  std::cin.ignore ();
  clearServerInput (server);
}

int main(int argc, char** argv)
{
  // Number of measure to get in a position.
  int N = 1000;
  // Number of steps
  int steps = 6;

  int port = 6000;
  std::string host = "0.0.0.0";

  switch (argc) {
    default:
    case 4:
      N = atoi(argv[3]);
    case 3:
      port = atoi(argv[2]);
    case 2:
      host = std::string (argv[1]);
      break;
    case 1: // argv[0] contains the executable name.
      std::cout << "Usage: " << argv[0] << " [host [port [measurement_number]]]" << std::endl;
      break;
    case 0: // This should not happen
      break;
  }
  std::cout << "Listening on " << host << ":" << port << std::endl;
  std::cout << "Number of measurements: " << N << std::endl;
  UDPServer server (host, port);

  MeasurementVector uncalib (N, 3);
  ExtentedMeasurementVector uncalibMean (steps, 4);
  MeasurementVector calib (steps, 3);
  calib.setZero ();
  uncalibMean.col (3).setOnes ();

  // Step 1
  getReady ("Put your phone Z-axis up on a flat surface and wait.", server);
  getMeasurements (uncalib, server);
  uncalibMean.row(0).leftCols (3) = ((double) 1/N ) * Eigen::VectorXd::Ones (N).transpose () * uncalib;
  calib (0, 2) = stateObservation::cst::gravityConstant;

  // Step 2
  getReady ("Put your phone Z-axis down on a flat surface and wait.", server);
  getMeasurements (uncalib, server);
  uncalibMean.row(1).leftCols (3) = ((double) 1/N ) * Eigen::VectorXd::Ones (N).transpose () * uncalib;
  calib (1, 2) = - stateObservation::cst::gravityConstant;

  // Step 3
  getReady ("Put your phone X-axis up on a flat surface and wait.", server);
  getMeasurements (uncalib, server);
  uncalibMean.row(2).leftCols (3) = ((double) 1/N ) * Eigen::VectorXd::Ones (N).transpose () * uncalib;
  calib (2, 0) = + stateObservation::cst::gravityConstant;

  // Step 4
  getReady ("Put your phone X-axis down on a flat surface and wait.", server);
  getMeasurements (uncalib, server);
  uncalibMean.row(3).leftCols (3) = ((double) 1/N ) * Eigen::VectorXd::Ones (N).transpose () * uncalib;
  calib (3, 0) = - stateObservation::cst::gravityConstant;

  // Step 5
  getReady ("Put your phone Y-axis up on a flat surface and wait.", server);
  getMeasurements (uncalib, server);
  uncalibMean.row(4).leftCols (3) = ((double) 1/N ) * Eigen::VectorXd::Ones (N).transpose () * uncalib;
  calib (4, 1) = + stateObservation::cst::gravityConstant;

  // Step 6
  getReady ("Put your phone Y-axis down on a flat surface and wait.", server);
  getMeasurements (uncalib, server);
  uncalibMean.row(5).leftCols (3) = ((double) 1/N ) * Eigen::VectorXd::Ones (N).transpose () * uncalib;
  calib (5, 1) = - stateObservation::cst::gravityConstant;

  Eigen::Matrix <double, 4, Eigen::Dynamic> MT = uncalibMean.transpose ();
  Eigen::Matrix4d MTxM = MT*uncalibMean;
  std::cout << "M" << std::endl;
  std::cout << uncalibMean << std::endl;
  std::cout << "MTxM" << std::endl;
  std::cout << MTxM << std::endl;
  std::cout << "det(MTxM)" << std::endl;
  std::cout << MTxM.determinant () << std::endl;
  CalibrationMatrix K = MTxM.inverse () * MT * calib;

  std::cout << "The calibration matrix is:" << std::endl;
  std::cout << K << std::endl;
}
