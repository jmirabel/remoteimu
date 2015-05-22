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

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include<signal.h>

#include <remoteimu/udpserver.hh>
#include <remoteimu/udpclient.hh>

using namespace remoteimu;

bool sigInt = false;

void sig_handler(int signo)
{
  if (signo == SIGINT) {
    std::cout << "Exiting..." << std::endl;
    sigInt = true;
  }
}

void replay (std::string filename, std::string host, int port)
{
  std::ifstream f;
  f.open (filename.c_str());
  if (!f.is_open ()) exit (1);

  std::cout << "Sending to " << host << ":" << port << std::endl;
  std::cout << "Reading datas from file " << filename << std::endl;
  
  UDPClient client (host, port);
  char tts[256];
  char sendline[1000];

  while (!sigInt) {
    if (f.eof ()) break;
    f.getline (tts, 256, ' ').getline (sendline, 1000);
    usleep (atoi (tts));
    client.send (sendline, f.gcount ());
  }
  
  f.close ();
}

void save (std::string filename, std::string host, int port)
{
  std::ofstream f;
  f.open (filename.c_str());
  if (!f.is_open ()) exit (1);

  std::cout << "Listening on " << host << ":" << port << std::endl;
  std::cout << "Writing datas to file " << filename << std::endl;

  struct timeval last, now;  
  long int elapsed;
  gettimeofday(&last, NULL);  

  UDPServer server (host, port);
  char recvline[1000];
  int len;

  while (!sigInt) {
    len = server.recv (recvline, 999);
    gettimeofday (&now, NULL);
    recvline[len] = '\0';
    elapsed = (now.tv_sec - last.tv_sec) * 1000000 + (now.tv_usec - last.tv_usec);
    last = now;
    f << elapsed << ", " << recvline << std::endl;
  }

  f.close ();
}

int main(int argc, char** argv)
{
  if (signal(SIGINT, sig_handler) == SIG_ERR)
    std::cout << "Cannot catch SIGINT signals" << std::endl;

  bool isReplay = true;
  int port = 6000;
  std::string host = "0.0.0.0";
  std::string filename;
  switch (argc) {
    default:
    case 5:
      port = atoi (argv[4]);
    case 4:
      host = std::string (argv[3]);
    case 3:
      filename = std::string (argv[2]);
    case 2:
      if (strcmp ("replay", argv[1]) == 0) {
        isReplay = true;
        break;
      } else if (strcmp ("save", argv[1]) == 0) {
        isReplay = false;
        break;
      }
    case 1: // argv[0] contains the executable name.
    case 0: // This should not happen
      std::cout << "Usage: " << argv[0] << " [save|replay] file host port" << std::endl;
      exit (1);
      break;
  }
  if (isReplay)
    replay (filename, host, port);
  else
    save (filename, host, port);
}
