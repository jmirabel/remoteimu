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

#ifndef REMOTEIMU_UDPSERVER_HH
#define REMOTEIMU_UDPSERVER_HH

#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <string>

namespace remoteimu {
  class UDPServer
  {
    public:
      UDPServer(const std::string& addr, int port);
      ~UDPServer();

      int get_socket() const;
      int get_port() const;
      std::string get_addr() const;

      int recv(char *msg, size_t max_size);
      int timed_recv(char *msg, size_t max_size, int max_wait_ms);

    private:
      int f_socket;
      int f_port;
      std::string f_addr;
      struct addrinfo *f_addrinfo;
  };
} // namespace remoteimu

#endif // REMOTEIMU_UDPSERVER_HH
