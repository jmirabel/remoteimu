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

#include "remoteimu/udpclient.hh"

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdexcept>

namespace remoteimu {
  UDPClient::UDPClient(const std::string& addr, int port)
    : f_port(port)
      , f_addr(addr)
  {
    char decimal_port[16];
    snprintf(decimal_port, sizeof(decimal_port), "%d", f_port);
    decimal_port[sizeof(decimal_port) / sizeof(decimal_port[0]) - 1] = '\0';
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    int r(getaddrinfo(addr.c_str(), decimal_port, &hints, &f_addrinfo));
    if(r != 0 || f_addrinfo == NULL)
    {
      throw std::runtime_error(("invalid address or port: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
    f_socket = socket(f_addrinfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
    if(f_socket == -1)
    {
      freeaddrinfo(f_addrinfo);
      throw std::runtime_error(("could not create socket for: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
  }

  UDPClient::~UDPClient()
  {
    freeaddrinfo(f_addrinfo);
    close(f_socket);
  }

  int UDPClient::get_socket() const
  {
    return f_socket;
  }

  int UDPClient::get_port() const
  {
    return f_port;
  }

  std::string UDPClient::get_addr() const
  {
    return f_addr;
  }

  int UDPClient::send(const char *msg, size_t size)
  {
    return sendto(f_socket, msg, size, 0, f_addrinfo->ai_addr, f_addrinfo->ai_addrlen);
  }
} // namespace remoteimu
