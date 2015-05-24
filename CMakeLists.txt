## Copyright (c) 2015, Joseph Mirabel
## Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
##
## This file is part of remoteimu.
## remoteimu is free software: you can redistribute it
## and/or modify it under the terms of the GNU Lesser General Public
## License as published by the Free Software Foundation, either version
## 3 of the License, or (at your option) any later version.
##
## remoteimu is distributed in the hope that it will be
## useful, but WITHOUT ANY WARRANTY; without even the implied warranty
## of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
## General Lesser Public License for more details.  You should have
## received a copy of the GNU Lesser General Public License along with
## remoteimu. If not, see <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/eigen.cmake)

SET(PROJECT_NAME remoteimu)
SET(PROJECT_DESCRIPTION "")
SET(PROJECT_URL "")

SET(${PROJECT_NAME}_HEADERS
  include/remoteimu/mouse.hh
  include/remoteimu/udpclient.hh
  include/remoteimu/udpserver.hh
)

SETUP_PROJECT()

SEARCH_FOR_EIGEN()
ADD_REQUIRED_DEPENDENCY("state-observation")
ADD_REQUIRED_DEPENDENCY("gepetto-viewer-corba")
FIND_PACKAGE ("X11")

ADD_SUBDIRECTORY (src)

PKG_CONFIG_APPEND_LIBS (${PROJECT_NAME})

SETUP_PROJECT_FINALIZE()
