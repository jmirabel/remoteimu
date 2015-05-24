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

SET(LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY(${LIBRARY_NAME} SHARED
  mouse.cc
  udpserver.cc
  udpclient.cc
  )

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} state-observation)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)

ADD_EXECUTABLE(${PROJECT_NAME}-viz
  remoteimu-viz.cc
  )

ADD_EXECUTABLE(${PROJECT_NAME}-calibration
  remoteimu-calibration.cc
  )

ADD_EXECUTABLE(${PROJECT_NAME}-replay
  remoteimu-replay.cc
  )

ADD_EXECUTABLE(${PROJECT_NAME}-pointer
  remoteimu-pointer.cc
  )

TARGET_LINK_LIBRARIES (${PROJECT_NAME}-viz ${LIBRARY_NAME})
TARGET_LINK_LIBRARIES (${PROJECT_NAME}-calibration ${LIBRARY_NAME})
TARGET_LINK_LIBRARIES (${PROJECT_NAME}-replay ${LIBRARY_NAME})
TARGET_LINK_LIBRARIES (${PROJECT_NAME}-pointer ${LIBRARY_NAME} "X11")

PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} state-observation)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} gepetto-viewer-corba)

INSTALL(TARGETS ${PROJECT_NAME}-viz DESTINATION bin)
INSTALL(TARGETS ${PROJECT_NAME}-calibration DESTINATION bin)
INSTALL(TARGETS ${PROJECT_NAME}-replay DESTINATION bin)
INSTALL(TARGETS ${PROJECT_NAME}-pointer DESTINATION bin)
