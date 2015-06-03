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

#include <stdio.h>
#include <iostream>

#include <X11/X.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>

#ifdef Success
#undef Success
#endif

#include <remoteimu/mouse.hh>

using namespace remoteimu;

class PointerEventSender : public MouseEventSender
{
  private:
    Display* dpy;
    Window rootWindow;

  public:
    const int W_center, H_center;
    const double W_pixel_per_mm, H_pixel_per_mm;
    const double alpha;

    PointerEventSender (int refreshRate) :
      dpy (XOpenDisplay (0)), rootWindow (XRootWindow (dpy, 0)),
      W_center (XDisplayWidth (dpy, 0) / 2), H_center (XDisplayHeight (dpy, 0) / 2),
      W_pixel_per_mm (2*W_center / XDisplayWidthMM (dpy, 0)),
      H_pixel_per_mm (2*H_center / XDisplayHeightMM (dpy, 0)),
      alpha (5. / (double)refreshRate), px_1 (0), py_1 (0)
    {
      /*
       * TODO it is not a good idea to initialize dpy in the constructor as
       * error are not handled.
       * */
      /*
      dpy = XOpenDisplay (0);
      rootWindow = XRootWindow (dpy, 0);

      W_center = XDisplayWidth (dpy, 0) / 2;
      H_center = XDisplayHeight (dpy, 0) / 2;
      W_pixel_per_mm = XDisplayWidth (dpy, 0) / XDisplayWidthMM (dpy, 0);
      H_pixel_per_mm = XDisplayHeight (dpy, 0) / XDisplayHeightMM (dpy, 0);

      // */
    }

    void mouseEvent (const MouseEventSender::Event e) {
      // Move the pointer.
      Eigen::Quaternion<double> q (e.ori[0],e.ori[1],e.ori[2],e.ori[3]);
      const Eigen::Matrix3d M = q.toRotationMatrix ();
      // std::cout << M.block <3,1> (0, 1) << std::endl;
      const double d = 1000; // Distance to the screen in mm.
      double x = d * M(0,1) / M(1,1), y = - d * M(2, 1) / M(1,1);
      x = (1 - alpha) * px_1 + alpha * x;
      y = (1 - alpha) * py_1 + alpha * y;
      px_1 = x; py_1 = y;
      int px = (int)(x * W_pixel_per_mm) + W_center,
          py = (int)(y * H_pixel_per_mm) + H_center;
      // std::cout << x << "," << y << " => ";
      // std::cout << px << "," << py << std::endl;
      XSelectInput(dpy, rootWindow, KeyReleaseMask);
      XWarpPointer(dpy, None, rootWindow, 0, 0, 0, 0, px, py);
      XFlush(dpy);
    }

  private:
    double px_1, py_1;
};

int main(int argc, char** argv)
{
  int port = 6000;
  std::string host = "0.0.0.0";
  switch (argc) {
    default:
    case 3:
      port = atoi(argv[2]);
    case 2:
      host = std::string (argv[1]);
      break;
    case 1: // argv[0] contains the executable name.
    case 0: // This should not happen
      std::cout << "Usage: " << argv[0] << " [host [port]]" << std::endl;
      break;
  }
  std::cout << "Listening on " << host << ":" << port << std::endl;
  UDPServer* server (new UDPServer (host, port));

  int rate = 60;
  Mouse mouse (server, rate);

  PointerEventSender es (rate);
  mouse.setMouseEventSender (&es);

  mouse.handleEvents (true);
}
