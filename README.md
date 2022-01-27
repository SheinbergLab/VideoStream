# VideoStream

Stand alone program to acquire video frames using the FLIR Spinnaker library or OpenCV device, display, and store to disk.

For control, the program opens a TCP/IP communication port, and receives Tcl scripts across the port.  Metadata about frames are stored alongside the video using the "dgz" format.  Dependencies are [Tcl](https://tcl.tk), [sockpp](https://github.com/fpagliughi/sockpp), and [libdg](https://github.com/sheinb/libdg).
