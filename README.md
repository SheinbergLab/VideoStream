# VideoStream

Stand alone program to acquire video frames using the FLIR Spinnaker library or OpenCV device, display, and store to disk.

For control, the program opens a TCP/IP communication port, and receives Tcl scripts across the port.  Metadata about frames are stored alongside the video using the "dgz" format.  Dependencies are [Tcl](https://tcl.tk), [sockpp](https://github.com/fpagliughi/sockpp), and [libdg](https://github.com/sheinb/libdg).

## Installation

### Linux

A number of readily available packages are required to build the VideoStream app.

#### Camera support
For FLIR camera control, the Spinnaker SDK should be installed.  This does not require the SpinView app to be installed.  For USB Webcam support, any camera that can be found using OpenCV will work.

#### Development Libs
Standard libs that should be installed include:

* essential build tools (gcc)
* cmake
* the Tcl development environment
* the OpenCV Devlopment libraries

`
sudo apt install build-essential cmake tcl-dev libopendev-dev
`

#### sockpp

For TCP/IP communications, VideoStream uses the socket wrapper called [sockpp](https://github.com/fpagliughi/sockpp). Compile and install this using cmake.

#### libdg

This is a library written to allow storing and reading complex binary data file structures.  The C API in [this repo](https://github.com/sheinb/libdg) should be built and installed.

## General Functions
```
 vstream::fileOpen
 vstream::fileClose
 vstream::domainSocketOpen
 vstream::domainSocketClose
 vstream::inObs
 vstream::fourcc
 vstream::addShutdownCmd
 vstream::displayOpen
 vstream::displayClose
```

## FLIR Camera Functions
```
vstream::configureExposure
vstream::configureGain
vstream::configureFrameRate
vstream::configureROI
```
