#pragma once
// DNS-SD (mDNS / Bonjour / Avahi) on the local link:
//   - browse for dservs (`_dserv._tcp`, advertised by dserv's mesh
//     subprocess) so the browser can offer them for "connect to dserv";
//   - advertise this VideoStream (`_videostream._tcp` on the web port).
//
// Uses dns_sd.h: part of libSystem on macOS; on Linux Avahi's Bonjour
// compatibility library (libavahi-compat-libdnssd), the same as dserv. Built
// without it (HAVE_DNSSD undefined), everything here is a no-op and state()
// says so. All of it runs on one background thread; dservs() is safe to call
// from any thread.

#include <string>
#include <vector>

struct DservInfo {
  std::string name;       // service instance name (usually the host's name)
  std::string host;       // .local host name the service resolved to
  std::string ip;         // IPv4 address, "" if the host name didn't resolve
  int port = 0;           // dserv's command port (the advertised SRV port)
  int dp_port = 4620;     // datapoint port (TXT dp), what VideoStream connects to
  int web_port = 0;       // TXT web
  bool ssl = false;       // TXT ssl
  std::string workgroup;  // TXT wg
  std::string version;    // TXT ver
};

namespace discovery {

// Start browsing and advertising. web_port/tcl_port go in our own record;
// apps is a comma-separated list of the browser apps served (e.g.
// "eyetracker"). Safe to call once; later calls are ignored.
void start(int web_port, int tcl_port, const std::string& version,
           const std::string& apps);
void stop();

// dservs seen on the link right now, sorted by name.
std::vector<DservInfo> dservs();

// "browsing" when running, otherwise why not (not built in, no mDNS daemon...).
std::string state();

}  // namespace discovery
