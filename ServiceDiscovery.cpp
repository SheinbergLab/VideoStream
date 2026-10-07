#include "ServiceDiscovery.h"

#include <algorithm>
#include <atomic>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <thread>

#ifdef HAVE_DNSSD

#include <arpa/inet.h>   // htons/ntohs: Apple's dns_sd.h pulls it in, Avahi's does not
#include <cerrno>
#include <netdb.h>
#include <sys/select.h>
#include <cstdlib>
#include <cstring>
#include <dns_sd.h>

namespace {

const char* kDservType = "_dserv._tcp";
const char* kOwnType = "_videostream._tcp";

struct Resolve {
  DNSServiceRef ref = nullptr;
  std::string name;
  bool done = false;
};

struct State {
  std::mutex mutex;                          // guards found + status
  std::map<std::string, DservInfo> found;    // by instance name
  std::string status = "not started";

  // Below: discovery thread only.
  std::map<std::string, std::set<uint32_t>> interfaces;  // where each name is seen
  std::vector<Resolve*> resolves;
  DNSServiceRef browse = nullptr;
  DNSServiceRef own = nullptr;
};

State g;
std::thread g_thread;
std::atomic<bool> g_running{false};
std::atomic<bool> g_started{false};

void set_status(const std::string& s) {
  std::lock_guard<std::mutex> lock(g.mutex);
  g.status = s;
}

std::string txt_value(uint16_t len, const unsigned char* txt, const char* key) {
  uint8_t vlen = 0;
  const void* v = TXTRecordGetValuePtr(len, txt, key, &vlen);
  return v ? std::string(static_cast<const char*>(v), vlen) : std::string();
}

int txt_int(uint16_t len, const unsigned char* txt, const char* key, int dflt) {
  std::string s = txt_value(len, txt, key);
  if (s.empty()) return dflt;
  char* end = nullptr;
  long n = std::strtol(s.c_str(), &end, 10);
  return (end && *end == '\0' && n > 0 && n < 65536) ? int(n) : dflt;
}

// IPv4 for a .local name. macOS resolves .local itself; Linux needs nss-mdns
// (libnss-mdns, which avahi-daemon recommends). Runs on the discovery thread.
std::string resolve_ipv4(const std::string& host) {
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* res = nullptr;
  if (getaddrinfo(host.c_str(), nullptr, &hints, &res) != 0 || !res) return "";
  char buf[INET_ADDRSTRLEN] = {0};
  auto* sin = reinterpret_cast<sockaddr_in*>(res->ai_addr);
  inet_ntop(AF_INET, &sin->sin_addr, buf, sizeof(buf));
  freeaddrinfo(res);
  return buf;
}

void DNSSD_API on_resolve(DNSServiceRef, DNSServiceFlags, uint32_t,
                          DNSServiceErrorType err, const char*,
                          const char* hosttarget, uint16_t port_be,
                          uint16_t txt_len, const unsigned char* txt,
                          void* context) {
  auto* r = static_cast<Resolve*>(context);
  r->done = true;
  if (err != kDNSServiceErr_NoError) return;
  if (!g.interfaces.count(r->name)) return;  // removed while resolving

  DservInfo d;
  d.name = r->name;
  d.host = hosttarget ? hosttarget : "";
  if (!d.host.empty() && d.host.back() == '.') d.host.pop_back();
  d.port = ntohs(port_be);
  d.dp_port = txt_int(txt_len, txt, "dp", 4620);
  d.web_port = txt_int(txt_len, txt, "web", 0);
  d.ssl = txt_value(txt_len, txt, "ssl") == "1";
  d.workgroup = txt_value(txt_len, txt, "wg");
  d.version = txt_value(txt_len, txt, "ver");
  d.ip = resolve_ipv4(d.host);

  std::lock_guard<std::mutex> lock(g.mutex);
  g.found[d.name] = d;
}

void DNSSD_API on_browse(DNSServiceRef, DNSServiceFlags flags, uint32_t iface,
                         DNSServiceErrorType err, const char* name,
                         const char* type, const char* domain, void*) {
  if (err != kDNSServiceErr_NoError) {
    set_status("browse error " + std::to_string(err));
    return;
  }
  std::string n(name);
  if (flags & kDNSServiceFlagsAdd) {
    // The same instance is announced once per interface; resolve it once.
    bool first = g.interfaces[n].empty();
    g.interfaces[n].insert(iface);
    if (!first) return;
    auto* r = new Resolve;
    r->name = n;
    if (DNSServiceResolve(&r->ref, 0, iface, name, type, domain, on_resolve, r) !=
        kDNSServiceErr_NoError) {
      delete r;
      return;
    }
    g.resolves.push_back(r);
  } else {
    auto it = g.interfaces.find(n);
    if (it == g.interfaces.end()) return;
    it->second.erase(iface);
    if (!it->second.empty()) return;
    g.interfaces.erase(it);
    std::lock_guard<std::mutex> lock(g.mutex);
    g.found.erase(n);
  }
}

void DNSSD_API on_register(DNSServiceRef, DNSServiceFlags, DNSServiceErrorType err,
                           const char* name, const char*, const char*, void*) {
  if (err == kDNSServiceErr_NoError)
    std::cout << "mDNS: advertising " << kOwnType << " as \"" << name << "\"" << std::endl;
  else
    std::cout << "mDNS: advertisement failed (dns_sd error " << err << ")" << std::endl;
}

void run(int web_port, int tcl_port, std::string version, std::string apps) {
  DNSServiceErrorType err =
      DNSServiceBrowse(&g.browse, 0, 0, kDservType, nullptr, on_browse, nullptr);
  if (err != kDNSServiceErr_NoError) {
    // Avahi's shim reports a missing avahi-daemon this way.
    set_status("off: no mDNS service (dns_sd error " + std::to_string(err) +
               "; on Linux, is avahi-daemon running?)");
    std::cout << "mDNS: dserv discovery unavailable (dns_sd error " << err << ")"
              << std::endl;
    g.browse = nullptr;
    return;
  }
  set_status("browsing");

  TXTRecordRef txt;
  TXTRecordCreate(&txt, 0, nullptr);
  std::string tcl = std::to_string(tcl_port);
  TXTRecordSetValue(&txt, "tcl", uint8_t(tcl.size()), tcl.data());
  if (!apps.empty()) TXTRecordSetValue(&txt, "apps", uint8_t(std::min<size_t>(apps.size(), 255)), apps.data());
  if (!version.empty()) TXTRecordSetValue(&txt, "ver", uint8_t(version.size()), version.data());
  if (DNSServiceRegister(&g.own, 0, 0, nullptr, kOwnType, nullptr, nullptr,
                         htons(uint16_t(web_port)), TXTRecordGetLength(&txt),
                         TXTRecordGetBytesPtr(&txt), on_register, nullptr) !=
      kDNSServiceErr_NoError)
    g.own = nullptr;
  TXTRecordDeallocate(&txt);

  while (g_running.load()) {
    fd_set fds;
    FD_ZERO(&fds);
    int maxfd = -1;
    auto add = [&](DNSServiceRef ref) {
      if (!ref) return;
      int fd = DNSServiceRefSockFD(ref);
      if (fd < 0) return;
      FD_SET(fd, &fds);
      maxfd = std::max(maxfd, fd);
    };
    add(g.browse);
    add(g.own);
    for (auto* r : g.resolves) add(r->ref);

    timeval tv{0, 250000};  // wake to notice stop()
    int n = select(maxfd + 1, &fds, nullptr, nullptr, &tv);
    if (n < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (n == 0) continue;

    auto ready = [&](DNSServiceRef ref) {
      return ref && FD_ISSET(DNSServiceRefSockFD(ref), &fds);
    };
    if (ready(g.browse) && DNSServiceProcessResult(g.browse) != kDNSServiceErr_NoError) {
      set_status("off: mDNS service went away");
      break;
    }
    if (ready(g.own)) DNSServiceProcessResult(g.own);
    // Resolves can be appended by on_browse above; only walk the ones that
    // existed when select() ran (their fds are the ones in the set).
    for (auto* r : std::vector<Resolve*>(g.resolves)) {
      if (ready(r->ref) && DNSServiceProcessResult(r->ref) != kDNSServiceErr_NoError)
        r->done = true;
    }
    for (auto it = g.resolves.begin(); it != g.resolves.end();) {
      if ((*it)->done) {
        DNSServiceRefDeallocate((*it)->ref);
        delete *it;
        it = g.resolves.erase(it);
      } else {
        ++it;
      }
    }
  }

  for (auto* r : g.resolves) {
    DNSServiceRefDeallocate(r->ref);
    delete r;
  }
  g.resolves.clear();
  if (g.own) DNSServiceRefDeallocate(g.own);
  if (g.browse) DNSServiceRefDeallocate(g.browse);
  g.own = g.browse = nullptr;
}

}  // namespace

namespace discovery {

void start(int web_port, int tcl_port, const std::string& version,
           const std::string& apps) {
  if (g_started.exchange(true)) return;
  setenv("AVAHI_COMPAT_NOWARN", "1", 0);  // Avahi's shim nags on stderr otherwise
  g_running = true;
  g_thread = std::thread(run, web_port, tcl_port, version, apps);
}

void stop() {
  g_running = false;
  if (g_thread.joinable()) g_thread.join();
}

std::vector<DservInfo> dservs() {
  std::lock_guard<std::mutex> lock(g.mutex);
  std::vector<DservInfo> out;
  for (const auto& [name, d] : g.found) out.push_back(d);
  return out;
}

std::string state() {
  std::lock_guard<std::mutex> lock(g.mutex);
  return g.status;
}

}  // namespace discovery

#else  // !HAVE_DNSSD

namespace discovery {
void start(int, int, const std::string&, const std::string&) {}
void stop() {}
std::vector<DservInfo> dservs() { return {}; }
std::string state() {
  return "off: built without dns_sd (Linux: apt install libavahi-compat-libdnssd-dev)";
}
}  // namespace discovery

#endif
