#include "ViewerMediaApi.h"
#include "SourceManager.h"

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <set>
#include <vector>

namespace fs = std::filesystem;

#ifdef USE_FLIR
#include <Spinnaker.h>
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
#endif

#ifdef USE_LUCID
#include "LucidCameraSource.h"
#endif

static const std::set<std::string> kVideoExt = {
    ".mp4", ".avi", ".mkv", ".mov", ".m4v", ".webm", ".mpg", ".mpeg"};

static bool has_video_ext(const fs::path& p) {
  std::string ext = p.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return (char) std::tolower(c); });
  return kVideoExt.count(ext) != 0;
}

// Set from Tcl on the main thread, read by the web thread.
static std::mutex g_places_mutex;
static MediaPlaces g_places;

void viewer_set_media_places(MediaPlaces places) {
  std::lock_guard<std::mutex> lock(g_places_mutex);
  g_places = std::move(places);
}

MediaPlaces viewer_media_places() {
  std::lock_guard<std::mutex> lock(g_places_mutex);
  return g_places;
}

static std::vector<fs::path> allowed_browse_roots(const SourceManager& sm) {
  std::vector<fs::path> roots;
  auto add = [&](const fs::path& p) {
    if (p.empty()) return;
    std::error_code ec;
    fs::path abs = fs::weakly_canonical(p, ec);
    if (ec) abs = fs::absolute(p, ec);
    if (ec) return;
    for (const auto& r : roots) {
      if (r == abs) return;
    }
    roots.push_back(abs);
  };

  if (const char* home = std::getenv("HOME")) add(fs::path(home));
  add(fs::path("/mnt/c/Users"));  // WSL: the Windows user folders
  for (const auto& [label, path] : viewer_media_places()) add(fs::path(path));
  auto params = sm.getSourceParams();
  if (params.count("file")) {
    add(fs::path(params.at("file")).parent_path());
  }
  return roots;
}

static bool path_under_root(const fs::path& canonical,
                            const fs::path& root) {
  auto rit = root.begin();
  auto cit = canonical.begin();
  for (; rit != root.end(); ++rit, ++cit) {
    if (cit == canonical.end() || *cit != *rit) return false;
  }
  return true;
}

static bool browse_path_allowed(const fs::path& canonical,
                                const SourceManager& sm) {
  for (const auto& root : allowed_browse_roots(sm)) {
    if (path_under_root(canonical, root)) return true;
  }
  return false;
}

static void append_camera(json_t* arr, const char* vendor, int id,
                          const std::string& label,
                          const std::string& model,
                          const std::string& serial) {
  json_t* o = json_object();
  json_object_set_new(o, "vendor", json_string(vendor));
  json_object_set_new(o, "id", json_integer(id));
  json_object_set_new(o, "label", json_string(label.c_str()));
  if (!model.empty())
    json_object_set_new(o, "model", json_string(model.c_str()));
  if (!serial.empty())
    json_object_set_new(o, "serial", json_string(serial.c_str()));
  json_array_append_new(arr, o);
}

// On Linux a webcam is /dev/video<N>. Opening an index with no device makes
// OpenCV spew GStreamer and V4L2 warnings for every attempt, so skip those.
static bool webcam_device_exists(int i) {
#ifdef __linux__
  std::error_code ec;
  return fs::exists("/dev/video" + std::to_string(i), ec);
#else
  (void) i;
  return true;
#endif
}

static void probe_webcams(json_t* cameras) {
  for (int i = 0; i < 4; ++i) {
    if (!webcam_device_exists(i)) continue;
    cv::VideoCapture cap(i, cv::CAP_ANY);
    if (!cap.isOpened()) continue;
    cap.release();
    append_camera(cameras, "webcam", i, "Webcam " + std::to_string(i), "", "");
  }
}

ViewerCameraPick viewer_probe_first_camera() {
  ViewerCameraPick pick;
  for (int i = 0; i < 4; ++i) {
    if (!webcam_device_exists(i)) continue;
    cv::VideoCapture cap(i, cv::CAP_ANY);
    if (!cap.isOpened()) continue;
    cap.release();
    pick.found = true;
    pick.vendor = "webcam";
    pick.id = i;
    return pick;
  }
#ifdef USE_FLIR
  try {
    SystemPtr system = System::GetInstance();
    CameraList camList = system->GetCameras();
    if (camList.GetSize() > 0) {
      pick.found = true;
      pick.vendor = "flir";
      pick.id = 0;
      try {
        CameraPtr cam = camList.GetByIndex(0);
        INodeMap& tl = cam->GetTLDeviceNodeMap();
        CStringPtr pSerial = tl.GetNode("DeviceSerialNumber");
        if (pSerial.IsValid() && IsReadable(pSerial))
          pick.serial = pSerial->GetValue().c_str();
      } catch (...) {
      }
    }
    camList.Clear();
    system->ReleaseInstance();
    if (pick.found) return pick;
  } catch (...) {
  }
#endif
#ifdef USE_LUCID
  std::vector<LucidDeviceSummary> lucid = lucidListDevices();
  if (!lucid.empty()) {
    pick.found = true;
    pick.vendor = "lucid";
    pick.id = 0;
    pick.serial = lucid[0].serial;
    return pick;
  }
#endif
  return pick;
}

#ifdef USE_FLIR
static void probe_flir(json_t* cameras) {
  try {
    SystemPtr system = System::GetInstance();
    CameraList camList = system->GetCameras();
    const unsigned n = camList.GetSize();
    for (unsigned i = 0; i < n; ++i) {
      CameraPtr cam = camList.GetByIndex(i);
      std::string model;
      std::string serial;
      try {
        INodeMap& tl = cam->GetTLDeviceNodeMap();
        CStringPtr pModel = tl.GetNode("DeviceModelName");
        CStringPtr pSerial = tl.GetNode("DeviceSerialNumber");
        if (pModel.IsValid() && IsReadable(pModel))
          model = pModel->GetValue().c_str();
        if (pSerial.IsValid() && IsReadable(pSerial))
          serial = pSerial->GetValue().c_str();
      } catch (...) {
      }
      std::string label = "Blackfly";
      if (!model.empty()) label += " — " + model;
      else if (!serial.empty()) label += " — " + serial;
      append_camera(cameras, "flir", (int) i, label, model, serial);
    }
    camList.Clear();
    system->ReleaseInstance();
  } catch (const std::exception& e) {
    std::cerr << "FLIR probe: " << e.what() << std::endl;
  }
}
#endif

#ifdef USE_LUCID
static void probe_lucid(json_t* cameras) {
  std::vector<LucidDeviceSummary> devices = lucidListDevices();
  for (size_t i = 0; i < devices.size(); ++i) {
    const std::string& model = devices[i].model;
    const std::string& serial = devices[i].serial;
    std::string label = "Triton2";
    if (!model.empty()) label += " — " + model;
    else if (!serial.empty()) label += " — " + serial;
    append_camera(cameras, "lucid", (int) i, label, model, serial);
  }
}
#endif

json_t* viewer_build_sources_json(const SourceManager& sm) {
  json_t* root = json_object();
  json_t* cap = json_object();
  json_object_set_new(cap, "playback", json_true());
  json_object_set_new(cap, "webcam", json_true());
#ifdef USE_FLIR
  json_object_set_new(cap, "flir", json_true());
#else
  json_object_set_new(cap, "flir", json_false());
#endif
#ifdef USE_LUCID
  json_object_set_new(cap, "lucid", json_true());
#else
  json_object_set_new(cap, "lucid", json_false());
#endif
  json_object_set_new(root, "capabilities", cap);

  json_t* cameras = json_array();
  probe_webcams(cameras);
#ifdef USE_FLIR
  probe_flir(cameras);
#endif
#ifdef USE_LUCID
  probe_lucid(cameras);
#endif
  json_object_set_new(root, "source_active",
                      json_boolean(sm.getState() != SOURCE_IDLE));
  json_object_set_new(root, "cameras", cameras);
  json_object_set_new(root, "type", json_string("sources"));
  json_object_set_new(root, "status", json_string("ok"));
  return root;
}

std::string viewer_default_browse_path(const SourceManager& sm) {
  auto params = sm.getSourceParams();
  if (params.count("file")) {
    fs::path p = fs::path(params.at("file")).parent_path();
    std::error_code ec;
    if (fs::is_directory(p, ec)) return p.string();
  }
  for (const auto& [label, path] : viewer_media_places()) {
    std::error_code ec;
    if (fs::is_directory(path, ec)) return path;
  }
  if (const char* home = std::getenv("HOME")) {
    fs::path v = fs::path(home) / "Videos";
    std::error_code ec;
    if (fs::is_directory(v, ec)) return v.string();
    return home;
  }
  return "/";
}

static int64_t file_mtime_unix(const fs::path& p) {
  std::error_code ec;
  auto ft = fs::last_write_time(p, ec);
  if (ec) return 0;
  auto sys = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
      ft - fs::file_time_type::clock::now() + std::chrono::system_clock::now());
  return std::chrono::duration_cast<std::chrono::seconds>(sys.time_since_epoch())
      .count();
}

static json_t* browse_places_json(const SourceManager& sm) {
  json_t* places = json_array();
  std::vector<std::string> seen;
  auto add = [&](const std::string& label, const fs::path& p) {
    std::error_code ec;
    if (p.empty() || !fs::is_directory(p, ec)) return;
    fs::path c = fs::weakly_canonical(p, ec);
    if (ec || !browse_path_allowed(c, sm)) return;
    const std::string s = c.string();
    if (std::find(seen.begin(), seen.end(), s) != seen.end()) return;
    seen.push_back(s);
    json_t* o = json_object();
    json_object_set_new(o, "label", json_string(label.c_str()));
    json_object_set_new(o, "path", json_string(s.c_str()));
    json_array_append_new(places, o);
  };

  auto params = sm.getSourceParams();
  if (params.count("file"))
    add("Current video folder", fs::path(params.at("file")).parent_path());
  for (const auto& [label, path] : viewer_media_places()) add(label, path);
  add("Windows user folders", "/mnt/c/Users");
  if (const char* home = std::getenv("HOME")) {
    add("Linux home", fs::path(home));
    add("Linux videos", fs::path(home) / "Videos");
  }
  return places;
}

json_t* viewer_browse_media_json(const std::string& path_in,
                                 const SourceManager& sm,
                                 std::string& error_out) {
  error_out.clear();
  std::string use = path_in.empty() ? viewer_default_browse_path(sm) : path_in;

  fs::path p(use);
  std::error_code ec;
  fs::path canonical = fs::weakly_canonical(p, ec);
  if (ec) {
    error_out = "Invalid path";
    return nullptr;
  }
  if (!fs::is_directory(canonical, ec) || ec) {
    error_out = "Not a directory";
    return nullptr;
  }
  if (!browse_path_allowed(canonical, sm)) {
    error_out = "Path not allowed";
    return nullptr;
  }

  json_t* root = json_object();
  json_object_set_new(root, "type", json_string("browse"));
  json_object_set_new(root, "status", json_string("ok"));
  json_object_set_new(root, "path", json_string(canonical.string().c_str()));

  fs::path parent = canonical.parent_path();
  if (parent != canonical && browse_path_allowed(parent, sm))
    json_object_set_new(root, "parent", json_string(parent.string().c_str()));
  else
    json_object_set_new(root, "parent", json_null());

  struct Row {
    std::string name;
    std::string path;
    bool dir;
    uintmax_t size;
    int64_t mtime;
  };
  std::vector<Row> rows;
  for (const auto& ent : fs::directory_iterator(canonical, ec)) {
    if (ec) break;
    const fs::path ep = ent.path();
    const std::string name = ep.filename().string();
    if (name.empty() || name[0] == '.') continue;
    std::error_code eec;
    if (ent.is_directory(eec)) {
      rows.push_back({name, ep.string(), true, 0, file_mtime_unix(ep)});
    } else if (ent.is_regular_file(eec) && has_video_ext(ep)) {
      uintmax_t sz = ent.file_size(eec);
      if (eec) sz = 0;
      rows.push_back({name, ep.string(), false, sz, file_mtime_unix(ep)});
    }
  }
  std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) {
    if (a.dir != b.dir) return a.dir > b.dir;
    return a.name < b.name;
  });

  json_t* entries = json_array();
  for (const auto& r : rows) {
    json_t* o = json_object();
    json_object_set_new(o, "name", json_string(r.name.c_str()));
    json_object_set_new(o, "path", json_string(r.path.c_str()));
    json_object_set_new(o, "dir", json_boolean(r.dir));
    if (!r.dir) json_object_set_new(o, "size", json_integer((json_int_t) r.size));
    if (r.mtime) json_object_set_new(o, "mtime", json_integer((json_int_t) r.mtime));
    json_array_append_new(entries, o);
  }
  json_object_set_new(root, "entries", entries);
  json_object_set_new(root, "places", browse_places_json(sm));
  return root;
}
