#ifndef VIEWER_MEDIA_API_H
#define VIEWER_MEDIA_API_H

#include <jansson.h>
#include <string>
#include <utility>
#include <vector>

class SourceManager;

struct ViewerCameraPick {
  bool found = false;
  std::string vendor;
  int id = 0;
  std::string serial;
};

// First camera in probe order (webcam, then FLIR, then Lucid).
ViewerCameraPick viewer_probe_first_camera();

// JSON for WS cmd "sources": capabilities + cameras[].
json_t* viewer_build_sources_json(const SourceManager& sm);

// JSON for WS cmd "browse": path, parent, entries[]; nullptr + error string on failure.
json_t* viewer_browse_media_json(const std::string& path_in,
                                 const SourceManager& sm,
                                 std::string& error_out);

// Default directory when the client omits path (playback dir, the first
// existing media place, ~/Videos, or $HOME).
std::string viewer_default_browse_path(const SourceManager& sm);

// Extra folders for the viewer's file picker, set by the startup script via
// vstream::mediaPlaces (e.g. serve.tcl adds the eye-tracking data share), so
// the core carries no application paths. Each is {label, path}: listed under
// places, allowed for browsing, and a default when no video is open.
using MediaPlaces = std::vector<std::pair<std::string, std::string>>;
void viewer_set_media_places(MediaPlaces places);
MediaPlaces viewer_media_places();

#endif
