#ifndef VIEWER_MEDIA_API_H
#define VIEWER_MEDIA_API_H

#include <jansson.h>
#include <string>

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

// Default directory when the client omits path (playback dir, Videos, or $HOME).
std::string viewer_default_browse_path(const SourceManager& sm);

#endif
