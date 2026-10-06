#pragma once
// The browser viewer (web/dist) built into the binary; the table is generated
// at build time by cmake/EmbedDir.cmake. app_file_count is 0 when web/ wasn't
// built before VideoStream (then only --www-dir can serve the viewer).

#include <cstddef>

namespace embedded {

struct AppFile {
  const char *path;            // relative to dist/, e.g. "assets/index-xyz.js"
  const unsigned char *data;
  std::size_t size;
};

extern const AppFile app_files[];
extern const std::size_t app_file_count;

}  // namespace embedded
