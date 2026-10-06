# Generate a C++ source embedding every file under SRC_DIR (except source maps)
# as a table of { path, bytes, size } for EmbeddedApp.h. Run in script mode:
#
#   cmake -DSRC_DIR=<dir> -DOUT=<file.cpp> -P EmbedDir.cmake
#
# A missing or empty SRC_DIR yields an empty table (app_file_count == 0).

set(_files "")
if(IS_DIRECTORY "${SRC_DIR}")
    file(GLOB_RECURSE _files RELATIVE "${SRC_DIR}" "${SRC_DIR}/*")
    list(FILTER _files EXCLUDE REGEX "\\.map$")
    list(FILTER _files EXCLUDE REGEX "(^|/)\\.DS_Store$")
    list(SORT _files)
endif()

set(_body "// Auto-generated from ${SRC_DIR} by EmbedDir.cmake. Do not edit!\n")
string(APPEND _body "#include \"EmbeddedApp.h\"\n\nnamespace embedded {\n\n")

set(_table "")
set(_i 0)
foreach(_rel IN LISTS _files)
    file(READ "${SRC_DIR}/${_rel}" _hex HEX)
    string(LENGTH "${_hex}" _hexlen)
    math(EXPR _size "${_hexlen} / 2")
    if(_size EQUAL 0)
        set(_bytes "0")
    else()
        # 32 bytes per line keeps the generated file readable in an editor.
        # (CMake regexes have no {n} repetition, hence string(REPEAT).)
        string(REPEAT "[0-9a-f][0-9a-f]" 32 _line_re)
        string(REGEX REPLACE "(${_line_re})" "\\1\n" _hex "${_hex}")
        string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _bytes "${_hex}")
    endif()
    string(APPEND _body "// ${_rel}\nstatic const unsigned char f${_i}[] = {\n${_bytes}\n};\n\n")
    string(APPEND _table "  { \"${_rel}\", f${_i}, ${_size} },\n")
    math(EXPR _i "${_i} + 1")
endforeach()

if(_i EQUAL 0)
    set(_table "  { nullptr, nullptr, 0 },\n")
endif()
string(APPEND _body "const AppFile app_files[] = {\n${_table}};\n")
string(APPEND _body "const std::size_t app_file_count = ${_i};\n\n} // namespace embedded\n")

file(WRITE "${OUT}" "${_body}")
