# find python by path
set(Python_FIND_REGISTRY LAST)
# Options
option(CHECK_OPTIONAL_LIBS "Try to download / enable all optional libraries (use only EPL clean libraries, if set to false)" ON)
option(MULTITHREADED_BUILD "Use all available cores for building (applies to Visual Studio only)")
option(ISOLATED_BUILD "Do not pollute the source directory on building")
option(INSTALL_DLL_RUNTIMES "Copy debug and release runtimes for MSVC" ON)
option(PROFILING "Enable output of profiling data (applies to gcc/clang builds only)")
option(PPROF "Link the pprof profiler library (applies to gcc/clang builds only)")
option(COVERAGE "Enable output of coverage data (applies to gcc/clang builds only)")
option(SUMO_UTILS "Enable generation of a shared library for the utility functions for option handling, XML parsing etc.")
option(FMI "Enable generation of an FMI library for SUMO" ON)
option(NETEDIT "Enable build of netedit" ON)
option(ENABLE_PYTHON_BINDINGS "Build Python Bindings" ON)
option(ENABLE_JAVA_BINDINGS "Build Java Bindings" ON)
option(ENABLE_CS_BINDINGS "Build C# Bindings" ON)
option(CCACHE_SUPPORT "Enable ccache support if installed" ON)
option(TCMALLOC "Use tcmalloc if installed" ON)
option(PARQUET "Compile with Parquet support" ON)
option(VERBOSE_SUB "Let sub-commands be more verbose")
set(BINARY_SUFFIX "" CACHE STRING "Append the suffix to every generated binary")
set(COMPILE_DEFINITIONS "" CACHE STRING "Macros or defines to add when compiling")
set(JUPEDSIM_CUSTOMDIR "" CACHE PATH "Custom install location of jupedsim")
set(PROJECT_NAME "SUMO" CACHE STRING "Project/Solution name")

# Set a default build type if none was specified
# you may use -DCMAKE_BUILD_TYPE:STRING=Debug from the command line
set(default_build_type "Release")

set(CMAKE_ORIGINAL_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "RelWithDebInfo")
endif ()

# the part after ... sets all cmake policies to "new" which have been introduced up to that cmake version
cmake_minimum_required(VERSION 3.5...3.21 FATAL_ERROR)
project("${PROJECT_NAME}" CXX C)
set(PACKAGE_VERSION "git")

# Check if libraries have to be found, depending on SUMO_LIBRARIES
set(SUMO_LIBRARIES "$ENV{SUMO_LIBRARIES}" CACHE PATH "Location of SUMOLibraries dependencies")

# some user place SUMOLibraries in the same SUMO folder
if (NOT SUMO_LIBRARIES AND EXISTS "${CMAKE_SOURCE_DIR}/../SUMOLibraries")
    set(SUMO_LIBRARIES "${CMAKE_SOURCE_DIR}/../SUMOLibraries")
endif ()

if (CCACHE_SUPPORT)
    if (MSVC)
        file(GLOB SCCACHE_ROOT "${SUMO_LIBRARIES}/sccache-*")
        find_program(SCCACHE_PROGRAM sccache ${SCCACHE_ROOT})
        if (SCCACHE_PROGRAM)
            message(STATUS "Found sccache: ${SCCACHE_PROGRAM}")
            set(CMAKE_C_COMPILER_LAUNCHER "${SCCACHE_PROGRAM}")
            set(CMAKE_CXX_COMPILER_LAUNCHER "${SCCACHE_PROGRAM}")
            if (CMAKE_BUILD_TYPE STREQUAL "Debug")
                string(REPLACE "/Zi" "/Z7" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
                string(REPLACE "/Zi" "/Z7" CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG}")
            elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
                string(REPLACE "/Zi" "/Z7" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
                string(REPLACE "/Zi" "/Z7" CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO}")
            endif()
        endif()
    else()
        find_program(CCACHE_PROGRAM ccache)
        if (CCACHE_PROGRAM)
            message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
            set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
            set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
        endif()
    endif()
endif()

set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/build_config/cmake_modules/")

set(ENABLED_FEATURES "${CMAKE_SYSTEM} ${CMAKE_SYSTEM_PROCESSOR} ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION} ${CMAKE_BUILD_TYPE}")

if (COMPILE_DEFINITIONS)
    add_compile_definitions(${COMPILE_DEFINITIONS})
endif ()

# declare flags for compilers
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set (GNU_COMPILER True)
endif ()
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set (CLANG_COMPILER True)
endif ()

# C++17 is needed by std::filesystem and parquet.
# C++14 is needed by Google Test >= 1.13, for all the other parts C++11 should be enough.
# This will silently fall back to C++14 or even C++11, whatever is supported by the compiler.
if (${CMAKE_VERSION} VERSION_LESS 3.8.0)
    set(CMAKE_CXX_STANDARD 14)
else()
    set(CMAKE_CXX_STANDARD 17)
endif()
if (NOT "cxx_std_17" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
    message(WARNING "Your compiler does not support C++17. Please consider upgrading your compiler toolchain.")
endif()

# compiler specific flags
if (GNU_COMPILER OR CLANG_COMPILER)
    # set flags for clang in windows
    if (CLANG_COMPILER AND WIN32)
        # flags for clang in windows
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unsafe-buffer-usage")
        # disable warnings
        add_compile_options("/clang:-Wno-unsafe-buffer-usage")
        add_compile_options("/clang:-Wno-reserved-identifier")
        add_compile_options("/clang:-Wno-c++98-compat")
        # disable debug build (due a problem with Runtime Libraries)
        set(CMAKE_CONFIGURATION_TYPES "Release")
        set(CMAKE_BUILD_TYPE "Release")
    else ()
        # flags for clang and gcc in linux
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -Wall -pedantic -Wextra")
    endif ()
    if (GNU_COMPILER)
        # see https://stackoverflow.com/questions/23499909/adjust-variable-tracking-assignment-length
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --param=max-vartrack-size=5000000")
        if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9.9 AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
            # see the last bulletpoint about ABI at https://gcc.gnu.org/gcc-10/changes.html#cxx
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-psabi")
        endif()
    endif ()
    if (PROFILING)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
        set(ENABLED_FEATURES "${ENABLED_FEATURES} Profiling")
        set(BINARY_SUFFIX "${BINARY_SUFFIX}P")
    endif ()
    if (COVERAGE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
        set(ENABLED_FEATURES "${ENABLED_FEATURES} Coverage")
    endif ()
elseif (MSVC)
    # enabling /WX is not possible due to warnings in external headers
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        # this is a workaround to disable lots of warnings for replacing /W3 with /W4 in VS 2019
        # there is a policy for that starting with CMake 3.15
        if (MSVC_VERSION GREATER 1900)
            string(REPLACE "/W3" "/Wall" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        else ()
            string(REPLACE "/W3" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
        endif ()
    else ()
        # /Wall brings MSVC 2013 to complete halt
        if (MSVC_VERSION GREATER 1900)
            set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /Wall")
        else ()
            set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /W4")
        endif ()
    endif ()
    if (MULTITHREADED_BUILD)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
    endif ()
    # SWIG generates large obj files
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
    # backward compatible mutex API, see https://github.com/actions/runner-images/issues/10004
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR")
    # exporting symbols for shared libraries needs to enabled explicitly
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    # add option for enabling console in release mode
    option(CONSOLE_RELEASE "Enable console in SUMO-GUI and NETEDIT in release (use only for debug purposes)" false)
endif ()

# special debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")
if (CLANG_COMPILER)
    if (WIN32)
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=undefined,integer -fsanitize-blacklist=${CMAKE_SOURCE_DIR}/build_config/clang_sanitize_blacklist.txt")
        add_compile_definitions(_ITERATOR_DEBUG_LEVEL=0)
    else ()
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=undefined,address,integer -fno-omit-frame-pointer -fsanitize-blacklist=${CMAKE_SOURCE_DIR}/build_config/clang_sanitize_blacklist.txt")
    endif ()
endif ()

# we need to build position independent code when generating a shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if (SUMO_UTILS)
    set(ENABLED_FEATURES "${ENABLED_FEATURES} SumoUtilsLibrary")
endif ()
if (FMI)
    set(ENABLED_FEATURES "${ENABLED_FEATURES} FMI")
endif ()

if (NOT ISOLATED_BUILD)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
    # force Visual Studio to leave out the Release / Debug dirs
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin)
endif()

# Debug messages
message(STATUS "CMAKE_BINARY_DIR: " ${CMAKE_BINARY_DIR})
message(STATUS "CMAKE_SOURCE_DIR: " ${CMAKE_SOURCE_DIR})
message(STATUS "")
message(STATUS "Platform: ")
message(STATUS "    Host: " ${CMAKE_HOST_SYSTEM} " " ${CMAKE_HOST_SYSTEM_PROCESSOR})
message(STATUS "    Target: " ${CMAKE_SYSTEM} " " ${CMAKE_SYSTEM_PROCESSOR})
message(STATUS "    CMake: " ${CMAKE_VERSION})
message(STATUS "    CMake generator: " ${CMAKE_GENERATOR})
message(STATUS "    CMake build tool: " ${CMAKE_BUILD_TOOL})
message(STATUS "    Compiler: " ${CMAKE_CXX_COMPILER_ID} " " ${CMAKE_CXX_COMPILER_VERSION})
if (CMAKE_GENERATOR MATCHES Xcode)
    message(STATUS "    Xcode: " ${XCODE_VERSION})
endif ()
message(STATUS "")

if (${CMAKE_VERSION} VERSION_LESS 3.12.0)
    find_package(PythonInterp REQUIRED)
    # define variable for compatibility
    set(Python_EXECUTABLE ${PYTHON_EXECUTABLE} CACHE FILEPATH "Python executable")
    message(WARNING "Found Python in legacy mode, consider upgrading CMake to at least 3.18: " ${Python_EXECUTABLE})
else()
    # we search for Interpreter _and_ Development to make sure the versions match
    find_package(Python REQUIRED COMPONENTS Interpreter OPTIONAL_COMPONENTS Development.Module)
endif()
if (${CMAKE_VERSION} VERSION_LESS 3.18.0 AND (ENABLE_PYTHON_BINDINGS OR ENABLE_JAVA_BINDINGS OR ENABLE_CS_BINDINGS))
    message(WARNING "Language bindings require at least CMake 3.18.")
    set(ENABLE_PYTHON_BINDINGS OFF)
    set(ENABLE_JAVA_BINDINGS OFF)
    set(ENABLE_CS_BINDINGS OFF)
endif()
if (MSVC AND ENABLE_PYTHON_BINDINGS)
    # recheck that the platform of the generator and python matches
    execute_process(COMMAND ${Python_EXECUTABLE} -c "import sys; print(sys.maxsize > 2**32)"
                    OUTPUT_VARIABLE _is_python64 OUTPUT_STRIP_TRAILING_WHITESPACE)
    if (${_is_python64} STREQUAL "False" OR NOT ${CMAKE_MODULE_LINKER_FLAGS} STREQUAL "/machine:x64")
        message(WARNING "Python language bindings are only supported for 64 bit.")
        set(ENABLE_PYTHON_BINDINGS OFF)
    endif()
    if (NOT PYTHON_DEBUG_LIBRARY AND "${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        message(WARNING "Did not find Python debug library. Please reinstall your Python and enable the Python debug libraries in the installer.")
        set(ENABLE_PYTHON_BINDINGS OFF)
    endif()
endif()

# hatchling is required for doing a build without access to pypi.org (offline on open build service, launchpad etc.)
execute_process(COMMAND ${Python_EXECUTABLE} -c "import build, hatchling, pip; build.__version__"
                RESULT_VARIABLE PYTHON_BUILD_MISSING ERROR_QUIET)
if (PYTHON_BUILD_MISSING)
    message(STATUS "Python build, hatchling or pip are missing (sumo packages will not be installable via cmake).")
endif()

# exclusive from Apple
if (APPLE)
    # we know that openGL is deprecated for newer MacOS
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGL_SILENCE_DEPRECATION")

    # Xcode 15 on macOS sonoma complains about duplicate libraries for the linker, so we disable the warning for xcode >= 15
    if (CLANG_COMPILER)
        string(REGEX MATCH "^[0-9]+" ClangMajorVersion "${CMAKE_CXX_COMPILER_VERSION}")
        if (ClangMajorVersion GREATER_EQUAL 15)
            set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-no_warn_duplicate_libraries")
        endif()
    endif()

    # Recommendation, also add a "brew --prefix" custom command to detect a homebrew build environment
    execute_process(COMMAND brew --prefix RESULT_VARIABLE DETECT_BREW OUTPUT_VARIABLE BREW_PREFIX ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if (${DETECT_BREW} EQUAL 0)
        link_directories(${BREW_PREFIX}/lib)
        include_directories(${BREW_PREFIX}/include)
        message(STATUS "Brew detected: ${BREW_PREFIX}")
    endif ()

    # Detect if the "port" command is valid on this system; if so, return full path
    execute_process(COMMAND which port RESULT_VARIABLE DETECT_MACPORTS OUTPUT_VARIABLE MACPORTS_PREFIX ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if (${DETECT_MACPORTS} EQUAL 0)
        # MACPORTS_PREFIX contains now something like "/opt/local/bin/port", so we get the parent directory
        get_filename_component(MACPORTS_PREFIX ${MACPORTS_PREFIX} DIRECTORY)
        # "/opt/local" is where MacPorts lives, add `/lib` suffix and link
        # http://stackoverflow.com/questions/1487752/how-do-i-instruct-cmake-to-look-for-libraries-installed-by-macports
        link_directories(/opt/X11/lib ${MACPORTS_PREFIX}/../lib)
        include_directories(SYSTEM /opt/X11/include ${MACPORTS_PREFIX}/../include)
        message(STATUS "Macports detected: ${MACPORTS_PREFIX}")
    endif ()
endif (APPLE)

# Specifically define variable WIN32 for compilations under windows (due an error in Shawn)
if (GNU_COMPILER AND WIN32)
    option(USE_MINGW_64BITS "Use 64 bits libraries for the compilation with MinGW" true)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWIN32 -D_WIN32_WINNT=0x0501")
    set(MINGW32 1)
endif ()

if (NOT JUPEDSIM_CUSTOMDIR)
    # check if set JuPedSim directory from SUMOLibraries
    if (SUMO_LIBRARIES AND WIN32)
        file(GLOB JUPEDSIM_CUSTOMDIR_SUMOLIBRARIES "${SUMO_LIBRARIES}/JuPedSim-*")
        set(JUPEDSIM_CUSTOMDIR "${JUPEDSIM_CUSTOMDIR_SUMOLIBRARIES}")
    else ()
        if (EXISTS "${CMAKE_SOURCE_DIR}/../jupedsim-install")
            set(JUPEDSIM_CUSTOMDIR "${CMAKE_SOURCE_DIR}/../jupedsim-install")
        endif ()
    endif ()
    if (JUPEDSIM_CUSTOMDIR)
        message(STATUS "Using JuPedSim from ${JUPEDSIM_CUSTOMDIR}")
    endif ()
endif ()

# check if SUMOLibraries was found (Only in Windows)
if (SUMO_LIBRARIES AND WIN32)
    # Special option for MinGW32
    message(STATUS "Using 64 bit libraries from SUMO_LIBRARIES placed in " ${SUMO_LIBRARIES})
    file(GLOB XERCES_PATH "${SUMO_LIBRARIES}/xerces-c-3.?.?")
    file(GLOB ZLIB_PATH "${SUMO_LIBRARIES}/3rdPartyLibs/zlib-?.?.*")
    file(GLOB PROJ_PATH "${SUMO_LIBRARIES}/proj-?.?.?")
    file(GLOB FOX_PATH "${SUMO_LIBRARIES}/fox-?.?.??")
    file(GLOB FREETYPE_PATH "${SUMO_LIBRARIES}/3rdPartyLibs/freetype-2.??.?")
    file(GLOB EIGEN3_PATH "${SUMO_LIBRARIES}/eigen-3.?.?")
    file(GLOB GETTEXT_PATH "${SUMO_LIBRARIES}/gettext-?.??")
    file(GLOB ARROW_PATH "${SUMO_LIBRARIES}/apache-arrow-??.?.?")
    file(GLOB LZ4_PATH "${SUMO_LIBRARIES}/3rdPartyLibs/lz4-?.??.?")       # needed for Arrow
    file(GLOB ZSTD_PATH "${SUMO_LIBRARIES}/3rdPartyLibs/zstd-?.?.?")      # needed for Arrow
    file(GLOB BZIP2_PATH "${SUMO_LIBRARIES}/3rdPartyLibs/bzip2-?.?.?")    # needed for Arrow
    file(GLOB THRIFT_PATH "${SUMO_LIBRARIES}/3rdPartyLibs/thrift-?.??.?") # needed for Arrow

    # set all paths in prefix
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${EIGEN3_PATH};${XERCES_PATH};${ZLIB_PATH};${PROJ_PATH};${FOX_PATH};${FREETYPE_PATH};${GETTEXT_PATH};${GETTEXT_PATH}/tools/gettext;${ARROW_PATH};${LZ4_PATH};${ZSTD_PATH};${BZIP2_PATH};${THRIFT_PATH}")
    # save in variable SUMO_LIBRARIES_DLL all paths to dll files
    file(GLOB SUMO_LIBRARIES_DLL "${XERCES_PATH}/bin/*.dll"
                                 "${PROJ_PATH}/bin/*.dll"
                                 "${FOX_PATH}/bin/*.dll"
                                 "${ZLIB_PATH}/bin/*.dll"
                                 "${FREETYPE_PATH}/bin/*.dll"
                                 "${GETTEXT_PATH}/bin/*.dll"
                                 "${GDAL_PATH}/bin/*.dll"
                                 "${ARROW_PATH}/bin/*.dll"
                                 "${SUMO_LIBRARIES}/libspatialite-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/geos-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/bzip2-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/curl-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/libpng-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/libssh2-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/openssl-*/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/sqlite-*/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/tiff-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/libxml2-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/libiconv-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/openssl-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/sqlite-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/freexl-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/libexpat-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/librttopo-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/minizip-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/thrift-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/zstd-*/bin/*.dll"
                                 "${SUMO_LIBRARIES}/3rdPartyLibs/lz4-*/bin/*.dll"
                                 )
    # check if MSVC dll runtimes have to be copied
    if (INSTALL_DLL_RUNTIMES)
        file(GLOB MSVC_DLLS "${SUMO_LIBRARIES}/runtimes/*.dll")
        list(APPEND SUMO_LIBRARIES_DLL ${MSVC_DLLS})
    endif(INSTALL_DLL_RUNTIMES)
    if (MSVC_VERSION GREATER 1919)
        # fmt in SUMOLibraries only works with MSVC 2019 and later
        file(GLOB fmt_PATH "${SUMO_LIBRARIES}/fmt-??.?.?")
    endif ()
    # declare flag for use google test
    set(USE_GOOGLETEST true)
    # check if use google test
    if (WIN32)
        if (GNU_COMPILER AND NOT USE_MINGW_64BITS)
            message(STATUS "Disabled Google Test in Mingw32")
            set(USE_GOOGLETEST false)
        elseif(CLANG_COMPILER)
            message(STATUS "Disabled Google Test in Clang (Windows)")
            set(USE_GOOGLETEST false)
        endif ()
    endif()
    if (USE_GOOGLETEST)
        file(GLOB GTEST_ROOT "${SUMO_LIBRARIES}/googletest-?.??.?")
        message(STATUS "Found Google test: " ${GTEST_ROOT})
        include_directories("${GTEST_ROOT}/include")
        # set both google test library
        set(GTEST_BOTH_LIBRARIES "${GTEST_ROOT}/lib/gtest.lib" "${GTEST_ROOT}/lib/gtest_main.lib")
        # set google test DLLs
        file(GLOB GTEST_DLL "${GTEST_ROOT}/bin/gtest*.dll")
        set(GTEST_FOUND true)
    endif ()
    file(GLOB SWIG_EXECUTABLE "${SUMO_LIBRARIES}/swigwin-*/swig.exe")
    file(GLOB MVN_EXECUTABLE "${SUMO_LIBRARIES}/apache-maven-*/bin/mvn.cmd")
else ()
    # for Linux and Mac only
    find_package(GTest)
endif ()

find_package(XercesC REQUIRED)
if (XercesC_FOUND)
    include_directories(SYSTEM ${XercesC_INCLUDE_DIRS})
endif (XercesC_FOUND)

find_package(Proj)
if (PROJ_FOUND)
    include_directories(SYSTEM ${PROJ_INCLUDE_DIR})
    set(ENABLED_FEATURES "${ENABLED_FEATURES} Proj")
endif (PROJ_FOUND)

find_package(FOX)
if (FOX_FOUND)
    set(OpenGL_GL_PREFERENCE LEGACY)
    find_package(OpenGL)
    if (OPENGL_FOUND AND OPENGL_GLU_FOUND)
        include_directories(SYSTEM ${FOX_INCLUDE_DIR})
        add_definitions(${FOX_CXX_FLAGS})
        add_definitions(-DFLOAT_MATH_FUNCTIONS)
        set(HAVE_FOX 1)
        set(ENABLED_FEATURES "${ENABLED_FEATURES} GUI")
    else ()
        message(WARNING "Fox is present but OpenGL or GLU cannot be found, GUI will be disabled.")
    endif ()
endif (FOX_FOUND)

find_package(Freetype)
if (FREETYPE_FOUND)
    include_directories(SYSTEM ${FREETYPE_INCLUDE_DIRS})
else (FREETYPE_FOUND)
    message(WARNING "FreeType cannot be found. If you are using SUMO libraries, update SUMO_LIBRARIES folder with git pull. Otherwise, specify manually FreeType include folder and libs.")
endif (FREETYPE_FOUND)

if (SUMO_LIBRARIES AND WIN32)
    find_package(fmt PATHS ${fmt_PATH} NO_DEFAULT_PATH)
    if (fmt_FOUND)
        message(STATUS "Found FMT (SUMOLibraries): " ${fmt_DIR})
        include_directories(SYSTEM "${fmt_PATH}/include")
        set(FMT_LIBRARY fmt::fmt)
        set(HAVE_FMT 1)
        set(ENABLED_FEATURES "${ENABLED_FEATURES} FMT")
    endif (fmt_FOUND)
elseif (NOT MSVC)
    find_package(fmt QUIET)
    if (fmt_FOUND)
        set(HAVE_FMT 1)
        set(FMT_LIBRARY fmt::fmt)
        # next 2 lines could fix the include path for mambaforge fmt on DLR machines (maybe outdated cmake?)
        # get_target_property(fmt_INCLUDE_DIR fmt::fmt INTERFACE_INCLUDE_DIRECTORIES)
        # include_directories(SYSTEM ${fmt_INCLUDE_DIR})
        set(ENABLED_FEATURES "${ENABLED_FEATURES} FMT")
    endif (fmt_FOUND)
endif ()

find_package(X11)
if (X11_FOUND)
    link_directories(${X11_LIBRARY_DIR})
    include_directories(SYSTEM ${X11_INCLUDE_DIR})
endif (X11_FOUND)

find_package(ZLIB)
if (ZLIB_FOUND)
    set(HAVE_ZLIB 1)
    link_directories(${ZLIB_LIBRARY_DIR})
    include_directories(SYSTEM ${ZLIB_INCLUDE_DIR})
else ()
    set(ZLIB_LIBRARIES "")
endif ()

find_package(Intl)
if (Intl_FOUND)
    set(HAVE_INTL 1)
    link_directories(${Intl_LIBRARY_DIR})
    include_directories(SYSTEM ${Intl_INCLUDE_DIRS})
    set(ENABLED_FEATURES "${ENABLED_FEATURES} Intl")
endif ()

find_package(SWIG 3.0.12)
if (SWIG_FOUND)
    set(ENABLED_FEATURES "${ENABLED_FEATURES} SWIG")
endif ()

if (TCMALLOC)
    find_library(TCMALLOC_LIBRARY NAMES tcmalloc_minimal)
    if (TCMALLOC_LIBRARY)
        set(ENABLED_FEATURES "${ENABLED_FEATURES} tcmalloc")
    else ()
        set(TCMALLOC_LIBRARY "")
    endif ()
endif ()

if (PARQUET)
    set(lz4Alt_FIND_QUIETLY True)
    set(re2Alt_FIND_QUIETLY True)
    set(ThriftAlt_FIND_QUIETLY True)
    # needed for parquet
    find_package(Arrow)
    find_package(Parquet)
    if (Arrow_FOUND AND Parquet_FOUND)
        set(ENABLED_FEATURES "${ENABLED_FEATURES} Parquet")
        set(PARQUET_LIBRARY Parquet::parquet_shared)
        include_directories(SYSTEM ${ARROW_INCLUDE_DIR})
        # if (GTEST_FOUND)
        #     add_definitions("GTest_SOURCE=Bundled")
        # endif()
        # this is for gtest compatibility
    else()
        message(WARNING "Parquet support requested but Arrow or Parquet libraries not found. Disabling Parquet support.")
        set(PARQUET_LIBRARY "")
    endif()
else()
    set(PARQUET_LIBRARY "")
endif()

# Eigen (for overhead wire / electric circuit simulation)
find_package(Eigen3 3.2)
if (EIGEN3_FOUND)
    include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
    set(HAVE_EIGEN 1)  # see config.h.cmake for #cmakedefine
    set(ENABLED_FEATURES "${ENABLED_FEATURES} Eigen")
endif (EIGEN3_FOUND)

if (CHECK_OPTIONAL_LIBS)
    file(GLOB GDAL_PATH "${SUMO_LIBRARIES}/gdal-?.?.?")
    file(GLOB FFMPEG_PATH "${SUMO_LIBRARIES}/FFMPEG-?.?.?")
    file(GLOB OSG_PATH "${SUMO_LIBRARIES}/OSG-?.?.?")
    file(GLOB GL2PS_PATH "${SUMO_LIBRARIES}/gl2ps-?.?.?")
    file(GLOB GEOS_PATH "${SUMO_LIBRARIES}/geos-?.??.?")
    file(GLOB BOOST_PATH "${SUMO_LIBRARIES}/boost-?.??.?")
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${GDAL_PATH};${FFMPEG_PATH};${OSG_PATH};${GL2PS_PATH};${GEOS_PATH};${BOOST_PATH}")
    file(GLOB SUMO_OPTIONAL_LIBRARIES_DLL "${GDAL_PATH}/bin/*.dll" "${FFMPEG_PATH}/bin/*.dll" "${OSG_PATH}/bin/*.dll" "${GL2PS_PATH}/bin/*.dll" "${JUPEDSIM_CUSTOMDIR}/bin/*.dll")

    # GDAL (for geopositioning)
    find_package(GDAL)
    if (GDAL_FOUND)
        include_directories(SYSTEM ${GDAL_INCLUDE_DIR})
        set(HAVE_GDAL 1)
        set(ENABLED_FEATURES "${ENABLED_FEATURES} GDAL")
    else (GDAL_FOUND)
        set(GDAL_LIBRARY "")
    endif (GDAL_FOUND)

    if (FOX_FOUND)
        # FFMPEG (for recording videos)
        find_package(FFMPEG)
        if (FFMPEG_FOUND)
            include_directories(SYSTEM ${FFMPEG_INCLUDE_DIR})
            set(HAVE_FFMPEG 1)
            set(ENABLED_FEATURES "${ENABLED_FEATURES} FFmpeg")
        endif ()

        # OSG (For 3D view)
        find_package(OpenSceneGraph 3.4.0 COMPONENTS osgGA osgViewer osgUtil osgDB osgText)
        if (OPENSCENEGRAPH_FOUND)
            include_directories(SYSTEM ${OPENSCENEGRAPH_INCLUDE_DIRS})
            set(HAVE_OSG 1)
            set(ENABLED_FEATURES "${ENABLED_FEATURES} OSG")
        endif ()

        # GL2PS (deprecated, will be changed by FreeType)
        find_package(GL2PS)
        if (GL2PS_FOUND)
            include_directories(SYSTEM ${GL2PS_INCLUDE_DIR})
            set(HAVE_GL2PS 1)
            set(ENABLED_FEATURES "${ENABLED_FEATURES} GL2PS")
        endif (GL2PS_FOUND)
    endif ()

    find_package(jupedsim CONFIG HINTS ${JUPEDSIM_CUSTOMDIR} QUIET)
    if (jupedsim_FOUND)
        find_package(GEOS 3.3.0)
        if (GEOS_FOUND)
            include_directories(SYSTEM ${GEOS_INCLUDE_DIR})
            set(JPS_VERSION "${jupedsim_VERSION_MAJOR}${jupedsim_VERSION_MINOR}${jupedsim_VERSION_PATCH}")
            set(ENABLED_FEATURES "${ENABLED_FEATURES} JuPedSim")
            message(STATUS "Found JuPedSim: ${jupedsim_DIR} ${jupedsim_VERSION}")
        else (GEOS_FOUND)
            message(STATUS "Could NOT find GEOS! JuPedSim requires GEOS.")
        endif (GEOS_FOUND)
    else (jupedsim_FOUND)
        message(STATUS "Could NOT find JuPedSim! Skipping JuPedSim integration.")
    endif (jupedsim_FOUND)

    # add optional libraries
    list(APPEND SUMO_LIBRARIES_DLL ${SUMO_OPTIONAL_LIBRARIES_DLL})

    # BOOST (needed for stacktrace, minimum version 1.80.0)
    find_package(boost_filesystem CONFIG QUIET)
    find_package(boost_process CONFIG QUIET)
    find_package(boost_headers CONFIG QUIET)

    # init boost library
    set(BOOST_LIBRARIES "")

    # continue if boost was found
    if(boost_filesystem_FOUND AND boost_process_FOUND AND boost_headers_FOUND)
        message(STATUS "Boost headers, filesystem and process " ${boost_headers_VERSION} " found, enabling stacktrace support")
        set(HAVE_BOOST 1)
        # set includes
        get_target_property(Boost_include Boost::headers INTERFACE_INCLUDE_DIRECTORIES)
        include_directories(${Boost_include})
        # add boost libraries
        list(APPEND BOOST_LIBRARIES Boost::filesystem Boost::process Boost::headers)
    else()
        message(STATUS "Boost not found, building without stacktrace support")
    endif()

endif (CHECK_OPTIONAL_LIBS)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

set(commonlibs
        utils_distribution utils_handlers utils_shapes utils_options
        utils_xml utils_geom utils_common utils_importio utils_iodevices utils_traction_wire foreign_tcpip
        ${XercesC_LIBRARIES} ${ZLIB_LIBRARIES} ${PROJ_LIBRARY} ${Intl_LIBRARIES} ${FMT_LIBRARY} ${PARQUET_LIBRARY} ${CMAKE_DL_LIBS})
if (WIN32)
    set(commonlibs ${commonlibs} ws2_32)
endif ()
if (PPROF)
    set(commonlibs ${commonlibs} profiler)
endif ()
set(commonvehiclelibs
        utils_emissions foreign_phemlight foreign_phemlight_V5 utils_vehicle ${commonlibs} ${FOX_LIBRARY})

# set custom name and folder for ALL_BUILD and ZERO_CHECK in visual studio solutions
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMake")

# installation
if (MSVC)
    if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
        set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/sumo-${PACKAGE_VERSION}")
    endif()
    if (ISOLATED_BUILD)
        set(SUMO_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/bin/)
    else()
        set(SUMO_BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/)
    endif()
    install(DIRECTORY ${SUMO_BIN_DIR} DESTINATION bin COMPONENT dll
            FILES_MATCHING
            PATTERN "*.bat"
            PATTERN "*.dll"
            PATTERN "*d.dll" EXCLUDE
            PATTERN "*_Debug.dll" EXCLUDE
            PATTERN "gtest*.dll" EXCLUDE)
    # files which end with "d" but are not debug
    install(DIRECTORY ${SUMO_BIN_DIR} DESTINATION bin COMPONENT dll
            FILES_MATCHING
            PATTERN "thriftmd.dll"
            PATTERN "zstd.dll")
else ()
    include(GNUInstallDirs)
endif ()
if (SKBUILD OR MSVC)
    set(DATA_PATH "")
else ()
    set(DATA_PATH "share/sumo/")
endif ()
if (SKBUILD)
    set(EXCLUDE_LIBSUMO "libsumo")
    set(EXCLUDE_LIBTRACI "libtraci")
endif ()
install(DIRECTORY data/ DESTINATION ${DATA_PATH}data COMPONENT data)
install(DIRECTORY tools/ DESTINATION ${DATA_PATH}tools
        USE_SOURCE_PERMISSIONS COMPONENT runtime
        PATTERN "build_config/sumo" EXCLUDE  # needed for the scikit build
        PATTERN "calibration" EXCLUDE
        PATTERN "lisum-core" EXCLUDE
        PATTERN "lisum-gui" EXCLUDE
        PATTERN "sumolib4matlab/src" EXCLUDE
        PATTERN "${EXCLUDE_LIBSUMO}" EXCLUDE
        PATTERN "${EXCLUDE_LIBTRACI}" EXCLUDE
        PATTERN "traas" EXCLUDE
        PATTERN "traci4matlab/src" EXCLUDE
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
        PATTERN "*.egg-info" EXCLUDE
        PATTERN ".git" EXCLUDE)
if (${CMAKE_VERSION} VERSION_GREATER 3.6.0)
    # this is a hack which works only with a non-isolated build
    install(DIRECTORY tools/libsumo/ DESTINATION libsumo COMPONENT tools_libsumo EXCLUDE_FROM_ALL
            PATTERN "__pycache__" EXCLUDE
            PATTERN "*.pyc" EXCLUDE)
    if (PROJ_DATA_DIR)
        install(DIRECTORY ${PROJ_DATA_DIR}/ DESTINATION ${DATA_PATH}data/proj COMPONENT proj_data EXCLUDE_FROM_ALL
                PATTERN "*.tif" EXCLUDE
                PATTERN "*.txt" EXCLUDE)
    endif ()
endif ()
install(CODE "
    execute_process(COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/build_config/version.py --pep440 \$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${DATA_PATH}tools/sumolib/version.py)
" COMPONENT runtime)
if (DATA_PATH)
    install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ../../bin \$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/${DATA_PATH}bin)" COMPONENT runtime)
endif ()

install(EXPORT SUMOConfig DESTINATION ${DATA_PATH}cmake NAMESPACE SUMO:: COMPONENT runtime)

set(SUMO_TOOLS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tools")
if (${CMAKE_VERSION} VERSION_GREATER 3.6.0 AND NOT ${PYTHON_BUILD_MISSING})
    # no isolated build here because we need to be able to do it offline (on open build service, launchpad etc.)
    # furthermore several commands in the same execute_process are not guaranteed to be executed sequentially
    install(CODE "
        execute_process(COMMAND ${Python_EXECUTABLE} tools/build_config/version.py --pep440 build_config/pyproject/sumolib.toml pyproject.toml
                        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
        execute_process(COMMAND ${Python_EXECUTABLE} -m build --no-isolation --wheel . -o ${CMAKE_BINARY_DIR}
                        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
        execute_process(COMMAND ${Python_EXECUTABLE} tools/build_config/version.py --pep440 build_config/pyproject/traci.toml pyproject.toml
                        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
        execute_process(COMMAND ${Python_EXECUTABLE} -m build --no-isolation --wheel . -o ${CMAKE_BINARY_DIR}
                        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
        execute_process(COMMAND ${Python_EXECUTABLE} -m pip install --prefix=\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX} -f ${CMAKE_BINARY_DIR} --no-index traci
                        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    " COMPONENT python_package EXCLUDE_FROM_ALL)
endif ()

if (SUMO_LIBRARIES AND WIN32)
    # filter release DLLs
    execute_process(COMMAND ${Python_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/build_config/filterDebugDLL.py ${SUMO_LIBRARIES_DLL}
                    OUTPUT_VARIABLE SUMO_LIBRARIES_RELEASE_DLL)
    if (ISOLATED_BUILD)
        set(TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR})
    else()
        set(TARGET_DIR ${CMAKE_CURRENT_SOURCE_DIR})
    endif()
    # copy debug dlls
    foreach(DLL_PATH ${SUMO_LIBRARIES_DLL} ${GTEST_DLL})
        get_filename_component(DLL_BASENAME ${DLL_PATH} NAME)
        set(TARGET_DLL ${TARGET_DIR}/bin/${DLL_BASENAME})
        list(FIND TARGET_ALL_DLLS ${TARGET_DLL} DLL_KNOWN)
        list(FIND SUMO_LIBRARIES_RELEASE_DLL ${DLL_PATH} IS_RELEASE_DLL)
        list(FIND GTEST_DLL ${DLL_PATH} IS_TEST_DLL)
        if (DLL_KNOWN EQUAL -1)
            add_custom_command(OUTPUT ${TARGET_DLL}
                               COMMAND ${CMAKE_COMMAND} -E copy ${DLL_PATH} ${TARGET_DLL}
                               DEPENDS ${DLL_PATH})
            if (IS_TEST_DLL GREATER -1)
                list(APPEND TARGET_TEST_DLLS ${TARGET_DLL})
            elseif (IS_RELEASE_DLL GREATER -1)
                list(APPEND TARGET_ALL_DLLS ${TARGET_DLL})
            else ()
                list(APPEND TARGET_ALL_DLLS $<$<CONFIG:Debug>:${TARGET_DLL}>)
            endif ()
        endif()
    endforeach(DLL_PATH)
    # copy OSG dlls
    if (HAVE_OSG)
        file(GLOB OSG_PLUGIN_DLL "${OSG_PATH}/bin/osgPlugins*/osgdb_osg.dll")
        get_filename_component(OSG_PLUGIN_DLL_BASENAME ${OSG_PLUGIN_DLL} NAME)
        get_filename_component(OSG_PLUGIN_DIR ${OSG_PLUGIN_DLL} DIRECTORY)
        get_filename_component(OSG_PLUGIN_DIR ${OSG_PLUGIN_DIR} NAME)
        set(OSG_TARGET ${TARGET_DIR}/bin/${OSG_PLUGIN_DIR}/${OSG_PLUGIN_DLL_BASENAME})
        add_custom_command(OUTPUT ${OSG_TARGET}
                           COMMAND ${CMAKE_COMMAND} -E copy_directory ${OSG_PATH}/bin/${OSG_PLUGIN_DIR} ${TARGET_DIR}/bin/${OSG_PLUGIN_DIR}
                           DEPENDS ${OSG_PLUGIN_DLL})
    endif (HAVE_OSG)
    if (PROJ_DATA_DIR)
        # proj 7 needs to copy "share/proj" folder in SUMO_HOME/share
        set(PROJ_DATA ${TARGET_DIR}/share/proj/proj.db)
        add_custom_command(OUTPUT ${PROJ_DATA}
                           COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJ_DATA_DIR} ${TARGET_DIR}/share/proj
                           DEPENDS ${PROJ_DATA_DIR}/proj.db)
        install(DIRECTORY ${TARGET_DIR}/share/ DESTINATION ${DATA_PATH}share COMPONENT data)
    endif ()
endif()
# set install dll targets, for non-Windows they depend on nothing
add_custom_target(install_dll DEPENDS ${TARGET_ALL_DLLS} ${OSG_TARGET} ${PROJ_DATA})
add_custom_target(install_test_dll DEPENDS ${TARGET_TEST_DLLS})
set_property(TARGET install_dll PROPERTY FOLDER "CMake")
set_property(TARGET install_test_dll PROPERTY FOLDER "CMake")

# installing desktop files and other Linux specific stuff
if (CMAKE_VERSION VERSION_GREATER 3.6.0 AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    install(DIRECTORY build_config/package/ DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/profile.d
            COMPONENT linux_integration EXCLUDE_FROM_ALL
            FILES_MATCHING PATTERN "*sh" PATTERN "jupedsim" EXCLUDE)
    install(DIRECTORY build_config/package/ DESTINATION ${CMAKE_INSTALL_DATADIR}/applications
            COMPONENT linux_integration EXCLUDE_FROM_ALL
            FILES_MATCHING PATTERN "*.desktop" PATTERN "jupedsim" EXCLUDE)
    install(DIRECTORY build_config/package/ DESTINATION ${CMAKE_INSTALL_DATADIR}/pixmaps
            COMPONENT linux_integration EXCLUDE_FROM_ALL
            FILES_MATCHING PATTERN "*.png" PATTERN "jupedsim" EXCLUDE)
    install(DIRECTORY build_config/package/ DESTINATION ${CMAKE_INSTALL_DATADIR}/mime/application
            COMPONENT linux_integration EXCLUDE_FROM_ALL
            FILES_MATCHING PATTERN "org.eclipse.sumo.xml" PATTERN "jupedsim" EXCLUDE)
endif()

# i18n target
find_package(Gettext)
if (Gettext_FOUND)
    if (ISOLATED_BUILD)
        set(LOCALE_DIR ${CMAKE_BINARY_DIR}/data)
        install(DIRECTORY ${LOCALE_DIR}/locale/ DESTINATION ${DATA_PATH}data/locale COMPONENT data)
    else()
        set(LOCALE_DIR ${CMAKE_SOURCE_DIR}/data)
    endif()
    set(I18N_DATA ${LOCALE_DIR}/locale/tr/LC_MESSAGES/sumo.mo)
    add_custom_command(OUTPUT ${I18N_DATA}
                       COMMAND ${Python_EXECUTABLE} ${CMAKE_SOURCE_DIR}/tools/build_config/i18n.py -m -o ${LOCALE_DIR}
                       DEPENDS ${CMAKE_SOURCE_DIR}/data/po/tr_gui.po)
else ()
    message(WARNING "Gettext tools not found, translation files will not be generated.")
endif()
add_custom_target(install_mo DEPENDS ${I18N_DATA})

# java targets
find_program(MVN_EXECUTABLE mvn)
if (NOT VERBOSE_SUB)
    set(MVN_OPTS "-q")
endif ()
find_package(Java COMPONENTS Development)
if (MVN_EXECUTABLE AND Java_FOUND)
    if (NOT DEFINED ENV{JAVA_HOME})
        get_filename_component(JAVA_BIN ${Java_JAVAC_EXECUTABLE} DIRECTORY)
        get_filename_component(JAVA_HOME ${JAVA_BIN} DIRECTORY)
        set(MVN_COMMAND_PREFIX ${CMAKE_COMMAND} -E env JAVA_HOME=${JAVA_HOME})
        set(MVN_REPO "-Dmaven.repo.local=${CMAKE_BINARY_DIR}/m2")
    endif ()
    add_custom_target(traas
        COMMAND ${MVN_COMMAND_PREFIX} ${MVN_EXECUTABLE} ${MVN_REPO} ${MVN_OPTS} --batch-mode -f tools/contributed/traas/pom.xml clean install
        COMMAND ${CMAKE_COMMAND} -E copy tools/contributed/traas/target/traas-1.1.jar bin/TraaS.jar
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    add_custom_target(lisum
        COMMAND ${MVN_COMMAND_PREFIX} ${MVN_EXECUTABLE} ${MVN_REPO} ${MVN_OPTS} --batch-mode -f tools/contributed/lisum/pom.xml clean install
        COMMAND ${CMAKE_COMMAND} -E copy tools/contributed/lisum/lisum-core/target/lisum-core-1.0.2.jar bin/lisum-core.jar
        COMMAND ${CMAKE_COMMAND} -E copy tools/contributed/lisum/lisum-gui/target/lisum-gui-1.1.jar bin/lisum-gui.jar
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    add_dependencies(lisum traas)
    set(JAVA_TARGETS traas lisum)
    foreach (JAVAT ${JAVA_TARGETS})
        set_property(TARGET ${JAVAT} PROPERTY EXCLUDE_FROM_DEFAULT_BUILD TRUE)
        set_property(TARGET ${JAVAT} PROPERTY FOLDER "java")
    endforeach ()
endif ()

# doc targets
add_custom_target(doxygen
    COMMAND rm -rf docs/doxygen
    COMMAND mkdir docs/doxygen
    COMMAND doxygen sumo.doxyconf > doxygen.log 2>&1
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
set_property(TARGET doxygen PROPERTY EXCLUDE_FROM_DEFAULT_BUILD TRUE)
set_property(TARGET doxygen PROPERTY FOLDER "doc")

add_custom_target(userdoc
    COMMAND ../../tools/build_config/buildPyDoc.py ../pydoc --clean
    COMMAND mkdocs build -d ../userdoc
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/docs/web)
set_property(TARGET userdoc PROPERTY EXCLUDE_FROM_DEFAULT_BUILD TRUE)
set_property(TARGET userdoc PROPERTY FOLDER "doc")

if (MVN_EXECUTABLE)
    add_custom_target(javadoc
        COMMAND rm -rf docs/javadoc
        COMMAND mkdir docs/javadoc
        COMMAND ${MVN_EXECUTABLE} ${MVN_OPTS} --batch-mode -f tools/contributed/traas/pom.xml javadoc:javadoc
        COMMAND mv tools/contributed/traas/target/site/apidocs docs/javadoc/traas
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    set_property(TARGET javadoc PROPERTY EXCLUDE_FROM_DEFAULT_BUILD TRUE)
    set_property(TARGET javadoc PROPERTY FOLDER "doc")
    set(JAVADOC_TARGET javadoc)
endif ()

find_program(HELPMAN_EXECUTABLE help2man)
if (HELPMAN_EXECUTABLE)
    add_custom_target(man
        COMMAND rm -rf docs/man
        COMMAND mkdir docs/man
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "A microscopic, multi-modal traffic simulation" bin/sumo > docs/man/sumo.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "GUI version of the simulation SUMO" bin/sumo-gui > docs/man/sumo-gui.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Builds vehicle routes for SUMO using detector values" bin/dfrouter > docs/man/dfrouter.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Shortest path router and DUE computer for the microscopic traffic simulation SUMO" bin/duarouter > docs/man/duarouter.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Router for the microscopic traffic simulation SUMO based on junction turning ratios" bin/jtrrouter > docs/man/jtrrouter.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Import O/D-matrices and trips using macroscopic traffic assignment for SUMO" bin/marouter > docs/man/marouter.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Generates routes of persons throughout a day for the microscopic traffic simulation SUMO" bin/activitygen > docs/man/activitygen.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Importer of O/D-matrices for the traffic simulation SUMO" bin/od2trips > docs/man/od2trips.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Road network importer / builder for the traffic simulation SUMO" bin/netconvert > docs/man/netconvert.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Road network editor for the traffic simulation SUMO" bin/netedit > docs/man/netedit.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Road network generator for the microscopic traffic simulation SUMO" bin/netgenerate > docs/man/netgenerate.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Importer of polygons and POIs for the traffic simulation SUMO" bin/polyconvert > docs/man/polyconvert.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "TraCITestClient for the traffic simulation SUMO" bin/TraCITestClient > docs/man/TraCITestClient.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Builds and writes an emissions map for SUMO\\\'s emission models" bin/emissionsMap > docs/man/emissionsMap.1
        COMMAND ${HELPMAN_EXECUTABLE} -N -n "Computes emissions by driving a time line using SUMO\\\'s emission models" bin/emissionsDrivingCycle > docs/man/emissionsDrivingCycle.1
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    set_property(TARGET man PROPERTY EXCLUDE_FROM_DEFAULT_BUILD TRUE)
    set_property(TARGET man PROPERTY FOLDER "doc")
    set(MAN_TARGET man)
    if (CMAKE_VERSION VERSION_GREATER 3.6.0 AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
        install(DIRECTORY docs/man/ DESTINATION ${CMAKE_INSTALL_MANDIR}/man1
                COMPONENT linux_integration EXCLUDE_FROM_ALL)
    endif()
endif ()

add_custom_target(doc)
add_dependencies(doc doxygen userdoc ${JAVADOC_TARGET} ${MAN_TARGET})
set_property(TARGET doc PROPERTY EXCLUDE_FROM_DEFAULT_BUILD TRUE)
set_property(TARGET doc PROPERTY FOLDER "doc")

# coverage targets
if (COVERAGE)
    find_program(LCOV_EXECUTABLE lcov)
    if (LCOV_EXECUTABLE)
        add_custom_target(lcov
            COMMAND rm -rf docs/lcov
            COMMAND mkdir docs/lcov
            COMMAND ${LCOV_EXECUTABLE} -d . --capture --output-file docs/lcov/lcov.info
            COMMAND ${LCOV_EXECUTABLE} --remove docs/lcov/lcov.info '/usr/*' --output-file docs/lcov/lcov.info
            COMMAND genhtml -o docs/lcov/html docs/lcov/lcov.info
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
        add_custom_target(lcov-reset
            COMMAND ${LCOV_EXECUTABLE} -d . -z
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    else ()
        message(WARNING "COVERAGE is enabled but lcov was not found.")
    endif ()
endif ()

# testing
enable_testing()
add_subdirectory(src)
if (GTEST_FOUND)
    add_subdirectory(unittest)
endif ()

# example and dist targets
add_custom_target(examples
    COMMAND ${Python_EXECUTABLE} tools/extractTest.py -x -f tests/examples.txt -p docs/examples/runAll.py
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
add_dependencies(examples sumo netconvert dfrouter duarouter jtrrouter)
set_property(TARGET examples PROPERTY EXCLUDE_FROM_DEFAULT_BUILD TRUE)
set_property(TARGET examples PROPERTY FOLDER "doc")

find_program(PYINSTALLER_FOUND "pyinstaller")
if (PYINSTALLER_FOUND AND EXISTS ${CMAKE_SOURCE_DIR}/tools/game/runner.spec)
    if (OPENSCENEGRAPH_FOUND)
        set(PYINSTALLER_ARGS -- osg)
    endif()
    add_custom_target(game
        COMMAND pyinstaller --noconfirm ${CMAKE_SOURCE_DIR}/tools/game/runner.spec ${PYINSTALLER_ARGS}
    )
endif()

add_test(exampletest ${Python_EXECUTABLE} ${CMAKE_SOURCE_DIR}/docs/examples/runAll.py)
find_package(Git)
if (GIT_FOUND)
    add_custom_target(dist
        COMMAND rm -rf sumo-${PACKAGE_VERSION} sumo-${PACKAGE_VERSION}.zip sumo-src-${PACKAGE_VERSION}.tar.gz sumo-src-${PACKAGE_VERSION}.zip sumo-gui-macos-${PACKAGE_VERSION}.zip
        COMMAND ${GIT_EXECUTABLE} archive --prefix sumo-${PACKAGE_VERSION}/ -o sumo-${PACKAGE_VERSION}.zip HEAD
        COMMAND unzip -q sumo-${PACKAGE_VERSION}.zip
        COMMAND rm -r sumo-${PACKAGE_VERSION}/tests
        COMMAND cp -a docs/tutorial docs/examples sumo-${PACKAGE_VERSION}/docs
        COMMAND find tools/contributed/saga/ tools/contributed/traci4matlab -type f | grep -v .git | xargs cp --parents --target-dir sumo-${PACKAGE_VERSION}
        COMMAND ${Python_EXECUTABLE} tools/build_config/version.py --pep440 sumo-${PACKAGE_VERSION}/tools/sumolib/version.py
        COMMAND ${Python_EXECUTABLE} tools/build_config/version.py --pep440 sumo-${PACKAGE_VERSION}/tools/traci/__init__.py
        COMMAND ${Python_EXECUTABLE} tools/build_config/version.py --pep440 sumo-${PACKAGE_VERSION}/tools/libsumo/__init__.py
        COMMAND mkdir sumo-${PACKAGE_VERSION}/include
        COMMAND cp ${CMAKE_BINARY_DIR}/src/version.h sumo-${PACKAGE_VERSION}/include
        COMMAND zip -rq sumo-src-${PACKAGE_VERSION}.zip sumo-${PACKAGE_VERSION}
        COMMAND tar -czf sumo-src-${PACKAGE_VERSION}.tar.gz sumo-${PACKAGE_VERSION}
        COMMAND cp -a docs/userdoc docs/pydoc docs/javadoc docs/man sumo-${PACKAGE_VERSION}/docs
        COMMAND rm -r sumo-${PACKAGE_VERSION}/docs/web/docs/images
        COMMAND ln -s ../../userdoc/images sumo-${PACKAGE_VERSION}/docs/web/docs/images
        COMMAND tar -czf sumo_${PACKAGE_VERSION}.orig.tar.gz --exclude "*.jar" sumo-${PACKAGE_VERSION}
        COMMAND rm -rf sumo-${PACKAGE_VERSION} sumo-${PACKAGE_VERSION}.zip
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    add_dependencies(dist examples doc ${JAVA_TARGETS})
    set_property(TARGET dist PROPERTY FOLDER "CMake")
    set_property(TARGET dist PROPERTY EXCLUDE_FROM_DEFAULT_BUILD TRUE)

    add_custom_target(distcheck
        COMMAND rm -rf sumo-${PACKAGE_VERSION}
        COMMAND unzip -q sumo-src-${PACKAGE_VERSION}.zip
        COMMAND cd sumo-${PACKAGE_VERSION} && mkdir _cmake_build _cmake_install && cd _cmake_build
                && cmake -DCMAKE_INSTALL_PREFIX=../_cmake_install -DCHECK_OPTIONAL_LIBS=${CHECK_OPTIONAL_LIBS} .. || (echo "ERROR: the cmake configuration failed." && false)
                && make -j8 || (echo "ERROR: the compilation failed." && false)
                && make test || (echo "ERROR: the test suite failed." && false)
                && make install || (echo "ERROR: the install target failed." && false)
        COMMAND rm -rf sumo-${PACKAGE_VERSION}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
    add_dependencies(distcheck dist)
    set_property(TARGET distcheck PROPERTY FOLDER "CMake")
    set_property(TARGET distcheck PROPERTY EXCLUDE_FROM_DEFAULT_BUILD TRUE)
endif ()
message(STATUS "Enabled features: ${ENABLED_FEATURES}")
