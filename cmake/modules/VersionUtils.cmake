# ==============================================================================
# MODULE: VersionUtils
# ==============================================================================
set(VERSION_FILE "${CMAKE_SOURCE_DIR}/Inc/version.h")

# WATCH FOR HEADER CHANGES
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${VERSION_FILE}")

# EXTRACT VERSION NUMBERS
if(EXISTS "${VERSION_FILE}")
    file(STRINGS "${VERSION_FILE}" V_MAJ_LINE REGEX "^#define[ \t]+VERSION_MAJOR[ \t]+([0-9]+)")
    string(REGEX REPLACE "^#define[ \t]+VERSION_MAJOR[ \t]+([0-9]+).*" "\\1" V_MAJ "${V_MAJ_LINE}")

    file(STRINGS "${VERSION_FILE}" V_MIN_LINE REGEX "^#define[ \t]+VERSION_MINOR[ \t]+([0-9]+)")
    string(REGEX REPLACE "^#define[ \t]+VERSION_MINOR[ \t]+([0-9]+).*" "\\1" V_MIN "${V_MIN_LINE}")

    set(FIRMWARE_VERSION "${V_MAJ}.${V_MIN}")
else()
    set(FIRMWARE_VERSION "0.00")
endif()

# GIT INFO & SMART DATING
find_package(Git)
set(IS_DIRTY FALSE)

if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
    # A. Git appends '-dirty' if uncommitted changes exist
    execute_process(COMMAND ${GIT_EXECUTABLE} describe --always --dirty 
        OUTPUT_VARIABLE GIT_HASH 
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(GIT_HASH MATCHES "-dirty$")
        set(IS_DIRTY TRUE)
    endif()

    # C. Determine Date/Time Source
    if(IS_DIRTY)
        # Dirty Build -> Use System Time (Shows "Now")
        message(STATUS "[Version] Dirty build detected. Using system time.")
        string(TIMESTAMP REPRO_DATE "%b %d %Y")
        string(TIMESTAMP REPRO_TIME "%H:%M:%S")
    else()
        # Clean Build -> Use Commit Time (Reproducible)
        execute_process(COMMAND ${GIT_EXECUTABLE} log -1 --format=%cd --date=format:"%b %d %Y"
            OUTPUT_VARIABLE REPRO_DATE
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        execute_process(COMMAND ${GIT_EXECUTABLE} log -1 --format=%cd --date=format:"%H:%M:%S"
            OUTPUT_VARIABLE REPRO_TIME
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )
    endif()
else()
    # No Git -> Use System Time
    string(TIMESTAMP REPRO_DATE "%b %d %Y")
    string(TIMESTAMP REPRO_TIME "%H:%M:%S")
endif()

# 4. EXPORT DEFINITIONS
# Overwrites standard macros. Safe because -Wno-builtin-macro-redefined is in flags.
add_compile_definitions(
    "__DATE__=\"${REPRO_DATE}\""
    "__TIME__=\"${REPRO_TIME}\""
)

# How this behaves now:
# Clean Build:
#   Hash: a1b2c3d
#   Date: Jan 01 2024 (The day you actually committed the code).
#   Result: Exact binary reproducibility. Anyone building this commit gets the exact same bytes.
#
# Dirty Build (Uncommitted changes):
#   Hash: a1b2c3d-dirty
#   Date: Jan 18 2026 (Today/Now).
#   Result: The date correctly tells you "This is a custom modification made on this day," preventing confusion with the official commit.