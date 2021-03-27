# Tries to find SFML
#
# SFML_FOUND          - set to true if SFML was found
# SFML_LIBRARIES      - link these to use SFML
# SFML_INCLUDE_DIR    - path to SFML header files

if (SFML_LIBRARIES AND SFML_INCLUDE_DIR)

  # cached by cmake already
  set(SFML TRUE)

else (SFML_LIBRARIES AND SFML_INCLUDE_DIR)

  set(CANDIDATE_LIB_DIR
    /usr/lib
    /usr/local/lib
    /usr/lib/SFML
    /usr/lib/i386-linux-gnu/
    /usr/lib/x86_64-linux-gnu/
    /opt/SFML/lib
  )

  set(CANDIDATE_INC_DIR
    /usr/include/
    /usr/include/SFML/
    /usr/local/include/
    /usr/local/include/SFML/
    /usr/local/SFML/include/
    /opt/SFML/include/
  )

  find_path(SFML_INCLUDE_DIR Main.hpp ${CANDIDATE_INC_DIR})

  find_library(SFML_AUDIO sfml-audio ${CANDIDATE_LIB_DIR})
  find_library(SFML_GRAPHICS sfml-graphics ${CANDIDATE_LIB_DIR})
  find_library(SFML_NETWORK sfml-network ${CANDIDATE_LIB_DIR})
  find_library(SFML_SYSTEM sfml-system ${CANDIDATE_LIB_DIR})
  find_library(SFML_WINDOW sfml-window ${CANDIDATE_LIB_DIR})


  set(SFML_LIBRARIES
    ${SFML_AUDIO}
    ${SFML_GRAPHICS}
    ${SFML_NETWORK}
    ${SFML_SYSTEM}
    ${SFML_WINDOW}
  )

  # output
  include(FindPackageHandleStandardArgs)

  find_package_handle_standard_args(SFML
    DEFAULT_MSG
    SFML_INCLUDE_DIR
    SFML_AUDIO
    SFML_GRAPHICS
    SFML_NETWORK
    SFML_SYSTEM
    SFML_WINDOW
  )

  mark_as_advanced(
    SFML_INCLUDE_DIR
    SFML_LIBRARIES
  )

endif (SFML_LIBRARIES AND SFML_INCLUDE_DIR)
