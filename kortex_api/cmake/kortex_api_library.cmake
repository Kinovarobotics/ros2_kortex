if(NOT TARGET kortex_api::KortexApiCpp)
  get_filename_component(_kortex_prefix "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
  add_library(kortex_api::KortexApiCpp STATIC IMPORTED)
  set_target_properties(kortex_api::KortexApiCpp PROPERTIES
    IMPORTED_LOCATION "${_kortex_prefix}/lib/kortex_api/libKortexApiCpp.a"
    INTERFACE_LINK_LIBRARIES pthread
  )
  unset(_kortex_prefix)
endif()
