if(TARGET fastcdr::fastcdr AND NOT TARGET fastcdr)
    add_library(fastcdr INTERFACE IMPORTED)
    set_property(TARGET fastcdr PROPERTY INTERFACE_LINK_LIBRARIES fastcdr::fastcdr)
endif()
