# ############################### Setting targets ####################################################################

set(TARGET
    esmini-zenoh)

# ############################### Loading desired rules ##############################################################

include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_static_analysis.cmake)
include(${CMAKE_SOURCE_DIR}/support/cmake/rule/disable_iwyu.cmake)

# ############################### Zenoh ##############################################################################
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

# ############################### Setting target files ###############################################################

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

# ############################### Creating executable ################################################################

add_executable(
    ${TARGET}
    ${SOURCES})

# embed $origin (location of exe file) and install (bin) dirs as execution dyn lib search paths
set(RPATH_DIRS
    "$ORIGIN:${INSTALL_PATH}")

if(DYN_PROTOBUF)
    # add OSI library folder to execution lib search paths
    set(RPATH_DIRS
        "${RPATH_DIRS}:${EXTERNALS_OSI_LIBRARY_PATH}")
endif()

set_target_properties(
    ${TARGET}
    PROPERTIES BUILD_WITH_INSTALL_RPATH
               true
               INSTALL_RPATH
               "$ORIGIN:${INSTALL_PATH}")

target_compile_definitions(${TARGET}
    PRIVATE ZENOHCXX_ZENOHC=1  # Use Zenoh-C backend
    # PRIVATE ZENOHCXX_ZENOHPICO=1  # Uncomment to use Zenoh-Pico backend
)

target_include_directories(
    ${TARGET}
    PRIVATE ${ESMINI_LIB_PATH}
            ${COMMON_MINI_PATH})

target_include_directories(
    ${TARGET}
    SYSTEM
    PUBLIC ${EXTERNALS_OSI_INCLUDES})

target_link_libraries(
    ${TARGET}
    PRIVATE esminiLib
    PRIVATE CommonMini
    PRIVATE ${TIME_LIB}
    zenohcxx::zenohc)
# PRIVATE ${FMT_LIBRARIES})

if(USE_OSI)
    target_link_libraries(
        ${TARGET}
        PRIVATE ${OSI_LIBRARIES})
endif()

if(USE_IMPLOT)
    target_link_libraries(
        ${TARGET}
        PRIVATE ${IMPLOT_LIBRARIES})
endif()

# ############################### Install ############################################################################

install(
    TARGETS ${TARGET}
    DESTINATION "${INSTALL_PATH}")
