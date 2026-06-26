enable_language(CXX)

set(SIASUN_TINYXML2_DIR "${CMAKE_CURRENT_LIST_DIR}/thirdparty/tinyxml2")

add_executable(ec_siasun
    src/main.cpp
    src/axis_config.cpp
    src/ethercat_app.cpp
    src/sdo_parameter.cpp
    "${SIASUN_TINYXML2_DIR}/tinyxml2.cpp"
)

set_target_properties(ec_siasun PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

target_include_directories(ec_siasun PRIVATE
    include
    "${SIASUN_TINYXML2_DIR}"
)

target_link_libraries(ec_siasun PRIVATE
    soem
    pthread
    m
    rt
)

target_compile_options(ec_siasun PRIVATE
    -Wall
    -Wextra
)

install(TARGETS ec_siasun DESTINATION bin)
