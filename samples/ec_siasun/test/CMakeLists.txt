add_executable(ec_siasun_communication_test
    communication_test.cpp
)

set_target_properties(ec_siasun_communication_test PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

target_link_libraries(ec_siasun_communication_test PRIVATE
    ec_siasun_core
)

target_compile_options(ec_siasun_communication_test PRIVATE
    -Wall
    -Wextra
)
