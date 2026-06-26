add_executable(ec_demo ec_demo.c)
target_link_libraries(ec_demo soem m)
install(TARGETS ec_demo DESTINATION bin)
