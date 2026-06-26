add_executable(ec_communication ec_communication.c)
target_link_libraries(ec_communication soem)
install(TARGETS ec_communication DESTINATION bin)
