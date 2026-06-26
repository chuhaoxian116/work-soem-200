add_executable(ec_pdoconfig ec_pdoconfig.c)
target_link_libraries(ec_pdoconfig soem)
install(TARGETS ec_pdoconfig DESTINATION bin)
