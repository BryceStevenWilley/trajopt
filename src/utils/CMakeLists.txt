add_library(utils clock.cpp config.cpp logging.cpp)
target_link_libraries(utils ${Boost_PROGRAM_OPTIONS_LIBRARY})
