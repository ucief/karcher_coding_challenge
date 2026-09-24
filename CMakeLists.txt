cmake_minimum_required(VERSION 3.16)
project(path_analysis LANGUAGES CXX)

find_package(Boost 1.75 REQUIRED COMPONENTS json)

add_library(path_analysis src/path.cpp src/json_file.cpp)
target_include_directories(path_analysis PUBLIC include)
target_link_libraries(path_analysis PUBLIC Boost::headers PRIVATE Boost::json)
target_compile_features(path_analysis PUBLIC cxx_std_17)

add_executable(analyze_path src/main.cpp)
target_link_libraries(analyze_path PRIVATE path_analysis)

include(CTest)
if(BUILD_TESTING)
    foreach(name IN ITEMS path json_file)
        add_executable(test_${name} tests/test_${name}.cpp)
        target_link_libraries(test_${name} PRIVATE path_analysis)
        add_test(NAME ${name} COMMAND test_${name})
    endforeach()
    target_compile_definitions(test_json_file PRIVATE
        TEST_DATA_FILE="${CMAKE_CURRENT_SOURCE_DIR}/data/short.json")
endif()
