if(PTEX_BUILD_STATIC_LIBS)
    add_definitions(-DPTEX_STATIC)
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_executable(wtest_plane wtest_plane.cpp)
add_executable(wtest wtest.cpp)
add_executable(rtest rtest.cpp)
add_executable(ftest ftest.cpp)
add_executable(halftest halftest.cpp)
add_executable(ptxtransfer ptxtransfer.cpp)

find_package(CGAL REQUIRED)
target_link_libraries(ptxtransfer CGAL::CGAL)

target_link_libraries(wtest_plane ${PTEX_LIBRARY})
target_link_libraries(wtest ${PTEX_LIBRARY})
target_link_libraries(rtest ${PTEX_LIBRARY})
target_link_libraries(ftest ${PTEX_LIBRARY})
target_link_libraries(halftest ${PTEX_LIBRARY})
target_link_libraries(ptxtransfer ${PTEX_LIBRARY})

# create a function to add tests that compare output
# file results
function(add_compare_test test_name)
    add_test(NAME ${test_name}
        COMMAND ${CMAKE_COMMAND}
        -DOUT=${CMAKE_CURRENT_BINARY_DIR}/${test_name}.out
        -DDATA=${CMAKE_CURRENT_SOURCE_DIR}/${test_name}ok.dat
        -DCMD=$<TARGET_FILE:${test_name}>
        -P ${CMAKE_CURRENT_SOURCE_DIR}/compare_test.cmake)
endfunction()

# add all the tests
add_test(NAME wtest_plane COMMAND wtest_plane)
add_test(NAME wtest COMMAND wtest)
add_compare_test(rtest)
add_compare_test(ftest)
add_test(NAME halftest COMMAND halftest)
add_test(NAME ptxtransfer COMMAND ptxtransfer)
