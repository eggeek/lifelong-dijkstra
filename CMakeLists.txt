cmake_minimum_required(VERSION 3.5)
project(road-cpd)

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS "-DUSE_PARALLELISM -DEXTRACT_ALL_AT_ONCE")
# if(APPLE)
#     set(CMAKE_CXX_FLAGS "-L/usr/local/opt/libomp/lib -I/usr/local/opt/libomp/include ${CMAKE_CXX_FLAGS} -lomp")
# else (APPLE)
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
# endif(APPLE)

set(DEV_CXX_FLAGS "-g -ggdb -O0 -fno-omit-frame-pointer")
set(TEST_CXX_FLAGS "-O3")
set(RELEASE_CXX_FLAGS "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} ${DEV_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_TEST "${CMAKE_CXX_FLAGS} ${TEST_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${RELEASE_CXX_FLAGS}")
include_directories(src)
include_directories(utility)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/../bin)



set(SOURCE_FILES
    src/adj_array.h
    src/adj_graph.h
    src/dijkstra.h
    src/lifelong_dij.h
    src/list_graph.h
    utility/range.h
    utility/heap.h
    utility/coord.h
    utility/RoadNetworkLoader.h)

add_executable(main
        ${SOURCE_FILES}
        main.cpp)

# target_link_libraries(main ${OpenMP_omp_LIBRARY})
