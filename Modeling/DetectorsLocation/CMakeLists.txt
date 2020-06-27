project(LDTC_MODELING_DETECTORS)

find_package(OpenCV REQUIRED)

set(HEADERS
        Src/DetectionMap.h
        Src/MapIsolineVisualization.h
        Src/MapVisualization.h
        Src/Mat.h
        )

set(EXECUTABLE
        Src/DetectionMap.cpp
        Src/MapIsolineVisualization.cpp
        Src/MapVisualization.cpp
        )

find_package (Threads)

add_executable(${PROJECT_NAME} main.cpp ${HEADERS} ${EXECUTABLE})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} LDTC ${CMAKE_THREAD_LIBS_INIT})
