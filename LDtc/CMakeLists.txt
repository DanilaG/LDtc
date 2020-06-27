project(LDTC)

set(HEADERS
        Debug/Log.h
        Detection/DetectionMath.h
        Detection/DirectMethod.h
        Detection/QuadMethod.h
        Detection/VectorMethod.h
        Structures/Geometric/Points/Point.h
        Structures/Geometric/Points/TimePoint.h
        )

set(EXECUTABLE
        Debug/Log.cpp
        Detection/DirectMethod.cpp
        Detection/QuadMethod.cpp
        Detection/VectorMethod.cpp
        )

add_library(${PROJECT_NAME} ${HEADERS} ${EXECUTABLE})
