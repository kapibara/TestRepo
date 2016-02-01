set(LIBKINECT2 libkinect2)
set(EXECTEST test)

set(HEADERS ../include/kinect2videosource.h ../include/stdafx.h ../include/bufferwrapper.h)
set(SOURCE kinect2videosource.cpp)

set(SOURCE_TEST main.cpp)

add_library(${LIBKINECT2} ${SOURCE} ${HEADERS})
add_executable(${EXECTEST} ${SOURCE_TEST} ${SOURCE} ${HEADERS})

target_link_libraries(${LIBKINECT2} ${KINECT2_LIBS})
target_link_libraries(${EXECTEST} ${KINECT2_LIBS})

install(TARGETS ${LIBKINECT2} DESTINATION lib)
install(TARGETS ${EXECTEST} DESTINATION bin)
