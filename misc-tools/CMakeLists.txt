
# provide path to dependency libraries

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src)

ADD_EXECUTABLE(flexEstimatorTester model-base-flex-estimator-file-tester.cpp ${${PROJECT_NAME}_ABSOLUTE_HEADERS})
ADD_EXECUTABLE(fileSplitter file-splitter.cpp ${${PROJECT_NAME}_ABSOLUTE_HEADERS})

TARGET_LINK_LIBRARIES(flexEstimatorTester ${Boost_LIBRARIES} ${PROJECT_NAME})
TARGET_LINK_LIBRARIES(fileSplitter ${Boost_LIBRARIES} ${PROJECT_NAME})