FIND_PACKAGE(OpenCV REQUIRED)


ADD_LIBRARY(RGBDFrame RGBDFrame.cpp)
TARGET_LINK_LIBRARIES(RGBDFrame ${OpenCV_LIBS})
# ADD_EXECUTABLE(test_RGBDFrame test_rgbdframe.cpp)
# TARGET_LINK_LIBRARIES(test_RGBDFrame RGBDFrame ${OpenCV_LIBS})
#ADD_EXECUTABLE(train_abw train_abw.cpp)
#TARGET_LINK_LIBRARIES(train_abw RGBDFrame ${OpenCV_LIBS})
ADD_EXECUTABLE(train_example train_example.cpp)
TARGET_LINK_LIBRARIES(train_example RGBDFrame ${OpenCV_LIBS})
#ADD_EXECUTABLE(train_nyu train_nyu.cpp)
#TARGET_LINK_LIBRARIES(train_nyu RGBDFrame ${OpenCV_LIBS})

ADD_EXECUTABLE(train_uw train_uw.cpp)
TARGET_LINK_LIBRARIES(train_uw RGBDFrame ${OpenCV_LIBS})