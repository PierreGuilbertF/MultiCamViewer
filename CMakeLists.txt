cmake_minimum_required (VERSION 3.0)

project(MultiCamViewer CXX)

if (MSVC)
	# set cxx 11 flaf
    set(CMAKE_CXX_FLAGS "-std=c++11")
	
	# set include dir to gui and core sources
    set(core_sources_include_dir core_sources)
	set(gui_sources_include_dir gui_sources)
    
    set (CMAKE_PREFIX_PATH "C:\\Qt\\Qt5.9.0\\5.9\\msvc2015_64")

    find_package(OpenCV REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(Qt5Core)
    find_package(Qt5Gui)
    find_package(Qt5Widgets)

    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(FORMS mainwindow.ui calibrationparametersdialog.ui distanceinfodialog.ui simulatedialog.ui colorcalibrationdialog.ui exportvideodialog.ui synchronizationdialog.ui)

    
    include_directories(Qt5::Core)
    include_directories(${MORE_INCLUDE})
    include_directories(${FFMPEG_INCLUDE})
    include_directories(${OpenCV_INCLUDE_DIRS} ${core_sources_include_dir} ${gui_sources_include_dir} ${Eigen3_DIR})
	
    QT5_WRAP_UI(FORMS_HEADERS ${FORMS})

    if( OpenCV_FOUND AND OpenCV_VERSION VERSION_LESS "3.1.0" )
        message(WARNING "Minimum support OpenCV version is 3.1.0, found ${OpenCV_VERSION}")
        set(OpenCV_FOUND FALSE)
    endif()

    add_executable( ${PROJECT_NAME} main.cxx
	        ${gui_sources_include_dir}/mainwindow.cpp
			${gui_sources_include_dir}/calibrationparametersdialog.cpp
			${gui_sources_include_dir}/distanceinfodialog.cpp
			${gui_sources_include_dir}/simulatedialog.cpp
			${gui_sources_include_dir}/colorcalibrationdialog.cpp
			${gui_sources_include_dir}/exportvideodialog.cpp
			${gui_sources_include_dir}/synchronizationdialog.cpp
            ${core_sources_include_dir}/Calibrator.cxx
            ${core_sources_include_dir}/ManualMatcher.cxx
            ${core_sources_include_dir}/Toolkit.cxx
            ${core_sources_include_dir}/NumericalAnalysisTools.cxx
            ${core_sources_include_dir}/StereoVideoManager.cxx
            ${core_sources_include_dir}/ColorCalibrator.cxx
            ${FORMS_HEADERS})

    QT5_USE_MODULES(MultiCamViewer Core Gui Widgets)

    target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBS})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
    target_link_libraries(${PROJECT_NAME} Qt5::Core)
    
    set (install_location "${CMAKE_CURRENT_BINARY_DIR}/install")
    message (${install_location})
else()
    SET(CMAKE_CXX_FLAGS "-std=c++0x")
    Set(dataPOCIncludeDir ../sources)

    find_package( OpenCV REQUIRED )
    find_package( Eigen3 REQUIRED )
    find_package(Qt5Core)
    find_package(Qt5Gui)
    find_package(Qt5Widgets)

    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(FORMS mainwindow.ui calibrationparametersdialog.ui)

    include_directories(Qt5::Core)
    include_directories( ${OpenCV_INCLUDE_DIRS} ${dataPOCIncludeDir} ${EIGEN_INCLUDE_DIR})

    QT5_WRAP_UI(FORMS_HEADERS ${FORMS})

    if( OpenCV_FOUND AND OpenCV_VERSION VERSION_LESS "3.1.0" )
        message(WARNING "Minimum support OpenCV version is 3.1.0, found ${OpenCV_VERSION}")
        set(OpenCV_FOUND FALSE)
    endif()

    add_executable( ${PROJECT_NAME} main.cxx mainwindow.cpp mainwindow.h calibrationparametersdialog.cpp calibrationparametersdialog.h
            ${dataPOCIncludeDir}/Calibrator.cxx
            ${dataPOCIncludeDir}/ManualMatcher.cxx
            ${dataPOCIncludeDir}/Toolkit.cxx
            ${dataPOCIncludeDir}/NumericalAnalysisTools.cxx
            ${dataPOCIncludeDir}/StereoVideoManager.cxx
            ${dataPOCIncludeDir}/AudioExtractor.cxx
            ${dataPOCIncludeDir}/ColorCalibrator.cxx
            ${FORMS_HEADERS})

    QT5_USE_MODULES(MultiCamViewer Core Gui Widgets)

    target_link_libraries(${PROJECT_NAME} avformat avcodec avutil swscale swresample)
    target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS})
    target_link_libraries( ${PROJECT_NAME} Qt5::Core)
endif()
