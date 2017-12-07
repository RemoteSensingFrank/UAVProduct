# ==============================================================================
# Generic CMake File for QTBased project
# ==============================================================================

find_package( Qt5Widgets )

if(Qt5Widgets_FOUND)
    if(POLICY CMP0020)
        cmake_policy(SET CMP0020 OLD)
    endif(POLICY CMP0020)

    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_AUTOMOC ON)
    # The name of my project.
    SET(ProjectName GUI_GCPs)

    # All Source Files here
    SET(SourceFiles
            main.cpp
            mainLayout.cpp
            GraphicsMainWindow.cpp
            ControlPointTableView.cpp
            node.cpp)

    # All non moc header files here
    SET(HeaderFiles
            mainLayout.hpp
            GraphicsMainWindow.hpp
            node.hpp
            document.hpp)

    # ==============================================================================
    # Add the following files, modules and libraries to the project.
    # ==============================================================================
    # Files
    ADD_EXECUTABLE(${ProjectName}
            ${SourceFiles}
            ${HeaderFiles} )
    #${MocSourceFiles})

    # Linking with Qt Libraries.
    TARGET_LINK_LIBRARIES(${ProjectName} Qt5::Widgets)
endif(Qt5Widgets_FOUND)
