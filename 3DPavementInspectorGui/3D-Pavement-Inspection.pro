QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

INCLUDEPATH = controllers

SOURCES += \
    controllers/CameraManager.cpp \
    controllers/ProjectManager.cpp \
    controllers/filemanager.cpp \
    main.cpp \
    mainwindow.cpp \
    widgets/NewSolutionDialog.cpp

HEADERS += \
    controllers/CameraManager.h \
    controllers/ProjectManager.h \
    controllers/filemanager.h \
    mainwindow.h \
    widgets/NewSolutionDialog.h

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    qdarkstyle/style.qrc \
    resource.qrc

#win32:CONFIG(release, debug|release): LIBS += -LC:/envs/opencv4.5.5/buildv2/install/x64/vc17/lib/ -lopencv_world455
#else:win32:CONFIG(debug, debug|release): LIBS += -LC:/envs/opencv4.5.5/buildv2/install/x64/vc17/lib/ -lopencv_world455d
#else:unix: LIBS += -LC:/envs/opencv4.5.5/buildv2/install/x64/vc17/lib/ -lopencv_world455

#INCLUDEPATH += C:/envs/opencv4.5.5/buildv2/install/include
#DEPENDPATH += C:/envs/opencv4.5.5/buildv2/install/include




DISTFILES += \
    qdarkstyle/rc/arrow_down.png \
    qdarkstyle/rc/arrow_down@2x.png \
    qdarkstyle/rc/arrow_down_disabled.png \
    qdarkstyle/rc/arrow_down_disabled@2x.png \
    qdarkstyle/rc/arrow_down_focus.png \
    qdarkstyle/rc/arrow_down_focus@2x.png \
    qdarkstyle/rc/arrow_down_pressed.png \
    qdarkstyle/rc/arrow_down_pressed@2x.png \
    qdarkstyle/rc/arrow_left.png \
    qdarkstyle/rc/arrow_left@2x.png \
    qdarkstyle/rc/arrow_left_disabled.png \
    qdarkstyle/rc/arrow_left_disabled@2x.png \
    qdarkstyle/rc/arrow_left_focus.png \
    qdarkstyle/rc/arrow_left_focus@2x.png \
    qdarkstyle/rc/arrow_left_pressed.png \
    qdarkstyle/rc/arrow_left_pressed@2x.png \
    qdarkstyle/rc/arrow_right.png \
    qdarkstyle/rc/arrow_right@2x.png \
    qdarkstyle/rc/arrow_right_disabled.png \
    qdarkstyle/rc/arrow_right_disabled@2x.png \
    qdarkstyle/rc/arrow_right_focus.png \
    qdarkstyle/rc/arrow_right_focus@2x.png \
    qdarkstyle/rc/arrow_right_pressed.png \
    qdarkstyle/rc/arrow_right_pressed@2x.png \
    qdarkstyle/rc/arrow_up.png \
    qdarkstyle/rc/arrow_up@2x.png \
    qdarkstyle/rc/arrow_up_disabled.png \
    qdarkstyle/rc/arrow_up_disabled@2x.png \
    qdarkstyle/rc/arrow_up_focus.png \
    qdarkstyle/rc/arrow_up_focus@2x.png \
    qdarkstyle/rc/arrow_up_pressed.png \
    qdarkstyle/rc/arrow_up_pressed@2x.png \
    qdarkstyle/rc/base_icon.png \
    qdarkstyle/rc/base_icon@2x.png \
    qdarkstyle/rc/base_icon_disabled.png \
    qdarkstyle/rc/base_icon_disabled@2x.png \
    qdarkstyle/rc/base_icon_focus.png \
    qdarkstyle/rc/base_icon_focus@2x.png \
    qdarkstyle/rc/base_icon_pressed.png \
    qdarkstyle/rc/base_icon_pressed@2x.png \
    qdarkstyle/rc/branch_closed.png \
    qdarkstyle/rc/branch_closed@2x.png \
    qdarkstyle/rc/branch_closed_disabled.png \
    qdarkstyle/rc/branch_closed_disabled@2x.png \
    qdarkstyle/rc/branch_closed_focus.png \
    qdarkstyle/rc/branch_closed_focus@2x.png \
    qdarkstyle/rc/branch_closed_pressed.png \
    qdarkstyle/rc/branch_closed_pressed@2x.png \
    qdarkstyle/rc/branch_end.png \
    qdarkstyle/rc/branch_end@2x.png \
    qdarkstyle/rc/branch_end_disabled.png \
    qdarkstyle/rc/branch_end_disabled@2x.png \
    qdarkstyle/rc/branch_end_focus.png \
    qdarkstyle/rc/branch_end_focus@2x.png \
    qdarkstyle/rc/branch_end_pressed.png \
    qdarkstyle/rc/branch_end_pressed@2x.png \
    qdarkstyle/rc/branch_line.png \
    qdarkstyle/rc/branch_line@2x.png \
    qdarkstyle/rc/branch_line_disabled.png \
    qdarkstyle/rc/branch_line_disabled@2x.png \
    qdarkstyle/rc/branch_line_focus.png \
    qdarkstyle/rc/branch_line_focus@2x.png \
    qdarkstyle/rc/branch_line_pressed.png \
    qdarkstyle/rc/branch_line_pressed@2x.png \
    qdarkstyle/rc/branch_more.png \
    qdarkstyle/rc/branch_more@2x.png \
    qdarkstyle/rc/branch_more_disabled.png \
    qdarkstyle/rc/branch_more_disabled@2x.png \
    qdarkstyle/rc/branch_more_focus.png \
    qdarkstyle/rc/branch_more_focus@2x.png \
    qdarkstyle/rc/branch_more_pressed.png \
    qdarkstyle/rc/branch_more_pressed@2x.png \
    qdarkstyle/rc/branch_open.png \
    qdarkstyle/rc/branch_open@2x.png \
    qdarkstyle/rc/branch_open_disabled.png \
    qdarkstyle/rc/branch_open_disabled@2x.png \
    qdarkstyle/rc/branch_open_focus.png \
    qdarkstyle/rc/branch_open_focus@2x.png \
    qdarkstyle/rc/branch_open_pressed.png \
    qdarkstyle/rc/branch_open_pressed@2x.png \
    qdarkstyle/rc/checkbox_checked.png \
    qdarkstyle/rc/checkbox_checked@2x.png \
    qdarkstyle/rc/checkbox_checked_disabled.png \
    qdarkstyle/rc/checkbox_checked_disabled@2x.png \
    qdarkstyle/rc/checkbox_checked_focus.png \
    qdarkstyle/rc/checkbox_checked_focus@2x.png \
    qdarkstyle/rc/checkbox_checked_pressed.png \
    qdarkstyle/rc/checkbox_checked_pressed@2x.png \
    qdarkstyle/rc/checkbox_indeterminate.png \
    qdarkstyle/rc/checkbox_indeterminate@2x.png \
    qdarkstyle/rc/checkbox_indeterminate_disabled.png \
    qdarkstyle/rc/checkbox_indeterminate_disabled@2x.png \
    qdarkstyle/rc/checkbox_indeterminate_focus.png \
    qdarkstyle/rc/checkbox_indeterminate_focus@2x.png \
    qdarkstyle/rc/checkbox_indeterminate_pressed.png \
    qdarkstyle/rc/checkbox_indeterminate_pressed@2x.png \
    qdarkstyle/rc/checkbox_unchecked.png \
    qdarkstyle/rc/checkbox_unchecked@2x.png \
    qdarkstyle/rc/checkbox_unchecked_disabled.png \
    qdarkstyle/rc/checkbox_unchecked_disabled@2x.png \
    qdarkstyle/rc/checkbox_unchecked_focus.png \
    qdarkstyle/rc/checkbox_unchecked_focus@2x.png \
    qdarkstyle/rc/checkbox_unchecked_pressed.png \
    qdarkstyle/rc/checkbox_unchecked_pressed@2x.png \
    qdarkstyle/rc/line_horizontal.png \
    qdarkstyle/rc/line_horizontal@2x.png \
    qdarkstyle/rc/line_horizontal_disabled.png \
    qdarkstyle/rc/line_horizontal_disabled@2x.png \
    qdarkstyle/rc/line_horizontal_focus.png \
    qdarkstyle/rc/line_horizontal_focus@2x.png \
    qdarkstyle/rc/line_horizontal_pressed.png \
    qdarkstyle/rc/line_horizontal_pressed@2x.png \
    qdarkstyle/rc/line_vertical.png \
    qdarkstyle/rc/line_vertical@2x.png \
    qdarkstyle/rc/line_vertical_disabled.png \
    qdarkstyle/rc/line_vertical_disabled@2x.png \
    qdarkstyle/rc/line_vertical_focus.png \
    qdarkstyle/rc/line_vertical_focus@2x.png \
    qdarkstyle/rc/line_vertical_pressed.png \
    qdarkstyle/rc/line_vertical_pressed@2x.png \
    qdarkstyle/rc/radio_checked.png \
    qdarkstyle/rc/radio_checked@2x.png \
    qdarkstyle/rc/radio_checked_disabled.png \
    qdarkstyle/rc/radio_checked_disabled@2x.png \
    qdarkstyle/rc/radio_checked_focus.png \
    qdarkstyle/rc/radio_checked_focus@2x.png \
    qdarkstyle/rc/radio_checked_pressed.png \
    qdarkstyle/rc/radio_checked_pressed@2x.png \
    qdarkstyle/rc/radio_unchecked.png \
    qdarkstyle/rc/radio_unchecked@2x.png \
    qdarkstyle/rc/radio_unchecked_disabled.png \
    qdarkstyle/rc/radio_unchecked_disabled@2x.png \
    qdarkstyle/rc/radio_unchecked_focus.png \
    qdarkstyle/rc/radio_unchecked_focus@2x.png \
    qdarkstyle/rc/radio_unchecked_pressed.png \
    qdarkstyle/rc/radio_unchecked_pressed@2x.png \
    qdarkstyle/rc/toolbar_move_horizontal.png \
    qdarkstyle/rc/toolbar_move_horizontal@2x.png \
    qdarkstyle/rc/toolbar_move_horizontal_disabled.png \
    qdarkstyle/rc/toolbar_move_horizontal_disabled@2x.png \
    qdarkstyle/rc/toolbar_move_horizontal_focus.png \
    qdarkstyle/rc/toolbar_move_horizontal_focus@2x.png \
    qdarkstyle/rc/toolbar_move_horizontal_pressed.png \
    qdarkstyle/rc/toolbar_move_horizontal_pressed@2x.png \
    qdarkstyle/rc/toolbar_move_vertical.png \
    qdarkstyle/rc/toolbar_move_vertical@2x.png \
    qdarkstyle/rc/toolbar_move_vertical_disabled.png \
    qdarkstyle/rc/toolbar_move_vertical_disabled@2x.png \
    qdarkstyle/rc/toolbar_move_vertical_focus.png \
    qdarkstyle/rc/toolbar_move_vertical_focus@2x.png \
    qdarkstyle/rc/toolbar_move_vertical_pressed.png \
    qdarkstyle/rc/toolbar_move_vertical_pressed@2x.png \
    qdarkstyle/rc/toolbar_separator_horizontal.png \
    qdarkstyle/rc/toolbar_separator_horizontal@2x.png \
    qdarkstyle/rc/toolbar_separator_horizontal_disabled.png \
    qdarkstyle/rc/toolbar_separator_horizontal_disabled@2x.png \
    qdarkstyle/rc/toolbar_separator_horizontal_focus.png \
    qdarkstyle/rc/toolbar_separator_horizontal_focus@2x.png \
    qdarkstyle/rc/toolbar_separator_horizontal_pressed.png \
    qdarkstyle/rc/toolbar_separator_horizontal_pressed@2x.png \
    qdarkstyle/rc/toolbar_separator_vertical.png \
    qdarkstyle/rc/toolbar_separator_vertical@2x.png \
    qdarkstyle/rc/toolbar_separator_vertical_disabled.png \
    qdarkstyle/rc/toolbar_separator_vertical_disabled@2x.png \
    qdarkstyle/rc/toolbar_separator_vertical_focus.png \
    qdarkstyle/rc/toolbar_separator_vertical_focus@2x.png \
    qdarkstyle/rc/toolbar_separator_vertical_pressed.png \
    qdarkstyle/rc/toolbar_separator_vertical_pressed@2x.png \
    qdarkstyle/rc/transparent.png \
    qdarkstyle/rc/transparent@2x.png \
    qdarkstyle/rc/transparent_disabled.png \
    qdarkstyle/rc/transparent_disabled@2x.png \
    qdarkstyle/rc/transparent_focus.png \
    qdarkstyle/rc/transparent_focus@2x.png \
    qdarkstyle/rc/transparent_pressed.png \
    qdarkstyle/rc/transparent_pressed@2x.png \
    qdarkstyle/rc/window_close.png \
    qdarkstyle/rc/window_close@2x.png \
    qdarkstyle/rc/window_close_disabled.png \
    qdarkstyle/rc/window_close_disabled@2x.png \
    qdarkstyle/rc/window_close_focus.png \
    qdarkstyle/rc/window_close_focus@2x.png \
    qdarkstyle/rc/window_close_pressed.png \
    qdarkstyle/rc/window_close_pressed@2x.png \
    qdarkstyle/rc/window_grip.png \
    qdarkstyle/rc/window_grip@2x.png \
    qdarkstyle/rc/window_grip_disabled.png \
    qdarkstyle/rc/window_grip_disabled@2x.png \
    qdarkstyle/rc/window_grip_focus.png \
    qdarkstyle/rc/window_grip_focus@2x.png \
    qdarkstyle/rc/window_grip_pressed.png \
    qdarkstyle/rc/window_grip_pressed@2x.png \
    qdarkstyle/rc/window_minimize.png \
    qdarkstyle/rc/window_minimize@2x.png \
    qdarkstyle/rc/window_minimize_disabled.png \
    qdarkstyle/rc/window_minimize_disabled@2x.png \
    qdarkstyle/rc/window_minimize_focus.png \
    qdarkstyle/rc/window_minimize_focus@2x.png \
    qdarkstyle/rc/window_minimize_pressed.png \
    qdarkstyle/rc/window_minimize_pressed@2x.png \
    qdarkstyle/rc/window_undock.png \
    qdarkstyle/rc/window_undock@2x.png \
    qdarkstyle/rc/window_undock_disabled.png \
    qdarkstyle/rc/window_undock_disabled@2x.png \
    qdarkstyle/rc/window_undock_focus.png \
    qdarkstyle/rc/window_undock_focus@2x.png \
    qdarkstyle/rc/window_undock_pressed.png \
    qdarkstyle/rc/window_undock_pressed@2x.png \
    qdarkstyle/style.qss






win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../envs/opencv455_contrib/install/x64/vc17/lib/ -lopencv_world455
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../envs/opencv455_contrib/install/x64/vc17/lib/ -lopencv_world455d
else:unix: LIBS += -L$$PWD/../../../../../envs/opencv455_contrib/install/x64/vc17/lib/ -lopencv_world455

INCLUDEPATH += $$PWD/../../../../../envs/opencv455_contrib/install/include
DEPENDPATH += $$PWD/../../../../../envs/opencv455_contrib/install/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../librealsense_build/release/ -lrealsense2
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../librealsense_build/debug/ -lrealsense2
else:unix: LIBS += -L$$PWD/../../librealsense_build/ -lrealsense2

INCLUDEPATH += $$PWD/../../librealsense/include
DEPENDPATH += $$PWD/../../librealsense/include
