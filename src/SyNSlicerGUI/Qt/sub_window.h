#ifndef SYNSLICER_PATHPLANNING_SUBWINDOW_H_
#define SYNSLICER_PATHPLANNING_SUBWINDOW_H_

#include <iostream>

#include <qapplication.h>
#include <qmainwindow.h>
#include <qwidget.h>
#include <qgridlayout.h>

#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <QVTKOpenGLNativeWidget.h>

namespace SyNSlicerGUI {

    //! This class defines a independent window for debug purpose.
    class SubWindow : public QMainWindow
    {
        Q_OBJECT

    public:
        SubWindow();
        ~SubWindow();

        vtkRenderer *getRenderer();
        void resetCamera();
        void render();

    private:
        QWidget *mp_central_widget;
        QGridLayout *mp_central_widget_layout;
        QVTKOpenGLNativeWidget *mp_opengl_native_widget;
        vtkGenericOpenGLRenderWindow *mp_render_window;
        vtkRenderer *mp_renderer;
    };
}
#endif  // SYNSLICER_PATHPLANNING_SUBWINDOW_H_
