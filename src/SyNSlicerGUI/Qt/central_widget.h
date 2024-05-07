#ifndef SYNSLICERGUI_CENTRALWIDGET_H_
#define SYNSLICERGUI_CENTRALWIDGET_H_


#include <iostream>

#include <qwidget.h>
#include <qgridlayout.h>

#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <QVTKOpenGLNativeWidget.h>

namespace SyNSlicerGUI {

	class CentralWidget : public QWidget
	{
	public:
		CentralWidget(QWidget *parent = nullptr);
		~CentralWidget();

		//! Return VTKRenderer
		vtkRenderer *getVTKRenderer();

	protected:
		QGridLayout m_central_widget_layout;
		QVTKOpenGLNativeWidget m_opengl_native_widget;
		vtkGenericOpenGLRenderWindow *mp_render_window;
		vtkRenderer *mp_renderer;
	};
}

#endif  // SYNSLICERGUI_CENTRALWIDGET_H_