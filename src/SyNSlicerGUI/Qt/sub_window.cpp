#include "sub_window.h"

using SyNSlicerGUI::SubWindow;

SubWindow::SubWindow()
{
    resize(1280,720);
	move(500, 100);
    setWindowTitle("Debug Window");

	mp_central_widget = new QWidget(this);
	setCentralWidget(mp_central_widget);
	mp_central_widget_layout = new QGridLayout();
	mp_opengl_native_widget = new QVTKOpenGLNativeWidget(this);
	mp_render_window = vtkGenericOpenGLRenderWindow::New();
	mp_renderer = vtkRenderer::New();
	vtkNew<vtkNamedColors> colors;
	mp_renderer->SetBackground(colors->GetColor3d("White").GetData());
	mp_central_widget->setLayout(mp_central_widget_layout);
	mp_central_widget_layout->addWidget(mp_opengl_native_widget, 0, 0);
	mp_opengl_native_widget->setRenderWindow(mp_render_window);
	mp_render_window->AddRenderer(mp_renderer);
	mp_renderer->SetBackground2(colors->GetColor3d("Cadetblue").GetData());
	mp_renderer->SetGradientBackground(1);
	mp_render_window->Render();
}

SubWindow::~SubWindow()
{

}

vtkRenderer *SubWindow::getRenderer()
{
	return mp_renderer;
}

void SubWindow::resetCamera()
{
	mp_renderer->ResetCamera();
}

void SubWindow::render()
{
	mp_render_window->Render();
}