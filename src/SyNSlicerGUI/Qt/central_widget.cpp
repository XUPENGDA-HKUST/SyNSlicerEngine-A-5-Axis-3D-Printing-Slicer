#include "central_widget.h"

using SyNSlicerGUI::CentralWidget;

CentralWidget::CentralWidget(QWidget* parent)
	: QWidget(parent)
	, m_central_widget_layout(this)
	, m_opengl_native_widget(this)
	, mp_render_window(vtkGenericOpenGLRenderWindow::New())
	, mp_renderer(vtkRenderer::New())
{
	vtkNew<vtkNamedColors> colors;
	mp_renderer->SetBackground(colors->GetColor3d("White").GetData());
	this->setLayout(&m_central_widget_layout);
	m_central_widget_layout.addWidget(&m_opengl_native_widget);
	m_opengl_native_widget.setRenderWindow(mp_render_window);
	mp_render_window->AddRenderer(mp_renderer);
	mp_renderer->SetBackground2(colors->GetColor3d("Cadetblue").GetData());
	mp_renderer->SetGradientBackground(1);
	mp_render_window->Render();
}

CentralWidget::~CentralWidget()
{
	mp_render_window->Delete();
	mp_renderer->Delete();
}

vtkRenderer *CentralWidget::getVTKRenderer()
{
	return mp_renderer;
}
