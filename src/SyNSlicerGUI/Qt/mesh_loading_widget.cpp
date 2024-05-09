#include "mesh_loading_widget.h"

using SyNSlicerGUI::MeshLoadingWidget;

MeshLoadingWidget::MeshLoadingWidget(QWidget *parent)
	: QWidget(parent)
	, m_layout(QBoxLayout::TopToBottom)
	, m_open_mesh_button(tr("Open Mesh"))
	, m_open_default_mesh_button(tr("Open Default Mesh"))
{
	this->setLayout(&m_layout);
	m_layout.addWidget(&m_open_mesh_button);
	m_layout.addWidget(&m_open_default_mesh_button);
}

MeshLoadingWidget::~MeshLoadingWidget()
{

}