#include "menu_bar.h"

using SyNSlicerGUI::MenuBar;

MenuBar::MenuBar(QWidget *parent)
	: QMenuBar(parent)
	, m_file_menu(tr("File"))
	, m_action_open_mesh(tr("Open Mesh"))

{
	this->addMenu(&m_file_menu);
	m_file_menu.addAction(&m_action_open_mesh);
}

MenuBar::~MenuBar()
{
}

QAction &MenuBar::getActionOpenMesh()
{
	return m_action_open_mesh;
}
