#ifndef SYNSLICERGUI_MENUBAR_H_
#define SYNSLICERGUI_MENUBAR_H_

#include "qmenubar.h"
#include "qmenu.h"
#include "qaction.h"
#include "qstring.h"
#include "qfiledialog.h"

namespace SyNSlicerGUI {

	class MenuBar : public QMenuBar
	{
		Q_OBJECT

	public:
		MenuBar(QWidget *parent = nullptr);
		~MenuBar();

		QAction &getActionOpenMesh();

	private slots:


	protected:
		QMenu m_file_menu;
		QAction m_action_open_mesh;
	};
}

#endif  // SYNSLICERGUI_MENUBAR_H_