#ifndef SYNSLICERGUI_MESHLOADINGWIDGET_H_
#define SYNSLICERGUI_MESHLOADINGWIDGET_H_

#include "qwidget.h"
#include "qpushbutton.h"
#include "qboxlayout.h"

namespace SyNSlicerGUI {

	class MeshLoadingWidget :public QWidget
	{
		Q_OBJECT

	public:
		MeshLoadingWidget(QWidget *parent = nullptr);
		~MeshLoadingWidget();

		QPushButton m_open_mesh_button;
		QPushButton m_open_default_mesh_button;

	Q_SIGNALS:


	protected Q_SLOTS:

		
	protected:
		QBoxLayout m_layout;

	};
}

#endif  // SYNSLICERGUI_MESHLOADINGWIDGET_H_