#ifndef SYNSLICER_PATHPLANNING_GUI_MESHHIERARCHYTREEWIDGET_H_
#define SYNSLICER_PATHPLANNING_GUI_MESHHIERARCHYTREEWIDGET_H_

#include <vector>

#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkCamera.h"
#include "qapplication.h"
#include "qtreewidget.h"
#include "qpushbutton.h"

#include "SynSlicerEngine/Object/partition_collection.h"

#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerGUI {

	//! This class is used to show and control the partitions.
	class MeshHierarchyTreeWidget : public QTreeWidget
	{
		Q_OBJECT

	public:
		MeshHierarchyTreeWidget(QWidget *parent = nullptr);
		~MeshHierarchyTreeWidget();

		void addTopLevelItem(std::string name);
		void addChildItems(int top_item_index, int number_of_items_added);

	public Q_SLOTS:
		void itemCheckStatusChanged(QTreeWidgetItem *p_widget, int colume);

	Q_SIGNALS:
		void changePartitionVisibility(int partition_index, bool visible);

	protected:
		//! Not necessary.
		std::vector<QTreeWidgetItem *> m_top_items;

	};
}


#endif  // SYNSLICER_PATHPLANNING_GUI_MESHHIERARCHYTREEWIDGET_H_