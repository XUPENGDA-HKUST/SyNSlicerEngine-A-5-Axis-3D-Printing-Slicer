#ifndef SYNSLICERGUI_MESHHIERARCHYTREEWIDGET_H_
#define SYNSLICERGUI_MESHHIERARCHYTREEWIDGET_H_

#include <vector>

#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkCamera.h"
#include "qapplication.h"
#include "qtreewidget.h"
#include "qpushbutton.h"
#include "qlabel.h"
#include "qgridlayout.h"

#include "SynSlicerEngine/Object/partition_collection.h"

#include "object_drawer.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerGUI {

	class QPushButtonWithIndex : public QPushButton
	{
		Q_OBJECT			

	public:

		QPushButtonWithIndex() = delete;
		QPushButtonWithIndex(int index)
			: m_index(index)
		{
			QObject::connect(this, SIGNAL(clicked()), 
				this, SLOT(emitIndex()));
		};

		~QPushButtonWithIndex() {};

	Q_SIGNALS:
		void clipPartition(int index);

	protected Q_SLOTS:
		void emitIndex()
		{
			emit clipPartition(m_index);
		};

	protected:
		int m_index;

	};

	//! This class is used to show and control the partitions.
	class MeshHierarchyTreeWidget : public QTreeWidget
	{
		Q_OBJECT

	public:
		MeshHierarchyTreeWidget(QWidget *parent = nullptr);
		~MeshHierarchyTreeWidget();

		void addTopLevelItem(std::string name);
		void addChildItems(int top_item_index, int number_of_items_added);

		void setButtonsEnabled(bool enabled);

		void reset() override;

	public Q_SLOTS:
		void itemCheckStatusChanged(QTreeWidgetItem *p_widget, int colume);
		void partitionButtonClicked(int index);

	Q_SIGNALS:
		void changePartitionVisibility(int partition_index, bool visible);
		void clipPartition(int partition_index);

	protected:
		std::vector<QPushButtonWithIndex *> m_buttons;
	};
}

#endif  // SYNSLICERGUI_MESHHIERARCHYTREEWIDGET_H_