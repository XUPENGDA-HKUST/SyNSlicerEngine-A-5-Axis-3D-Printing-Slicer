#ifndef SYNSLICERGUI_PATHPLANNINGDOCKWIDGET_H_
#define SYNSLICERGUI_PATHPLANNINGDOCKWIDGET_H_

//!  A Customized QDockWidget class. 
/*!
  A Customized QDockWidget class controlling all the functions in path planning operation.
*/

#include <iostream>
#include <vector>
#include <array>

#include "qapplication.h"
#include "qwidget.h"
#include "qdockwidget.h"
#include "qgridlayout.h"
#include "qpushbutton.h"
#include "qfiledialog.h"
#include "qtreewidget.h"
#include "qstackedwidget.h"
#include "qslider.h"

#include "vtkNew.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyleTrackballCamera.h"

#include "SyNSlicerEngine/Object/partition_collection.h"

#include "object_drawer.h"
#include "mesh_hierarchy_tree_widget.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerGUI {

	class PathPlanningDockWidget : public QDockWidget
	{
		Q_OBJECT
	public:
		PathPlanningDockWidget(vtkRenderer *input_renderer, QWidget *parent = nullptr);
		~PathPlanningDockWidget();

	protected slots:
		void changeShowOrHidePartition(int partition_index, bool visible);

	protected:
		void updateTreeWidget();

		vtkRenderer *mp_renderer;
		vtkRenderWindow *mp_render_window;
		vtkRenderWindowInteractor *mp_render_window_interactor;
		vtkInteractorStyleTrackballCamera *mp_default_interactor_style;
		ObjectDrawer m_drawer;

		QWidget m_widget;
		QGridLayout m_grid_layout;
		MeshHierarchyTreeWidget m_tree_widget;
		QPushButton m_reset_button;

		std::string model_name;
		SO::PartitionCollection<CgalMesh_EPICK> m_partitions;
	};
}
#endif  // SYNSLICERGUI_PATHPLANNINGDOCKWIDGET_H_