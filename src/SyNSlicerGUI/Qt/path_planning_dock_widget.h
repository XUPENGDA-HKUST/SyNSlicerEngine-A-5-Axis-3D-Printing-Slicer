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

#include <VTK_CORE_CLASS>

#include "SyNSlicerEngine/Object/partition_collection.h"
#include "SyNSlicerEngine/Algorithm/mesh_clipper.h"

#include "object_drawer.h"
#include "mesh_hierarchy_tree_widget.h"
#include "path_planning_stacked_widget.h"
#include "model_clipping_interactor_style.h"
#include "auto_slicer_gui.h"
#include "auto_partitioner_gui.h"
#include "printing_sequence_determinator_gui.h"
#include "support_generator_gui.h"
#include "toolpath_generator_gui.h"
#include "gcode_generator_gui.h"
#include "mesh_clipper_gui.h"

namespace SO = SyNSlicerEngine::Object;
namespace SA = SyNSlicerEngine::Algorithm;

namespace SyNSlicerGUI {

	class PathPlanningDockWidget : public QDockWidget
	{
		Q_OBJECT

	public:
		PathPlanningDockWidget(vtkRenderer *input_renderer, QWidget *parent = nullptr);
		~PathPlanningDockWidget();

	protected Q_SLOTS:
		void reset();
		void openMesh();
		void openDefaultMesh();
		void changeShowOrHidePartition(int partition_index, bool visible);
		void setClipPartition(int partition_index);
		void clipPartition();
		void clipPartitionWithFinitePlane();
		void runAutoPartition();
		void confirmPartition();
		void resetPartitionResult();
		void exitPartition();
		void generateToolPath(std::tuple<int, int, int, int, int, int> toolpath_setting);
		void changePreviewLayer(int value);
		void returnToToolpathGenerationWidget();
		void generateGcode();

	protected:

		void updateTreeWidget();
		void drawPartitions();
		void drawPartitionsInPartitioning();
		void updatePrintingPath();

		vtkRenderer *mp_renderer;
		vtkRenderWindow *mp_render_window;
		vtkRenderWindowInteractor *mp_render_window_interactor;
		vtkInteractorStyleTrackballCamera *mp_default_interactor_style;
		ObjectDrawer m_drawer;
		ObjectDrawer m_drawer_for_preview;

		QWidget m_widget;
		QGridLayout m_grid_layout;
		MeshHierarchyTreeWidget m_tree_widget;
		PathPlanningStackedWidget m_stacked_widget;

		QPushButton m_reset_button;

		std::string model_name;
		SO::PartitionCollection<CgalMesh_EPICK> m_partitions;
		SO::Nozzle m_nozzle;

		ModelClippingInteractorStyle m_mesh_clipping_interactor_style;
		int m_operating_partition_index;
		SO::PartitionCollection<CgalMesh_EPICK> m_partitions_in_partitioning;

		SO::Partition<CgalMesh_EPICK> *mp_on_clipping_partition;
		MeshClipperGUI m_mesh_clipper;

	};
}
#endif // SYNSLICERGUI_PATHPLANNINGDOCKWIDGET_H_