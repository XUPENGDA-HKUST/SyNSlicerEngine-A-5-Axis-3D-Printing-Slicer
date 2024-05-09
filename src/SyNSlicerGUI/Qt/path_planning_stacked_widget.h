#ifndef SYNSLICERGUI_PATHPLANNINGSTACKEDWIDGET_H_
#define SYNSLICERGUI_PATHPLANNINGSTACKEDWIDGET_H_

#include "qstackedwidget.h"

#include "mesh_loading_widget.h"
#include "model_partitioning_widget.h"
#include "toolpath_generating_widget.h"
#include "preview_widget.h"

namespace SyNSlicerGUI {

	class PathPlanningStackedWidget :public QStackedWidget
	{

		Q_OBJECT

	public:
		PathPlanningStackedWidget(QWidget *parent = nullptr);
		~PathPlanningStackedWidget();

		MeshLoadingWidget m_mesh_loading_widget;
		ModelPartitioningWidget m_model_partition_widget;
		ToolpathGeneratingWidget m_toolpath_generating_widget;
		PreviewWidget m_preview_widget;

	protected Q_SLOTS:

	protected:

	};
}

#endif  // SYNSLICERGUI_PATHPLANNINGSTACKEDWIDGET_H_