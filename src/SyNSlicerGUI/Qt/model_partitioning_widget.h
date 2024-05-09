#ifndef SYNSLICERGUI_MODELPARTITIONING_H_
#define SYNSLICERGUI_MODELPARTITIONING_H_

#include <iostream>

#include "QT_CORE_CLASS"
#include "qstackedwidget.h"

namespace SyNSlicerGUI
{
	//! This class is used to control the model clipping process.
	class ModelPartitioningWidget : public QWidget
	{

		Q_OBJECT

	public:
		ModelPartitioningWidget(QWidget *parent = nullptr);
		~ModelPartitioningWidget();

		QComboBox m_partition_mode_combo_box;
		QPushButton m_auto_partition_button;
		QPushButton m_infinite_plane_clipping_confirm_line_button;
		QPushButton m_finite_plane_clipping_confirm_line_button;
		QPushButton m_confirm_paritition_selection;
		QPushButton m_confirm_result_button;
		QPushButton m_reset_button;
		QPushButton m_exit_button;

	protected:
		enum ClippingMode {
			FinitePlane = 0,
			InfinitePlane = 1,
			AutoPartition = 2
		};
		
		ClippingMode current_clipping_mode = FinitePlane;

	protected Q_SLOTS:

	protected:

		QBoxLayout m_layout;
		QLabel m_title;

		QStackedWidget m_stacked_widget;

		QWidget m_empty_widget;

		QWidget m_auto_partition_widget;
		QBoxLayout m_auto_partition_layout;

		QWidget m_infinite_plane_clipping_widget;
		QBoxLayout m_infinite_plane_clipping_layout;

		QWidget m_finite_plane_clipping_widget;
		QBoxLayout m_finite_plane_clipping_layout;
	};
}

#endif  // SYNSLICERGUI_MODELPARTITIONING_H_