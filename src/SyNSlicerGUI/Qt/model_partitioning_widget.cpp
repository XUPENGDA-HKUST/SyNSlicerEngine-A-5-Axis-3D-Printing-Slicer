#include "model_partitioning_widget.h"

using SyNSlicerGUI::ModelPartitioningWidget;

ModelPartitioningWidget::ModelPartitioningWidget(QWidget *parent)
	: QWidget(parent)
	, m_layout(QBoxLayout::TopToBottom)
	, m_auto_partition_layout(QBoxLayout::TopToBottom)
	, m_infinite_plane_clipping_layout(QBoxLayout::TopToBottom)
	, m_finite_plane_clipping_layout(QBoxLayout::TopToBottom)
{
	m_title.setText("Model partition");
	m_title.setAlignment(Qt::AlignCenter);

	m_partition_mode_combo_box.addItem("(Select a parition method)");
	m_partition_mode_combo_box.addItem("Auto Partition");
	m_partition_mode_combo_box.addItem("Infinite Plane Clipping");
	//m_partition_mode_combo_box.addItem("Finite Plane Clipping");

	m_confirm_result_button.setText("Confirm");
	m_confirm_result_button.setEnabled(false);

	m_reset_button.setText("Reset");
	m_reset_button.setEnabled(false);

	m_exit_button.setText("Exit");

	this->setLayout(&m_layout);
	m_layout.addWidget(&m_title);
	m_layout.addWidget(&m_partition_mode_combo_box);
	m_layout.addWidget(&m_stacked_widget);
	m_layout.addWidget(&m_confirm_result_button);
	m_layout.addWidget(&m_reset_button);
	m_layout.addWidget(&m_exit_button);

	m_auto_partition_button.setText("Auto Partition");

	m_auto_partition_widget.setLayout(&m_auto_partition_layout);
	m_auto_partition_layout.addWidget(&m_auto_partition_button);

	m_infinite_plane_clipping_confirm_line_button.setText("Confirm Line");

	m_infinite_plane_clipping_widget.setLayout(&m_infinite_plane_clipping_layout);
	m_infinite_plane_clipping_layout.addWidget(&m_infinite_plane_clipping_confirm_line_button);

	m_finite_plane_clipping_confirm_line_button.setText("Confirm Line");
	m_confirm_paritition_selection.setText("Confirm Partition");

	m_finite_plane_clipping_widget.setLayout(&m_finite_plane_clipping_layout);
	m_finite_plane_clipping_layout.addWidget(&m_finite_plane_clipping_confirm_line_button);
	m_finite_plane_clipping_layout.addWidget(&m_confirm_paritition_selection);

	m_stacked_widget.addWidget(&m_empty_widget);
	m_stacked_widget.addWidget(&m_auto_partition_widget);
	m_stacked_widget.addWidget(&m_infinite_plane_clipping_widget);
	m_stacked_widget.addWidget(&m_finite_plane_clipping_widget);

	QObject::connect(
		&m_partition_mode_combo_box, SIGNAL(currentIndexChanged(int)),
		&m_stacked_widget, SLOT(setCurrentIndex(int)));
}

ModelPartitioningWidget::~ModelPartitioningWidget()
{

}
