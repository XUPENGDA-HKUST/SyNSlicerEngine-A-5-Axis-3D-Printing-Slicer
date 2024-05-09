#include "preview_widget.h"

using SyNSlicerGUI::PreviewWidget;

PreviewWidget::PreviewWidget(QWidget *parent)
	: m_slider_for_selecting_printing_layer(Qt::Vertical)
	, m_space_item_0(20, 20, QSizePolicy::Maximum, QSizePolicy::Maximum)
{
	setLayout(&m_layout);

	m_title.setText("ToolPath_Preview");
	m_title.setAlignment(Qt::AlignCenter);
	m_partition_ID_label.setText("Partition: ");
	m_partition_ID.setDisabled(true);
	m_layer_ID_label.setText("Layer: ");
	m_layer_ID.setRange(0, 10000);
	m_layer_ID.setDisabled(true);
	m_prev_page.setText("Previous Page");
	m_generate_gcode_button.setText("Generate Gcode");

	m_spinboxs_widget.setLayout(&m_spinboxs_layout);
	m_spinboxs_layout.addWidget(&m_partition_ID_label, 0, 0, 1, 1);
	m_spinboxs_layout.addWidget(&m_partition_ID, 0, 1, 1, 1);
	m_spinboxs_layout.addWidget(&m_layer_ID_label, 1, 0, 1, 1);
	m_spinboxs_layout.addWidget(&m_layer_ID, 1, 1, 1, 1);

	m_layout.addWidget(&m_title, 0, 0, 1, 5);
	m_layout.addWidget(&m_slider_for_selecting_printing_layer, 1, 1, 2, 1);
	m_layout.addWidget(&m_spinboxs_widget, 1, 3, 2, 2);
	m_layout.addItem(&m_space_item_0, 3, 0, 1, 5);
	m_layout.addWidget(&m_prev_page, 4, 0, 1, 5);
	m_layout.addWidget(&m_generate_gcode_button, 5, 0, 1, 5);
}

PreviewWidget::~PreviewWidget()
{

}