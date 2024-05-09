#include "toolpath_generating_widget.h"

using SyNSlicerGUI::ToolpathGeneratingWidget;

ToolpathGeneratingWidget::ToolpathGeneratingWidget(QWidget *parent)
	: QWidget(parent)
	, m_spacer_item_0(20, 20, QSizePolicy::Maximum, QSizePolicy::Maximum)
	, m_spacer_item_1(20, 20, QSizePolicy::Maximum, QSizePolicy::Maximum)
	, m_spacer_item_2(20, 20, QSizePolicy::Maximum, QSizePolicy::Maximum)
{
	setLayout(&m_layout);

	m_title.setText("ToolPath Generation");
	m_title.setAlignment(Qt::AlignCenter);
	m_model_toolpath_setting_label.setText("Model:");
	m_wall_count_label.setText("Wall count: ");
	m_wall_count_spinbox.setValue(1);
	m_wall_count_spinbox.setMinimum(1);
	m_infill_pattern_label.setText("Infill pattern: ");
	m_infill_pattern_combobox.addItem("Contour Parallel");
	m_infill_pattern_combobox.addItem("Zig Zag");
	m_infill_pattern_combobox.addItem("Grid");
	m_infill_density_label.setText("Infill Density: ");
	m_infill_density_spinbox.setValue(70);
	m_infill_density_spinbox.setRange(0, 100);

	m_support_toolpath_setting_label.setText("Support:");
	m_wall_count_support_label.setText("Wall count: ");
	m_wall_count_support_spinbox.setValue(1);
	m_wall_count_support_spinbox.setMinimum(1);
	m_infill_pattern_support_label.setText("Infill pattern: ");
	m_infill_pattern_support_combobox.addItem("Contour Parallel");
	m_infill_pattern_support_combobox.addItem("Zig Zag");
	m_infill_pattern_support_combobox.addItem("Grid");
	m_infill_density_support_label.setText("Infill Density: ");
	m_infill_density_support_spinbox.setValue(70);
	m_infill_density_support_spinbox.setRange(0, 100);

	m_confirm_button.setText("Generate Toolpath");

	m_layout.addWidget(&m_title, 0, 0, 1, 2);

	m_layout.addWidget(&m_model_toolpath_setting_label, 1, 0, 1, 2);
	m_layout.addWidget(&m_wall_count_label, 2, 0, 1, 1);
	m_layout.addWidget(&m_wall_count_spinbox, 2, 1, 1, 1);
	m_layout.addWidget(&m_infill_pattern_label, 3, 0, 1, 1);
	m_layout.addWidget(&m_infill_pattern_combobox, 3, 1, 1, 1);
	m_layout.addWidget(&m_infill_density_label, 4, 0, 1, 1);
	m_layout.addWidget(&m_infill_density_spinbox, 4, 1, 1, 1);

	m_layout.addWidget(&m_support_toolpath_setting_label, 5, 0, 1, 2);
	m_layout.addWidget(&m_wall_count_support_label, 6, 0, 1, 1);
	m_layout.addWidget(&m_wall_count_support_spinbox, 6, 1, 1, 1);
	m_layout.addWidget(&m_infill_pattern_support_label, 7, 0, 1, 1);
	m_layout.addWidget(&m_infill_pattern_support_combobox, 7, 1, 1, 1);
	m_layout.addWidget(&m_infill_density_support_label, 8, 0, 1, 1);
	m_layout.addWidget(&m_infill_density_support_spinbox, 8, 1, 1, 1);

	m_layout.addWidget(&m_confirm_button, 9, 1, 1, 1);

	QObject::connect(&m_confirm_button, SIGNAL(clicked()),
		this, SLOT(confirmButtonClicked()));
}

ToolpathGeneratingWidget::~ToolpathGeneratingWidget()
{

}

void ToolpathGeneratingWidget::confirmButtonClicked()
{
	/*
	emit emitToolpathSetting(
		m_wall_count_spinbox.value(),
		m_infill_pattern_combobox.currentIndex(), 
		m_infill_density_spinbox.value(),
		m_wall_count_support_spinbox.value(), 
		m_infill_pattern_support_combobox.currentIndex(), 
		m_infill_density_support_spinbox.value());
	*/

	std::tuple<int, int, int, int, int, int> toolpath_setting
		= std::make_tuple(m_wall_count_spinbox.value(),
			m_infill_pattern_combobox.currentIndex(),
			m_infill_density_spinbox.value(),
			m_wall_count_support_spinbox.value(),
			m_infill_pattern_support_combobox.currentIndex(),
			m_infill_density_support_spinbox.value());

	emit emitToolpathSetting(toolpath_setting);
}
