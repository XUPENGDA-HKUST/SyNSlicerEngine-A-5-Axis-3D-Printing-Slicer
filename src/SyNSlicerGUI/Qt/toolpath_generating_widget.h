#ifndef SYNSLICERGUI_TOOLPATHGENERATING_H_
#define SYNSLICERGUI_TOOLPATHGENERATING_H_

#include <memory>

#include "QT_CORE_CLASS"


namespace SyNSlicerGUI
{

	class ToolpathGeneratingWidget : public QWidget
	{

		Q_OBJECT

	public:
		ToolpathGeneratingWidget(QWidget *parent = nullptr);
		~ToolpathGeneratingWidget();

		QPushButton m_confirm_button;

	public Q_SLOTS:
		void confirmButtonClicked();

	Q_SIGNALS:
		void emitToolpathSetting(
			int number_of_contour_parallel_path, int path_type,  int infill_density,
			int number_of_contour_parallel_path_support, int path_type_support,  int infill_density_support);

		void emitToolpathSetting(std::tuple<int, int, int, int, int, int> toolpath_setting);

	protected:
		QGridLayout m_layout;
		QLabel m_title;

		QLabel m_model_toolpath_setting_label;
		QLabel m_wall_count_label;
		QSpinBox m_wall_count_spinbox;
		QLabel m_infill_pattern_label;
		QComboBox m_infill_pattern_combobox;
		QLabel m_infill_density_label;
		QSpinBox m_infill_density_spinbox;

		QLabel m_support_toolpath_setting_label;
		QLabel m_wall_count_support_label;
		QSpinBox m_wall_count_support_spinbox;
		QLabel m_infill_pattern_support_label;
		QComboBox m_infill_pattern_support_combobox;
		QLabel m_infill_density_support_label;
		QSpinBox m_infill_density_support_spinbox;



		QSpacerItem m_spacer_item_0;
		QSpacerItem m_spacer_item_1;
		QSpacerItem m_spacer_item_2;
	};
}

#endif  // SYNSLICERGUI_TOOLPATHGENERATING_H_