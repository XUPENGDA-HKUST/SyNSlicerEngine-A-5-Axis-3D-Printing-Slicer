#ifndef SYNSLICERGUI_PREVIEWWIDGET_H_
#define SYNSLICERGUI_PREVIEWWIDGET_H_

#include "QT_CORE_CLASS"

namespace SyNSlicerGUI
{
	class PreviewWidget : public QWidget
	{
	public:
		PreviewWidget(QWidget *parent = nullptr);
		~PreviewWidget();

		QPushButton m_prev_page;
		QPushButton m_generate_gcode_button;
		QSlider m_slider_for_selecting_printing_layer;
		QSpinBox m_partition_ID;
		QSpinBox m_layer_ID;

	protected:
		QGridLayout m_layout;
		QLabel m_title;

		QWidget m_spinboxs_widget;
		QGridLayout m_spinboxs_layout;
		QLabel m_partition_ID_label;
		QLabel m_layer_ID_label;
		QSpacerItem m_space_item_0;
	};
}

#endif  // SYNSLICERGUI_PREVIEWWIDGET_H_