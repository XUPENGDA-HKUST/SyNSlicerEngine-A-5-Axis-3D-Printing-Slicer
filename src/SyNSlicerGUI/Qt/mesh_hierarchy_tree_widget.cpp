#include "mesh_hierarchy_tree_widget.h"

using SyNSlicerGUI::MeshHierarchyTreeWidget;

MeshHierarchyTreeWidget::MeshHierarchyTreeWidget(QWidget *parent)
	: QTreeWidget(parent)
{
	this->setMinimumWidth(370);
	this->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	this->setColumnWidth(0,150);
	this->setColumnCount(2); //very important, otherwise, the treewidget only shows one column
	this->setHeaderHidden(true);
	QObject::connect(
		this, SIGNAL(itemChanged(QTreeWidgetItem *, int)), 
		this, SLOT(itemCheckStatusChanged(QTreeWidgetItem *, int)));
}

MeshHierarchyTreeWidget::~MeshHierarchyTreeWidget()
{

}

void MeshHierarchyTreeWidget::addTopLevelItem(std::string name)
{
	QTreeWidgetItem *item = new QTreeWidgetItem();
	item->setText(0, QString::fromStdString(name));
	QTreeWidget::addTopLevelItem(item);
}

void MeshHierarchyTreeWidget::addChildItems(int top_item_index, int number_of_items_added)
{
	assert(top_item_index < QTreeWidget::topLevelItemCount());
	m_buttons.clear();
	QTreeWidgetItem *top_item = QTreeWidget::topLevelItem(top_item_index);
	for (int i = 0; i < number_of_items_added; i++)
	{
		QTreeWidgetItem *item = new QTreeWidgetItem();
		item->setText(0, QString::fromStdString("Partition" + std::to_string(i)));
		item->setCheckState(0, Qt::Checked);
		top_item->addChild(item);
		QPushButtonWithIndex *partition_button = new QPushButtonWithIndex(i);
		partition_button->setText("Partition");
		QTreeWidget::setItemWidget(item, 1, partition_button);
		m_buttons.push_back(partition_button);
		QObject::connect(partition_button, SIGNAL(clipPartition(int)),
			this, SLOT(partitionButtonClicked(int)));
	}
	QTreeWidget::expandAll();
}

void MeshHierarchyTreeWidget::setButtonsEnabled(bool enabled)
{
	for (size_t i = 0; i < m_buttons.size(); i++)
	{
		m_buttons[i]->setEnabled(enabled);
	}
}

void MeshHierarchyTreeWidget::itemCheckStatusChanged(QTreeWidgetItem *p_widget, int colume)
{
	QTreeWidgetItem *top_item = QTreeWidget::topLevelItem(0);
	for (int i = 0; i < top_item->childCount(); i++)
	{
		if (top_item->child(i) == p_widget)
		{
			if (p_widget->checkState(0) == Qt::Checked)
			{
				emit changePartitionVisibility(i, true);
			}
			else
			{
				emit changePartitionVisibility(i, false);
			}
		}
	}
}

void MeshHierarchyTreeWidget::reset()
{
	for (size_t i = 0; i < this->topLevelItemCount(); i++)
	{
		this->takeTopLevelItem(i);
	}
}

void MeshHierarchyTreeWidget::partitionButtonClicked(int index)
{
	emit clipPartition(index);
}
