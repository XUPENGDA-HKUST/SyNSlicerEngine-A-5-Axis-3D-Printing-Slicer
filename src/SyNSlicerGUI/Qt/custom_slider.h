#pragma once

#include <qapplication.h>
#include <qobject.h>
#include <qslider.h>

#include "Object/partition_collection.h"
#include "Object/printing_layer_collection.h"
#include "object_drawer.h"

namespace SyNSlicerGUI {

    class LayerPreviewSlider : public QSlider
    {
        Q_OBJECT

    public:
        LayerPreviewSlider(SO::PartitionCollection<CgalMesh_EPICK> &partitions, vtkRenderer *renderer)
            : m_partition_list(partitions)
            , m_drawer(renderer)
        {
            for (size_t i = 0; i < m_partition_list.numberOfPartitions(); i++)
            {
                for (size_t j = 0; j < m_partition_list[i].getPrintingLayers().size(); j++)
                {
                    m_printing_layers.addPrintingLayer(m_partition_list[i].getPrintingLayers()[j]);
                }
            }

            QObject::connect(this, SIGNAL(valueChanged(int)),
                this, SLOT(changePreviewLayer(int)));

            this->setRange(0, m_printing_layers.getNumberOfLayers() - 1);
            this->setValue(0);

            this->setFixedSize(400, 400);

            this->show();
        };

        ~LayerPreviewSlider() {};

    private slots:

        void changePreviewLayer(int value)
        {
            current_value = value;
            if (m_drawer.numberOfObjectsDrawn() > 1)
            {
                m_drawer.removeAllObjectsDrawn();
            }
            std::string name = "S" + std::to_string(current_value);
            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPaths().getSurface(), name);
            m_drawer.setColor(name, 1, 0, 0);

            for (size_t i = 0; i < m_printing_layers[current_value].getPrintingPaths().getWall().size(); i++)
            {
                name = "W" + std::to_string(i) + std::to_string(current_value);
                m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPaths().getWall()[i], name);
                m_drawer.setColor(name, 0, 1, 0);
            }

            name = "BT" + std::to_string(current_value);
            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPaths().getBottomTopUnion(), name);
            m_drawer.setColor(name, 1, 1, 0);

            name = "I" + std::to_string(current_value);
            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPaths().getInfill(), name);
            m_drawer.setColor(name, (double)255 / 255, (double)165 / 255, 0);

            name = "SS" + std::to_string(current_value);
            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPathsForSupport().getSurface(), name);
            m_drawer.setColor(name, 1, 0, 0);

            for (size_t i = 0; i < m_printing_layers[current_value].getPrintingPathsForSupport().getWall().size(); i++)
            {
                name = "WW" + std::to_string(i) + std::to_string(current_value);
                m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPathsForSupport().getWall()[i], name);
                m_drawer.setColor(name, 0, 1, 0);
            }

            name = "BTBT" + std::to_string(current_value);
            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPathsForSupport().getBottomTopUnion(), name);
            m_drawer.setColor(name, 1, 1, 0);

            name = "II" + std::to_string(current_value);
            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPathsForSupport().getInfill(), name);
            m_drawer.setColor(name, (double)255 / 255, (double)165 / 255, 0);

            last_value = value;
        }

    private:
        QSlider slider;
        SyNSlicerGUI::ObjectDrawer m_drawer;
        SO::PartitionCollection<CgalMesh_EPICK> &m_partition_list;
        SO::PrintingLayerCollection m_printing_layers;

        int last_value = 0;
        int current_value = 0;

    };
}
