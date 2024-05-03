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

            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPaths().getSurface(), "S" + std::to_string(current_value));
            m_drawer.setColor("S" + std::to_string(current_value), 1, 0, 0);

            for (size_t i = 0; i < m_printing_layers[current_value].getPrintingPaths().getWall().size(); i++)
            {
                m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPaths().getWall()[i], "W" + std::to_string(i) + std::to_string(current_value));
                m_drawer.setColor("W" + std::to_string(i) + std::to_string(current_value), 0, 1, 0);
            }

            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPaths().getBottomTopUnion(), "BT" + std::to_string(current_value));
            m_drawer.setColor("BT" + std::to_string(current_value), 1, 1, 0);

            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPaths().getInfill(), "I" + std::to_string(current_value));
            m_drawer.setColor("I" + std::to_string(current_value), (double)255 / 255, (double)165 / 255, 0);

            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPathsForSupport().getSurface(), "SS" + std::to_string(current_value));
            m_drawer.setColor("SS" + std::to_string(current_value), 1, 0, 0);

            for (size_t i = 0; i < m_printing_layers[current_value].getPrintingPathsForSupport().getWall().size(); i++)
            {
                m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPathsForSupport().getWall()[i], "WW" + std::to_string(i) + std::to_string(current_value));
                m_drawer.setColor("WW" + std::to_string(i) + std::to_string(current_value), 0, 1, 0);
            }

            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPathsForSupport().getBottomTopUnion(), "BTBT" + std::to_string(current_value));
            m_drawer.setColor("BTBT" + std::to_string(current_value), 1, 1, 0);

            m_drawer.drawPolygons(m_printing_layers[current_value].getPrintingPathsForSupport().getInfill(), "II" + std::to_string(current_value));
            m_drawer.setColor("II" + std::to_string(current_value), (double)255 / 255, (double)165 / 255, 0);

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
