#include <iostream>
#include <cstdio>

#include <qapplication.h>

#include "sub_window.h"

#include "spdlog/sinks/basic_file_sink.h"

#include "Object_drawer.h"
#include "Object/partition.h"
#include "Algorithm/auto_partitioner.h"
#include "Algorithm/auto_slicer.h"
#include "Algorithm/slicer.h"
#include "Algorithm/support_generator.h"
#include "Algorithm/printing_sequence_determinator.h"
#include "Algorithm/toolpath_generator.h"
#include "Algorithm/gcode_generator.h"
#include "custom_slider.h"
#include "SyNSlicerGUI/Algorithm/auto_partitioner_gui.h"
#include "SyNSlicerGUI/Algorithm/auto_slicer_gui.h"
#include "SyNSlicerGUI/Algorithm/printing_sequence_determinator_gui.h"
#include "SyNSlicerGUI/Algorithm/slicer_gui.h"
#include "SyNSlicerGUI/Algorithm/support_generator_gui.h"
#include "SyNSlicerGUI/Algorithm/toolpath_generator_gui.h"
#include "SyNSlicerGUI/Algorithm/gcode_generator_gui.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SyNSlicerGUI::SubWindow main_window;
    main_window.move(300, 50);
    main_window.getRenderer()->GetActiveCamera()->ParallelProjectionOn();
    main_window.resetCamera();
    main_window.render();
    main_window.show();
    QApplication::processEvents();

    auto my_logger = spdlog::basic_logger_mt("basic_logger", "logs/basic.txt", true);
    spdlog::get("basic_logger")->info("Start program.");

    SyNSlicerGUI::ObjectDrawer drawer(main_window.getRenderer());


    if (false)
    {
        Eigen::Vector3d origin(0, 0, 0);
        Eigen::Vector3d pt0(0, 0, 0);
        Eigen::Vector3d pt1(2, 0, 0);
        Eigen::Vector3d pt2(2, 2, 0);
        Eigen::Vector3d pt3(0, 2, 0);

        SO::Polygon polygon;
        polygon.setPlane(SO::Plane());
        polygon.addPointToBack(pt0 + origin);
        polygon.addPointToBack(pt1 + origin);
        polygon.addPointToBack(pt2 + origin);
        polygon.addPointToBack(pt3 + origin);

        SO::PolygonCollection polygons;
        polygons.addPolygon(polygon);
        polygons.save("Polygons.txt");

        SO::PolygonCollection polygons_loaded("Polygons.txt");
        drawer.drawPolygons(polygons_loaded, "123");
        drawer.setColor("123", 1, 0, 0);
    }
    else
    {
        SO::PartitionCollection<CgalMesh_EPICK> result;
        if (false)
        {
            // Step 1: Load mesh.
        // SO::Partition<CgalMesh_EPICK> source_epick("../data/firebird_1mm_zero.stl");
            SO::Partition<CgalMesh_EPICK> source_epick("../data/bunny_Z_zero_1500.stl");
            // SO::Partition<CgalMesh_EPICK> source_epick("../data/L_shape.stl");
            // SO::Partition<CgalMesh_EPICK> source_epick("../data/triceratops.stl");

            source_epick.setBasePlane(SO::Plane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)));

            // Step 2: Parition the mesh.
            SO::Nozzle nozzle(0.4, 10, 10);
            SyNSlicerGUI::AutoPartitionerGUI auto_p(source_epick, nozzle, main_window.getRenderer());
            auto_p.partition();
            result = auto_p.getResult();

            for (int i = 0; i < result.numberOfPartitions(); i++)
            {
                SyNSlicerGUI::AutoSlicerGUI auto_s(result[i], main_window.getRenderer(), 0.3, 0.4);
                auto_s.slice();
            }

            // Step 3: Determine printing sequence.
            SyNSlicerGUI::PrintingSequenceDeterminatorGUI printing_sequence(result, nozzle, main_window.getRenderer());
            printing_sequence.determinePrintingSequence();
        }
        else
        {
            result = SO::PartitionCollection<CgalMesh_EPICK>(std::string("partition_list.txt"));
            for (int i = 0; i < result.numberOfPartitions(); i++)
            {
                SyNSlicerGUI::SlicerGUI slicer(result[i], main_window.getRenderer());
                slicer.slice();
            }
        }       

        // Step 4: Generate support.
        SyNSlicerGUI::SupportGeneratorGUI support_generator(result, main_window.getRenderer());
        support_generator.generateSupportStructure();

        // Step 5: Generate toolpath.
        for (int i = 0; i < result.numberOfPartitions(); i++)
        {
            SyNSlicerGUI::ToolpathGeneratorGUI generator(result[i], main_window.getRenderer(), true);
            generator.setPathPropertyForModel(2, 3, 3, 0, 30, 0.4);
            generator.setPathPropertyForSupport(2, 3, 3, 0, 30, 0.4);
            generator.generatePath();
        }

        // Step 6: Generate G-code.
        SyNSlicerGUI::GcodeGeneratorGUI gcode_generator(result, main_window.getRenderer());
        gcode_generator.generateToolpathForEachLayer();
        gcode_generator.generateCompletedToolpath();
        gcode_generator.writeGcode();
        gcode_generator.getCompletedToolpath();

        for (int i = 0; i < result.numberOfPartitions(); i++)
        {
            drawer.drawMesh(result[i].getEPICKMesh(), std::string("Mesh") + std::to_string(i));
            drawer.setColor(std::string("Mesh") + std::to_string(i),
                255.0 / result.numberOfPartitions() * i,
                255.0 / result.numberOfPartitions() * i,
                255.0 / result.numberOfPartitions() * i);
            drawer.setOpacity(std::string("Mesh") + std::to_string(i), 0.2);
        }

        /*
        for (int i = 0; i < result.numberOfPartitions(); i++)
        {
            SO::PrintingLayerCollection printing_layers = result[i].getPrintingLayers();
            printing_layers.update();
            drawer.drawPolylines(printing_layers.getContours(), "Contour" + std::to_string(i));
            drawer.setColor("Contour" + std::to_string(i), 1, 0, 0);
            drawer.drawPolylines(printing_layers.getSupportContours(), "Support" + std::to_string(i));
            drawer.setColor("Support" + std::to_string(i), 0, 1, 1);
        }
        */

        SyNSlicerGUI::LayerPreviewSlider *layer_preview = new SyNSlicerGUI::LayerPreviewSlider(result, main_window.getRenderer());
    }

    main_window.resetCamera();
    main_window.render();
    spdlog::info("Done");

    QApplication::processEvents();
    return a.exec();
}