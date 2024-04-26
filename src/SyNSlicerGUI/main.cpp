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
#include "auto_partitioner_debug.h"
#include "printing_sequence_determinator_debug.h"
#include "support_generator_debug.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    GUI::SubWindow main_window;
    main_window.move(300, 50);
    main_window.getRenderer()->GetActiveCamera()->ParallelProjectionOn();
    main_window.resetCamera();
    main_window.render();
    main_window.show();
    QApplication::processEvents();

    auto my_logger = spdlog::basic_logger_mt("basic_logger", "logs/basic.txt", true);
    spdlog::get("basic_logger")->info("Start program.");

    SyNSlicerEngine::GUI::ObjectDrawer drawer(main_window.getRenderer());


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

        SO::PolygonCollection polygons("support_contours.txt");
        SO::PolygonCollection pos;
        SO::PolygonCollection neg;
        SO::Plane clip_plane("plane_below.txt");
        polygons.clipWithPlane(clip_plane, pos, neg);

        drawer.drawPlane(clip_plane, "clip_plane");
        drawer.drawPolygons(pos, "pos");
        drawer.setColor("pos", 1, 0, 0);
        drawer.drawPolygons(neg, "neg");
        drawer.setColor("neg", 0, 0, 1);
    }
    else
    {
        /*
    // SO::Partition<CgalMesh_EPICK> source_epick("../data/firebird_1mm_zero.stl");
    SO::Partition<CgalMesh_EPICK> source_epick("../data/bunny_Z_zero_1500.stl");
    // SO::Partition<CgalMesh_EPICK> source_epick("../data/L_shape.stl");
    // SO::Partition<CgalMesh_EPICK> source_epick("../data/triceratops.stl");
    // SO::Partition<CgalMesh_EPICK> source_epick("../data/check/Part_2.stl");

    SO::Plane plane_1(
        Eigen::Vector3d(1.9189308881759644, 0.23120105266571045, 2.2115540504455566),
        Eigen::Vector3d(0.999742587463422, -0.005949421356898948, -0.02189436451372865));

    //source_epick.setBasePlane(plane_1);
    source_epick.setBasePlane(SO::Plane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)));

    SO::Nozzle nozzle(0.4, 10, 10);

    //SyNSlicerEngine::Algorithm::AutoPartitioner auto_p(source_epick, nozzle);
    SyNSlicerEngine::GUI::AutoPartitionerDebug auto_p(source_epick, nozzle, main_window.getRenderer());
    auto_p.partition();
    SO::PartitionCollection<CgalMesh_EPICK> result = auto_p.getResult();
 
    for (int i = 0; i < result.numberOfPartitions(); i++)
    {
        //if (i != 2)
        {
            //drawer.drawPolygons(result[i].getBaseContours(), "BaseContours" + std::to_string(i));
            //drawer.setColor("BaseContours" + std::to_string(i), 1, 0, 0);
            //drawer.drawPlane(result[i].getBasePlane(), "plane");
            drawer.drawMesh(result[i].getEPICKMesh(), std::string("Mesh") + std::to_string(i));
            drawer.setColor(std::string("Mesh") + std::to_string(i),
                255.0 / result.numberOfPartitions() * i,
                255.0 / result.numberOfPartitions() * i,
                255.0 / result.numberOfPartitions() * i);
            drawer.setOpacity(std::string("Mesh") + std::to_string(i), 0.2);

            SyNSlicerEngine::Algorithm::AutoSlicer auto_s(result[i], 0.3, 0.4);
            auto_s.slice();
            SO::PrintingLayerCollection printing_layers = result[i].getPrintingLayers();
            printing_layers.update();
            //drawer.drawPolylines(printing_layers.getContours(), "Contour" + std::to_string(i));
            //drawer.setColor("Contour" + std::to_string(i), 1, 0, 0);

        }
    }
    
    SyNSlicerEngine::Algorithm::PrintingSequenceDeterminator printing_sequence(result, nozzle);

    //SyNSlicerEngine::GUI::PrintingSequenceDeterminatorDebug printing_sequence(result, main_window.getRenderer());
    printing_sequence.determinePrintingSequence();

    result.save(std::string("partition_list.txt"));
    */
    //SyNSlicerEngine::Algorithm::SupportGenerator support_generator(result);

        SO::PartitionCollection<CgalMesh_EPICK> result(std::string("partition_list.txt"));
 
        for (int i = 0; i < result.numberOfPartitions(); i++)
        {
            SyNSlicerEngine::Algorithm::Slicer slicer(result[i]);
            slicer.slice();

            drawer.drawMesh(result[i].getEPICKMesh(), std::string("Mesh") + std::to_string(i));
            drawer.setColor(std::string("Mesh") + std::to_string(i),
                255.0 / result.numberOfPartitions() * i,
                255.0 / result.numberOfPartitions() * i,
                255.0 / result.numberOfPartitions() * i);
            drawer.setOpacity(std::string("Mesh") + std::to_string(i), 0.2);

            SO::PrintingLayerCollection printing_layers = result[i].getPrintingLayers();
            printing_layers.update();
            //drawer.drawPolylines(printing_layers.getContours(), "Contour" + std::to_string(i));
            //drawer.setColor("Contour" + std::to_string(i), 1, 0, 0);

            // drawer.drawPolylines(printing_layers.getSupportContours(), "Support" + std::to_string(i));
            // drawer.setColor("Support" + std::to_string(i), 0, 1, 1);
        }

        SyNSlicerEngine::Algorithm::SupportGenerator support_generator(result);
        //SyNSlicerEngine::GUI::SupportGeneratorDebug support_generator(result, main_window.getRenderer());
        support_generator.generateSupportStructure();
       
        /*
        for (int i = 0; i < result.numberOfPartitions(); i++)
        {
            //int i = 4;
            SO::PrintingLayerCollection printing_layers = result[i].getPrintingLayers();
            printing_layers.update();
            drawer.drawPolylines(printing_layers.getContours(), "Contour" + std::to_string(i));
            drawer.setColor("Contour" + std::to_string(i), 1, 0, 0);
            drawer.drawPolylines(printing_layers.getSupportContours(), "Support" + std::to_string(i));
            drawer.setColor("Support" + std::to_string(i), 0, 1, 1);
        }
        */
        
        for (int i = 0; i < result.numberOfPartitions(); i++)
        {
            SyNSlicerEngine::Algorithm::ToolpathGenerator generator(result[i], true);
            generator.setPathPropertyForModel(2, 3, 3, 0, 30, 0.4);
            generator.setPathPropertyForSupport(2, 3, 3, 0, 30, 0.4);
            generator.generatePath();
        }

        SyNSlicerEngine::Algorithm::GcodeGenerator gcode_generator(result);
        gcode_generator.generateToolpathForEachLayer();
        gcode_generator.generateCompletedToolpath();
        gcode_generator.writeGcode();

        SO::Toolpath m_completed_tool_path = gcode_generator.getCompletedToolpath();
        SO::Polyline polyline;
        for (size_t i = 0; i < m_completed_tool_path.size(); i++)
        {
            polyline.push_back(m_completed_tool_path[i].getPosition());
        }

        //drawer.drawPolyline(polyline, "ToolPath");
        //drawer.setColor("ToolPath", 1, 0, 0);
 
        SyNSlicerEngine::GUI::LayerPreviewSlider *layer_preview = new SyNSlicerEngine::GUI::LayerPreviewSlider(result, main_window.getRenderer());
        
     }

    main_window.resetCamera();
    main_window.render();
    spdlog::info("Done");

    QApplication::processEvents();
    return a.exec();
}