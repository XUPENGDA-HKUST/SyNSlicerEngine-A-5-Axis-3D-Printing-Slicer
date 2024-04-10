#include <iostream>
#include <cstdio>

#include <qapplication.h>

#include "sub_window.h"

#include "spdlog/sinks/basic_file_sink.h"

#include "Object_drawer.h"
#include "Object/partition.h"
#include "auto_partitioner.h"
#include "auto_slicer.h"
#include "support_generator.h"
#include "printing_sequence_determinator.h"
#include "toolpath_generator.h"

#define getName(var) #var

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

    GUI::ObjectDrawer drawer(main_window.getRenderer());
    /*
    Eigen::Vector3d origin(10, 10, 10);
    Eigen::Vector3d pt0(0, 0, 0);
    Eigen::Vector3d pt1(2, 0, 2);
    Eigen::Vector3d pt2(2, 2, 2);
    Eigen::Vector3d pt3(0, 2, 0);

    SO::Polygon polygon;
    polygon.setPlane(SO::Plane(origin, pt1.cross(pt3)));
    polygon.addPointToBack(pt0 + origin);
    polygon.addPointToBack(pt1 + origin);
    polygon.addPointToBack(pt2 + origin);
    polygon.addPointToBack(pt3 + origin);
    drawer.drawPolygon(polygon, "Check1");
    drawer.setColor("Check1", 0, 1, 0);
    
    polygon = polygon.getTransformedPolygon(SO::Plane(origin, Eigen::Vector3d(0, 0, 1)));
    drawer.drawPolygon(polygon, "Check2");
    drawer.setColor("Check2", 1, 0, 0);
    */
    // SO::Partition<CgalMesh_EPICK> source_epick("../data/firebird_1mm_zero.stl");
    // SO::Partition<CgalMesh_EPICK> source_epick("../data/bunny_Z_zero_1500.stl");
    SO::Partition<CgalMesh_EPICK> source_epick("../data/L_shape.stl");
    // SO::Partition<CgalMesh_EPICK> source_epick("../data/Check/Part_0.stl");
    
    // SO::Partition source("../data/triceratops.stl");
    SO::Plane plane_0(
        Eigen::Vector3d(-9.047619819641113, -7.621689796447754, 7.983455181121826),
        Eigen::Vector3d(0.1388610896344402, -0.9749322997521669, 0.17385226108822147));

    SO::Plane plane_6(
        Eigen::Vector3d(-12.08036994934082, 0.8999999761581421, 8.406791687011719),
        Eigen::Vector3d(-0.8110223783607358, -3.8766091184389854e-05, 0.5850151282618988));
     
    SO::Plane plane_up_6(
        Eigen::Vector3d(-11.542960166931152, 9.788363968254998e-05, 22.389612197875977),
        Eigen::Vector3d(0.17851260290771584, - 0.002996044446590417, 0.9839330639432676));   

    //source_epick.setBasePlane(plane_up_6);
    source_epick.setBasePlane(SO::Plane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)));

    drawer.drawMesh(source_epick.getEPICKMesh(), "123");
    drawer.setOpacity("123", 0.2);

    SyNSlicerEngine::Algorithm::AutoSlicer auto_s(source_epick, 0.3, 0.4, main_window.getRenderer());
    SO::PrintingLayerCollection printing_layers = source_epick.getPrintingLayers();
    printing_layers.update();
    drawer.drawPolylines(printing_layers.getContours(), "Contour");
    drawer.setColor("Contour", 1, 0, 0);
    SO::PartitionCollection<CgalMesh_EPICK> result;
    result.addPartition(source_epick);

    /*
    SO::Nozzle nozzle(0.4, 10, 10);

    SyNSlicerEngine::Algorithm::AutoPartitioner auto_p(source_epick, nozzle, main_window.getRenderer());
    auto_p.partition();
    SO::PartitionCollection<CgalMesh_EPICK> result = auto_p.getResult();

    for (int i = 0; i < result.numberOfPartitions(); i++)
    {
        //if (i == 3)
        {
            drawer.drawMesh(result[i].getEPICKMesh(), std::string("Mesh") + std::to_string(i));
            drawer.setColor(std::string("Mesh") + std::to_string(i),
                255.0 / result.numberOfPartitions() * i,
                255.0 / result.numberOfPartitions() * i,
                255.0 / result.numberOfPartitions() * i);
            drawer.setOpacity(std::string("Mesh") + std::to_string(i), 0.2);
            SyNSlicerEngine::Algorithm::AutoSlicer auto_s(result[i], 0.3, 0.4, main_window.getRenderer());
            SO::PrintingLayerCollection printing_layers = result[i].getPrintingLayers();
            printing_layers.update();
            //drawer.drawPolylines(printing_layers.getContours(), "Contour" + std::to_string(i));
            //drawer.setColor("Contour" + std::to_string(i), 1, 0, 0);
        }
    }
    */
    SyNSlicerEngine::Algorithm::SupportGenerator support_generator(result, main_window.getRenderer());

    for (int i = 0; i < result.numberOfPartitions(); i++)
    {
        SO::PrintingLayerCollection printing_layers = result[i].getPrintingLayers();
        printing_layers.update();

        //drawer.drawPolylines(printing_layers.getSupportContours(), "Support" + std::to_string(i));
        //drawer.setColor("Support" + std::to_string(i), 0, 1, 1);
    }

    for (int i = 0; i < result.numberOfPartitions(); i++)
    {
        SyNSlicerEngine::Algorithm::ToolpathGenerator generator(result[i], false, main_window.getRenderer());
        generator.setPathPropertyForModel(2, 3, 3, 1, 100, 0.4);
        generator.generatePath();
    }

    for (int i = 0; i < result.numberOfPartitions(); i++)
    {
        for (int j = 0; j < result[i].getPrintingLayers().size(); j++)
        {
            /*
            drawer.drawPolygons(result[i].getPrintingLayers()[j].getPrintingPaths().getSurface(), "S" + std::to_string(i) + std::to_string(j));
            drawer.setColor("S" + std::to_string(i) + std::to_string(j), 1, 0, 0);

            for (int k = 0; k < result[i].getPrintingLayers()[j].getPrintingPaths().getWall().size(); k++)
            {
                drawer.drawPolygons(result[i].getPrintingLayers()[j].getPrintingPaths().getWall()[k],
                    "W" + std::to_string(i) + std::to_string(j) + std::to_string(k));
                drawer.setColor("W" + std::to_string(i) + std::to_string(j) + std::to_string(k), 0, 0, 1);
            }

            drawer.drawPolygons(result[i].getPrintingLayers()[j].getPrintingPaths().getBottomTopUnion(),
                "I" + std::to_string(i) + std::to_string(j));
            drawer.setColor("I" + std::to_string(i) + std::to_string(j), 0, 1, 0);
            */
        }
    }

    main_window.resetCamera();
    main_window.render();
    spdlog::info("Done");

    QApplication::processEvents();
    return a.exec();
}