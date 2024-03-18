#include <iostream>
#include <cstdio>

#include <qapplication.h>

#include "sub_window.h"

#include "spdlog/sinks/basic_file_sink.h"

#include "Object_drawer.h"
#include "Object/partition.h"
#include "auto_partitioner.h"
#include "auto_slicer.h"

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

    SO::Partition<CgalMesh_EPICK> source_epick("../data/firebird_1mm_zero.stl");
    // SO::Partition<CgalMesh_EPICK> source_epick("../data/bunny_Z_zero_1500.stl");
    //SO::Partition<CgalMesh_EPICK> source_epick("../data/Check/Low_4.stl");
    
    // SO::Partition source("../data/triceratops.stl");
    source_epick.setBasePlane(SO::Plane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)));
    /*
    CgalMesh_EPICK mesh = source_epick.getEPICKMesh();
    CGAL::Polygon_mesh_slicer<CgalMesh_EPICK, EPICK> m_slicer(mesh);
    SO::Plane plane = source_epick.getBasePlane();
    plane.offset(0.3);

    std::vector<CgalPolyline_EPICK> cgal_polylines;
    m_slicer(EPICK::Plane_3(
        plane.a(), plane.b(),
        plane.c(), plane.d()), std::back_inserter(cgal_polylines));

    SO::PolygonCollection contours;
    contours.setPlane(plane);

    for (auto &cgal_polygon : cgal_polylines)
    {
        SO::Polygon contour;
        contour.setPlane(plane);
        for (size_t i = 0; i < cgal_polygon.size(); i++)
        {
            contour.addPointToBack(Eigen::Vector3d(cgal_polygon[i].x(), cgal_polygon[i].y(), cgal_polygon[i].z()));
        }
        contours.addPolygon(contour);
    }

    drawer.drawMesh(mesh, "Check1");
    drawer.drawPolygons(contours, "Check1");
    drawer.setColor("Check1", 1, 0, 0);
    */

    //SyNSlicerEngine::Algorithm::AutoSlicer auto_s(source_epick, 0.3, 0.4, main_window.getRenderer());


    SyNSlicerEngine::Algorithm::AutoPartitioner auto_p(source_epick, main_window.getRenderer());
    auto_p.partition();
    SO::PartitionCollection<CgalMesh_EPICK> result = auto_p.getResult();

    for (int i = 0; i < result.numberOfPartitions(); i++)
    {
        drawer.drawMesh(result[i].getEPICKMesh(), std::string("Mesh") + std::to_string(i));
        drawer.setColor(std::string("Mesh") + std::to_string(i), 
            255.0 / result.numberOfPartitions() * i, 
            255.0 / result.numberOfPartitions() * i, 
            255.0 / result.numberOfPartitions() * i);
        drawer.setOpacity(std::string("Mesh") + std::to_string(i), 0.2);
        SyNSlicerEngine::Algorithm::AutoSlicer auto_s(result[i], 0.3, 0.4, main_window.getRenderer());
    }

    main_window.resetCamera();
    main_window.render();

    QApplication::processEvents();
    return a.exec();
}