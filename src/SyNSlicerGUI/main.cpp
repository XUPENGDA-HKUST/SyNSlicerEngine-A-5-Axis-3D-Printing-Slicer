#include <iostream>

#include <qapplication.h>

#include "sub_window.h"

#include "Object_drawer.h"
#include "Object/partition.h"

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

    GUI::ObjectDrawer drawer(main_window.getRenderer());

    SO::Partition<CgalMesh_EPICK> source_epick("../data/firebird_1mm_zero.stl");
    SO::Partition<CgalMesh_EPECK> source_epeck("../data/firebird_1mm_zero.stl");
    // SO::Partition source("../data/triceratops.stl");
    source_epick.setBasePlane(SO::Plane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)));
    source_epeck.setBasePlane(SO::Plane(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 1)));

    main_window.resetCamera();
    main_window.render();

    QApplication::processEvents();
    return a.exec();
}