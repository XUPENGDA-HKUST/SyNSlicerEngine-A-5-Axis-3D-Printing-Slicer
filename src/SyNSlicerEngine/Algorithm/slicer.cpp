#include "slicer.h"

using SyNSlicerEngine::Algorithm::Slicer;

Slicer::Slicer(SO::Partition<CgalMesh_EPICK> &p_partition)
    : mp_operating_partition(&p_partition)
    , m_mesh(mp_operating_partition->getEPICKMesh())
    , m_slicer(m_mesh)
{

}

Slicer::~Slicer()
{ 

}

void Slicer::slice()
{
    SO::PrintingLayerCollection &printing_layers = mp_operating_partition->getPrintingLayers();
    for (int i = 0; i < printing_layers.size(); i++)
    {
        SO::PolygonCollection contours = this->slice(printing_layers[i].getSlicingPlane());
        printing_layers[i] = SO::PrintingLayer(contours, printing_layers[i].getSlicingPlane(), printing_layers[i].getPrevSlicingPlane());
    }
}

SO::PolygonCollection Slicer::slice(SO::Plane plane)
{
    if (plane.isValid() == false)
    {
        return SO::PolygonCollection();
    }

    std::vector<CgalPolyline_EPICK> cgal_polylines;
    m_slicer(EPICK::Plane_3(
        plane.a(), plane.b(),
        plane.c(), plane.d()), std::back_inserter(cgal_polylines));

    SO::PolygonCollection contours;
    contours.setPlane(plane);

    if (!cgal_polylines.size())
    {
        return contours;
    }

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

    contours.setPlane(SO::Plane(contours.centroid(), contours.getPlane().getNormal()));

    return contours;
}