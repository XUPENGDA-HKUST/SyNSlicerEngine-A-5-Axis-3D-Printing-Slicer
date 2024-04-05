#include "auto_slicer.h"

using SyNSlicerEngine::Algorithm::AutoSlicer;

AutoSlicer::AutoSlicer(SO::Partition<CgalMesh_EPICK> &p_partition, double target_layer_thickness, double side_step, vtkRenderer *p_renderer)
    : mp_operating_partition(&p_partition)
    , m_mesh(mp_operating_partition->getEPICKMesh())
    , m_slicer(m_mesh)
    , m_tree(faces(m_mesh).first, faces(m_mesh).second, m_mesh)
    , m_layer_thickness(target_layer_thickness)
    , m_side_step(side_step)
    , m_drawer(p_renderer)
{
    m_max_layer_thickness = 0.35;
    m_min_layer_thickness = 0.25;

    SO::Plane base_plane = mp_operating_partition->getBasePlane();
    SO::PolygonCollection current_contours = mp_operating_partition->getBaseContours();

    Eigen::Vector3d new_origin = current_contours.centroid();
    new_origin = base_plane.getProjectionOfPointOntoPlane(new_origin);
    current_contours.setPlane(SO::Plane(new_origin, base_plane.getNormal()));

    m_temp_slicing_result.emplace_back(current_contours);
    
    SO::PolygonCollection next_contours;
    while (this->determineNextSlicingPlane(current_contours, next_contours) == true)
    {
        current_contours = next_contours;
    }

    for (auto &slicing_result : m_temp_slicing_result)
    {
        m_slicing_result.emplace_back(slicing_result);
    }

    SO::PrintingLayerCollection printing_layers;
    for (size_t i = 1; i < m_slicing_result.size(); i++)
    {   
        if (!this->isSlicingPlaneValid(m_slicing_result[i].getPlane(), m_slicing_result[i - 1].getPlane()))
        {
            std::cout << "Layer " << i << " not_valid" << std::endl;
        }
        SO::PrintingLayer printing_layer(m_slicing_result[i], m_slicing_result[i].getPlane(), m_slicing_result[i - 1].getPlane());
        printing_layers.addPrintingLayer(printing_layer);
    }

    p_partition.setPrintingLayers(printing_layers);
}

AutoSlicer::~AutoSlicer()
{ 

}

static int debug = 0;
static int aaa = 0; 
static int bbb = 0;

bool AutoSlicer::determineNextSlicingPlane(SO::PolygonCollection &current_contours, SO::PolygonCollection &next_contours)
{
    next_contours.reset();

    SO::PolygonCollection contours_below = current_contours;
    SO::Plane plane_up = current_contours.getPlane();
    plane_up.offset(0.3);
    SO::PolygonCollection contours_up = this->slice(plane_up);
    if (!contours_up.get().size())
    {
        return false;
    }

    SO::PolygonCollection support_contours;

    this->checkSupportNeeded(contours_below, contours_up, support_contours);

    if (support_contours.get().size())
    {
        SO::Polygon largest_polygon = support_contours.getLargestPolygon();
        Eigen::Vector3d target_point = largest_polygon.centroid();

        SO::Polygon cloest_polygon = contours_below.getPolygonCloestToPolygon(largest_polygon);
        Eigen::Vector3d source_point = cloest_polygon.centroid();

        SO::Plane plane_0(source_point, target_point - source_point);
        SO::Plane plane_1(source_point, plane_0.getNormal().cross(plane_up.getNormal()));

        std::vector<Eigen::Vector3d> result_below = contours_below.getIntersectionWithPlane(plane_1);
        std::vector<Eigen::Vector3d> result_up = contours_up.getIntersectionWithPlane(plane_1);

        bool p0_found = false;
        Eigen::Vector3d p0;
        p0_found = plane_0.getMostPositivePoint(result_below, p0);

        bool p1_found = false;
        Eigen::Vector3d p1;
        p1_found = plane_0.getMostPositivePoint(result_up, p1);

        if (p0_found == false || p1_found == false)
        {
            goto block_1;
        }

        SO::Plane original_plane = plane_up;

        plane_up.setNormal(p1 - p0);
        plane_up.setOrigin(p1);

        // check if contour m_temp_slicing_result.back() has point on positive side.

        for (auto &poly: m_temp_slicing_result.back().get())
        {
            for (auto &pt : poly.get())
            {
                if (plane_up.getPositionOfPointWrtPlane(pt) == 1)
                {
                    plane_up = original_plane;
                    goto block_1;
                }
            }
        }

        if (aaa == -50)
        {
            m_drawer.drawPlane(plane_up, "Plane_up");
            m_drawer.drawPlane(m_temp_slicing_result.front().getPlane(), "Plane_1");
        }

        SO::Line intersecting_line = m_temp_slicing_result.front().getPlane().getIntersectionWithPlane(plane_up);
        double angle_of_rotation = m_temp_slicing_result.front().getPlane().getAngleOfRotation(plane_up);
        Eigen::Vector3d axis_of_rotation = m_temp_slicing_result.front().getPlane().getAxisOfRotation(plane_up);

        if (isnan(axis_of_rotation[0]) || isnan(axis_of_rotation[1]) || isnan(axis_of_rotation[2]))
        {
            goto block_1;
        }

        if (!this->getIntermediatePlanes(plane_up, m_temp_slicing_result.front().getPlane()))
        {
            goto block_1;
        };

        for (int i = 0; i < m_temp_slicing_result.size(); i++)
        {
            m_slicing_result.emplace_back(m_temp_slicing_result[i]); 
        }

        m_temp_slicing_result.clear();
        m_temp_slicing_result.emplace_back(this->slice(plane_up));

        if (aaa == 2)
        { 
            //return false;
        }
        std::cout << aaa << std::endl;
        aaa += 1;
    }
    else
    {
    block_1:
        m_temp_slicing_result.emplace_back(this->slice(plane_up));
    }

    next_contours = m_temp_slicing_result.back();
    return true;
}

bool AutoSlicer::checkSupportNeeded(SO::PolygonCollection &contours_below, SO::PolygonCollection &contours_up, SO::PolygonCollection &support_contours)
{
    bool support_needed = false;
    double side_step = 0.3;

    Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;
    Eigen::Transform<double, 3, Eigen::Affine> inverse_transformation_matrix;
    Eigen::Vector3d origin(contours_below.getPlane().getOrigin());

    transformation_matrix = contours_below.getPlane().getTransformationMatrix(SO::Plane());
    inverse_transformation_matrix = transformation_matrix.inverse();

    Clipper2Lib::PathsD subject, clip, solution;

    SO::PolygonCollection projected_contours;
    projected_contours = contours_up.projectToOtherPlane(contours_below.getPlane());
    projected_contours = projected_contours.getTransformedPolygons(SO::Plane(origin, Eigen::Vector3d(0, 0, 1)));

    for (auto &contour : projected_contours.get())
    {
        Clipper2Lib::PathD temp_path;
        for (auto &point : contour.get())
        {
            temp_path.push_back(Clipper2Lib::PointD(point[0], point[1]));
        }
        subject.push_back(temp_path);
    }

    SO::PolygonCollection contours_from_layer_i_minus_1 = contours_below;
    contours_from_layer_i_minus_1 = contours_from_layer_i_minus_1.getTransformedPolygons(SO::Plane(origin, Eigen::Vector3d(0, 0, 1)));

    for (auto &contour : contours_from_layer_i_minus_1.get())
    {
        Clipper2Lib::PathD temp_path;
        for (auto &point : contour.get())
        {
            temp_path.push_back(Clipper2Lib::PointD(point[0], point[1]));
        }
        clip.push_back(temp_path);
    }

    clip = Clipper2Lib::InflatePaths(clip, 0.25 * side_step, Clipper2Lib::JoinType::Miter, Clipper2Lib::EndType::Polygon);
    clip = Clipper2Lib::SimplifyPaths(clip, 0.1);
  
    solution = Difference(subject, clip, Clipper2Lib::FillRule::NonZero);
    solution = Clipper2Lib::SimplifyPaths(solution, 0.01);

    if (solution.size() > 0)
    {
        support_contours.reset();
        support_needed = true;
        for (int i = 0; i < solution.size(); i++)
        {
            SO::Polygon temp_contour;
            temp_contour.setPlane(contours_below.getPlane());
            for (int j = 0; j < solution[i].size(); j++)
            {
                Eigen::Vector3d point(solution[i][j].x, solution[i][j].y, contours_below.getPlane().getOrigin()[2]);
                point = inverse_transformation_matrix * (point - origin) + origin;
                temp_contour.addPointToBack(point);
            }
            support_contours.addPolygon(temp_contour);
        }
    }
    return support_needed;
}

bool AutoSlicer::getIntermediatePlanes(SO::Plane &plane_up, SO::Plane plane_below)
{
    int number_of_gaps = 0;
    std::vector<SO::Plane> slicing_planes;

    if (!this->tunePlaneUpUntilMimimumSideValid(plane_up, plane_below, slicing_planes))
    {
        return false;
    }

    SO::Line intersecting_line;
    if (!plane_below.isIntersectedWithPlane(plane_up, intersecting_line))
    {
        return false;
    }

    m_temp_slicing_result.clear();
    m_temp_slicing_result.emplace_back(this->slice(plane_below));
    int j = 0;
    for (int i = 1; i < slicing_planes.size(); i++)
    {
        if (aaa == 9 && i == slicing_planes.size() - 1)
        {
            debug = 1;
        }
        if (tuneConsecutivePlanesValid(slicing_planes[i], slicing_planes[j]))
        {
            m_temp_slicing_result.emplace_back(this->slice(slicing_planes[i]));
        }
        else
        {
            plane_up = m_temp_slicing_result.back().getPlane();
            m_temp_slicing_result.pop_back();
            return true;
        }
        j = i;
    }
    m_temp_slicing_result.pop_back();
    plane_up = slicing_planes.back();

    return true;
}

bool AutoSlicer::tunePlaneUpUntilMimimumSideValid(SO::Plane &plane_up, SO::Plane plane_below, std::vector<SO::Plane> &slicing_planes)
{
    SO::Line intersecting_line;
    if (!plane_below.isIntersectedWithPlane(plane_up, intersecting_line))
    {
        return false;
    }

    SO::PolygonCollection contours = this->slice(plane_up);
    contours.removePolygonsBelowPlane(plane_below);

    Eigen::Vector3d closest_point;
    contours.getClosestPointFromLine(intersecting_line, closest_point);
    double a = plane_below.getDistanceFromPointToPlane(closest_point);
    Eigen::Vector3d point_at_plane_below = plane_below.getProjectionOfPointOntoPlane(closest_point);

    slicing_planes.clear();
    slicing_planes.emplace_back(plane_below);
    double distance = 0.0;

    if (a < distance)
    {
        std::cout << aaa << " a < distance" << std::endl;
    }

    while (distance < a)
    {
        distance += m_min_layer_thickness;
        point_at_plane_below = point_at_plane_below + m_min_layer_thickness * plane_below.getNormal();
        Eigen::Vector3d new_normal = (point_at_plane_below - intersecting_line.getSource()).cross(intersecting_line.getDirection());
        if (new_normal.dot(plane_below.getNormal()) < 0.0)
        {
            new_normal = -new_normal;
        }
        SO::Plane slicing_plane(intersecting_line.getSource(), new_normal);
        SO::PolygonCollection temp_contours = this->slice(slicing_plane);

        if (temp_contours.numberOfPolygons() > 0)
        {
            slicing_planes.emplace_back(slicing_plane);
        }
        else
        {
            break;
        }
    }

    plane_up = slicing_planes.back();
    return true;
}

bool AutoSlicer::tuneConsecutivePlanesValid(SO::Plane &plane_up, SO::Plane plane_below)
{
    SO::PolygonCollection contours = this->slice(plane_up);  
    SO::PolygonCollection original_contours = contours;
    contours.removePolygonsBelowPlane(plane_below);

    Eigen::Vector3d min_point;
    double min = contours.getMinimumDistanceFromPlane(plane_below, min_point);
    plane_up.setOrigin(min_point);
    contours = this->slice(plane_up);
    contours.removePolygonsBelowPlane(plane_below);

    double min_last_time = 0;
    int time = 0;

    while (min < m_min_layer_thickness && abs(min - min_last_time) > 1e-6)
    {
        Eigen::Vector3d new_origin = plane_below.getProjectionOfPointOntoPlane(min_point) 
            + m_min_layer_thickness * plane_below.getNormal();
        plane_up.setOrigin(new_origin);
        contours = this->slice(plane_up);
        contours.removePolygonsBelowPlane(plane_below);

        min_last_time = min;
        min = contours.getMinimumDistanceFromPlane(plane_below, min_point);
    }

    Eigen::Vector3d max_point;
    double max = contours.getMaximumDistanceFromPlane(plane_below, max_point);
    double max_last_time = std::numeric_limits<double>::max();

    SO::Plane plane_before = plane_up;
    SO::PolygonCollection contours_before = contours;
    // some problem in this step 
    SO::Line intersecting_line;
    if (plane_below.isIntersectedWithPlane(plane_up, intersecting_line))
    {
        Eigen::Vector3d new_point;
        m_temp_slicing_result.back().getFurthestPointFromLine(intersecting_line, new_point);
        new_point = new_point + m_max_layer_thickness * plane_up.getNormal();

        Eigen::Vector3d new_normal = (new_point - min_point).cross(intersecting_line.getDirection());
        if (new_normal.dot(plane_below.getNormal()) < 0.0)
        {
            new_normal = -new_normal;
        }
        plane_up.setOrigin(min_point);
        plane_up.setNormal(new_normal);

        contours = this->slice(plane_up);
        contours.removePolygonsBelowPlane(plane_below);

        min = contours.getMinimumDistanceFromPlane(plane_below, min_point);
        min_last_time = 0;

        // Sometime min > m_max_layer_thickness  
        while (min > m_max_layer_thickness && min != std::numeric_limits<double>::max())
        {
            Eigen::Vector3d new_origin = plane_below.getProjectionOfPointOntoPlane(min_point);
            new_origin = new_origin + m_min_layer_thickness * plane_below.getNormal();
            plane_up.setOrigin(new_origin);
            contours = this->slice(plane_up);
            contours.removePolygonsBelowPlane(plane_below);

            min = contours.getMinimumDistanceFromPlane(plane_below, min_point);
        }

        while (min < m_min_layer_thickness && abs(min - min_last_time) > 1e-6)
        {
            Eigen::Vector3d new_origin = plane_below.getProjectionOfPointOntoPlane(min_point)
                + m_min_layer_thickness * plane_below.getNormal();
            plane_up.setOrigin(new_origin);
            contours = this->slice(plane_up);
            contours.removePolygonsBelowPlane(plane_below);

            min_last_time = min;
            min = contours.getMinimumDistanceFromPlane(plane_below, min_point);
        }

        max_last_time = max;
        max = contours.getMaximumDistanceFromPlane(plane_below, max_point);
    }
    
    if (plane_up.getPositionOfPointWrtPlane(contours_before.centroid()) == -1)
    {
        plane_up = plane_before;
        contours = contours_before;
    }

    while (max > m_max_layer_thickness && abs(max - max_last_time) > 1e-6)
    {
        SO::Line intersecting_line;
        if (plane_below.isIntersectedWithPlane(plane_up, intersecting_line))
        {
            Eigen::Vector3d new_point;
            contours.getMaximumDistanceFromPlane(plane_below, new_point);
            new_point = plane_below.getIntersectionWithRay(SO::Line(new_point, -plane_up.getNormal(), 1));
            new_point = new_point + m_min_layer_thickness * plane_up.getNormal();

            Eigen::Vector3d new_normal = (new_point - min_point).cross(intersecting_line.getDirection());
            if (new_normal.dot(plane_below.getNormal()) < 0.0)
            {
                new_normal = -new_normal;
            }
            plane_up.setNormal(new_normal);

            contours = this->slice(plane_up);
            contours.removePolygonsBelowPlane(plane_below);

            min = contours.getMinimumDistanceFromPlane(plane_below, min_point);
            min_last_time = 0;

            // Sometime min > m_max_layer_thickness  
            while (min > m_max_layer_thickness && min != std::numeric_limits<double>::max())
            {
                Eigen::Vector3d new_origin = plane_below.getProjectionOfPointOntoPlane(min_point);
                new_origin = new_origin + m_min_layer_thickness * plane_below.getNormal();
                plane_up.setOrigin(new_origin);
                contours = this->slice(plane_up);
                contours.removePolygonsBelowPlane(plane_below);

                min = contours.getMinimumDistanceFromPlane(plane_below, min_point);
            }

            while (min < m_min_layer_thickness && abs(min - min_last_time) > 1e-6)
            {
                Eigen::Vector3d new_origin = plane_below.getProjectionOfPointOntoPlane(min_point)
                    + m_min_layer_thickness * plane_below.getNormal();
                plane_up.setOrigin(new_origin);
                contours = this->slice(plane_up);
                contours.removePolygonsBelowPlane(plane_below);

                min_last_time = min;
                min = contours.getMinimumDistanceFromPlane(plane_below, min_point);
            }

            max_last_time = max;
            max = contours.getMaximumDistanceFromPlane(plane_below, max_point);
        }
    }

    if (contours.numberOfPolygons() < 1)
    {     
        return false;
    }

    return true;
}

bool AutoSlicer::isSlicingPlaneValid(SO::Plane plane_up, SO::Plane plane_below)
{
    SO::PolygonCollection contours = this->slice(plane_up);

    double min = std::numeric_limits<double>::max();
    double max = -std::numeric_limits<double>::max();
    for (auto &contour : contours.get())
    {
        for (auto &point : contour.get())
        {
            Eigen::Vector3d projected_point = plane_below.getProjectionOfPointOntoPlane(point);
            double distance = (point - projected_point).norm();
            if (distance > max)
            {
                max = distance;
            }
            if (distance < min)
            {
                min = distance;
            }
        }
    }
    //std::cout << min << " " << max << std::endl;
    if ((max - m_max_layer_thickness) > 1e-3 || (m_min_layer_thickness - min) > 1e-3)
    {
        std::cout << min << " " << max << std::endl;
        return false;
    }

    return true;
}

SO::PolygonCollection AutoSlicer::slice(SO::Plane plane)
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