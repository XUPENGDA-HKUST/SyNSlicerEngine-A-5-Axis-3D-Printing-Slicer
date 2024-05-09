#include "auto_slicer.h"

using SyNSlicerEngine::Algorithm::AutoSlicer;

AutoSlicer::AutoSlicer(SO::Partition<CgalMesh_EPICK> &p_partition, double target_layer_thickness, double side_step,
    double min_layer_thickness, double max_layer_thickness)
    : mp_operating_partition(&p_partition)
    , m_mesh(mp_operating_partition->getEPICKMesh())
    , m_slicer(m_mesh)
    , m_target_layer_thickness(target_layer_thickness)
    , m_side_step(side_step)
    , m_min_layer_thickness(min_layer_thickness)
    , m_max_layer_thickness(max_layer_thickness)
{

}

AutoSlicer::~AutoSlicer()
{ 

}

void AutoSlicer::slice()
{
    SO::Plane base_plane = mp_operating_partition->getBasePlane();
    SO::PolygonCollection current_contours = mp_operating_partition->getBaseContours();

    Eigen::Vector3d new_origin = current_contours.getCentroid();
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
    for (int i = 1; i < m_slicing_result.size(); i++)
    {
        if (!this->isSlicingPlaneValid(m_slicing_result[i].getPlane(), m_slicing_result[i - 1].getPlane()))
        {
            std::cout << "Layer " << i << " not_valid" << std::endl;
        }

        SO::PrintingLayer printing_layer(m_slicing_result[i], m_slicing_result[i].getPlane(), m_slicing_result[i - 1].getPlane());

        if (i != 1)
        {
            SO::PolygonCollection support_contours;
            this->checkSupportNeeded(m_slicing_result[i - 1], m_slicing_result[i], support_contours);
            printing_layers.back().addSupportStructureContours(support_contours);
        }

        printing_layers.addPrintingLayer(printing_layer);
    }

    mp_operating_partition->setPrintingLayers(printing_layers);
}

bool AutoSlicer::determineNextSlicingPlane(SO::PolygonCollection &current_starting_contours, SO::PolygonCollection &next_starting_contours)
{
    next_starting_contours.reset();

    SO::PolygonCollection contours_below = current_starting_contours;
    SO::Plane plane_up = current_starting_contours.getPlane();
    plane_up.offset(m_target_layer_thickness);
    SO::PolygonCollection contours_up = this->slice(plane_up);

    // Slicing end here.
    if (!contours_up.size())
    {
        return false;
    }

    SO::PolygonCollection support_contours;

    this->checkSupportNeeded(contours_below, contours_up, support_contours, 0.25);

    if (support_contours.size())
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

        SO::Line intersecting_line;
        m_temp_slicing_result.front().getPlane().isIntersectedWithPlane(plane_up, intersecting_line);
        double angle_of_rotation = m_temp_slicing_result.front().getPlane().getAngleOfRotation(plane_up);
        Eigen::Vector3d axis_of_rotation = m_temp_slicing_result.front().getPlane().getAxisOfRotation(plane_up);

        if (isnan(axis_of_rotation[0]) || isnan(axis_of_rotation[1]) || isnan(axis_of_rotation[2]))
        {
            goto block_1;
        }

        if (!this->calculateIntermediatePlanes(plane_up, m_temp_slicing_result.front().getPlane()))
        {
            goto block_1;
        };

        for (int i = 0; i < m_temp_slicing_result.size(); i++)
        {
            m_slicing_result.emplace_back(m_temp_slicing_result[i]); 
        }

        m_temp_slicing_result.clear();
        m_temp_slicing_result.emplace_back(this->slice(plane_up));
    }
    else
    {
    block_1:
        m_temp_slicing_result.emplace_back(this->slice(plane_up));
    }

    if (plane_up == current_starting_contours.getPlane())
    {
        return false;
    }

    next_starting_contours = m_temp_slicing_result.back();
    return true;
}

bool AutoSlicer::checkSupportNeeded(SO::PolygonCollection &contours_below, SO::PolygonCollection &contours_up, SO::PolygonCollection &support_contours, double coefficient)
{
    bool support_needed = false;

    Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;
    Eigen::Transform<double, 3, Eigen::Affine> inverse_transformation_matrix;
    Eigen::Vector3d origin(contours_below.getPlane().getOrigin());

    transformation_matrix = contours_below.getPlane().getTransformationMatrix(SO::Plane());
    inverse_transformation_matrix = transformation_matrix.inverse();

    Clipper2Lib::PathsD subject, clip, solution;

    SO::PolygonCollection projected_contours;
    projected_contours = contours_up.projectToOtherPlane(contours_below.getPlane());

    SO::PolygonCollection contours_from_layer_i_minus_1 = contours_below;

    // 0.25 control how the slicing planes sensitive to model surface
    contours_from_layer_i_minus_1 = contours_from_layer_i_minus_1.getOffset(coefficient * m_side_step);
    support_contours = projected_contours.getDifference(contours_from_layer_i_minus_1);

    if (support_contours.size())
    {
        support_needed = true;
    }

    return support_needed;
}

bool AutoSlicer::calculateIntermediatePlanes(SO::Plane &plane_up, SO::Plane plane_below)
{
    int number_of_gaps = 0;
    std::vector<SO::Plane> slicing_planes;

    if (!this->computeDefaultIntermediatePlanes(plane_up, plane_below, slicing_planes))
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
        if (tuneConsecutivePlanesValid(slicing_planes[i], slicing_planes[j]))
        {
            m_temp_slicing_result.emplace_back(this->slice(slicing_planes[i]));
        }
        else
        {
            plane_up = m_temp_slicing_result.back().getPlane();
            m_temp_slicing_result.pop_back();
            return false;
        }
        j = i;
    }
    m_temp_slicing_result.pop_back();
    plane_up = slicing_planes.back();

    return true;
}

bool AutoSlicer::computeDefaultIntermediatePlanes(SO::Plane &plane_up, SO::Plane plane_below, std::vector<SO::Plane> &slicing_planes)
{
    SO::Line intersecting_line;
    if (!plane_below.isIntersectedWithPlane(plane_up, intersecting_line))
    {
        return false;
    }

    SO::PolygonCollection contours = this->slice(plane_up);
    if (contours.size() < 1)
    {
        return false;
    }

    contours.removePolygonsBelowPlane(plane_below);

    Eigen::Vector3d closest_point;
    contours.getClosestPointFromLine(intersecting_line, closest_point);
    double a = plane_below.getDistanceFromPointToPlane(closest_point);
    Eigen::Vector3d point_at_plane_below = plane_below.getProjectionOfPointOntoPlane(closest_point);

    slicing_planes.clear();
    slicing_planes.emplace_back(plane_below);
    double distance = 0.0;

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

    // Make sure smallest layer thickness greater than the minimum layer thickness
    // that the printer can print
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

    SO::Line intersecting_line;
    if (plane_below.isIntersectedWithPlane(plane_up, intersecting_line))
    {
        Eigen::Vector3d new_point;
        // m_temp_slicing_result.back() is this->slice(plane_below)
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
    
    // Eliminate wrong result for plane up.
    if (plane_up.getPositionOfPointWrtPlane(contours_before.centroid()) == -1)
    {
        plane_up = plane_before;
        contours = contours_before;
    }

    // Adjust plane up to meet the requirement of max and min layer thickness iteratively until
    // both requirements are met
    while (max > m_max_layer_thickness && abs(max - max_last_time) > 1e-6)
    {
        SO::Line intersecting_line;
        if (plane_below.isIntersectedWithPlane(plane_up, intersecting_line))
        {
            Eigen::Vector3d new_point;
            contours.getMaximumDistanceFromPlane(plane_below, new_point);
            plane_below.isIntersectedWithRay(SO::Line(new_point, -plane_up.getNormal(), 1), new_point);
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

    // if Plane_up z-normal < 0, assertion will trigger in GcodeGenerator.
    // Method to fix:
    // Step 1: Set z-normal to zero.
    // Step 2: Offset the plane to met the min layer thickness requirement.
    if (plane_up.getNormal()[2] < 0.0)
    {
        spdlog::warn("plane_up.getNormal()[2] < 0.0");
        plane_up.setNormal(Eigen::Vector3d(plane_up.getNormal()[0], plane_up.getNormal()[1], 1e-5));
        contours = this->slice(plane_up);
        contours.removePolygonsBelowPlane(plane_below);
        min = contours.getMinimumDistanceFromPlane(plane_below, min_point);
        if (min < m_min_layer_thickness)
        {
            plane_up.offset(m_min_layer_thickness - min);
            contours = this->slice(plane_up);
            contours.removePolygonsBelowPlane(plane_below);
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
    if ((max - m_max_layer_thickness) > 1e-3 || (m_min_layer_thickness - min) > 1e-3 || plane_up.getNormal()[2] < 0.0)
    {
        std::cout << min << " " << max << " " << plane_up.getNormal()[2] << std::endl;
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