#include "auto_slicer.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_3.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

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
            std::cout << "Plane_not_valid" << std::endl;
        }
        SO::PrintingLayer printing_layer(m_slicing_result[i], m_slicing_result[i].getPlane(), m_slicing_result[i - 1].getPlane());
        printing_layers.addPrintingLayer(printing_layer);
    }

    p_partition.setPrintingLayers(printing_layers);

    printing_layers.update();
    m_drawer.drawPolylines(printing_layers.getContours(), "Check1");
    m_drawer.setColor("Check1", 1, 0, 0);
}

AutoSlicer::~AutoSlicer()
{ 

}

static int aaa = 0;

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
        Eigen::Vector3d source_point = contours_below.centroid();

        SO::Plane plane_0(plane_up.getOrigin(), target_point - source_point);
        SO::Plane plane_1(plane_up.getOrigin(), plane_0.getNormal().cross(plane_up.getNormal()));

        std::vector<Eigen::Vector3d> result_below = contours_below.getIntersectionWithPlane(plane_1);
        std::vector<Eigen::Vector3d> result_up = contours_up.getIntersectionWithPlane(plane_1);

        bool p0_found = false;
        Eigen::Vector3d p0;
        for (auto &point : result_below)
        {
            if (plane_0.getPositionOfPointWrtPlane(point) == 1)
            {
                p0 = point;
                p0_found = true;
            }
        }

        bool p1_found = false;
        Eigen::Vector3d p1;
        for (auto &point : result_up)
        {
            if (plane_0.getPositionOfPointWrtPlane(point) == 1)
            {
                p1 = point;
                p1_found = true;
            }
        }

        if (p0_found == false || p1_found == false)
        {
            goto block_1;
        }


        plane_up.setNormal(p1 - p0);
        plane_up.setOrigin(p1);

        SO::Line intersecting_line = m_temp_slicing_result.front().getPlane().getIntersectionWithPlane(plane_up);
        double angle_of_rotation = m_temp_slicing_result.front().getPlane().getAngleOfRotation(plane_up);
        Eigen::Vector3d axis_of_rotation = m_temp_slicing_result.front().getPlane().getAxisOfRotation(plane_up);

        if (isnan(axis_of_rotation[0]) || isnan(axis_of_rotation[1]) || isnan(axis_of_rotation[2]))
        {
            goto block_1;
        }

        if (!this->determineIntermediatePlanes(plane_up, m_temp_slicing_result.front().getPlane()))
        {
            goto block_1;
        };

        for (int i = 0; i < m_temp_slicing_result.size(); i++)
        {
            m_slicing_result.emplace_back(m_temp_slicing_result[i]);
        }

        m_temp_slicing_result.clear();
        m_temp_slicing_result.emplace_back(this->slice(plane_up));
        if (aaa == 1)
        {
            //return false;
        }
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

    PathsD subject, clip, solution;

    SO::PolygonCollection projected_contours;
    this->projectContourOnOtherPlane(contours_up, -contours_up.getPlane().getNormal(), contours_below.getPlane(), projected_contours);
    this->transformContoursToXYPlane(projected_contours, transformation_matrix, origin);

    for (auto &contour : projected_contours.get())
    {
        PathD temp_path;
        for (auto &point : contour.get())
        {
            temp_path.push_back(Clipper2Lib::PointD(point[0], point[1]));
        }
        subject.push_back(temp_path);
    }

    SO::PolygonCollection contours_from_layer_i_minus_1 = contours_below;
    this->transformContoursToXYPlane(contours_from_layer_i_minus_1, transformation_matrix, origin);

    for (auto &contour : contours_from_layer_i_minus_1.get())
    {
        PathD temp_path;
        for (auto &point : contour.get())
        {
            temp_path.push_back(Clipper2Lib::PointD(point[0], point[1]));
        }
        clip.push_back(temp_path);
    }

    clip = Clipper2Lib::InflatePaths(clip, 0.25 * side_step, JoinType::Miter, Clipper2Lib::EndType::Polygon);
    clip = Clipper2Lib::SimplifyPaths(clip, 0.1);
  
    solution = Difference(subject, clip, FillRule::NonZero);
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

    if (max > m_max_layer_thickness || min < m_min_layer_thickness)
    {
        std::cout << min << " " << max << std::endl;
        return false;
    }

    return true;
}

bool AutoSlicer::isPlaneUpValid(SO::Plane &plane_up, SO::Plane plane_below)
{
    SO::PolygonCollection contours = this->slice(plane_up);
    SO::Line intersecting_line = plane_below.getIntersectionWithPlane(plane_up);

    if (intersecting_line.getLength() < 1e-6)
    {
        // Valid
        return true;
    }

    double angle_between_planes = plane_below.getAngleOfRotation(plane_up);

    Eigen::Vector3d closest_point = this->getClosestPointFromLine(contours, intersecting_line);
    Eigen::Vector3d furthest_point = this->getFurthestPointFromLine(contours, intersecting_line);

    double min = intersecting_line.getDistanceOfPoint(closest_point);
    double max = intersecting_line.getDistanceOfPoint(furthest_point);

    double a = max * tan(angle_between_planes);
    double b = min * tan(angle_between_planes);

    int max_number_of_clipping_plane = floor(b / (m_min_layer_thickness));

    if ((b / max_number_of_clipping_plane) < 0.25)
    {
        max_number_of_clipping_plane -= 1;
    }

    if ((b / max_number_of_clipping_plane) > 0.35)
    {
        max_number_of_clipping_plane += 1;
    }

    if (max_number_of_clipping_plane == 0)
    {
        max_number_of_clipping_plane = 1;
    }

    double max_layer_thickness = a / max_number_of_clipping_plane;

    if (max_layer_thickness <= m_max_layer_thickness)
    {
        return true;
    }

    double c = m_max_layer_thickness * max_number_of_clipping_plane;
    double angle_radian = atan(c / max);
    angle_radian = angle_between_planes - angle_radian;

    Eigen::Vector3d new_normal = plane_up.getNormal();
    Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;
    transformation_matrix = Eigen::AngleAxis<double>(angle_radian, -intersecting_line.getDirection());
    new_normal = transformation_matrix.linear() * new_normal;
    SO::Plane temp_plane(intersecting_line.getSource(), new_normal);
    SO::PolygonCollection temp_contours = this->slice(temp_plane);
    Eigen::Vector3d temp_furthest_point = this->getFurthestPointFromLine(temp_contours, intersecting_line);

    new_normal = (temp_furthest_point - plane_up.getOrigin()).cross(intersecting_line.getDirection());

    if (new_normal.dot(plane_up.getNormal()) < 1e-6)
    {
        new_normal = -new_normal;
    }
    plane_up.setNormal(new_normal);

    if (this->isPlaneUpValid(plane_up, plane_below))
    {
        return true;
    }

    return false;  
}

bool AutoSlicer::determineIntermediatePlanes(SO::Plane &plane_up, SO::Plane plane_below)
{   
    this->isPlaneUpValid(plane_up, plane_below);

    SO::PolygonCollection contours = this->slice(plane_up);
    
    SO::Line intersecting_line = plane_below.getIntersectionWithPlane(plane_up);

    if (intersecting_line.getLength() < 1e-6)
    {
        return true;
    }

    double angle_between_planes = plane_below.getAngleOfRotation(plane_up);

    Eigen::Vector3d closest_point = this->getClosestPointFromLine(contours, intersecting_line);
    Eigen::Vector3d furthest_point = this->getFurthestPointFromLine(contours, intersecting_line);

    double min = intersecting_line.getDistanceOfPoint(closest_point);
    double max = intersecting_line.getDistanceOfPoint(furthest_point);

    double a = max * tan(angle_between_planes);
    double b = min * tan(angle_between_planes);

    int max_number_of_clipping_plane = floor(b / (m_min_layer_thickness));

    if ((b / max_number_of_clipping_plane) < 0.25)
    {
        max_number_of_clipping_plane -= 1;
    }

    if ((b / max_number_of_clipping_plane) > 0.35)
    {      
        max_number_of_clipping_plane += 1;
    }

    if (max_number_of_clipping_plane == 0)
    {
        max_number_of_clipping_plane = 1;
    }

    double max_layer_thickness = a / max_number_of_clipping_plane;

    Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;

    if (!this->isPlaneUpValid(plane_up, plane_below))
    {
        double c = m_max_layer_thickness * max_number_of_clipping_plane;
        double angle_radian = atan(c / max);
        angle_radian = angle_between_planes - angle_radian;

        Eigen::Vector3d new_normal = plane_up.getNormal();
        transformation_matrix = Eigen::AngleAxis<double>(angle_radian, -intersecting_line.getDirection());
        new_normal = transformation_matrix.linear() * new_normal;
        SO::Plane temp_plane(intersecting_line.getSource(), new_normal);
        SO::PolygonCollection temp_contours = this->slice(temp_plane);
        Eigen::Vector3d temp_furthest_point = this->getFurthestPointFromLine(temp_contours, intersecting_line);

        new_normal = (temp_furthest_point - plane_up.getOrigin()).cross(intersecting_line.getDirection());

        if (new_normal.dot(plane_up.getNormal()) < 1e-6)
        {
            new_normal = -new_normal;
        }
        plane_up.setNormal(new_normal);

        intersecting_line = plane_below.getIntersectionWithPlane(plane_up);
    }

    double angle_of_rotation = plane_below.getAngleOfRotation(plane_up);
    Eigen::Vector3d axis_of_rotation = plane_below.getAxisOfRotation(plane_up);

    m_temp_slicing_result.clear();
    m_temp_slicing_result.emplace_back(this->slice(plane_below));
    for (int i = 1; i < max_number_of_clipping_plane; i++)
    {
        double angle = angle_of_rotation / max_number_of_clipping_plane * i;
        transformation_matrix = Eigen::AngleAxis<double>(angle, axis_of_rotation);
        Eigen::Vector3d new_normal = transformation_matrix.linear() * plane_below.getNormal();
        SO::Plane new_plane(intersecting_line.getSource(), new_normal);
        m_temp_slicing_result.emplace_back(this->slice(new_plane));
    }

    return true;
}

Eigen::Vector3d AutoSlicer::getClosestPointFromLine(SO::PolygonCollection &contours, SO::Line &line)
{
    Eigen::Vector3d closest_point;
    double min = std::numeric_limits<double>::max();
    for (auto &contour : contours.get())
    {
        for (auto &point : contour.get())
        {
            double distance = line.getDistanceOfPoint(point);
            if (distance < min)
            {
                closest_point = point;
                min = distance;
            }
        }
    }
    return closest_point;
}

Eigen::Vector3d AutoSlicer::getFurthestPointFromLine(SO::PolygonCollection &contours, SO::Line &line)
{
    Eigen::Vector3d furthest_point;
    double max = -std::numeric_limits<double>::max();
    for (auto &contour : contours.get())
    {
        for (auto &point : contour.get())
        {
            double distance = line.getDistanceOfPoint(point);
            if (distance > max)
            {
                furthest_point = point;
                max = distance;
            }
        }
    }
    return furthest_point;
}

void AutoSlicer::projectContourOnOtherPlane(
    const SO::PolygonCollection &contours,
    const Eigen::Vector3d &direction,
    const SO::Plane &plane,
    SO::PolygonCollection &projected_contours)
{
    projected_contours.reset();
    for (auto &contour : contours.get())
    {
        SO::Polygon projected_contour;
        projected_contour.setPlane(plane);
        for (auto &point : contour.get())
        {
            Eigen::Vector3d temp_point = point;
            SO::Line ray(temp_point, direction, 1);
            temp_point = plane.getIntersectionWithRay(ray);
            projected_contour.addPointToBack(temp_point);
        }
        projected_contours.addPolygon(projected_contour);
    }
}

void AutoSlicer::transformContoursToXYPlane(
    SO::PolygonCollection &contours,
    const Eigen::Transform<double, 3, Eigen::Affine> &transformation_matrix,
    const Eigen::Vector3d &origin)
{
    SO::PolygonCollection new_contours;
    for (auto &contour : contours.get())
    {
        SO::Polygon new_polygon;
        new_polygon.setPlane(contours.getPlane());
        for (auto &point : contour.get())
        {
            new_polygon.addPointToBack(transformation_matrix * (point - origin) + origin);
        }
        new_contours.addPolygon(new_polygon);
    }
    contours = new_contours;
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