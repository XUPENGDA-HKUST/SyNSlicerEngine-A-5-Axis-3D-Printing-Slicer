#include "toolpath_generator.h"

using SyNSlicerEngine::Algorithm::ToolpathGenerator;

ToolpathGenerator::ToolpathGenerator(SO::Partition<CgalMesh_EPICK> &partition, bool with_support, vtkRenderer *p_renderer)
    : mp_partition(&partition)
    , m_drawer(p_renderer)
    , m_side_step(0.4)
{
    m_with_support = with_support;
};

ToolpathGenerator::~ToolpathGenerator()
{
}

void ToolpathGenerator::setPathPropertyForModel(int wall_count, int bottom_count, int top_count, int path_type, int infill_density, double side_step)
{
    m_infill_type = path_type;
    m_wall_count = wall_count;
    m_bottom_count = bottom_count;
    m_top_count = top_count;
    m_infill_density = infill_density;
    m_side_step = side_step;
    m_property_for_model_setup = true;
}

void ToolpathGenerator::setPathPropertyForSupport(int wall_count, int bottom_count, int top_count, int path_type, int infill_density, double side_step)
{
    m_infill_type_support = path_type;
    m_wall_count_support = wall_count;
    m_bottom_count_support = bottom_count;
    m_top_count_support = top_count;
    m_infill_density_support = infill_density;
    m_side_step = side_step;
    m_property_for_support_setup = true;
}

void ToolpathGenerator::generatePath()
{
    if (!m_property_for_model_setup)
    {
        return;
    }
    if (m_with_support && !m_property_for_support_setup)
    {
        return;
    }

    this->determineOriginOfAllPrintingLayers();
    this->generateSurfaceForModel();
    this->generateWallForModel(m_wall_count);
    this->generateBottomForModel(m_wall_count, m_bottom_count);
    this->generateTopForModel(m_wall_count, m_top_count);
    this->generateTopBottomUnionAndInfillContoursForModel(m_wall_count);
    this->generateInfillForModel(2, m_infill_type);

    if (m_with_support == true)
    {
        this->generateSurfaceForSupport();
        this->generateWallForSupport(m_wall_count_support);
        this->generateBottomForSupport(m_wall_count_support, m_bottom_count_support);
        this->generateTopForSupport(m_wall_count_support, m_top_count_support);
        this->generateTopBottomUnionAndInfillContoursForSupport(m_wall_count_support);
        this->generateInfillForSupport(m_wall_count_support, m_infill_type_support);
    }
}

void ToolpathGenerator::determineOriginOfAllPrintingLayers()
{
    if (mp_partition->getPrintingLayers().size() < 1)
    {
        return;
    }

    SO::PrintingLayer &first_printing_layer = mp_partition->getPrintingLayers()[0];
    SO::PolygonCollection contours_of_the_first_layer = first_printing_layer.getContours();

    SO::Line intersecting_line;
    if (first_printing_layer.getSlicingPlane().isIntersectedWithPlane(first_printing_layer.getPrevSlicingPlane(), intersecting_line))
    {
        SO::Plane target_plane(intersecting_line.getSource(), first_printing_layer.getPrevSlicingPlane().getNormal());
        contours_of_the_first_layer = contours_of_the_first_layer.getTransformedPolygons(target_plane);
    }

    if (first_printing_layer.getPrevSlicingPlane().isIntersectedWithPlane(SO::Plane(), intersecting_line))
    {
        SO::Plane target_plane(intersecting_line.getSource(), Eigen::Vector3d::UnitZ());
        contours_of_the_first_layer = contours_of_the_first_layer.getTransformedPolygons(target_plane);
    }

    double bound[6];
    contours_of_the_first_layer.getBoundingBox(bound); // be careful
    m_center_of_infill_cutting_planes = Eigen::Vector3d((bound[1] + bound[0]) / 2, (bound[3] + bound[2]) / 2, (bound[5] + bound[4]) / 2);
    m_center_of_infill_cutting_planes[2] = 0;
    Eigen::Vector3d m_direction_1_target = m_center_of_infill_cutting_planes + Eigen::Vector3d(1, 0, 0);
    Eigen::Vector3d m_direction_2_target = m_center_of_infill_cutting_planes + Eigen::Vector3d(0, 1, 0);

    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        if (layer_index == 0)
        {
            current_layer.getOrigin() = this->transformPointFromPlaneToPlane(m_center_of_infill_cutting_planes, SO::Plane(), current_layer.getSlicingPlane());
            m_direction_1_target = this->transformPointFromPlaneToPlane(m_direction_1_target, SO::Plane(), current_layer.getSlicingPlane());
            m_direction_2_target = this->transformPointFromPlaneToPlane(m_direction_2_target, SO::Plane(), current_layer.getSlicingPlane());
            current_layer.getDirection1() = SO::Line(current_layer.getOrigin(), m_direction_1_target);
            current_layer.getDirection2() = SO::Line(current_layer.getOrigin(), m_direction_2_target);
        }
        else
        {
            SO::PrintingLayer &prev_layer = mp_partition->getPrintingLayers()[layer_index - 1];
            current_layer.getOrigin() = this->transformPointFromPlaneToPlane(prev_layer.getOrigin(), prev_layer.getSlicingPlane(), current_layer.getSlicingPlane());
            m_direction_1_target = this->transformPointFromPlaneToPlane(m_direction_1_target, prev_layer.getSlicingPlane(), current_layer.getSlicingPlane());
            m_direction_2_target = this->transformPointFromPlaneToPlane(m_direction_2_target, prev_layer.getSlicingPlane(), current_layer.getSlicingPlane());
            current_layer.getDirection1() = SO::Line(current_layer.getOrigin(), m_direction_1_target);
            current_layer.getDirection2() = SO::Line(current_layer.getOrigin(), m_direction_2_target);
        }   
    }
}

void ToolpathGenerator::determineCuttingPlanesZigzagInfill(SO::PolygonCollection &contours, int index)
{
    if (contours.numberOfPolygons() < 1)
    {
        return;
    }

    double bound[6];
    contours.getBoundingBox(bound);


    if (index % 2 == 0)
    {
        Eigen::Vector3d zigzag_direction_1(1, 1, 0);
        zigzag_direction_1 = zigzag_direction_1 / zigzag_direction_1.norm();
        std::deque<Eigen::Vector3d> cutting_planes_origin_direction_1;
        cutting_planes_origin_direction_1.push_back(m_center_of_infill_cutting_planes);

        Eigen::Vector3d plane_origin = m_center_of_infill_cutting_planes;
        while (plane_origin[0] > bound[0] || plane_origin[1] > bound[2])
        {
            plane_origin = plane_origin - zigzag_direction_1 * m_side_step;
            cutting_planes_origin_direction_1.push_front(plane_origin);
        }

        plane_origin = m_center_of_infill_cutting_planes;
        while (plane_origin[0] < bound[1] || plane_origin[1] < bound[3])
        {
            plane_origin = plane_origin + zigzag_direction_1 * m_side_step;
            cutting_planes_origin_direction_1.push_back(plane_origin);
        }
        m_cutting_planes.clear();
        for (size_t i = 0; i < cutting_planes_origin_direction_1.size(); i++)
        {
            m_cutting_planes.emplace_back(SO::Plane(cutting_planes_origin_direction_1[i], zigzag_direction_1));
        }
    }
    else
    {
        Eigen::Vector3d zigzag_direction_2(-1, 1, 0);
        zigzag_direction_2 = zigzag_direction_2 / zigzag_direction_2.norm();
        std::deque<Eigen::Vector3d> cutting_planes_origin_direction_2;
        cutting_planes_origin_direction_2.push_back(m_center_of_infill_cutting_planes);
        Eigen::Vector3d plane_origin = m_center_of_infill_cutting_planes;
        while (plane_origin[1] > bound[2] || plane_origin[0] < bound[1])
        {
            plane_origin = plane_origin - zigzag_direction_2 * m_side_step;
            cutting_planes_origin_direction_2.push_front(plane_origin);
        }

        plane_origin = m_center_of_infill_cutting_planes;
        while (plane_origin[1] < bound[3] || plane_origin[0] > bound[0])
        {
            plane_origin = plane_origin + zigzag_direction_2 * m_side_step;
            cutting_planes_origin_direction_2.push_back(plane_origin);
        }
        m_cutting_planes.clear();
        for (size_t i = 0; i < cutting_planes_origin_direction_2.size(); i++)
        {
            m_cutting_planes.emplace_back(SO::Plane(cutting_planes_origin_direction_2[i], zigzag_direction_2));
        }
    }
}

void ToolpathGenerator::determineCuttingPlanesGridInfill(SO::PolygonCollection &contours, int infill_density)
{
    if (contours.numberOfPolygons() < 1)
    {
        return;
    }

    double bound[6];
    contours.getBoundingBox(bound);

    double grid_size = (2 * m_side_step) / ((double)infill_density / 100);

    std::deque<Eigen::Vector3d> cutting_planes_origin_direction_1;
    std::deque<Eigen::Vector3d> cutting_planes_origin_direction_2;
    cutting_planes_origin_direction_1.push_back(m_center_of_infill_cutting_planes);
    cutting_planes_origin_direction_2.push_back(m_center_of_infill_cutting_planes);

    Eigen::Vector3d direction_1 = Eigen::Vector3d::UnitX();
    Eigen::Vector3d direction_2 = Eigen::Vector3d::UnitY();

    Eigen::Vector3d plane_origin = m_center_of_infill_cutting_planes;
    while (plane_origin[0] > bound[0])
    {
        plane_origin = plane_origin - direction_1 * grid_size;
        cutting_planes_origin_direction_1.push_front(plane_origin);
    }

    plane_origin = m_center_of_infill_cutting_planes;
    while (plane_origin[0] < bound[1])
    {
        plane_origin = plane_origin + direction_1 * grid_size;
        cutting_planes_origin_direction_1.push_back(plane_origin);
    }

    plane_origin = m_center_of_infill_cutting_planes;
    while (plane_origin[1] > bound[2])
    {
        plane_origin = plane_origin - direction_2 * grid_size;
        cutting_planes_origin_direction_2.push_front(plane_origin);
    }

    plane_origin = m_center_of_infill_cutting_planes;
    while (plane_origin[1] < bound[3])
    {
        plane_origin = plane_origin + direction_2 * grid_size;
        cutting_planes_origin_direction_2.push_back(plane_origin);
    }

    m_cutting_planes.clear();
    for (size_t i = 0; i < cutting_planes_origin_direction_1.size(); i++)
    {
        m_cutting_planes.emplace_back(SO::Plane(cutting_planes_origin_direction_1[i], direction_1));
    }
    for (size_t i = 0; i < cutting_planes_origin_direction_2.size(); i++)
    {
        m_cutting_planes.emplace_back(SO::Plane(cutting_planes_origin_direction_2[i], direction_2));
    }
}

void ToolpathGenerator::generateSurfaceForModel()
{
    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        current_layer.getPrintingPaths().reset();
        SO::PolygonCollection contours = current_layer.getContours();
        if (contours.numberOfPolygons() < 1)
        {
            continue;
        }
        Eigen::Vector3d local_center = current_layer.getOrigin();
        contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
        contours = contours.getOffset(-0.5 * m_side_step);
        contours = contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        contours = contours.getTranslatedPolygons(local_center);
        contours.closePolygons();
        current_layer.getPrintingPaths().getSurface() = contours;
    }
}

void ToolpathGenerator::generateWallForModel(int wall_count)
{
    // At this step, local_center is the matching with the m_center_of_infill_cutting_planes;
    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        SO::PolygonCollection contours = current_layer.getPrintingPaths().getSurface();
        if (contours.numberOfPolygons() < 1)
        {
            continue;
        }

        Eigen::Vector3d local_center = current_layer.getOrigin();
        contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);

        std::vector<SO::PolygonCollection> wall_paths;

        while (contours.numberOfPolygons() > 0 && wall_paths.size() < wall_count)
        {
            contours = contours.getOffset(-m_side_step);
            wall_paths.emplace_back(contours);
        }

        for (size_t i = 0; i < wall_paths.size(); i++)
        {
            contours = wall_paths[i];
            contours = contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
            contours = contours.getTranslatedPolygons(local_center);
            contours.closePolygons();
            current_layer.getPrintingPaths().getWall().push_back(contours);
        }
    }
}

void ToolpathGenerator::generateBottomForModel(int wall_count, int bottom_count)
{
    // The idea is find difference of current_layer and prev_layer
    SO::PolygonCollection difference;

    int prev_layer_index = 0;
    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        if (current_layer.getPrintingPaths().getWall().size() < wall_count)
        {
            prev_layer_index = layer_index;
            continue;
        }
        SO::PolygonCollection contours = current_layer.getPrintingPaths().getWall().back();
        if (contours.numberOfPolygons() < 1)
        {
            prev_layer_index = layer_index;
            continue;
        }

        if (layer_index == 0)
        {
            Eigen::Vector3d local_center = current_layer.getOrigin();
            contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            difference = contours.getOffset(-0.5 * m_side_step);
        }
        else
        {
            SO::PrintingLayer &prev_layer = mp_partition->getPrintingLayers()[prev_layer_index];
            // Question: what is the best prev_contours?
            // Surface? First Wall or the last wall?
            SO::PolygonCollection prev_contours;
            prev_contours = prev_layer.getPrintingPaths().getSurface();

            Eigen::Vector3d local_center = current_layer.getOrigin();
            contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            contours = contours.getOffset(-0.5 * m_side_step);

            local_center = prev_layer.getOrigin();
            prev_contours = prev_contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            prev_contours = prev_contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);

            difference = contours.getDifference(prev_contours);
            difference = difference.getOffset(3 * m_side_step);
            difference = difference.getIntersection(contours);
        }

        if (difference.numberOfPolygons() > 0)
        {
            for (int i = 0; i < bottom_count - 1; i++)
            {
                if (i < mp_partition->getPrintingLayers().size() - 1)
                {
                    SO::PrintingLayer &working_layer = mp_partition->getPrintingLayers()[layer_index + 1 + i];
                    if (working_layer.getPrintingPaths().getWall().size() >= wall_count)
                    {
                        // Transform the bottom contour to the working_layer and then computer the intersection;
                        SO::PolygonCollection working_contours = working_layer.getPrintingPaths().getWall()[wall_count - 1];
                        if (working_contours.numberOfPolygons() >= 0)
                        {
                            Eigen::Vector3d working_local_center = working_layer.getOrigin();
                            working_contours = working_contours.getTransformedPolygons(SO::Plane(working_local_center, Eigen::Vector3d::UnitZ()));
                            working_contours = working_contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
                            working_contours = working_contours.getIntersection(difference);
                            working_contours = working_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, working_layer.getSlicingPlane().getNormal()));
                            working_contours = working_contours.getTranslatedPolygons(working_local_center);
                            working_layer.getPrintingPaths().getBottom().emplace_back(working_contours);
                        };
                    }
                }
            }

            Eigen::Vector3d local_center = current_layer.getOrigin();
            difference = difference.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
            difference = difference.getTranslatedPolygons(local_center);
            current_layer.getPrintingPaths().getBottom().emplace_back(difference);
        }   
        prev_layer_index = layer_index;
    }

}

void ToolpathGenerator::generateTopForModel(int wall_count, int top_count)
{
    // At this step, local_center is the matching with the m_center_of_infill_cutting_planes;
    // The first layer must be a bottom.
    // The idea is find difference of current_layer and prev_layer
    SO::PolygonCollection difference;

    int prev_layer_index = mp_partition->getPrintingLayers().size() - 1;
    for (int layer_index = mp_partition->getPrintingLayers().size() - 1; layer_index >= 0; layer_index--)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        if (current_layer.getPrintingPaths().getWall().size() < wall_count)
        {
            prev_layer_index = layer_index;
            continue;
        }
        SO::PolygonCollection contours = current_layer.getPrintingPaths().getWall().back();
        if (contours.numberOfPolygons() < 1)
        {
            prev_layer_index = layer_index;
            continue;
        }

        if (layer_index == mp_partition->getPrintingLayers().size() - 1)
        {
            Eigen::Vector3d local_center = current_layer.getOrigin();
            contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            difference = contours.getOffset(-0.5 * m_side_step);
        }
        else
        {
            SO::PrintingLayer &prev_layer = mp_partition->getPrintingLayers()[prev_layer_index];

            // Question: what is the best prev_contours?
            // Surface? First Wall or the last wall?
            SO::PolygonCollection prev_contours;

            prev_contours = prev_layer.getPrintingPaths().getSurface();
            Eigen::Vector3d local_center = current_layer.getOrigin();
            contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            contours = contours.getOffset(-0.5 * m_side_step);

            local_center = prev_layer.getOrigin();
            prev_contours = prev_contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            prev_contours = prev_contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);

            difference = contours.getDifference(prev_contours);
            difference = difference.getOffset(3 * m_side_step);
            difference = difference.getIntersection(contours);
        }

        if (difference.numberOfPolygons() > 0)
        {
            for (int i = 0; i < top_count - 1; i++)
            {
                if ((layer_index - 1 - i) < (mp_partition->getPrintingLayers().size() - 1) && (layer_index - 1 - i) >= 0)
                {
                    SO::PrintingLayer &working_layer = mp_partition->getPrintingLayers()[layer_index - 1 - i];
                    if (working_layer.getPrintingPaths().getWall().size() >= wall_count)
                    {
                        // Transform the bottom contour to the working_layer and then computer the intersection;
                        SO::PolygonCollection working_contours = working_layer.getPrintingPaths().getWall()[wall_count - 1];
                        if (working_contours.numberOfPolygons() >= 0)
                        {
                            Eigen::Vector3d working_local_center = working_layer.getOrigin();
                            working_contours = working_contours.getTransformedPolygons(SO::Plane(working_local_center, Eigen::Vector3d::UnitZ()));
                            working_contours = working_contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
                            working_contours = working_contours.getIntersection(difference);
                            working_contours = working_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, working_layer.getSlicingPlane().getNormal()));
                            working_contours = working_contours.getTranslatedPolygons(working_local_center);
                            working_layer.getPrintingPaths().getTop().emplace_back(working_contours);
                        };
                    }
                }
            }

            Eigen::Vector3d local_center = current_layer.getOrigin();
            difference = difference.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
            difference = difference.getTranslatedPolygons(local_center);
            current_layer.getPrintingPaths().getTop().emplace_back(difference);
        }
        prev_layer_index = layer_index;
    }
}

void ToolpathGenerator::generateTopBottomUnionAndInfillContoursForModel(int wall_count)
{
    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        if (current_layer.getPrintingPaths().getWall().size() < wall_count)
        {
            continue;
        }
        SO::PolygonCollection contours = current_layer.getPrintingPaths().getWall()[wall_count - 1];
        if (contours.numberOfPolygons() < 1)
        {
            continue;
        }
        Eigen::Vector3d local_center = current_layer.getSlicingPlane().getOrigin();
        contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
        contours = contours.getOffset(-0.5 * m_side_step);

        SO::PolygonCollection union_contours;
        for (size_t i = 0; i < current_layer.getPrintingPaths().getBottom().size(); i++)
        {
            SO::PolygonCollection current_bottom = current_layer.getPrintingPaths().getBottom()[i];
            current_bottom = current_bottom.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            current_bottom = current_bottom.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            union_contours = current_bottom.getUnion(union_contours);

        }
        for (size_t i = 0; i < current_layer.getPrintingPaths().getTop().size(); i++)
        {
            SO::PolygonCollection current_top = current_layer.getPrintingPaths().getTop()[i];
            current_top = current_top.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            current_top = current_top.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            union_contours = current_top.getUnion(union_contours);
        }

        union_contours = union_contours.getOffset(3 * m_side_step);
        union_contours = union_contours.getIntersection(contours);

        SO::PolygonCollection infill_contour = contours.getDifference(union_contours);
        infill_contour = infill_contour.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        infill_contour = infill_contour.getTranslatedPolygons(local_center);
        current_layer.getPrintingPaths().getInfill() = infill_contour;

        union_contours = union_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        union_contours = union_contours.getTranslatedPolygons(local_center);
        current_layer.getPrintingPaths().getBottomTopUnion() = union_contours;
    }

    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        SO::PolygonCollection contours = current_layer.getPrintingPaths().getBottomTopUnion();
        if (contours.numberOfPolygons() < 1)
        {
            continue;
        }

        Eigen::Vector3d local_center = current_layer.getOrigin();
        contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
        contours = contours.getOffset(-0.5 * m_side_step);

        this->determineCuttingPlanesZigzagInfill(contours, layer_index);
        SO::PolygonCollection infill_contours = contours;

        contours = contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        contours = contours.getTranslatedPolygons(local_center);
        contours.closePolygons();

        current_layer.getPrintingPaths().getBottomTopUnion() = contours;

        InfillPathGenerator infill_generator(infill_contours, m_cutting_planes, m_side_step, 2, m_drawer.getRenderer());
        infill_generator.generateInfillPath();
        infill_generator.getOutput(infill_contours);
        infill_contours = infill_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        infill_contours = infill_contours.getTranslatedPolygons(local_center);

        current_layer.getPrintingPaths().getBottomTopUnion().addPolygons(infill_contours);
    }
}

void ToolpathGenerator::generateInfillForModel(int wall_count, int infill_type)
{
    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        SO::PolygonCollection contours = current_layer.getPrintingPaths().getInfill();
        if (contours.numberOfPolygons() < 1)
        {
            continue;
        }

        Eigen::Vector3d local_center = current_layer.getOrigin();
        contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);

        Eigen::Vector3d direction_1_target = this->transformPointFromPlaneToPlane(current_layer.getDirection1().getTarget(), 
            current_layer.getSlicingPlane(), SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        direction_1_target = direction_1_target - local_center + m_center_of_infill_cutting_planes;

        contours = contours.getTransformedPolygons(
            SO::Plane(m_center_of_infill_cutting_planes, direction_1_target - m_center_of_infill_cutting_planes),
            SO::Plane(m_center_of_infill_cutting_planes, Eigen::Vector3d::UnitX()));

        contours = contours.getOffset(-0.5 * m_side_step);
        
        if (m_infill_type == 1)
        {
            this->determineCuttingPlanesZigzagInfill(contours, layer_index);
        }
        else if (m_infill_type == 2)
        {
            this->determineCuttingPlanesGridInfill(contours, m_infill_density);
        }
        
        SO::PolygonCollection infill_contours = contours;

        contours = contours.getTransformedPolygons(
            SO::Plane(m_center_of_infill_cutting_planes, Eigen::Vector3d::UnitX()),
            SO::Plane(m_center_of_infill_cutting_planes, direction_1_target - m_center_of_infill_cutting_planes));
        contours = contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        contours = contours.getTranslatedPolygons(local_center);
        contours.closePolygons();
        current_layer.getPrintingPaths().getInfill() = contours;

        InfillPathGenerator infill_generator(infill_contours, m_cutting_planes, m_side_step, m_infill_type);
        infill_generator.generateInfillPath();
        infill_generator.getOutput(infill_contours);
        infill_contours = infill_contours.getTransformedPolygons(
            SO::Plane(m_center_of_infill_cutting_planes, Eigen::Vector3d::UnitX()),
            SO::Plane(m_center_of_infill_cutting_planes, direction_1_target - m_center_of_infill_cutting_planes));
        infill_contours = infill_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        infill_contours = infill_contours.getTranslatedPolygons(local_center);

        current_layer.getPrintingPaths().getInfill().addPolygons(infill_contours);
    }
    
}

void ToolpathGenerator::generateSurfaceForSupport()
{
    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        current_layer.getPrintingPathsForSupport().reset();
        SO::PolygonCollection contours = current_layer.getSupportStructureContours();
        if (contours.numberOfPolygons() < 1)
        {
            continue;
        }

        Eigen::Vector3d local_center = current_layer.getOrigin();
        contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
        contours = contours.getOffset(-0.5 * m_side_step);
        contours = contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        contours = contours.getTranslatedPolygons(local_center);
        contours.closePolygons();
        current_layer.getPrintingPathsForSupport().getSurface() = contours;
    }
}

void ToolpathGenerator::generateWallForSupport(int wall_count)
{
    // At this step, local_center is the matching with the m_center_of_infill_cutting_planes;
    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        SO::PolygonCollection contours = current_layer.getPrintingPathsForSupport().getSurface();
        if (contours.numberOfPolygons() < 1)
        {
            continue;
        }

        Eigen::Vector3d local_center = current_layer.getOrigin();
        contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);

        std::vector<SO::PolygonCollection> wall_paths;

        while (contours.numberOfPolygons() > 0 && wall_paths.size() < wall_count)
        {
            contours = contours.getOffset(-m_side_step);
            wall_paths.emplace_back(contours);
        }

        for (size_t i = 0; i < wall_paths.size(); i++)
        {
            contours = wall_paths[i];
            contours = contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
            contours = contours.getTranslatedPolygons(local_center);
            contours.closePolygons();
            current_layer.getPrintingPathsForSupport().getWall().push_back(contours);
        }
    }
}

void ToolpathGenerator::generateBottomForSupport(int wall_count, int bottom_count)
{
    if (mp_partition->getPrintingLayers().size() < 1)
    {
        return;
    }

    // The idea is find difference of current_layer and prev_layer
    SO::PolygonCollection difference;

    int prev_layer_index = 0;
    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        if (current_layer.getPrintingPathsForSupport().getWall().size() < wall_count)
        {
            prev_layer_index = layer_index;
            continue;
        }
        SO::PolygonCollection contours = current_layer.getPrintingPathsForSupport().getWall().back();
        if (contours.numberOfPolygons() < 1)
        {
            prev_layer_index = layer_index;
            continue;
        }

        if (layer_index == 0)
        {
            Eigen::Vector3d local_center = current_layer.getOrigin();
            contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            difference = contours.getOffset(-0.5 * m_side_step);
        }
        else
        {
            SO::PrintingLayer &prev_layer = mp_partition->getPrintingLayers()[prev_layer_index];
            // Question: what is the best prev_contours?
            // Surface? First Wall or the last wall?
            SO::PolygonCollection prev_contours;
            prev_contours = prev_layer.getPrintingPathsForSupport().getSurface();

            Eigen::Vector3d local_center = current_layer.getOrigin();
            contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            contours = contours.getOffset(-0.5 * m_side_step);

            local_center = prev_layer.getOrigin();
            prev_contours = prev_contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            prev_contours = prev_contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);

            difference = contours.getDifference(prev_contours);
            difference = difference.getOffset(3 * m_side_step);
            difference = difference.getIntersection(contours);
        }

        if (difference.numberOfPolygons() > 0)
        {
            for (int i = 0; i < bottom_count - 1; i++)
            {
                if (i < mp_partition->getPrintingLayers().size() - 1)
                {
                    SO::PrintingLayer &working_layer = mp_partition->getPrintingLayers()[layer_index + 1 + i];
                    if (working_layer.getPrintingPathsForSupport().getWall().size() >= wall_count)
                    {
                        // Transform the bottom contour to the working_layer and then computer the intersection;
                        SO::PolygonCollection working_contours = working_layer.getPrintingPathsForSupport().getWall()[wall_count - 1];
                        if (working_contours.numberOfPolygons() >= 0)
                        {
                            Eigen::Vector3d working_local_center = working_layer.getOrigin();
                            working_contours = working_contours.getTransformedPolygons(SO::Plane(working_local_center, Eigen::Vector3d::UnitZ()));
                            working_contours = working_contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
                            working_contours = working_contours.getIntersection(difference);
                            working_contours = working_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, working_layer.getSlicingPlane().getNormal()));
                            working_contours = working_contours.getTranslatedPolygons(working_local_center);
                            working_layer.getPrintingPathsForSupport().getBottom().emplace_back(working_contours);
                        };
                    }
                }
            }

            Eigen::Vector3d local_center = current_layer.getOrigin();
            difference = difference.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
            difference = difference.getTranslatedPolygons(local_center);
            current_layer.getPrintingPathsForSupport().getBottom().emplace_back(difference);
        }
        prev_layer_index = layer_index;
    }
}

void ToolpathGenerator::generateTopForSupport(int wall_count, int top_count)
{
    SO::PolygonCollection difference;

    int prev_layer_index = mp_partition->getPrintingLayers().size() - 1;
    for (int layer_index = mp_partition->getPrintingLayers().size() - 1; layer_index >= 0; layer_index--)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        if (current_layer.getPrintingPathsForSupport().getWall().size() < wall_count)
        {
            prev_layer_index = layer_index;
            continue;
        }
        SO::PolygonCollection contours = current_layer.getPrintingPathsForSupport().getWall().back();
        if (contours.numberOfPolygons() < 1)
        {
            prev_layer_index = layer_index;
            continue;
        }

        if (layer_index == mp_partition->getPrintingLayers().size() - 1)
        {
            Eigen::Vector3d local_center = current_layer.getOrigin();
            contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            difference = contours.getOffset(-0.5 * m_side_step);
        }
        else
        {
            SO::PrintingLayer &prev_layer = mp_partition->getPrintingLayers()[prev_layer_index];

            // Question: what is the best prev_contours?
            // Surface? First Wall or the last wall?
            SO::PolygonCollection prev_contours;

            prev_contours = prev_layer.getPrintingPathsForSupport().getSurface();
            Eigen::Vector3d local_center = current_layer.getOrigin();
            contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            contours = contours.getOffset(-0.5 * m_side_step);

            local_center = prev_layer.getOrigin();
            prev_contours = prev_contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            prev_contours = prev_contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);

            difference = contours.getDifference(prev_contours);
            difference = difference.getOffset(3 * m_side_step);
            difference = difference.getIntersection(contours);
        }

        if (difference.numberOfPolygons() > 0)
        {
            for (int i = 0; i < top_count - 1; i++)
            {
                if ((layer_index - 1 - i) < (mp_partition->getPrintingLayers().size() - 1) && (layer_index - 1 - i) >= 0)
                {
                    SO::PrintingLayer &working_layer = mp_partition->getPrintingLayers()[layer_index - 1 - i];
                    if (working_layer.getPrintingPathsForSupport().getWall().size() >= wall_count)
                    {
                        // Transform the bottom contour to the working_layer and then computer the intersection;
                        SO::PolygonCollection working_contours = working_layer.getPrintingPathsForSupport().getWall()[wall_count - 1];
                        if (working_contours.numberOfPolygons() >= 0)
                        {
                            Eigen::Vector3d working_local_center = working_layer.getOrigin();
                            working_contours = working_contours.getTransformedPolygons(SO::Plane(working_local_center, Eigen::Vector3d::UnitZ()));
                            working_contours = working_contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
                            working_contours = working_contours.getIntersection(difference);
                            working_contours = working_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, working_layer.getSlicingPlane().getNormal()));
                            working_contours = working_contours.getTranslatedPolygons(working_local_center);
                            working_layer.getPrintingPathsForSupport().getTop().emplace_back(working_contours);
                        };
                    }
                }
            }

            Eigen::Vector3d local_center = current_layer.getOrigin();
            difference = difference.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
            difference = difference.getTranslatedPolygons(local_center);
            current_layer.getPrintingPathsForSupport().getTop().emplace_back(difference);
        }
        prev_layer_index = layer_index;
    }
}

void ToolpathGenerator::generateTopBottomUnionAndInfillContoursForSupport(int wall_count)
{
    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        if (current_layer.getPrintingPathsForSupport().getWall().size() < wall_count)
        {
            continue;
        }
        SO::PolygonCollection contours = current_layer.getPrintingPathsForSupport().getWall()[wall_count - 1];
        if (contours.numberOfPolygons() < 1)
        {
            continue;
        }
        Eigen::Vector3d local_center = current_layer.getSlicingPlane().getOrigin();
        contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
        contours = contours.getOffset(-0.5 * m_side_step);

        SO::PolygonCollection union_contours;
        for (size_t i = 0; i < current_layer.getPrintingPathsForSupport().getBottom().size(); i++)
        {
            SO::PolygonCollection current_bottom = current_layer.getPrintingPathsForSupport().getBottom()[i];
            current_bottom = current_bottom.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            current_bottom = current_bottom.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            union_contours = current_bottom.getUnion(union_contours);

        }
        for (size_t i = 0; i < current_layer.getPrintingPathsForSupport().getTop().size(); i++)
        {
            SO::PolygonCollection current_top = current_layer.getPrintingPathsForSupport().getTop()[i];
            current_top = current_top.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
            current_top = current_top.getTranslatedPolygons(m_center_of_infill_cutting_planes);
            union_contours = current_top.getUnion(union_contours);
        }

        union_contours = union_contours.getOffset(3 * m_side_step);
        union_contours = union_contours.getIntersection(contours);

        SO::PolygonCollection infill_contour = contours.getDifference(union_contours);
        infill_contour = infill_contour.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        infill_contour = infill_contour.getTranslatedPolygons(local_center);
        current_layer.getPrintingPathsForSupport().getInfill() = infill_contour;

        union_contours = union_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        union_contours = union_contours.getTranslatedPolygons(local_center);
        current_layer.getPrintingPathsForSupport().getBottomTopUnion() = union_contours;
    }

    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        SO::PolygonCollection contours = current_layer.getPrintingPathsForSupport().getBottomTopUnion();
        if (contours.numberOfPolygons() < 1)
        {
            continue;
        }

        Eigen::Vector3d local_center = current_layer.getOrigin();
        contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);
        contours = contours.getOffset(-0.5 * m_side_step);

        this->determineCuttingPlanesZigzagInfill(contours, layer_index);
        SO::PolygonCollection infill_contours = contours;

        contours = contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        contours = contours.getTranslatedPolygons(local_center);
        contours.closePolygons();

        current_layer.getPrintingPathsForSupport().getBottomTopUnion() = contours;

        InfillPathGenerator infill_generator(infill_contours, m_cutting_planes, m_side_step, 2, m_drawer.getRenderer());
        infill_generator.generateInfillPath();
        infill_generator.getOutput(infill_contours);
        infill_contours = infill_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        infill_contours = infill_contours.getTranslatedPolygons(local_center);

        current_layer.getPrintingPathsForSupport().getBottomTopUnion().addPolygons(infill_contours);
    }
}

void ToolpathGenerator::generateInfillForSupport(int wall_count, int infill_type)
{
    for (int layer_index = 0; layer_index < mp_partition->getPrintingLayers().size(); layer_index++)
    {
        SO::PrintingLayer &current_layer = mp_partition->getPrintingLayers()[layer_index];
        SO::PolygonCollection contours = current_layer.getPrintingPathsForSupport().getInfill();
        if (contours.numberOfPolygons() < 1)
        {
            continue;
        }

        Eigen::Vector3d local_center = current_layer.getOrigin();
        contours = contours.getTransformedPolygons(SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        contours = contours.getTranslatedPolygons(m_center_of_infill_cutting_planes);

        Eigen::Vector3d direction_1_target = this->transformPointFromPlaneToPlane(current_layer.getDirection1().getTarget(),
            current_layer.getSlicingPlane(), SO::Plane(local_center, Eigen::Vector3d::UnitZ()));
        direction_1_target = direction_1_target - local_center + m_center_of_infill_cutting_planes;

        contours = contours.getTransformedPolygons(
            SO::Plane(m_center_of_infill_cutting_planes, direction_1_target - m_center_of_infill_cutting_planes),
            SO::Plane(m_center_of_infill_cutting_planes, Eigen::Vector3d::UnitX()));

        contours = contours.getOffset(-0.5 * m_side_step);

        if (m_infill_type == 1)
        {
            this->determineCuttingPlanesZigzagInfill(contours, layer_index);
        }
        else if (m_infill_type == 2)
        {
            this->determineCuttingPlanesGridInfill(contours, m_infill_density);
        }

        SO::PolygonCollection infill_contours = contours;

        contours = contours.getTransformedPolygons(
            SO::Plane(m_center_of_infill_cutting_planes, Eigen::Vector3d::UnitX()),
            SO::Plane(m_center_of_infill_cutting_planes, direction_1_target - m_center_of_infill_cutting_planes));
        contours = contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        contours = contours.getTranslatedPolygons(local_center);
        contours.closePolygons();
        current_layer.getPrintingPathsForSupport().getInfill() = contours;

        InfillPathGenerator infill_generator(infill_contours, m_cutting_planes, m_side_step, m_infill_type);
        infill_generator.generateInfillPath();
        infill_generator.getOutput(infill_contours);
        infill_contours = infill_contours.getTransformedPolygons(
            SO::Plane(m_center_of_infill_cutting_planes, Eigen::Vector3d::UnitX()),
            SO::Plane(m_center_of_infill_cutting_planes, direction_1_target - m_center_of_infill_cutting_planes));
        infill_contours = infill_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        infill_contours = infill_contours.getTranslatedPolygons(local_center);

        current_layer.getPrintingPathsForSupport().getInfill().addPolygons(infill_contours);
    }
}

Eigen::Vector3d ToolpathGenerator::transformPointFromPlaneToPlane(const Eigen::Vector3d &point, const SO::Plane &source_plane, const SO::Plane &target_plane)
{
    Eigen::Vector3d result_point(0, 0, 0);
    SO::Line intersectin_line;
    if (source_plane.isIntersectedWithPlane(target_plane, intersectin_line))
    {
        Eigen::Vector3d origin = intersectin_line.getSource();
        Eigen::Transform<double, 3, Eigen::Affine> transformation_matrix;
        transformation_matrix = source_plane.getTransformationMatrix(target_plane);
        result_point = (transformation_matrix * (point - origin)) + origin;
    }
    else
    {
        result_point = target_plane.getProjectionOfPointOntoPlane(point);
    }
    return result_point;
}