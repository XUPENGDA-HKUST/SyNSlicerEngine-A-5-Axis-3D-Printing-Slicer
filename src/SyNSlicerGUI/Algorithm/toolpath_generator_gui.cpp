#include "toolpath_generator_gui.h"

using SyNSlicerGUI::ToolpathGeneratorGUI;

ToolpathGeneratorGUI::ToolpathGeneratorGUI(SO::Partition<CgalMesh_EPICK> &partition, vtkRenderer *p_renderer, bool with_support)
	: ToolpathGenerator(partition, with_support)
	, m_drawer(new SyNSlicerGUI::ObjectDrawer(p_renderer))
	, m_should_drawer_delete_in_destructer(true)
{

}

ToolpathGeneratorGUI::ToolpathGeneratorGUI(SO::Partition<CgalMesh_EPICK> &partition, SyNSlicerGUI::ObjectDrawer *p_drawer, bool with_support)
	: ToolpathGenerator(partition, with_support)
	, m_drawer(p_drawer)
	, m_should_drawer_delete_in_destructer(false)
{

}

ToolpathGeneratorGUI::~ToolpathGeneratorGUI()
{
	if (m_should_drawer_delete_in_destructer == true)
	{
		delete m_drawer;
	}
}

void ToolpathGeneratorGUI::generateTopBottomUnionAndInfillContoursForModel(int wall_count)
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

        InfillPathGeneratorGUI infill_generator(infill_contours, m_cutting_planes, m_side_step, 1, m_drawer->getRenderer());
        infill_generator.generateInfillPath();
        infill_generator.getOutput(infill_contours);
        infill_contours = infill_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        infill_contours = infill_contours.getTranslatedPolygons(local_center);

        current_layer.getPrintingPaths().getBottomTopUnion().addPolygons(infill_contours);
    }
}

void ToolpathGeneratorGUI::generateInfillForModel(int wall_count, int infill_type)
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

        InfillPathGeneratorGUI infill_generator(infill_contours, m_cutting_planes, m_side_step, m_infill_type, m_drawer->getRenderer());
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

void ToolpathGeneratorGUI::generateTopBottomUnionAndInfillContoursForSupport(int wall_count)
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

        InfillPathGeneratorGUI infill_generator(infill_contours, m_cutting_planes, m_side_step, 2, m_drawer->getRenderer());
        infill_generator.generateInfillPath();
        infill_generator.getOutput(infill_contours);
        infill_contours = infill_contours.getTransformedPolygons(SO::Plane(m_center_of_infill_cutting_planes, current_layer.getSlicingPlane().getNormal()));
        infill_contours = infill_contours.getTranslatedPolygons(local_center);

        current_layer.getPrintingPathsForSupport().getBottomTopUnion().addPolygons(infill_contours);
    }
}

void ToolpathGeneratorGUI::generateInfillForSupport(int wall_count, int infill_type)
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

        InfillPathGeneratorGUI infill_generator(infill_contours, m_cutting_planes, m_side_step, m_infill_type, m_drawer->getRenderer());
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