#include "auto_partitioner_debug.h"

using SyNSlicerEngine::GUI::AutoPartitionerDebug;

AutoPartitionerDebug::AutoPartitionerDebug(const SO::Partition<CgalMesh_EPICK> &partition, const SO::Nozzle &nozzle, 
vtkRenderer *renderer, double overhanging_angle, double area_threshold_coefficient)
	: AutoPartitioner(partition, nozzle, overhanging_angle)
	, m_drawer(renderer)
{

}

AutoPartitionerDebug::~AutoPartitionerDebug()
{

}


void AutoPartitionerDebug::partitionMesh(SO::Partition<CgalMesh_EPECK> &partition, SO::PartitionCollection<CgalMesh_EPECK> &partition_list, std::vector<SO::PointCloud> &vertices_to_ignore_list)
{
    std::cout << std::endl;
    spdlog::info("The {} times of partition.", ++m_partition_time);

    // Self-intersection may occur during changing from CgalMesh_EPECK to CgalMesh_EPICK
    SO::Partition<CgalMesh_EPICK> epick_partition = SO::Partition<CgalMesh_EPICK>(partition.getEPICKMesh());
    if (epick_partition.getEPICKMesh().is_empty())
    {
        spdlog::error("AutoPartitioner::partitionMesh: Convert mesh from EPECK to EPICK fail!");
        return;
    }

    epick_partition.setBasePlane(partition.getBasePlane());

    SO::Plane clipping_plane;
    ResultOfDetermineClippingPlane result_of_determining_clipping_plane =
        this->determineClippingPlane(epick_partition, clipping_plane, vertices_to_ignore_list);

    if (result_of_determining_clipping_plane.status == false)
    {
        return;
    };

    SO::Partition<CgalMesh_EPECK> low_partition;
    SO::Partition<CgalMesh_EPECK> up_partition;


    if (this->clipPartition(partition, result_of_determining_clipping_plane,
        low_partition, up_partition))
    {
        partition.writeMeshToSTL(std::string("Check/Check_") + std::to_string(m_partition_time) + std::string(".stl"));
        low_partition.writeMeshToSTL(std::string("Check/Low_") + std::to_string(m_partition_time) + std::string(".stl"));
        up_partition.writeMeshToSTL(std::string("Check/Up_") + std::to_string(m_partition_time) + std::string(".stl"));

        if (partition_list.numberOfPartitions() > 0)
        {
            for (auto lock : partition_list.back().getLocks())
            {
                low_partition.addLock(lock);
            }

            for (auto key : partition_list.back().getKeys())
            {
                up_partition.addKey(key);
            }

            partition_list.pop_back();
        }

        spdlog::get("basic_logger")->info("The {} times:", m_partition_time);
        spdlog::get("basic_logger")->info("Base plane: {} {} {} {} {} {}",
            partition.getBasePlane().getOrigin()[0], partition.getBasePlane().getOrigin()[1], partition.getBasePlane().getOrigin()[2],
            partition.getBasePlane().getNormal()[0], partition.getBasePlane().getNormal()[1], partition.getBasePlane().getNormal()[2]);
        spdlog::get("basic_logger")->info("Clip plane: {} {} {} {} {} {}",
            clipping_plane.getOrigin()[0], clipping_plane.getOrigin()[1], clipping_plane.getOrigin()[2],
            clipping_plane.getNormal()[0], clipping_plane.getNormal()[1], clipping_plane.getNormal()[2]);
        spdlog::get("basic_logger")->flush();

        partition_list.addPartition(low_partition);
        this->partitionMesh(low_partition, partition_list, vertices_to_ignore_list);
        partition_list.addPartition(up_partition);
        std::vector<SO::PointCloud> vertices_to_ignore;
        this->partitionMesh(up_partition, partition_list, vertices_to_ignore);
    }
}

SA::AutoPartitioner::ResultOfDetermineClippingPlane AutoPartitionerDebug::determineClippingPlane(SO::Partition<CgalMesh_EPICK> &partition, SO::Plane &clipping_plane, std::vector<SO::PointCloud> &vertices_to_ignore_list)
{
    ResultOfDetermineClippingPlane result;

    if (partition.getBaseContours().numberOfPolygons() < 1)
    {
        spdlog::info("AutoPartitioner::determineClippingPlane(): return false because partition do not touch the base plane.");
        result.status = false;
        return result;
    }

    CgalMesh_EPICK mesh = partition.getEPICKMesh();
    SO::Plane base_plane = partition.getBasePlane();
    EigenPoint centroid_of_base_contours = partition.getBaseContours().getCentroid();

    std::vector<CgalMesh_EPICK::Face_index> face_ids;

    // Start
    // This if loop is to find all overhanging triangles and decide how many of them are used to calculate the clipping plane.
    // If number of base contours are greater than 1, it means some overhanging region cannot be remove by partitioning.
    // The region is all the triangle facets inside the convex hull of the base contours.
    // Base contours are fromed
    if (partition.getBaseContours().numberOfPolygons() > 1)
    {
        SO::Polygon convex_hull_base_contours;
        convex_hull_base_contours = partition.getBaseContours().getConvexHullPolygon();

        for (CgalMesh_EPICK::Face_index fd : mesh.faces())
        {
            SO::Triangle triangle(fd, mesh);
            if (triangle.getOverhangingAngle(base_plane) < (m_overhanging_angle / 180.0 * M_PI))
            {
                if (!triangle.isOneOfTheVerticesOnPlane(base_plane) && !convex_hull_base_contours.isOneOfTheVerticesOfTriangleInside(triangle))
                {
                    face_ids.emplace_back(fd);
                }
            }
        }
    }
    else
    {
        for (CgalMesh_EPICK::Face_index fd : mesh.faces())
        {
            SO::Triangle triangle(fd, mesh);
            if (triangle.getOverhangingAngle(base_plane) < (m_overhanging_angle / 180.0 * M_PI))
            {
                if (!triangle.isOneOfTheVerticesOnPlane(base_plane, 1e-3))
                {
                    face_ids.emplace_back(fd);
                }
            }
        }
    }
    // End

    // Overhanging triangle facets many locate in different regions
    // For each partition operation, deal with the largest overhanging region
    // Try to eliminate the largest overhanging region by partitioning
    OverhangingRegion result_largest_overhanging_region = this->findLargestOverhangingRegion(face_ids, mesh, base_plane, vertices_to_ignore_list, m_area_threshold_coefficient);

    if (result_largest_overhanging_region.faces.size() == 0)
    {
        spdlog::info("AutoPartitioner::determineClippingPlane(): Overhanging triangles too small.");
        result.status = false;
        return result;
    }

    // Start
    // Geodesic distance is used to determine the origin of the clipping plane.
    Heat_method hm(mesh);
    for (auto f_idx : mesh.faces())
    {
        bool point_on_plane = true;
        for (auto v_idx : mesh.vertices_around_face(mesh.halfedge(f_idx)))
        {
            point_on_plane = point_on_plane && base_plane.isPointOnPlane(mesh.point(v_idx), 1e-3);
        }
        if (point_on_plane)
        {
            for (auto v_idx : mesh.vertices_around_face(mesh.halfedge(f_idx)))
            {
                hm.add_source(v_idx);
            }
        }
    }
    auto m_vertex_distance = mesh.add_property_map<vertex_descriptor, double>("v:distance", 0).first;
    hm.estimate_geodesic_distances(m_vertex_distance);
    // End

    SO::Plane temp_clipping_plane;
    EigenPoints points_in_overhanging_triangles;
    double min = std::numeric_limits<double>::max();
    CgalMesh_EPICK::Vertex_index lowest_vertex;
    EigenPoint sum_point(0, 0, 0);

    std::vector<bool> is_vertices_recorded_list;
    is_vertices_recorded_list.resize(mesh.number_of_vertices());

    for (auto f_id : result_largest_overhanging_region.faces)
    {
        for (auto v : mesh.vertices_around_face(mesh.halfedge(CgalMesh_EPICK::Face_index(f_id))))
        {
            //if (is_vertices_recorded_list[v.id()] == false)
            {
                //is_vertices_recorded_list[v.id()] = true;
                points_in_overhanging_triangles.emplace_back(Eigen::Vector3d(mesh.point(v).x(), mesh.point(v).y(), mesh.point(v).z()));
                sum_point = sum_point + base_plane.getProjectionOfPointOntoPlane(points_in_overhanging_triangles.back());
                double distance = get(m_vertex_distance, v);

                if (distance < min)
                {
                    min = distance;
                    temp_clipping_plane.setOrigin(points_in_overhanging_triangles.back());
                    lowest_vertex = v;
                }
            }
        }
    }

    result.points_in_overhanging_triangles = points_in_overhanging_triangles;

    // sum_point is the middle of the largest_overhanging_region.
    sum_point = sum_point / (result_largest_overhanging_region.faces.size() * 3);
    Eigen::Vector3d sum_point_2 = partition.getBaseContours().centroid();
    Eigen::Vector3d axis_of_rotation = (sum_point - sum_point_2).cross(base_plane.getNormal());
    axis_of_rotation = axis_of_rotation / axis_of_rotation.norm();

    // reference_plane is the plane that the normal of the clipping must lie on.
    SO::Plane reference_plane(sum_point, axis_of_rotation);

    // Start
    // Find most_overhanging_face and then use it to determine the normal of the clipping plane.
    double max = std::numeric_limits<double>::min();
    CgalMesh_EPICK::Face_index most_overhanging_face;
    for (auto &f : result_largest_overhanging_region.faces)
    {
        SO::Triangle triangle(CgalMesh_EPICK::Face_index(f), mesh);
        double dot_product = triangle.getNormal().dot(-base_plane.getNormal());
        if (dot_product > max)
        {
            max = dot_product;
            most_overhanging_face = CgalMesh_EPICK::Face_index(f);
        }
    }
    // End

    // Start
    // Determine the origin of the clipping plane.
    // The origin is one of the vertices of the triangle facets in largest_overhanging_region
    // The vertices is the closest point to the reference_plane and having the least geodesic distance to the base plane
    min = std::numeric_limits<double>::max();
    for (auto f_id : result_largest_overhanging_region.faces)
    {
        for (auto v : mesh.vertices_around_face(mesh.halfedge(CgalMesh_EPICK::Face_index(f_id))))
        {
            Eigen::Vector3d temp_point(mesh.point(v).x(), mesh.point(v).y(), mesh.point(v).z());
            double perpendicular_distance = reference_plane.getDistanceFromPointToPlane(temp_point);
            perpendicular_distance = abs(perpendicular_distance);
            double distance = get(m_vertex_distance, v);
            if ((perpendicular_distance + distance) < min)
            {
                min = perpendicular_distance + distance;
                temp_clipping_plane.setOrigin(temp_point);
                lowest_vertex = v;
            }
        }
    }
    // End

    // Start
    // Determine the normal of the clipping plane.
    Eigen::Vector3d v1_s(0, 0, 0);
    SO::Triangle triangle(most_overhanging_face, mesh);
    Eigen::Vector3d v1_t = triangle.getNormal();
    v1_s = reference_plane.getProjectionOfPointOntoPlane(v1_s);
    v1_t = reference_plane.getProjectionOfPointOntoPlane(v1_t);
    temp_clipping_plane.setNormal((v1_s - v1_t).cross(axis_of_rotation));
    // End

    // Start
    // Store all the point in base contours into base_contour_points
    // And adjust the origin of the plane so that the clipping do not intersect the base contours
    auto is_two_point_cloud_has_comment_point = [](EigenPoints &points_1, EigenPoints &points_2) {
        for (auto &point_1 : points_1)
        {
            for (auto &point_2 : points_2)
            {
                if ((point_1 - point_2).norm() < 1e-3)
                {
                    return true;
                }
            }
        }
        return false;
    };

    EigenPoints base_contour_points;

    for (int i = 0; i < partition.getBaseContours().numberOfPolygons(); i++)
    {
        for (int j = 0; j < partition.getBaseContours()[i].numberOfPoints(); j++)
        {
            base_contour_points.emplace_back(partition.getBaseContours()[i][j]);
        }
    }

    result.points_in_base_contours = base_contour_points;

    if (is_two_point_cloud_has_comment_point(points_in_overhanging_triangles, base_contour_points))
    {
        double temp_max = -std::numeric_limits<double>::max();
        Eigen::Vector3d new_origin = temp_clipping_plane.getOrigin();
        for (auto &point : base_contour_points)
        {
            double distance = temp_clipping_plane.getDistanceFromPointToPlane(point);
            if (distance > temp_max)
            {
                temp_max = distance;
                new_origin = point;
            }
        }
        temp_clipping_plane.setOrigin(new_origin);
    }
    // End

    // Start
    // points_to_check is the vertices that we want them to be on the negative side of the clipping plane.
    EigenPoints points_to_check;
    SO::Plane plane_0 = temp_clipping_plane;
    if (partition.getBaseContours().numberOfPolygons() > 1)
    {
        std::vector<bool> vertices_status;
        vertices_status.reserve(mesh.number_of_vertices());
        vertices_status.resize(mesh.number_of_vertices());

        for (size_t i = 0; i < vertices_status.size(); i++)
        {
            vertices_status[i] = true;
        }

        for (auto f_id : result_largest_overhanging_region.faces)
        {
            for (auto v : mesh.vertices_around_face(mesh.halfedge(CgalMesh_EPICK::Face_index(f_id))))
            {
                vertices_status[v.id()] = false;
            }
        }

        for (size_t i = 0; i < vertices_status.size(); i++)
        {
            if (vertices_status[i])
            {
                auto &cgal_p = mesh.point(CgalMesh_EPICK::Vertex_index(i));
                EigenPoint p(cgal_p.x(), cgal_p.y(), cgal_p.z());
                points_to_check.emplace_back(p);
            }
        }
    }
    else
    {
        Skeleton_EPICK skeleton_epick;

        CGAL::extract_mean_curvature_flow_skeleton(mesh, skeleton_epick);
        SO::Skeleton skeleton(skeleton_epick, mesh, base_plane);

        std::vector<SO::SkeletonNode *> nodes_has_no_children;
        skeleton.skeleton_root->findNodesHaveNoChildren(nodes_has_no_children);

        std::vector<std::vector<int>> vertices_all;

        int skeleton_index = skeleton.findSkeletonVertex(lowest_vertex);
        if (skeleton.skeleton_root->findNode(skeleton_index) != nullptr)
        {
            for (auto node : skeleton.skeleton_root->findNode(skeleton_index)->m_nodes)
            {
                auto &cgal_p0 = skeleton.m_skeleton[Skeleton_EPICK::vertex_descriptor(skeleton_index)].point;
                auto &cgal_p1 = skeleton.m_skeleton[Skeleton_EPICK::vertex_descriptor(node->m_point)].point;
                EigenPoint p0(cgal_p0.x(), cgal_p0.y(), cgal_p0.z());
                EigenPoint p1(cgal_p1.x(), cgal_p1.y(), cgal_p1.z());
            }
            vertices_all = skeleton.skeleton_root->findPathsStartedFromNode(skeleton.skeleton_root->findNode(skeleton_index));
        }

        std::vector<bool> vertices_list;
        vertices_list.resize(mesh.number_of_vertices());

        for (auto &vertices : vertices_all)
        {
            for (auto &index : vertices)
            {
                for (auto &vd : skeleton.m_skeleton[index].vertices)
                {
                    vertices_list[vd.id()] = true;
                }
            }
        }

        std::vector<CgalMesh_EPICK::Vertex_index> vertices_n;
        for (int i = 0; i < vertices_list.size(); i++)
        {
            if (vertices_list[i] == false)
            {
                vertices_n.emplace_back(CgalMesh_EPICK::Vertex_index(i));
                auto &cgal_p = mesh.point(CgalMesh_EPICK::Vertex_index(i));
                EigenPoint p(cgal_p.x(), cgal_p.y(), cgal_p.z());
                points_to_check.emplace_back(p);
            }
        }
    }
    // End

    if (this->adjustPlaneNormalSoPointsAreOnNegativeSide(points_to_check, reference_plane, base_plane, temp_clipping_plane) == true)
    {
        if (temp_clipping_plane.getNormal().dot(base_plane.getNormal()) < 0)
        {
            if (temp_clipping_plane.getNormal().dot(plane_0.getNormal()) > 0)
            {
                temp_clipping_plane = plane_0;
            }
            else
            {
                temp_clipping_plane.setNormal(-temp_clipping_plane.getNormal());
            }
        }
    }

    SO::Plane plane_1 = temp_clipping_plane;

    this->adjustPlaneOriginSoPointsAreOnNegativeSide(base_contour_points, temp_clipping_plane);

    if (this->adjustPlaneOriginSoPointsAreOnPostiveSide(points_in_overhanging_triangles, temp_clipping_plane))
    {
        if (this->hasPointsOnPositiveSide(base_contour_points, temp_clipping_plane))
        {
            temp_clipping_plane = plane_1;
        }
    }

    // Problem 1: z-normal < 0

    // Problem 2: centroid on positive side
    if (temp_clipping_plane.getPositionOfPointWrtPlane(centroid_of_base_contours) == 1)
    {
        temp_clipping_plane.setNormal(-temp_clipping_plane.getNormal());
        if (temp_clipping_plane.getNormal().dot(Eigen::Vector3d::UnitZ()) < 0.0)
        {
            temp_clipping_plane.setNormal(Eigen::Vector3d(temp_clipping_plane.getNormal()[0], temp_clipping_plane.getNormal()[1], 0));
            if (temp_clipping_plane.getPositionOfPointWrtPlane(centroid_of_base_contours) == 1)
            {
                spdlog::error("Centroid on positive side");
                //m_drawer.drawPlane(temp_clipping_plane, "clipping_plane" + std::to_string(m_partition_time));
            }
        }
    }

    if (temp_clipping_plane.getNormal().dot(Eigen::Vector3d::UnitZ()) < 0.0)
    {
        spdlog::info("Clipping plane normal pointing to the build plate.");
        result.status = false;
        return result;
    }

    // Check nozzle intersect built plate
    Eigen::Vector3d direction = temp_clipping_plane.getNormal().cross(reference_plane.getNormal());
    direction = direction / direction.norm();

    Eigen::Vector3d point_to_check_below_built_plate = temp_clipping_plane.getOrigin() + m_nozzle.getX() * temp_clipping_plane.getNormal();
    point_to_check_below_built_plate = point_to_check_below_built_plate + m_nozzle.getY() / 2 * direction;

    if (point_to_check_below_built_plate[2] < 0.0)
    {
        spdlog::error("Clipping plane result to nozzle intersect built plate.");
    }

    result.clipping_plane = temp_clipping_plane;
    clipping_plane = temp_clipping_plane;
    vertices_to_ignore_list.emplace_back(SO::PointCloud(points_in_overhanging_triangles));
    result.status = true;
    return result;
}
