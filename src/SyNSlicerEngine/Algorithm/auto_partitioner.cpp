#include "auto_partitioner.h"

using SyNSlicerEngine::Algorithm::AutoPartitioner;

AutoPartitioner::AutoPartitioner(const SO::Partition<CgalMesh_EPICK> &partition,
    const SO::Nozzle &nozzle, double overhanging_angle, double area_threshold_coefficient)
    : m_partition(partition)
    , m_nozzle(nozzle)
    , m_overhanging_angle(overhanging_angle)
    , m_area_threshold_coefficient(area_threshold_coefficient)
    , m_partition_time(0)
{
    m_results.addPartition(partition);
}

AutoPartitioner::~AutoPartitioner()
{
}

void AutoPartitioner::partition()
{
    // Repair input mesh if needed.
    if (!m_partition.repaireSelfIntersection())
    {
        spdlog::error("Input partition self-intersected!");
    }

    m_partition.makeAsCleanAsPossible();

    SO::Partition<CgalMesh_EPECK> epeck_partition = SO::Partition<CgalMesh_EPECK>(m_partition.getEPECKMesh());
    if (epeck_partition.getEPECKMesh().is_empty())
    {
        spdlog::error("Empty epeck_partition");
    }

    epeck_partition.setBasePlane(m_partition.getBasePlane());

    // Partition the input mesh automatically.
    std::vector<SO::PointCloud> vertices_to_ignore;
    //EigenPoints vertices_to_ignore;
    this->partitionMesh(epeck_partition, m_partition_list, vertices_to_ignore);
}

SO::PartitionCollection<CgalMesh_EPICK> AutoPartitioner::getResultEPICK()
{
    m_results.reset();

    // Save of the output meshes.
    for (int i = 0; i < m_partition_list.numberOfPartitions(); i++)
    {
        if (CGAL::is_closed(m_partition_list[i].getEPECKMesh()))
        {
            if (!m_partition_list[i].save(std::string("Part_") + std::to_string(i) + std::string(".stl")))
            {
                spdlog::error("AutoPartitioner::writeSTL(): Fail!");
            }
            m_results.addPartition(SO::Partition<CgalMesh_EPICK>(std::string("Part_") + std::to_string(i) + std::string(".stl")));
            m_results[i].setBasePlane(m_partition_list[i].getBasePlane());
            for (auto lock : m_partition_list[i].getLocks())
            {
                m_results[i].addLock(lock);
            }

            for (auto key : m_partition_list[i].getKeys())
            {
                m_results[i].addKey(key);
            }
        }
    }
    return m_results;
}

SO::PartitionCollection<CgalMesh_EPECK> AutoPartitioner::getResultEPECK()
{
    return m_partition_list;
}

void AutoPartitioner::partitionMesh(SO::Partition<CgalMesh_EPECK> &partition, SO::PartitionCollection<CgalMesh_EPECK> &partition_list, std::vector<SO::PointCloud> &vertices_to_ignore_list)
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
        partition_list.addPartition(low_partition);
        this->partitionMesh(low_partition, partition_list, vertices_to_ignore_list);
        partition_list.addPartition(up_partition);
        std::vector<SO::PointCloud> vertices_to_ignore;
        this->partitionMesh(up_partition, partition_list, vertices_to_ignore);
    }
}

AutoPartitioner::ResultOfDetermineClippingPlane AutoPartitioner::determineClippingPlane(SO::Partition<CgalMesh_EPICK> &partition, SO::Plane &clipping_plane, std::vector<SO::PointCloud> &vertices_to_ignore_list)
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

    if (temp_clipping_plane.getPositionOfPointWrtPlane(centroid_of_base_contours) == 1)
    {
        temp_clipping_plane.setNormal(-temp_clipping_plane.getNormal());
        if (temp_clipping_plane.getNormal().dot(Eigen::Vector3d::UnitZ()) < 0.0)
        {
            temp_clipping_plane.setNormal(Eigen::Vector3d(temp_clipping_plane.getNormal()[0], temp_clipping_plane.getNormal()[1], 0));
            if (temp_clipping_plane.getPositionOfPointWrtPlane(centroid_of_base_contours) == 1)
            {
                spdlog::error("Centroid on positive side");
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

double AutoPartitioner::getAreaOfOverhangingTrianglesProjectedOnBasePlane(std::vector<int> faces, const CgalMesh_EPICK &mesh, const SO::Plane &base_plane)
{
    double total_area = 0.0;
    for (auto index : faces)
    {
        SO::Triangle triangle(CgalMesh_EPICK::Face_index(index), mesh);
        total_area += triangle.getArea();
    }
    return total_area;
}

AutoPartitioner::OverhangingRegion AutoPartitioner::findLargestOverhangingRegion(
    std::vector<CgalMesh_EPICK::Face_index> faces_to_search, CgalMesh_EPICK &mesh,
    const SO::Plane &base_plane, std::vector<SO::PointCloud> &vertices_to_ignore_list, double area_threshold_coefficient)
{
    OverhangingRegion overhanging_region;
    std::vector<bool> accessed_list;

    accessed_list.resize(mesh.number_of_faces());
    for (int i = 0; i < accessed_list.size(); i++)
    {
        accessed_list[i] = false;
    }
    for (auto &f : faces_to_search)
    {
        accessed_list[f.id()] = true;
    }

    std::vector<CgalMesh_EPICK::Face_index> seed_faces;

    auto find_seed_face = [](std::vector<bool> &accessed_list, std::vector<CgalMesh_EPICK::Face_index> &seed_faces) {
        seed_faces.clear();
        for (int i = 0; i < accessed_list.size(); ++i)
        {
            if (accessed_list[i] == true)
            {
                seed_faces.emplace_back(CgalMesh_EPICK::Face_index(i));
                accessed_list[i] = false;
                return true;
            }
        }
        return false;
    };

    double max = std::numeric_limits<double>::min();
    while (find_seed_face(accessed_list, seed_faces))
    {
        std::vector<int> resulted_faces;
        for (auto &seed_face : seed_faces)
        {
            resulted_faces.emplace_back(seed_face.id());
        }

        while (seed_faces.size())
        {
            std::vector<CgalMesh_EPICK::Face_index> new_seed_faces;
            for (auto &f : seed_faces)
            {
                for (auto &he : mesh.halfedges_around_face(mesh.halfedge(f)))
                {
                    for (auto neighbour_f : mesh.faces_around_target(he))
                    {
                        if (accessed_list[neighbour_f.id()] == true)
                        {
                            accessed_list[neighbour_f.id()] = false;
                            resulted_faces.emplace_back(neighbour_f);
                            new_seed_faces.emplace_back(neighbour_f);
                        }
                    }
                }
            }
            seed_faces.clear();
            seed_faces = new_seed_faces;
        }

        // Check if result faces has vertex in vertices_to_ignore_list
        // Performance issue here
        auto check_vertices_in_resulted_face_contained_in_ignore_list = [&resulted_faces, &mesh, &vertices_to_ignore_list]() {

            if (vertices_to_ignore_list.size() == 0)
            {
                return false;
            }

            std::vector<bool> is_vertices_recorded_list;
            is_vertices_recorded_list.resize(mesh.number_of_vertices());

            SO::PointCloud points_in_resulted_faces;
            for (auto &face : resulted_faces)
            {
                for (auto vertex : mesh.vertices_around_face(mesh.halfedge(CgalMesh_EPICK::Face_index(face))))
                {
                    if (is_vertices_recorded_list[vertex.id()] == false)
                    {
                        is_vertices_recorded_list[vertex.id()] = true;
                        auto &p = mesh.point(vertex);
                        EigenPoint temp_point(p.x(), p.y(), p.z());
                        points_in_resulted_faces.addPoint(temp_point);
                    }
                }
            }

            for (auto &point_cloud : vertices_to_ignore_list)
            {
                if (points_in_resulted_faces.hasCommonPoints(point_cloud))
                {
                    return true;
                };
            }

            return false;
        };

        if (check_vertices_in_resulted_face_contained_in_ignore_list())
        {
            spdlog::info("This overhanging part will not be considered.");
            continue;
        }

        double area = this->getAreaOfOverhangingTrianglesProjectedOnBasePlane(resulted_faces, mesh, base_plane);

        if (area > max)
        {
            max = area;
            overhanging_region.faces = resulted_faces;
            overhanging_region.area_projected_on_base_plane = area;
        }
    }

    double total_area = 0.0;
    for (CgalMesh_EPICK::Face_index fd : mesh.faces())
    {
        SO::Triangle triangle(fd, mesh);
        total_area += triangle.getArea();
    }

    double average_area = total_area / mesh.number_of_faces();

    if (max < area_threshold_coefficient * average_area)
    {
        overhanging_region.faces.clear();
        overhanging_region.area_projected_on_base_plane = 0;
    }

    return overhanging_region;
}

bool AutoPartitioner::clipPartition(SO::Partition<CgalMesh_EPECK> &partition, ResultOfDetermineClippingPlane &clipping_plane,
    SO::Partition<CgalMesh_EPECK> &partition_low, SO::Partition<CgalMesh_EPECK> &partition_up)
{
    CgalMesh_EPECK mesh = partition.getEPECKMesh();
    SO::Plane base_plane = partition.getBasePlane();

    if (CGAL::Polygon_mesh_processing::does_self_intersect(mesh))
    {
        spdlog::error("Abort AutoPartitioner::clipMesh() due to self-intersected input mesh!");
        return false;
    }

    CgalPlane_EPECK clip_plane(clipping_plane.clipping_plane.a(), clipping_plane.clipping_plane.b(),
        clipping_plane.clipping_plane.c(), clipping_plane.clipping_plane.d());
    CgalMesh_EPECK temp_low_mesh = mesh;
    CgalMesh_EPECK temp_up_mesh = mesh;
    bool result = CGAL::Polygon_mesh_processing::clip(temp_low_mesh, clip_plane, CGAL::parameters::throw_on_self_intersection(true).clip_volume(true)) &&
        CGAL::Polygon_mesh_processing::clip(temp_up_mesh, clip_plane.opposite(), CGAL::parameters::throw_on_self_intersection(true).clip_volume(true));

    if (!result)
    {
        spdlog::error("Abort AutoPartitioner::clipMesh() due to problem occur in CGAL::Polygon_mesh_processing::clip()!");
        return false;
    }

    auto is_mesh_touch_base_plane = [&base_plane](CgalMesh_EPECK &mesh) {

        for (auto f_idx : mesh.faces())
        {
            bool point_on_plane = true;
            for (auto v_idx : mesh.vertices_around_face(mesh.halfedge(f_idx)))
            {
                auto &p = mesh.point(v_idx);
                EigenPoint point(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
                point_on_plane = point_on_plane && base_plane.isPointOnPlane(point, 1e-3);
            }
            if (point_on_plane)
            {
                return true;
            }
        }
        return false;
    };

    CgalMesh_EPECK low_mesh;
    CgalMesh_EPECK up_mesh;
    if (is_mesh_touch_base_plane(temp_low_mesh))
    {
        low_mesh = temp_low_mesh;
        up_mesh = temp_up_mesh;
    }
    else
    {
        low_mesh = temp_up_mesh;
        up_mesh = temp_low_mesh;
        clipping_plane.clipping_plane.setNormal(-clipping_plane.clipping_plane.getNormal());
    }

    low_mesh.collect_garbage();
    up_mesh.collect_garbage();

    std::vector<CgalMesh_EPECK> low_meshes;
    CGAL::Polygon_mesh_processing::split_connected_components(low_mesh, low_meshes);

    std::vector<CgalMesh_EPECK> up_meshes;
    CGAL::Polygon_mesh_processing::split_connected_components(up_mesh, up_meshes);

    if (!up_meshes.size())
    {
        spdlog::info("Plane do not split the input mesh into meshes!");
        return false;
    }

    if (low_meshes.size() > 1)
    {
        spdlog::info("Low mesh has more than two parts!");

        std::vector<int> low_mesh_on_base_plane;
        for (int i = 0; i < low_meshes.size(); i++)
        {
            low_meshes[i].collect_garbage();
            if (is_mesh_touch_base_plane(low_meshes[i]))
            {
                low_mesh_on_base_plane.emplace_back(i);
            }
        }

        if (low_mesh_on_base_plane.size() > 1)
        {
            spdlog::error("Low mesh has more than one component touch the base plane!");
            return false;
        }

        if (!low_mesh_on_base_plane.size())
        {
            spdlog::error("Low mesh has no component touch the base plane!");
            return false;
        }

        low_mesh = low_meshes[low_mesh_on_base_plane[0]];
        low_meshes.erase(low_meshes.begin() + low_mesh_on_base_plane[0]);
    }
    else
    {
        low_meshes.clear();
    }

    if (up_meshes.size() > 1)
    {
        spdlog::info("Up mesh has more than two parts!");

        auto get_mesh_has_overhanging_triangles = [clipping_plane](std::vector<CgalMesh_EPECK> &meshes, CgalMesh_EPECK &output_mesh) {

            for (int i = 0; i < meshes.size(); i++)
            {
                meshes[i].collect_garbage();
                for (auto v : meshes[i].vertices())
                {
                    auto &p = meshes[i].point(v);
                    EigenPoint point(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
                    for (auto &point_in_overhanging_triangle : clipping_plane.points_in_overhanging_triangles)
                    {
                        if ((point - point_in_overhanging_triangle).norm() < 1e-3)
                        {
                            output_mesh = meshes[i];
                            meshes.erase(meshes.begin() + i);
                            return;
                        }
                    }
                }
            }
        };

        get_mesh_has_overhanging_triangles(up_meshes, up_mesh);

        for (auto &mesh : up_meshes)
        {
            CgalMesh_EPECK current_mesh = mesh;
            CGAL::Polygon_mesh_processing::corefine_and_compute_union(low_mesh, current_mesh, low_mesh);
        }

        for (auto &mesh : low_meshes)
        {
            CgalMesh_EPECK current_mesh = mesh;
            CgalMesh_EPECK result_mesh;

            CGAL::Polygon_mesh_processing::corefine_and_compute_union(up_mesh, current_mesh, result_mesh);

            std::vector<CgalMesh_EPECK> components;
            CGAL::Polygon_mesh_processing::split_connected_components(result_mesh, components);
            if (components.size() > 1)
            {
                CGAL::Polygon_mesh_processing::corefine_and_compute_union(low_mesh, current_mesh, low_mesh);
            }
            else
            {
                up_mesh = result_mesh;
            }
        }
    }
    else
    {
        for (auto &mesh : low_meshes)
        {
            CgalMesh_EPECK current_mesh = mesh;
            CGAL::Polygon_mesh_processing::corefine_and_compute_union(up_mesh, current_mesh, up_mesh);
        }
    }

    // Check whether clipping plane cut the base contour.
    auto is_plane_cut_base_contour = [clipping_plane]() {
        bool has_points_on_positive_side = false;
        bool has_points_on_negative_side = false;
        for (auto &point : clipping_plane.points_in_base_contours)
        {
            if (clipping_plane.clipping_plane.getDistanceFromPointToPlane(point) > 0.0)
            {
                has_points_on_positive_side = true;
            }
            else if (clipping_plane.clipping_plane.getDistanceFromPointToPlane(point) < 0.0)
            {
                has_points_on_negative_side = true;
            }

            if (has_points_on_positive_side && has_points_on_negative_side)
            {
                return true;
            }
        }
        return false;
    };

    spdlog::info("Clipping result:");
    spdlog::info("Low mesh has {} vertice, {} faces.", low_mesh.number_of_vertices(), low_mesh.number_of_faces());
    spdlog::info("Up mesh has {} vertice, {} faces.", up_mesh.number_of_vertices(), up_mesh.number_of_faces());

    if (low_mesh.number_of_vertices() == 0 || low_mesh.number_of_faces() == 0)
    {
        return false;
    };
    if (up_mesh.number_of_vertices() == 0 || up_mesh.number_of_faces() == 0)
    {
        return false;
    };

    auto mesh_volume = CGAL::to_double(CGAL::Polygon_mesh_processing::volume(mesh));
    auto up_mesh_volume = CGAL::to_double(CGAL::Polygon_mesh_processing::volume(up_mesh));

    if (up_mesh_volume / mesh_volume < 0.001)
    {
        spdlog::info("Up mesh too small, result abandon.");
        return false;
    }

    low_mesh.collect_garbage();
    partition_low = SO::Partition(low_mesh);
    partition_low.setBasePlane(base_plane);
    partition_low.addKey(m_partition_time);

    up_mesh.collect_garbage();
    partition_up = SO::Partition(up_mesh);
    partition_up.setBasePlane(clipping_plane.clipping_plane);
    partition_up.addLock(m_partition_time);

    spdlog::info("AutoPartitioner::clipPartition(): successful.");
    return true;
}

bool AutoPartitioner::hasPointsOnNegativeSide(const EigenPoints &points, SO::Plane &plane)
{
    for (const auto &point : points)
    {
        if (plane.getDistanceFromPointToPlane(point) < -1e-6)
        {
            return true;
        }
    }
    return false;
}

bool AutoPartitioner::hasPointsOnPositiveSide(const EigenPoints &points, SO::Plane &plane)
{
    for (const auto &point : points)
    {
        if (plane.getDistanceFromPointToPlane(point) > 1e-6)
        {
            return true;
        }
    }
    return false;
}

bool AutoPartitioner::adjustPlaneOriginSoPointsAreOnPostiveSide(const EigenPoints &points, SO::Plane &plane)
{
    double min = std::numeric_limits<double>::max();
    Eigen::Vector3d new_origin;
    bool new_origin_found = false;
    for (auto &point : points)
    {
        double distance = plane.getDistanceFromPointToPlane(point);
        if (distance < min)
        {
            min = distance;
            new_origin = point;
            new_origin_found = true;
        }
    }
    if (new_origin_found)
    {
        plane.setOrigin(new_origin);
        return true;
    }
    return false;
}

bool AutoPartitioner::adjustPlaneOriginSoPointsAreOnNegativeSide(const EigenPoints &points, SO::Plane &plane)
{
    double max = -std::numeric_limits<double>::max();
    Eigen::Vector3d new_origin;
    bool new_origin_found = false;
    for (auto &point : points)
    {
        double distance = plane.getDistanceFromPointToPlane(point);
        if (distance > max)
        {
            max = distance;
            new_origin = point;
            new_origin_found = true;
        }
    }
    if (new_origin_found)
    {
        plane.setOrigin(new_origin);
        return true;
    }
    return false;
}

bool AutoPartitioner::adjustPlaneNormalSoPointsAreOnNegativeSide(const EigenPoints &points, const SO::Plane &reference_plane, const SO::Plane &base_plane, SO::Plane &clipping_plane)
{
    SO::Plane temp_clipping_plane = clipping_plane;
    SO::Plane plane_to_check_up_and_down = clipping_plane;
    plane_to_check_up_and_down.setNormal(clipping_plane.getNormal().cross(reference_plane.getNormal()));
    if (plane_to_check_up_and_down.getNormal().dot(base_plane.getNormal()) < 0)
    {
        plane_to_check_up_and_down.setNormal(-plane_to_check_up_and_down.getNormal());
    }

    SO::Plane plane_to_check_up_and_down_2 = clipping_plane;
    plane_to_check_up_and_down_2.setNormal(base_plane.getNormal().cross(reference_plane.getNormal()));
    if (plane_to_check_up_and_down_2.getNormal().dot(clipping_plane.getNormal()) < 0)
    {
        plane_to_check_up_and_down_2.setNormal(-plane_to_check_up_and_down_2.getNormal());
    }

    EigenPoints points_to_check_0;
    EigenPoints points_to_check_1;
    EigenPoints points_to_check_2;
    for (auto &point : points)
    {
        if (temp_clipping_plane.getDistanceFromPointToPlane(point) > 1e-6)
        {
            if (plane_to_check_up_and_down.getDistanceFromPointToPlane(point) < -1e-6)
            {
                points_to_check_0.emplace_back(point);
            }
            else
            {
                if (plane_to_check_up_and_down_2.getDistanceFromPointToPlane(point) < -1e-6)
                {
                    points_to_check_1.emplace_back(point);
                }
                else
                {
                    points_to_check_2.emplace_back(point);
                }
            }
        }
    }
        
    Eigen::Vector3d most_positive_point;
    double max = std::numeric_limits<double>::min();
    if (points_to_check_0.size())
    {
        if (this->hasPointsOnPositiveSide(points_to_check_0, temp_clipping_plane))
        {
            max = std::numeric_limits<double>::min();
            Eigen::Vector3d new_normal = temp_clipping_plane.getNormal();
            for (auto &point : points_to_check_0)
            {
                most_positive_point = reference_plane.getProjectionOfPointOntoPlane(point);
                EigenPoint origin_projected_on_reference_plane = reference_plane.getProjectionOfPointOntoPlane(temp_clipping_plane.getOrigin());
                Eigen::Vector3d temp_new_normal = most_positive_point - origin_projected_on_reference_plane;
                if (temp_new_normal.norm() > 1e-6)
                {
                    temp_new_normal = temp_new_normal / temp_new_normal.norm();
                    temp_new_normal = temp_new_normal.cross(reference_plane.getNormal());
                    double value = temp_new_normal.cross(temp_clipping_plane.getNormal()).norm();
                    if (value <= sin(30 * M_PI / 180) && value > max)
                    {
                        max = value;
                        new_normal = temp_new_normal;
                    }
                }              
            }

            temp_clipping_plane.setNormal(new_normal);
            if (temp_clipping_plane.getNormal().dot(base_plane.getNormal()) < 0)
            {
                temp_clipping_plane.setNormal(-temp_clipping_plane.getNormal());
            }
        }

        clipping_plane = temp_clipping_plane;
    }
    else
    {
        if (points_to_check_2.size())
        {
            return false;
        }

        if (this->hasPointsOnPositiveSide(points_to_check_1, temp_clipping_plane))
        {
            max = std::numeric_limits<double>::min();
            for (auto &point : points_to_check_1)
            {
                double distance = temp_clipping_plane.getDistanceFromPointToPlane(point);
                if (distance > max)
                {
                    max = distance;
                    most_positive_point = point;
                }
            }
            most_positive_point = reference_plane.getProjectionOfPointOntoPlane(most_positive_point);
            EigenPoint origin_projected_on_reference_plane = reference_plane.getProjectionOfPointOntoPlane(temp_clipping_plane.getOrigin());
            temp_clipping_plane.setNormal((most_positive_point - origin_projected_on_reference_plane).cross(reference_plane.getNormal()));
            if (temp_clipping_plane.getNormal().dot(base_plane.getNormal()) < 0)
            {
                temp_clipping_plane.setNormal(-temp_clipping_plane.getNormal());
            }
        }
        clipping_plane = temp_clipping_plane;
    }

    return true;
}