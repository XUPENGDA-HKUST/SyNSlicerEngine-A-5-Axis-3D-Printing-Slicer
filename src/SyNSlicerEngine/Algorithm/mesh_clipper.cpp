#include "mesh_clipper.h"

using SyNSlicerEngine::Algorithm::MeshClipper;

MeshClipper::MeshClipper()
	: clipping_time(0)
	, mp_partition(nullptr)
{

}

MeshClipper::~MeshClipper()
{ 

}

void MeshClipper::setPartition(SO::Partition<CgalMesh_EPICK> *p_partition)
{
	mp_partition = p_partition;
}

bool MeshClipper::clipWithInfinitePlane(const SO::Plane &plane, SO::PartitionCollection<CgalMesh_EPICK> &result)
{
	result.clear();

	SO::Partition<CgalMesh_EPICK> &partition = *mp_partition;

	if (partition.getBaseContours().isIntersectedWithPlane(plane))
	{
		return false;
	};

	SO::Plane clip_plane = plane;

	if (clip_plane.getPositionOfPointWrtPlane(partition.getBaseContours().centroid()) == 1)
	{
		clip_plane.setNormal(-clip_plane.getNormal());
	}

	CgalMesh_EPICK pos_mesh = partition.getEPICKMesh();
	CgalMesh_EPICK neg_mesh = partition.getEPICKMesh();

	CgalPlane_EPICK cgal_plane(clip_plane.a(), clip_plane.b(), clip_plane.c(), clip_plane.d());

	CGAL::Polygon_mesh_processing::clip(neg_mesh, cgal_plane, CGAL::parameters::clip_volume(true));
	CGAL::Polygon_mesh_processing::clip(pos_mesh, cgal_plane.opposite(), CGAL::parameters::clip_volume(true));

	pos_mesh.collect_garbage();
	neg_mesh.collect_garbage();

	if (pos_mesh.is_empty())
	{
		return false;
	}

	SO::Partition<CgalMesh_EPICK> up_partition(pos_mesh);
	up_partition.setBasePlane(clip_plane);
	up_partition.addLock(clipping_time);
	SO::Partition<CgalMesh_EPICK> low_partition(neg_mesh);
	low_partition.setBasePlane(partition.getBasePlane());
	low_partition.addKey(clipping_time);

	clipping_time += 1;

	result.addPartition(low_partition);
	result.addPartition(up_partition);

	return true;
}

bool MeshClipper::clipWithFinitePlane(const SO::Line &line, const Eigen::Vector3d &camera_position, const SO::Plane &clipping_plane, SO::PartitionCollection<CgalMesh_EPICK> &result)
{
    std::vector<CgalMesh_EPICK::Face_index> triangle_contour;
    if (findTriangleContour(line, clipping_plane, triangle_contour))
    {
        if (findAllSperatedMeshes(triangle_contour, line, clipping_plane))
        {
            clipping_time += 1;
            result = m_partitions;
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

bool MeshClipper::findTriangleContour(const SO::Line &line, const SO::Plane &plane, std::vector<CgalMesh_EPICK::Face_index> &triangle_contour)
{
    std::vector<bool> triangle_list;
    CgalMesh_EPICK &m_operating_cgal_mesh = mp_partition->getEPICKMesh();
    triangle_list.resize(m_operating_cgal_mesh.number_of_faces());

    // Iterate all halfedge and then check whether the halfedge intersect the user-dragged line
    for (CgalMesh_EPICK::Halfedge_index halfedge_idx : m_operating_cgal_mesh.halfedges())
    {
        if (checkHalfedgeIntersectLine(halfedge_idx, m_operating_cgal_mesh, line, plane))
        {
            // If yes, check whether the face associated to this halfedge are located on the base plane
            if (isFaceContainedInPlane(halfedge_idx, m_operating_cgal_mesh, mp_partition->getBasePlane()))
            {
                std::cout << "Clipping Plane cross the base contour!" << std::endl;
                return false;
            }
            else
            {
                // If yes, record the face associated to this halfedge
                if (m_operating_cgal_mesh.face(halfedge_idx).idx() > m_operating_cgal_mesh.number_of_faces() - 1)
                {
                    std::cout << "Sth Wrong! " << m_operating_cgal_mesh.face(halfedge_idx).idx() << " " << m_operating_cgal_mesh.number_of_faces() << std::endl;
                    return false;
                }
                else
                {
                    triangle_list[m_operating_cgal_mesh.face(halfedge_idx).idx()] = true;
                }

            }
        }
    }

    // Create a vector to store all the faces recoreded
    triangle_contour.clear();
    for (int i = 0; i < triangle_list.size(); i++)
    {
        if (triangle_list[i])
        {
            CgalMesh_EPICK::Face_index fid(i);
            triangle_contour.push_back(fid);
        }
    }
    // extract the intersectin triangle to a independent mesh for future processing
    if (triangle_contour.size() != 0)
    {
        return true;
    }
    else
    {
        // The dragged line may have no intersection with the mesh, return false telling the input line is not valid.
        return false;
    }
}

bool MeshClipper::findAllSperatedMeshes(std::vector<CgalMesh_EPICK::Face_index> &triangle_contour, const SO::Line &line, const SO::Plane &plane)
{
    CgalMesh_EPICK &m_operating_cgal_mesh = mp_partition->getEPICKMesh();
    if (PMP::does_self_intersect(m_operating_cgal_mesh))
    {
        std::cout << "Original mesh self intersect!" << std::endl;
    }

    { /* Only clips the close loop triangle contour */
        std::vector<bool> triangle_list;
        triangle_list.resize(m_operating_cgal_mesh.number_of_faces());

        for (size_t i = 0; i < triangle_contour.size(); i++)
        {
            triangle_list[triangle_contour[i]] = true;
        }

        // Faces may form many contours, seperate them
        std::vector<std::vector<CgalMesh_EPICK::Face_index>> triangle_contours;
        triangle_contours = findNumberOfTriangleContours(triangle_list, m_operating_cgal_mesh);

        triangle_contour.clear();
        for (size_t i = 0; i < triangle_contours.size(); i++)
        {
            if (isTriangleContourCloseLoop(triangle_contours[i], m_operating_cgal_mesh, line, plane))
            {
                for (size_t j = 0; j < triangle_contours[i].size(); j++)
                {
                    triangle_contour.push_back(triangle_contours[i][j]);
                }
            }
        }
    }

    if (triangle_contour.size() != 0)
    {
        CgalMesh_EPICK temp_cgal_mesh = m_operating_cgal_mesh;
        for (size_t i = 0; i < triangle_contour.size(); i++)
        {
            CgalMesh_EPICK::Face_index fid(triangle_contour[i]);
            temp_cgal_mesh.remove_face(fid);
        }

        temp_cgal_mesh.collect_garbage();
        mp_partition->makeAsCleanAsPossible(temp_cgal_mesh);

        std::vector<CgalMesh_EPICK> meshes;
        CGAL::Polygon_mesh_processing::split_connected_components(temp_cgal_mesh, meshes);

        if (meshes.size() > 1)
        {
            CgalMesh_EPICK contour;
            this->extractFacetsFromMeshToNewMesh(triangle_contour, m_operating_cgal_mesh, contour);

            std::vector<CgalMesh_EPICK> contour_meshes;
            CGAL::Polygon_mesh_processing::split_connected_components(contour, contour_meshes);

            std::vector<CgalMesh_EPICK> contour_meshes_new;

            for (size_t i = 0; i < contour_meshes.size(); i++)
            {
                CgalMesh_EPICK temp_mesh_0 = contour_meshes[i];
                CgalMesh_EPICK temp_mesh_1 = contour_meshes[i];

                CgalPlane_EPICK cgal_plane(plane.a(), plane.b(), plane.c(), plane.d());
                CGAL::Polygon_mesh_processing::clip(temp_mesh_0, cgal_plane, CGAL::parameters::clip_volume(true));
                CGAL::Polygon_mesh_processing::clip(temp_mesh_1, cgal_plane.opposite(), CGAL::parameters::clip_volume(true));

                contour_meshes_new.push_back(temp_mesh_0);
                contour_meshes_new.push_back(temp_mesh_1);
            }

            std::vector<bool> meshes_status;
            meshes_status.resize(meshes.size());

            std::vector<bool> contour_meshes_new_status;
            contour_meshes_new_status.resize(contour_meshes_new.size());

            std::multimap<int, int> mapping;

            // Handle combination
            for (int i = 0; i < meshes.size(); i++)
            {
                for (int j = 0; j < contour_meshes_new.size(); j++)
                {
                    if (this->isTwoMeshesTouchEachOther(meshes[i], contour_meshes_new[j]))
                    {
                        mapping.insert({ i,j });
                    }
                }
            }

            for (std::pair<int, int> it : mapping)
            {
                this->combineTwoMeshes(meshes[it.first], contour_meshes_new[it.second]);
            }

            std::cout << meshes.size() << " models created!" << std::endl;
            m_partitions.clear();
            for (size_t i = 0; i < meshes.size(); i++)
            {    
                SO::Partition<CgalMesh_EPICK> partition(meshes[i]);
                this->determineBasePlane(partition, mp_partition->getBasePlane(), plane);
                m_partitions.addPartition(partition);
            }

            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

bool MeshClipper::checkHalfedgeIntersectLine(CgalMesh_EPICK::Halfedge_index halfedge_idx, const CgalMesh_EPICK &mesh,
    const SO::Line &line, const SO::Plane &plane)
{
    CgalPoint_EPICK pt0_cgal = mesh.point(mesh.vertex(mesh.edge(halfedge_idx), 0));
    Eigen::Vector3d pt0(pt0_cgal.x(), pt0_cgal.y(), pt0_cgal.z());

    CgalPoint_EPICK pt1_cgal = mesh.point(mesh.vertex(mesh.edge(halfedge_idx), 1)); // target
    Eigen::Vector3d pt1(pt1_cgal.x(), pt1_cgal.y(), pt1_cgal.z());

    double distance = 0.0;

    int a = plane.getPositionOfPointWrtPlane(pt0);
    int b = plane.getPositionOfPointWrtPlane(pt1); // change position of pt1

    if (a * b != 1)
    {
        bool c = line.isProjectionOfPointLieOnLineSegment(pt0);
        bool d = line.isProjectionOfPointLieOnLineSegment(pt1);

        if (c || d)
        {
            return true;
        };
    };
    return false;
}

bool MeshClipper::isFaceContainedInPlane(
    CgalMesh_EPICK::halfedge_index halfedge_idx,
    const CgalMesh_EPICK &mesh,
    const SO::Plane &plane)
{
    int number_of_vertex_contained_in_plane = 0;
    double distance = 0.0;
    for (CgalMesh_EPICK::Vertex_index vertex_idx : mesh.vertices_around_face(halfedge_idx))
    {
        Eigen::Vector3d vertex(mesh.point(vertex_idx).x(), mesh.point(vertex_idx).y(), mesh.point(vertex_idx).z());
        if (plane.isPointOnPlane(vertex))
        {
            ++number_of_vertex_contained_in_plane;
        }
    }
    if (number_of_vertex_contained_in_plane == 3)
    {
        return true;
    }
    else
    {
        return false;
    }
}

std::vector<std::vector<CgalMesh_EPICK::Face_index>> MeshClipper::findNumberOfTriangleContours(std::vector<bool> &triangle_list, const CgalMesh_EPICK &mesh)
{
    std::vector<CgalMesh_EPICK::Face_index> triangle_contour;
    std::vector<std::vector<CgalMesh_EPICK::Face_index>> triangle_contours;

    std::vector<CgalMesh_EPICK::Face_index> face_list_to_be_check;
    for (int i = 0; i < triangle_list.size(); i++)
    {
        if (triangle_list[i])
        {
            triangle_contour.clear();
            CgalMesh_EPICK::Face_index face_idx(i);
            face_list_to_be_check.push_back(face_idx);

            while (face_list_to_be_check.size() != 0)
            {
                for (size_t i = 0; i < face_list_to_be_check.size(); i++)
                {
                    triangle_contour.push_back(face_list_to_be_check[i]);
                }
                getNeighborTriangle(face_list_to_be_check, triangle_list, mesh);
            }
            triangle_contours.push_back(triangle_contour);
        }
    }
    return triangle_contours;
}

void MeshClipper::getNeighborTriangle(std::vector<CgalMesh_EPICK::Face_index> &face_list, std::vector<bool> &triangle_list, const CgalMesh_EPICK &mesh)
{
    std::vector<CgalMesh_EPICK::Face_index> new_face_list;
    for (size_t i = 0; i < face_list.size(); i++)
    {
        CgalMesh_EPICK::Halfedge_index halfedge_idx(mesh.halfedge(face_list[i]));
        for (CgalMesh_EPICK::Face_index face_idx : mesh.faces_around_face(halfedge_idx))
        {
            if (triangle_list[face_idx.idx()])
            {
                triangle_list[face_idx.idx()] = false;
                new_face_list.push_back(face_idx);
            }
        }
    }
    face_list.clear();
    face_list = new_face_list;
}

bool MeshClipper::isTriangleContourCloseLoop(std::vector<CgalMesh_EPICK::Face_index> triangle_contour, const CgalMesh_EPICK &mesh,
    const SO::Line &line, const SO::Plane &plane)
{
    std::vector<bool> triangle_list;
    triangle_list.resize(mesh.number_of_faces());
    for (size_t i = 0; i < triangle_list.size(); i++)
    {
        triangle_list[i] = false;
    }
    for (int i = 0; i < triangle_contour.size(); i++)
    {
        triangle_list[triangle_contour[i]] = true;
    }

    for (int i = 0; i < triangle_contour.size(); i++)
    {
        for (CgalMesh_EPICK::Halfedge_index halfedge_idx : mesh.halfedges_around_face(mesh.halfedge(triangle_contour[i])))
        {
            CgalPoint_EPICK pt0_cgal = mesh.point(mesh.vertex(mesh.edge(halfedge_idx), 0));
            Eigen::Vector3d pt0(pt0_cgal.x(), pt0_cgal.y(), pt0_cgal.z());

            CgalPoint_EPICK pt1_cgal = mesh.point(mesh.vertex(mesh.edge(halfedge_idx), 1));
            Eigen::Vector3d pt1(pt1_cgal.x(), pt1_cgal.y(), pt1_cgal.z());

            int a = plane.getPositionOfPointWrtPlane(pt0);
            int b = plane.getPositionOfPointWrtPlane(pt1);

            if (a * b != 1)
            {
                // Return false if the face associated to the opposition of current halfedge is not selected in step 1
                if (triangle_list[mesh.face(mesh.opposite(halfedge_idx))] != true)
                {
                    return false;
                };
            }
        }
    }
    return true;
}

void MeshClipper::determineBasePlane(SO::Partition<CgalMesh_EPICK> &model, const SO::Plane &base_plane, const SO::Plane &clipping_plane)
{
    double distance = 0.0;
    CgalMesh_EPICK &mesh = model.getEPICKMesh();
    for (auto it : mesh.vertices())
    {
        Eigen::Vector3d point(mesh.point(it).x(), mesh.point(it).y(), mesh.point(it).z());
        if (base_plane.isPointOnPlane(point, 1e-2))
        {
            model.setBasePlane(base_plane);
            model.addKey(clipping_time);
            return;
        }
    }
    model.setBasePlane(clipping_plane);
    model.addLock(clipping_time);
}

void MeshClipper::extractFacetsFromMeshToNewMesh(std::vector<CgalMesh_EPICK::Face_index> facets, const CgalMesh_EPICK &mesh_in, CgalMesh_EPICK &mesh_out)
{
    mesh_out = mesh_in;

    std::vector<bool> triangle_status_list;
    triangle_status_list.resize(mesh_out.number_of_faces());

    for (size_t i = 0; i < facets.size(); i++)
    {
        if (facets[i].idx() > triangle_status_list.size() - 1)
        {
            spdlog::error("Vector Index out of range! () ()", facets[i].idx(), triangle_status_list.size());
        }
        triangle_status_list[facets[i].idx()] = true;
    }

    for (size_t i = 0; i < triangle_status_list.size(); i++)
    {
        if (triangle_status_list[i] == false)
        {
            CgalMesh_EPICK::Face_index face_index(i);
            mesh_out.remove_face(face_index);
        }
    }

    mesh_out.collect_garbage();
    mp_partition->makeAsCleanAsPossible(mesh_out);
}

bool MeshClipper::isTwoMeshesTouchEachOther(const CgalMesh_EPICK &mesh_1, const CgalMesh_EPICK &mesh_2)
{
    for (CgalMesh_EPICK::Vertex_index vid_1 : mesh_1.vertices())
    {
        for (CgalMesh_EPICK::Vertex_index vid_2 : mesh_2.vertices())
        {
            Eigen::Vector3d p0(mesh_1.point(vid_1).x(), mesh_1.point(vid_1).y(), mesh_1.point(vid_1).z());
            Eigen::Vector3d p1(mesh_2.point(vid_2).x(), mesh_2.point(vid_2).y(), mesh_2.point(vid_2).z());
            if ((p0 - p1).norm() < 1e-6)
            {
                return true;
            }
        }
    }
    return false;
}

void MeshClipper::combineTwoMeshes(CgalMesh_EPICK &mesh_to_combined, CgalMesh_EPICK mesh_to_be_added)
{
    for (CgalMesh_EPICK::face_index face_idx : mesh_to_be_added.faces())
    {
        std::vector<CgalMesh_EPICK::vertex_index> vertex_list;
        for (CgalMesh_EPICK::vertex_index vertex_idx : mesh_to_be_added.vertices_around_face(mesh_to_be_added.halfedge(face_idx)))
        {
            vertex_list.push_back(addVertextoMesh(mesh_to_be_added.point(vertex_idx), mesh_to_combined));
        }
        mesh_to_combined.add_face(vertex_list[0], vertex_list[1], vertex_list[2]);
    }

    if (PMP::does_self_intersect(mesh_to_combined))
    {
        std::cout << "Self_intersect! " << mesh_to_combined.number_of_faces() << " " << mesh_to_combined.number_of_vertices() << std::endl;
        CGAL::IO::write_polygon_mesh("Self_intersect_mesh_when_combine.off", mesh_to_combined, CGAL::parameters::stream_precision(17));
    };
}

CgalMesh_EPICK::Vertex_index MeshClipper::addVertextoMesh(CgalPoint_EPICK point, CgalMesh_EPICK &mesh)
{
    Eigen::Vector3d p1(point.x(), point.y(), point.z());
    for (CgalMesh_EPICK::Vertex_index vid : mesh.vertices())
    {
        Eigen::Vector3d p0(mesh.point(vid).x(), mesh.point(vid).y(), mesh.point(vid).z());
        if ((p0 - p1).norm() < 1e-6)
        {
            return vid;
        };
    }
    CgalMesh_EPICK::Vertex_index a = mesh.add_vertex(point);
    return a;
}