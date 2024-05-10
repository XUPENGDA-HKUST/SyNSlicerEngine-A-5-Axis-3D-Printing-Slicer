#include "mesh_clipper_gui.h"

using SyNSlicerGUI::MeshClipperGUI;

MeshClipperGUI::MeshClipperGUI(vtkRenderer *p_renderer)
	: m_should_drawer_delete_in_destructer(true)
	, mp_drawer(new ObjectDrawer(p_renderer))
{

}

MeshClipperGUI::~MeshClipperGUI()
{
	if (m_should_drawer_delete_in_destructer)
	{
		delete mp_drawer;
	}
}

bool MeshClipperGUI::findAllSperatedMeshes(std::vector<CgalMesh_EPICK::Face_index> &triangle_contour, const SO::Line &line, const SO::Plane &plane)
{
    CgalMesh_EPICK m_operating_cgal_mesh = mp_partition->getEPICKMesh();
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