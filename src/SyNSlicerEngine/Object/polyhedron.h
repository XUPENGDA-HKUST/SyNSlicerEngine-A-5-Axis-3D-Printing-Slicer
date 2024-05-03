#ifndef SYNSLICERENGINE_OBJECT_POLYHEDRON_H_
#define SYNSLICERENGINE_OBJECT_POLYHEDRON_H_

#include <CGAL_CORE_CLASS>
#include "spdlog/spdlog.h"

namespace PMP = CGAL::Polygon_mesh_processing;

namespace SyNSlicerEngine::Object {

	//!  This class defines a polyhedron.
	/*!
		Polyhedron is store as a CGALMesh.
	*/
	template <class T> 
	class Polyhedron
	{
	public:
        //! Default constructor.
		Polyhedron();

        //! Copy constructor.
		Polyhedron(const Polyhedron &other);

        //! Constructor.
        /*!
            \brief  construct this by loading a .stl file.
            \param[in] file_path Path of the .stl file.
        */
		Polyhedron(std::string file_path);

        //! Constructor.
        /*!
            \brief  construct this by inputting a cgal mesh.
            \param[in] mesh CgalMesh_EPICK or CgalMesh_EPECK.
        */
		Polyhedron(const T &mesh);

        //! Destructor.
		~Polyhedron();

        //! Repaire self-intersection.
		bool repaireSelfIntersection();

        //! Remove duplicate vertices and faces.
		bool makeAsCleanAsPossible();

        //! Remove duplicate vertices and faces of the input mesh.
        /*!
            \param[in] mesh CgalMesh_EPICK.
        */
        bool makeAsCleanAsPossible(CgalMesh_EPICK &mesh);

        //! File holes in the mesh.
        bool fillHoles();

        //! Get the stored mesh.
        /*!
            \return \b CgalMesh_EPICK or \b CgalMesh_EPECK.
        */
		T getMesh() const;

        //! Save mesh to .stl file
        /*!
            \param[in] file_path Path of the .stl file.
        */
		bool save(std::string file_path);

        //! Loading mesh from a given file path
        /*!
            \param[in] file_path Path of the .stl file.
        */
        bool load(std::string file_path);

        //! Copy assignment operator.
		Polyhedron &operator = (const Polyhedron &other);

	protected:
        // Store the mesh.
		T m_mesh;
	};

	template<class T>
	inline Polyhedron<T>::Polyhedron()
	{
	}

	template<class T>
	inline Polyhedron<T>::Polyhedron(const Polyhedron &other)
	{
		*this = other;
	}

	template<class T>
	inline Polyhedron<T>::Polyhedron(std::string file_path)
	{
		this->load(file_path);
	}

	template<class T>
	inline Polyhedron<T>::Polyhedron(const T &mesh)
	{
		this->m_mesh = mesh;
	}

	template<class T>
	inline Polyhedron<T>::~Polyhedron()
	{
	}

    template<class T>
    inline bool Polyhedron<T>::repaireSelfIntersection()
    {
        return false;
    }

    template<>
    inline bool Polyhedron<CgalMesh_EPICK>::repaireSelfIntersection()
    {
        if (CGAL::Polygon_mesh_processing::does_self_intersect<CGAL::Parallel_if_available_tag>(this->m_mesh, CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, this->m_mesh))))
        {
            spdlog::info("Self-intersection detected, try to fix it!");
            CGAL::Polygon_mesh_processing::experimental::remove_self_intersections(this->m_mesh, CGAL::parameters::preserve_genus(true));
            if (CGAL::Polygon_mesh_processing::does_self_intersect<CGAL::Parallel_if_available_tag>(this->m_mesh, CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, this->m_mesh))))
            {
                spdlog::info("Self-intersection cannot be fixed!");
                return false;
            }
        }
        return true;
    }

    template<>
    inline bool Polyhedron<CgalMesh_EPICK>::makeAsCleanAsPossible()
    {
        struct Array_traits
        {
            struct Equal_3
            {
                bool operator()(const std::array<EPICK::FT, 3> &p, const std::array<EPICK::FT, 3> &q) const {
                    return p == q;
                }
            };
            struct Less_xyz_3
            {
                bool operator()(const std::array<EPICK::FT, 3> &p, const std::array<EPICK::FT, 3> &q) const {
                    return std::lexicographical_compare(p.begin(), p.end(), q.begin(), q.end());
                }
            };
            Equal_3 equal_3_object() const { return Equal_3(); }
            Less_xyz_3 less_xyz_3_object() const { return Less_xyz_3(); }
        };

        this->m_mesh.collect_garbage();

        std::vector<std::array<EPICK::FT, 3> > points;
        std::vector<std::vector<std::size_t>> polygons;

        CGAL::Polygon_mesh_processing::polygon_mesh_to_polygon_soup(this->m_mesh, points, polygons);

        CgalMesh_EPICK repaired_mesh;

        struct Visitor : public CGAL::Polygon_mesh_processing::Default_orientation_visitor
        {
            void non_manifold_edge(std::size_t id1, std::size_t id2, std::size_t nb_poly)
            {
                std::cout << std::setprecision(17);
                std::cout << "The edge " << id1 << ", " << id2 << " is not manifold: " << nb_poly << " incident polygons." << std::endl;
                std::cout << m_points[id1][0] << " ";
                std::cout << m_points[id1][1] << " ";
                std::cout << m_points[id1][2] << std::endl;
                std::cout << m_points[id2][0] << " ";
                std::cout << m_points[id2][1] << " ";
                std::cout << m_points[id2][2] << std::endl;
            }
            void non_manifold_vertex(std::size_t id, std::size_t nb_cycles)
            {
                std::cout << "The vertex " << id << " is not manifold: " << nb_cycles << " connected components of vertices in the link." << std::endl;
            }
            void duplicated_vertex(std::size_t v1, std::size_t v2)
            {
                std::cout << "The vertex " << v1 << " has been duplicated, its new id is " << v2 << "." << std::endl;
            }
            void vertex_id_in_polygon_replaced(std::size_t p_id, std::size_t i1, std::size_t i2)
            {
                std::cout << "In the polygon " << p_id << ", the index " << i1 << " has been replaced by " << i2 << "." << std::endl;
                std::cout << m_points[m_polygons[p_id][0]][0] << " " << m_points[m_polygons[p_id][0]][1] << " " << m_points[m_polygons[p_id][0]][2] << std::endl;
                std::cout << m_points[m_polygons[p_id][1]][0] << " " << m_points[m_polygons[p_id][1]][1] << " " << m_points[m_polygons[p_id][1]][2] << std::endl;
                std::cout << m_points[m_polygons[p_id][2]][0] << " " << m_points[m_polygons[p_id][2]][1] << " " << m_points[m_polygons[p_id][2]][2] << std::endl;
                std::cout << std::endl;
            }
            void polygon_orientation_reversed(std::size_t p_id)
            {
                std::cout << "The polygon " << p_id << " has been reversed." << std::endl;
            }
            Visitor(std::vector<std::array<EPICK::FT, 3> > points, std::vector<std::vector<std::size_t>> polygons)
            {
                m_points = points;
                m_polygons = polygons;
            };

            std::vector<std::array<EPICK::FT, 3>> m_points;
            std::vector<std::vector<std::size_t>> m_polygons;
        };


        if (!CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons))
        {
            spdlog::info("orient_polygon_soup() return false for the first time!");
        }

        CGAL::Polygon_mesh_processing::repair_polygon_soup(points, polygons, CGAL::parameters::geom_traits(Array_traits()));
        //CGAL::Polygon_mesh_processing::orient_triangle_soup_with_reference_triangle_mesh(mesh, points, polygons);

        if (!CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons))
        {
            spdlog::info("orient_polygon_soup() return false for the second time!");
            return false;
        }

        CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons, repaired_mesh);

        this->m_mesh = repaired_mesh;
        return true;
    }

    template<class T>
    inline bool Polyhedron<T>::makeAsCleanAsPossible(CgalMesh_EPICK &mesh)
    {
        return false;
    }

    template<>
    inline bool Polyhedron<CgalMesh_EPECK>::makeAsCleanAsPossible(CgalMesh_EPICK &mesh)
    {
        struct Array_traits
        {
            struct Equal_3
            {
                bool operator()(const std::array<EPICK::FT, 3> &p, const std::array<EPICK::FT, 3> &q) const {
                    return p == q;
                }
            };
            struct Less_xyz_3
            {
                bool operator()(const std::array<EPICK::FT, 3> &p, const std::array<EPICK::FT, 3> &q) const {
                    return std::lexicographical_compare(p.begin(), p.end(), q.begin(), q.end());
                }
            };
            Equal_3 equal_3_object() const { return Equal_3(); }
            Less_xyz_3 less_xyz_3_object() const { return Less_xyz_3(); }
        };

        mesh.collect_garbage();

        std::vector<std::array<EPICK::FT, 3> > points;
        std::vector<std::vector<std::size_t>> polygons;

        CGAL::Polygon_mesh_processing::polygon_mesh_to_polygon_soup(mesh, points, polygons);

        CgalMesh_EPICK repaired_mesh;

        struct Visitor : public CGAL::Polygon_mesh_processing::Default_orientation_visitor
        {
            void non_manifold_edge(std::size_t id1, std::size_t id2, std::size_t nb_poly)
            {
                std::cout << std::setprecision(17);
                std::cout << "The edge " << id1 << ", " << id2 << " is not manifold: " << nb_poly << " incident polygons." << std::endl;
                std::cout << m_points[id1][0] << " ";
                std::cout << m_points[id1][1] << " ";
                std::cout << m_points[id1][2] << std::endl;
                std::cout << m_points[id2][0] << " ";
                std::cout << m_points[id2][1] << " ";
                std::cout << m_points[id2][2] << std::endl;
            }
            void non_manifold_vertex(std::size_t id, std::size_t nb_cycles)
            {
                std::cout << "The vertex " << id << " is not manifold: " << nb_cycles << " connected components of vertices in the link." << std::endl;
            }
            void duplicated_vertex(std::size_t v1, std::size_t v2)
            {
                std::cout << "The vertex " << v1 << " has been duplicated, its new id is " << v2 << "." << std::endl;
            }
            void vertex_id_in_polygon_replaced(std::size_t p_id, std::size_t i1, std::size_t i2)
            {
                std::cout << "In the polygon " << p_id << ", the index " << i1 << " has been replaced by " << i2 << "." << std::endl;
                std::cout << m_points[m_polygons[p_id][0]][0] << " " << m_points[m_polygons[p_id][0]][1] << " " << m_points[m_polygons[p_id][0]][2] << std::endl;
                std::cout << m_points[m_polygons[p_id][1]][0] << " " << m_points[m_polygons[p_id][1]][1] << " " << m_points[m_polygons[p_id][1]][2] << std::endl;
                std::cout << m_points[m_polygons[p_id][2]][0] << " " << m_points[m_polygons[p_id][2]][1] << " " << m_points[m_polygons[p_id][2]][2] << std::endl;
                std::cout << std::endl;
            }
            void polygon_orientation_reversed(std::size_t p_id)
            {
                std::cout << "The polygon " << p_id << " has been reversed." << std::endl;
            }
            Visitor(std::vector<std::array<EPICK::FT, 3> > points, std::vector<std::vector<std::size_t>> polygons)
            {
                m_points = points;
                m_polygons = polygons;
            };

            std::vector<std::array<EPICK::FT, 3>> m_points;
            std::vector<std::vector<std::size_t>> m_polygons;
        };


        if (!CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons))
        {
            spdlog::info("orient_polygon_soup() return false for the first time!");
        }

        CGAL::Polygon_mesh_processing::repair_polygon_soup(points, polygons, CGAL::parameters::geom_traits(Array_traits()));
        //CGAL::Polygon_mesh_processing::orient_triangle_soup_with_reference_triangle_mesh(mesh, points, polygons);

        if (!CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons))
        {
            spdlog::info("orient_polygon_soup() return false for the second time!");
            //return false;
        }

        CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons, repaired_mesh);

        mesh = repaired_mesh;
        return true;
    }

    template<class T>
    inline bool Polyhedron<T>::fillHoles()
    {
        if (PMP::does_self_intersect(this->m_mesh))
        {
            std::cout << "Mesh is self-intersected, hole filling aborted!" << std::endl;
            return false;
        }

        unsigned int nb_holes = 0;
        std::vector<boost::graph_traits<CgalMesh_EPICK>::halfedge_descriptor> border_cycles;
        PMP::extract_boundary_cycles(this->m_mesh, std::back_inserter(border_cycles));

        for (boost::graph_traits<CgalMesh_EPICK>::halfedge_descriptor h : border_cycles)
        {
            std::vector<boost::graph_traits<CgalMesh_EPICK>::face_descriptor>  patch_facets;
            std::vector<boost::graph_traits<CgalMesh_EPICK>::vertex_descriptor> patch_vertices;
            PMP::triangulate_and_refine_hole(this->m_mesh,
                h,
                std::back_inserter(patch_facets),
                std::back_inserter(patch_vertices));
            ++nb_holes;
        }
        return true;
    }

	template<class T>
	inline T Polyhedron<T>::getMesh() const
	{
		return this->m_mesh;
	}

	template<class T>
	inline bool Polyhedron<T>::save(std::string file_path)
	{
		if (!CGAL::IO::write_polygon_mesh(file_path, this->m_mesh))
		{
			spdlog::info("Polyhedron::writeEPECKMeshToSTL: Fail!");
			return false;
		}
		return true;
	}

    template<class T>
    inline bool Polyhedron<T>::load(std::string file_path)
    {
        std::size_t found_0 = file_path.find(".stl");
        std::size_t found_1 = file_path.find(".STL");

        if (found_0 != std::string::npos || found_1 != std::string::npos)
        {
            if (CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(file_path, this->m_mesh))
            {
                spdlog::info("Read .stl to m_mesh OK");
                return true;
            }
            return false;
        }
        else
        {
            return false;
        }
    }

	template<class T>
	inline Polyhedron<T> &Polyhedron<T>::operator=(const Polyhedron &other)
	{
		this->m_mesh = other.m_mesh;
		return *this;
	}
}

#endif  // SYNSLICERENGINE_OBJECT_POLYHEDRON_H_