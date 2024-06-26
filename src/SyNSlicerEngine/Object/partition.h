#ifndef SYNSLICERENGINE_OBJECT_PARTITION_H_
#define SYNSLICERENGINE_OBJECT_PARTITION_H_

#include "CGAL/Surface_mesh.h"

#include "Object/polygon_collection.h"
#include "Object/polyhedron.h"
#include "Object/printing_layer_collection.h"

namespace SyNSlicerEngine::Object
{
	//! This class is used describe the 3D model after partitioning.
	/*!
		The difference between this and the superclass Model is that it stores members. \n
		(1) A contour obtain during parititioning used to calculate printing sequence. \n
		(2) Partition that should be printed before this partition does. \n
		(3) All printing layers obtained after slicing this partition.
	*/

	template <class T>
	class Partition : public Polyhedron<T>
	{
		using EigenPoint = Eigen::Vector3d;

	public:
        //! Default constructor.
		Partition();

        //! Copy constructor.
		Partition(const Partition<T> &other);

        //! Construct this by inputting a cgal mesh.
        /*!
            \param[in] mesh Cgal mesh, CgalMesh_EPICK or CgalMesh_EPECK.
        */
        Partition(const T &mesh);

        //! Construct this by inputting a .stl file.
        /*!
            \param[in] file_path System path of the file.
        */
		Partition(std::string file_path);

        //! Destructor.
		~Partition();

        //! Set base plane.
        /*!
            \param[in] base_plane Base plane.
        */
		void setBasePlane(const SO::Plane &base_plane);

        //! Get base plane.
        /*!
            \return Base plane.
        */
        SO::Plane getBasePlane();

        //! Get base contours.
        /*!
            \return Base contours.
        */
        PolygonCollection &getBaseContours();

        //! Add key.
        /*!
            Key is used to determine printing sequence. Key should be printed before lock.
            \param[in]  key     Key.
        */
        void addKey(int key);

        // Get keys.
        /*!
            \return     All keys.
        */
        std::vector<int> getKeys();

        //! Add lock.
        /*!
            Lock is used to determine printing sequence. Lock should be printed after key.
            \param[in]  lock     Lock.
        */
        void addLock(int lock);

        // Get Locks.
        /*!
            \return     All locks.
        */
        std::vector<int> getLocks();

        //! Get EPICK mesh.
        /*!
            \return     CgalMesh_EPICK.
        */
        CgalMesh_EPICK getEPICKMesh();

        //! Get EPECK mesh.
        /*!
            \return     CgalMesh_EPECK.
        */
        CgalMesh_EPECK getEPECKMesh();

        //! Set printing layers.
        /*!
            \param[in]  printing_layers     Printing layers.
        */
        void setPrintingLayers(const SO::PrintingLayerCollection &printing_layers);

        //! Get EPECK mesh.
        /*!
            \return     SO::PrintingLayerCollection.
        */
        SO::PrintingLayerCollection &getPrintingLayers();

        //! Copy assignment operators
		Partition<T> &operator=(const Partition<T> &other);

	protected:
        //! Calculate base contours.
		void determineBaseContours();

        //! Store base plane.
		SO::Plane m_base_plane;
		
        //! Store base contours.
		SO::PolygonCollection m_base_contours;

        //! Store printing layers.
		SO::PrintingLayerCollection m_printing_layers;

        //! Store keys.
        std::vector<int> m_keys;

        //! Store locks.
        std::vector<int> m_locks;
	};

    template<class T>
    inline Partition<T>::Partition()
    {
    }

    template<class T>
    inline Partition<T>::Partition(const Partition<T> &other)
    {
        *this = other;
    }

    template<class T>
    inline Partition<T>::Partition(const T &mesh)
        : Polyhedron<T>(mesh)
    {
        this->setBasePlane(SO::Plane());
    }

    template<class T>
    inline Partition<T>::Partition(std::string file_path)
        : Polyhedron<T>(file_path)
    {
        this->setBasePlane(SO::Plane());
    }

    template<class T>
    inline Partition<T>::~Partition()
    {
    }

    template<class T>
    inline void Partition<T>::setBasePlane(const SO::Plane &base_plane)
    {
        this->m_mesh.collect_garbage();
        this->m_base_plane = base_plane;
        this->determineBaseContours();
    }

    template<class T>
    inline SO::Plane Partition<T>::getBasePlane()
    {
        return this->m_base_plane;
    }

    template<class T>
    inline PolygonCollection &Partition<T>::getBaseContours()
    {
        return this->m_base_contours;
    }

    template<class T>
    inline void Partition<T>::addKey(int key)
    {
        m_keys.emplace_back(key);
    }

    template<class T>
    inline std::vector<int> Partition<T>::getKeys()
    {
        return m_keys;
    }

    template<class T>
    inline void Partition<T>::addLock(int lock)
    {
        m_locks.emplace_back(lock);
    }

    template<class T>
    inline std::vector<int> Partition<T>::getLocks()
    {
        return m_locks;
    }

    template<class T>
    inline CgalMesh_EPICK Partition<T>::getEPICKMesh()
    {
        return this->m_mesh;
    }

    template<>
    inline CgalMesh_EPICK Partition<CgalMesh_EPECK>::getEPICKMesh()
    {
        CgalMesh_EPICK epick_mesh;

        for (auto fid : this->m_mesh.faces())
        {
            std::vector<CgalMesh_EPICK::Vertex_index> vids;
            for (auto vid : this->m_mesh.vertices_around_face(this->m_mesh.halfedge(fid)))
            {
                vids.emplace_back(epick_mesh.add_vertex(CgalPoint_EPICK(CGAL::to_double(this->m_mesh.point(vid).x()), CGAL::to_double(this->m_mesh.point(vid).y()), CGAL::to_double(this->m_mesh.point(vid).z()))));
            }
            epick_mesh.add_face(vids[0], vids[1], vids[2]);
        }
        
        if (!this->makeAsCleanAsPossible(epick_mesh))
        {
            return CgalMesh_EPICK();
        }

        spdlog::info("EPICK Mesh has {} vertice, {} faces.", epick_mesh.number_of_vertices(), epick_mesh.number_of_faces());

        {
            CgalMesh_EPICK &m = epick_mesh;
            Plane &plane = this->m_base_plane;

            CgalMesh_EPICK::Property_map<CgalMesh_EPICK::Vertex_index, bool> vcm =
                m.add_property_map<CgalMesh_EPICK::Vertex_index, bool>("vcm", false).first;

            for (const auto &f : m.faces())
            {
                const auto &e1 = m.halfedge(f);
                const auto &e0 = m.prev(e1);
                const auto &e2 = m.next(e1);

                std::vector<CgalMesh_EPICK::Halfedge_index> edges;
                if (plane.isLineOnPlane(SO::Line(e1, m), 1e-3))
                {
                    edges.emplace_back(e1);
                }
                if (plane.isLineOnPlane(SO::Line(e0, m), 1e-3))
                {
                    edges.emplace_back(e0);
                }
                if (plane.isLineOnPlane(SO::Line(e2, m), 1e-3))
                {
                    edges.emplace_back(e2);
                }

                if (edges.size() == 1)
                {
                    vcm[m.source(edges[0])] = true;
                    vcm[m.target(edges[0])] = true;
                }
            }

            if (!CGAL::Polygon_mesh_processing::remove_almost_degenerate_faces(epick_mesh.faces(), epick_mesh, CGAL::parameters::cap_threshold(-0.99999).needle_threshold(50)))
            {
                spdlog::info("Degen faces cannot be removed.");
            };
        }

        if (CGAL::Polygon_mesh_processing::does_self_intersect<CGAL::Parallel_if_available_tag>(epick_mesh, CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, epick_mesh))))
        {
            spdlog::info("Self-intersection detected, try to fix it.");
            CGAL::Polygon_mesh_processing::experimental::remove_self_intersections(epick_mesh, CGAL::parameters::preserve_genus(true));

            if (CGAL::Polygon_mesh_processing::does_self_intersect<CGAL::Parallel_if_available_tag>(epick_mesh, CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, epick_mesh))))
            {
                std::cout << "Using parallel mode? " << std::is_same<CGAL::Parallel_if_available_tag, CGAL::Parallel_tag>::value << std::endl;

                std::vector<std::pair<CgalMesh_EPICK::Face_index, CgalMesh_EPICK::Face_index> > intersected_tris;
                CGAL::Polygon_mesh_processing::self_intersections<CGAL::Parallel_if_available_tag>(faces(epick_mesh), epick_mesh, std::back_inserter(intersected_tris));

                spdlog::info("Self-intersection cannot be fixed!");
                return CgalMesh_EPICK();
            }
            else
            {
                CgalMesh_EPICK &m = epick_mesh;
                Plane &plane = this->m_base_plane;

                CgalMesh_EPICK::Property_map<CgalMesh_EPICK::Vertex_index, bool> vcm =
                    m.add_property_map<CgalMesh_EPICK::Vertex_index, bool>("vcm", false).first;

                for (const auto &f : m.faces())
                {
                    const auto &e1 = m.halfedge(f);
                    const auto &e0 = m.prev(e1);
                    const auto &e2 = m.next(e1);

                    std::vector<CgalMesh_EPICK::Halfedge_index> edges;
                    if (plane.isLineOnPlane(SO::Line(e1, m), 1e-3))
                    {
                        edges.emplace_back(e1);
                    }
                    if (plane.isLineOnPlane(SO::Line(e0, m), 1e-3))
                    {
                        edges.emplace_back(e0);
                    }
                    if (plane.isLineOnPlane(SO::Line(e2, m), 1e-3))
                    {
                        edges.emplace_back(e2);
                    }

                    if (edges.size() == 1)
                    {
                        vcm[m.source(edges[0])] = true;
                        vcm[m.target(edges[0])] = true;
                    }
                }

                CGAL::Polygon_mesh_processing::remove_almost_degenerate_faces(epick_mesh.faces(), epick_mesh, CGAL::parameters::cap_threshold(-0.99999).needle_threshold(50).vertex_is_constrained_map(vcm));
                spdlog::info("Self-intersection fixed!");
            }
        }

        if (!CGAL::is_closed(epick_mesh))
        {

            return CgalMesh_EPICK();
            
        }

        spdlog::info("getEPICKMesh(): Success: {} vertices removed.", this->m_mesh.number_of_vertices() - epick_mesh.number_of_vertices());

        return epick_mesh;
    }

    template<class T>
    inline CgalMesh_EPECK Partition<T>::getEPECKMesh()
    {
        return this->m_mesh;
    }

    template<>
    inline CgalMesh_EPECK Partition<CgalMesh_EPICK>::getEPECKMesh()
    {
        CgalMesh_EPECK epeck_mesh;

        CGAL::Cartesian_converter<EPICK, EPECK> to_epeck;

        for (auto vid : this->m_mesh.vertices())
        {
            epeck_mesh.add_vertex(to_epeck(this->m_mesh.point(vid)));
        }

        for (auto fid : this->m_mesh.faces())
        {
            std::vector<CgalMesh_EPECK::Vertex_index> vids;
            for (auto vid : this->m_mesh.vertices_around_face(this->m_mesh.halfedge(fid)))
            {
                vids.emplace_back(CgalMesh_EPECK::Vertex_index(vid.id()));
            }
            epeck_mesh.add_face(vids[0], vids[1], vids[2]);
        }
        if (!CGAL::is_valid(epeck_mesh))
        {
            spdlog::info("getEPECKMesh(): Fail.");
        };
        return epeck_mesh;
    }

    template<class T>
    inline void Partition<T>::setPrintingLayers(const SO::PrintingLayerCollection &printing_layers)
    {
        m_printing_layers = printing_layers;
    }

    template<class T>
    inline SO::PrintingLayerCollection &Partition<T>::getPrintingLayers()
    {
        return m_printing_layers;
    }

    template<class T>
    inline Partition<T> &Partition<T>::operator=(const Partition<T> &other)
    {
        Polyhedron<T>::operator=(other);
        this->m_base_plane = other.m_base_plane;
        this->m_base_contours = other.m_base_contours;
        this->m_printing_layers = other.m_printing_layers;
        this->m_keys = other.m_keys;
        this->m_locks = other.m_locks;
        return *this;
    }

    template<class T>
    inline void Partition<T>::determineBaseContours()
    {
       
    }

    template<>
    inline void Partition<CgalMesh_EPICK>::determineBaseContours()
    {
        this->m_base_contours.reset();

        std::vector<int> unordered_base_contour_edges;
        std::vector<bool> edge_list;
        edge_list.resize(this->m_mesh.number_of_halfedges());

        for (const auto &f : this->m_mesh.faces())
        {
            const auto &e1 = this->m_mesh.halfedge(f);
            const auto &e0 = this->m_mesh.prev(e1);
            const auto &e2 = this->m_mesh.next(e1);

            std::vector<CgalMesh_EPICK::Halfedge_index> edges;
            if (this->m_base_plane.isLineOnPlane(Line(e1, this->m_mesh), 1e-2))
            {
                edges.emplace_back(e1);
            }
            if (this->m_base_plane.isLineOnPlane(Line(e0, this->m_mesh), 1e-2))
            {
                edges.emplace_back(e0);
            }
            if (this->m_base_plane.isLineOnPlane(Line(e2, this->m_mesh), 1e-2))
            {
                edges.emplace_back(e2);
            }

            if (edges.size() == 1)
            {
                unordered_base_contour_edges.emplace_back(edges[0].id());
                edge_list[edges[0].id()] = true;
            }
        }

        CgalMesh_EPICK::Halfedge_index source_he;
        CgalMesh_EPICK::Vertex_index source_v;
        CgalMesh_EPICK::Vertex_index end_v;
        SO::Polygon contour;
        contour.setPlane(this->m_base_plane);

        while (unordered_base_contour_edges.size())
        {
            auto it = std::find(edge_list.begin(), edge_list.end(), true);
            int index_to_start = std::distance(edge_list.begin(), it);
            source_he = CgalMesh_EPICK::Halfedge_index(index_to_start);
            source_v = this->m_mesh.source(source_he);
            end_v = this->m_mesh.target(source_he);

            const auto &cgal_point = this->m_mesh.point(source_v);
            contour.addPointToBack(Eigen::Vector3d(cgal_point.x(), cgal_point.y(), cgal_point.z()));

            auto it_source_he = std::find(unordered_base_contour_edges.begin(), unordered_base_contour_edges.end(), source_he.idx());
            if (it_source_he != unordered_base_contour_edges.end())
            {
                edge_list[*it_source_he] = false;
                unordered_base_contour_edges.erase(it_source_he);
            }

            while (end_v.idx() != source_v.idx())
            {
                for (const auto &he : this->m_mesh.halfedges_around_target(source_he))
                {
                    auto it = std::find(unordered_base_contour_edges.begin(), unordered_base_contour_edges.end(), this->m_mesh.opposite(he).idx());
                    if (it != unordered_base_contour_edges.end())
                    {
                        source_he = this->m_mesh.opposite(he);
                        end_v = this->m_mesh.target(source_he);

                        const auto &cgal_point = this->m_mesh.point(this->m_mesh.source(source_he));
                        contour.addPointToBack(Eigen::Vector3d(cgal_point.x(), cgal_point.y(), cgal_point.z()));
                        edge_list[*it] = false;
                        unordered_base_contour_edges.erase(it);
                        break;
                    }
                }
            }
            m_base_contours.addPolygon(contour);
            contour.reset();
            contour.setPlane(this->m_base_plane);
        }
    }

    template<>
    inline void Partition<CgalMesh_EPECK>::determineBaseContours()
    {
        this->m_base_contours.reset();

        std::vector<int> unordered_base_contour_edges;
        std::vector<bool> edge_list;
        edge_list.resize(this->m_mesh.number_of_halfedges());

        for (const auto &f : this->m_mesh.faces())
        {
            const auto &e1 = this->m_mesh.halfedge(f);
            const auto &e0 = this->m_mesh.prev(e1);
            const auto &e2 = this->m_mesh.next(e1);

            std::vector<CgalMesh_EPICK::Halfedge_index> edges;
            if (this->m_base_plane.isLineOnPlane(Line(e1, this->m_mesh)))
            {
                edges.emplace_back(e1);
            }
            if (this->m_base_plane.isLineOnPlane(Line(e0, this->m_mesh)))
            {
                edges.emplace_back(e0);
            }
            if (this->m_base_plane.isLineOnPlane(Line(e2, this->m_mesh)))
            {
                edges.emplace_back(e2);
            }

            if (edges.size() == 1)
            {
                unordered_base_contour_edges.emplace_back(edges[0].id());
                edge_list[edges[0].id()] = true;
            }
        }

        CgalMesh_EPICK::Halfedge_index source_he;
        CgalMesh_EPICK::Vertex_index source_v;
        CgalMesh_EPICK::Vertex_index end_v;
        SO::Polygon contour;
        contour.setPlane(this->m_base_plane);

        while (unordered_base_contour_edges.size())
        {
            auto it = std::find(edge_list.begin(), edge_list.end(), true);
            int index_to_start = std::distance(edge_list.begin(), it);
            source_he = CgalMesh_EPICK::Halfedge_index(index_to_start);
            source_v = this->m_mesh.source(source_he);
            end_v = this->m_mesh.target(source_he);

            const auto &cgal_point = this->m_mesh.point(source_v);
            Eigen::Vector3d p(
                CGAL::to_double(cgal_point.x()),
                CGAL::to_double(cgal_point.y()),
                CGAL::to_double(cgal_point.z()));
            contour.addPointToBack(p);

            auto it_source_he = std::find(unordered_base_contour_edges.begin(), unordered_base_contour_edges.end(), source_he.idx());
            if (it_source_he != unordered_base_contour_edges.end())
            {
                edge_list[*it_source_he] = false;
                unordered_base_contour_edges.erase(it_source_he);
            }

            while (end_v.idx() != source_v.idx())
            {
                for (const auto &he : this->m_mesh.halfedges_around_target(source_he))
                {
                    auto it = std::find(unordered_base_contour_edges.begin(), unordered_base_contour_edges.end(), this->m_mesh.opposite(he).idx());
                    if (it != unordered_base_contour_edges.end())
                    {
                        source_he = this->m_mesh.opposite(he);
                        end_v = this->m_mesh.target(source_he);

                        const auto &cgal_point = this->m_mesh.point(this->m_mesh.source(source_he));
                        Eigen::Vector3d p(
                            CGAL::to_double(cgal_point.x()),
                            CGAL::to_double(cgal_point.y()),
                            CGAL::to_double(cgal_point.z()));
                        contour.addPointToBack(p);
                        edge_list[*it] = false;
                        unordered_base_contour_edges.erase(it);
                        break;
                    }
                }
            }
            m_base_contours.addPolygon(contour);
            contour.reset();
            contour.setPlane(this->m_base_plane);
        }
    }
}

#endif  // SYNSLICERENGINE_OBJECT_PARTITION_H_