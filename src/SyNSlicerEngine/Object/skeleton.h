#ifndef SYNSLICERENGINE_OBJECT_SKELETON_H_
#define SYNSLICERENGINE_OBJECT_SKELETON_H_

#include <CGAL_CORE_CLASS>

#include "spdlog/spdlog.h"

#include "plane.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object
{
    //! A data structure to store a skeleton.
    class SkeletonNode
    {
    public:
        //! Default constructor is not allowed.
        SkeletonNode() = delete;

        //! Construct a node with skeleton point id and the skeleton.
        /*!
            \param[in]  input       Skeleton point id.
            \param[in]  skeleton    Cgal skeleton.
        */
        SkeletonNode(const int input, const Skeleton_EPICK &skeleton);

        //! Construct a root node and all the node of a skelton.
        /*!
            \brief  The root node will be the point that are closest to the plane.
            \param[in]  skeleton    Cgal skeleton.
            \param[in]  plane       Plane.
        */
        SkeletonNode(const Skeleton_EPICK &skeleton, const SO::Plane &plane);

        //! Construct a root node and all the node of a skelton.
        /*!
            \brief  The root node will be the point associated the point in the mesh that are closest to the plane.
            \param[in]  skeleton    Cgal skeleton.
            \param[in]  mesh        Cgal mesh.
            \param[in]  plane       Plane.
        */
        SkeletonNode(const Skeleton_EPICK &skeleton, const CgalMesh_EPICK &mesh, const SO::Plane &plane);

        //! Destructor.
        ~SkeletonNode();

        //! Add a node to the root node.
        /*!
            \brief  Create a node if point_1 or point_2 is a neighbour point of one of the nodes added.
            \param[in]  point_1 Point 1.
            \param[in]  point_2 Point 2.
            \return \b True if a new node is created. \n \b False if no new node is created. 
        */
        bool addNode(int point_1, int point_2);

        //! Find the node associated with the given id.
        /*!
            \param[in]  point   Point id.
            \return <b> SkeletonNode * </b> the node address.
        */
        SkeletonNode *findNode(int point);

        //! Find the parent node by giving child node.
        /*!
            \param[in]  child_node   The child node.
            \return <b> SkeletonNode * </b> the node address.
        */
        SkeletonNode *findParent(SkeletonNode *child_node);

        //! Find all nodes have no children.
        /*!
            \param[out]  nodes_has_no_children   All nodes have no children.
        */
        void findNodesHaveNoChildren(std::vector<SkeletonNode *> &nodes_has_no_children);

        //! Find the path from root node to the given node.
        /*!
            \param[out]     arr   All the point id in the nodes in the path.
            \param[in]      end   The end node of the path.
        */
        bool findPath(std::vector<int> &arr, SkeletonNode *end);

        //! Find the path start from the given node.
        /*!
            \param[in]  start   The starting node.
            \return \b std::vector<std::vector<int>> The point id on path.
        */
        std::vector<std::vector<int>> findPathsStartedFromNode(SkeletonNode *start);

        //! Find the longest path start from the root node.
        /*!
            \return \b std::vector<int> The longest path.
        */
        std::vector<int> getLongestPath();

        //! Point id of this node.
        int m_point;

        //! Children nodes.
        std::vector<SkeletonNode *> m_nodes;

        //! The cgal skeleton.
        const Skeleton_EPICK &m_skeleton;

    protected:
        //! Get the length of the a path.
        /*!
            \param[in]  path    Point id in the path.
            \return \b double Length.
        */
        double getLength(const std::vector<int> &path);
    };

    //! A skeleton
    class Skeleton
    {
    public:
        //! Default constructor is not allowed.
        Skeleton() = delete;

        //! Constructor.
        /*!
            \param[in]  skeleton    Cgal skeleton.
            \param[in]  plane       Plane.
        */
        Skeleton(const Skeleton_EPICK &skeleton, const Plane &plane);

        //! Constructor.
        /*!
            \param[in]  skeleton    Cgal skeleton.
            \param[in]  mesh        Cgal mesh.
            \param[in]  plane       Plane.
        */
        Skeleton(const Skeleton_EPICK &skeleton, const CgalMesh_EPICK &mesh, const Plane &plane);

        //! Destructor.
        ~Skeleton();

        //! Find skeleton vertex that is associated with the given vertex in a cgal mesh.
        /*!
            \param[in]  vertex    vertex in a cgal mesh.
            \return  \b Skeleton_EPICK::vertex_descriptor   Skeleton vertex.
        */
        Skeleton_EPICK::vertex_descriptor findSkeletonVertex(const CgalMesh_EPICK::Vertex_index &vertex);

        //! Get vertices associated to the same skeleton vertex.
        /*!
            \param[in]  vertex    vertex in a cgal mesh.
            \return  \b std::vector<CgalMesh_EPICK::Vertex_index>   Vertices in cgal mesh.
        */
        std::vector<CgalMesh_EPICK::Vertex_index> getNeighourVertices(const CgalMesh_EPICK::Vertex_index &vertex);

        //! Get vertices on skeleton that have only one neighbour.
        /*!
            \return  \b std::vector<Skeleton_EPICK::vertex_descriptor>   Vertices on skeleton.
        */
        std::vector<Skeleton_EPICK::vertex_descriptor> getEndPoints();

        //! Get vertices on cgal mesh that are associated with the end vertices on a skeleton.
        /*!
            \return  \b std::vector<CgalMesh_EPICK::Vertex_index>   Vertices on cgal mesh.
        */
        std::vector<CgalMesh_EPICK::Vertex_index> findEndVertices();

        //! Store the skeleton.
        Skeleton_EPICK m_skeleton;

        //! Store the root node of a skeleton.
        SkeletonNode *skeleton_root;
    };
};

#endif  // SYNSLICERENGINE_OBJECT_SKELETON_H_