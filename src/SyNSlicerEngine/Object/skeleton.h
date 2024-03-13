#ifndef SYNSLICERENGINE_OBJECT_SKELETON_H_
#define SYNSLICERENGINE_OBJECT_SKELETON_H_

#include <CGAL_CORE_CLASS>

#include "spdlog/spdlog.h"

#include "plane.h"

namespace SO = SyNSlicerEngine::Object;

namespace SyNSlicerEngine::Object
{
    class SkeletonNode
    {
    public:

        SkeletonNode(const int input, const Skeleton_EPICK &skeleton);
        SkeletonNode(const Skeleton_EPICK &skeleton, const SO::Plane &plane);
        SkeletonNode(const Skeleton_EPICK &skeleton, const CgalMesh_EPICK &mesh, const SO::Plane &plane);
        ~SkeletonNode();

        bool addNode(int point_1, int point_2);

        SkeletonNode *findNode(int point);
        SkeletonNode *findParent(SkeletonNode *child_node);

        void findNodesHaveNoChildren(std::vector<SkeletonNode *> &nodes_has_no_children);

        bool findPath(std::vector<int> &arr, SkeletonNode *end);
        std::vector<std::vector<int>> findPathsStartedFromNode(SkeletonNode *start);

        std::vector<int> getLongestPath();

        int m_point;
        std::vector<SkeletonNode *> m_nodes;
        const Skeleton_EPICK &m_skeleton;

    private:
        double getLength(const std::vector<int> &path);

    };

    class Skeleton
    {
    public:
        Skeleton(const Skeleton_EPICK &skeleton, const Plane &plane);
        Skeleton(const Skeleton_EPICK &skeleton, const CgalMesh_EPICK &mesh, const Plane &plane);
        ~Skeleton();

        Skeleton_EPICK::vertex_descriptor findSkeletonVertex(const CgalMesh_EPICK::Vertex_index &vertex);
        std::vector<CgalMesh_EPICK::Vertex_index> getNeighourVertices(const CgalMesh_EPICK::Vertex_index &vertex);
        std::vector<Skeleton_EPICK::vertex_descriptor> getEndPoints();
        std::vector<CgalMesh_EPICK::Vertex_index> findEndVertices();

        Skeleton_EPICK m_skeleton;
        SkeletonNode *skeleton_root;
    };
};

#endif  // SYNSLICERENGINE_OBJECT_SKELETON_H_