#include "skeleton.h"

using SyNSlicerEngine::Object::Skeleton;
using SyNSlicerEngine::Object::SkeletonNode;

SkeletonNode::SkeletonNode(const int input, const Skeleton_EPICK &skeleton)
    : m_point(input)
    , m_skeleton(skeleton)
{
};

SkeletonNode::SkeletonNode(const Skeleton_EPICK &skeleton, const SO::Plane &plane)
    : m_skeleton(skeleton)
{
    double distance = std::numeric_limits<double>::max();
    Skeleton_EPICK::vertex_descriptor closest_points;
    for (const Skeleton_EPICK::vertex_descriptor &v : CGAL::make_range(vertices(skeleton)))
    {
        Eigen::Vector3d temp_point(skeleton[v].point.x(), skeleton[v].point.y(), skeleton[v].point.z());
        double temp_distance = plane.getDistanceFromPointToPlane(temp_point);
        if (temp_distance < distance)
        {
            distance = temp_distance;
            closest_points = v;
        };
    }
    m_point = closest_points;

    std::vector<std::pair<int, int>> lines;
    for (const Skeleton_edge &e : CGAL::make_range(edges(skeleton)))
    {
        lines.emplace_back(std::make_pair<int, int>(source(e, skeleton), target(e, skeleton)));
    }

    while (lines.size())
    {
        for (int i = 0; i < lines.size(); i++)
        {
            if (this->addNode(lines[i].first, lines[i].second))
            {
                lines.erase(lines.begin() + i);
                break;
            }
        }
    }
}

SkeletonNode::SkeletonNode(const Skeleton_EPICK &skeleton, const CgalMesh_EPICK &mesh, const SO::Plane &plane)
    : m_skeleton(skeleton)
{
    auto find_root_vertex = [skeleton, mesh, plane]() {
        for (const auto &v : CGAL::make_range(vertices(skeleton)))
        {
            for (auto &vd : skeleton[v].vertices)
            {
                if (plane.isPointOnPlane(mesh.point(vd)))
                {
                    return v;
                }
            }
        }
    };

    m_point = find_root_vertex();

    std::vector<std::pair<int, int>> lines;
    for (const Skeleton_edge &e : CGAL::make_range(edges(skeleton)))
    {
        lines.emplace_back(std::make_pair<int, int>(source(e, skeleton), target(e, skeleton)));
    }

    
    while (lines.size())
    {
        bool exit_by_break = false;
        for (int i = 0; i < lines.size(); i++)
        {
            if (this->addNode(lines[i].first, lines[i].second))
            {
                lines.erase(lines.begin() + i);
                exit_by_break = true;
                break;
            }
        }
        if (exit_by_break == false)
        {
            spdlog::error("Skeleton split into 2 parts!");
            goto block_1;
        }
    }

block_1:
    int a = 0;
}

SkeletonNode::~SkeletonNode()
{
    for (auto node : m_nodes)
    {
        delete node;
    };
};

bool SkeletonNode::addNode(int point_1, int point_2)
{
    SkeletonNode *node_1 = findNode(point_1);
    if (node_1 != nullptr)
    {
        node_1->m_nodes.emplace_back(new SkeletonNode(point_2, m_skeleton));
        return true;
    };
    SkeletonNode *node_2 = findNode(point_2);
    if (node_2 != nullptr)
    {
        node_2->m_nodes.emplace_back(new SkeletonNode(point_1, m_skeleton));
        return true;
    };
    return false;
};

SkeletonNode *SkeletonNode::findNode(int point)
{
    SkeletonNode *pointer_to_return = nullptr;
    if (this->m_point == point)
    {
        pointer_to_return = this;
    }
    else
    {
        for (auto node : m_nodes)
        {
            pointer_to_return = node->findNode(point);
            if (pointer_to_return != nullptr)
            {
                return pointer_to_return;
            }
        };
    }
    return pointer_to_return;
}

SkeletonNode *SkeletonNode::findParent(SkeletonNode *child_node)
{
    for (auto node : m_nodes)
    {
        if (node == child_node)
        {
            return this;
        }
        else
        {
            node->findParent(child_node);
        }
    }
    return nullptr;
}

void SkeletonNode::findNodesHaveNoChildren(std::vector<SkeletonNode *> &nodes_has_no_children)
{
    if (this->m_nodes.size() == 0)
    {
        nodes_has_no_children.emplace_back(this);
    }
    for (auto node : m_nodes)
    {
        node->findNodesHaveNoChildren(nodes_has_no_children);
    };
}

bool SkeletonNode::findPath(std::vector<int> &arr, SkeletonNode *end)
{
    arr.emplace_back(this->m_point);

    if (this == end)
    {
        return true;
    }

    for (auto node : m_nodes)
    {
        if (node->findPath(arr, end))
        {
            return true;
        };
    };

    arr.pop_back();
    return false;
}

std::vector<std::vector<int>> SkeletonNode::findPathsStartedFromNode(SkeletonNode *start)
{
    std::vector<SkeletonNode *> nodes_has_no_children;
    start->findNodesHaveNoChildren(nodes_has_no_children);

    std::vector<std::vector<int>> paths;
    for (auto pointer : nodes_has_no_children)
    {
        std::vector<int> temp_path;
        start->findPath(temp_path, pointer);
        paths.emplace_back(temp_path);
    };

    return paths;
}

std::vector<int> SkeletonNode::getLongestPath()
{
    std::vector<SkeletonNode *> nodes_has_no_children_0;
    this->findNodesHaveNoChildren(nodes_has_no_children_0);

    std::vector<std::vector<int>> paths;
    for (auto pointer : nodes_has_no_children_0)
    {
        std::vector<int> temp_path;
        this->findPath(temp_path, pointer);
        paths.emplace_back(temp_path);
    };

    double distance = std::numeric_limits<double>::min();
    int index = 0;
    for (int i = 0; i < paths.size(); ++i)
    {
        double temp_length = this->getLength(paths[i]);
        if (temp_length > distance)
        {
            distance = temp_length;
            index = i;
        };
    };
    return paths[index];
};

double SkeletonNode::getLength(const std::vector<int> &path)
{
    double distance = 0.0;
    int j = 0;
    for (int i = 1; i < path.size(); i++)
    {
        auto cgal_pi = m_skeleton[path[i]].point;
        auto cgal_pj = m_skeleton[path[j]].point;
        Eigen::Vector3d pi(cgal_pi.x(), cgal_pi.y(), cgal_pi.z());
        Eigen::Vector3d pj(cgal_pj.x(), cgal_pj.y(), cgal_pj.z());
        distance += (pi - pj).norm();
        j = i;
    }

    return distance;
};

Skeleton::Skeleton(const Skeleton_EPICK &skeleton, const Plane &plane)
    : m_skeleton(skeleton)
    , skeleton_root(new SkeletonNode(skeleton, plane))
{
}

Skeleton::Skeleton(const Skeleton_EPICK &skeleton, const CgalMesh_EPICK &mesh, const Plane &plane)
    : m_skeleton(skeleton)
    , skeleton_root(new SkeletonNode(skeleton, mesh, plane))
{
}

Skeleton::~Skeleton()
{
}

Skeleton_EPICK::vertex_descriptor Skeleton::findSkeletonVertex(const CgalMesh_EPICK::Vertex_index &vertex)
{
    Skeleton_EPICK::vertex_descriptor target_vertex;
    bool vertex_found = false;
    for (Skeleton_EPICK::vertex_descriptor v : CGAL::make_range(vertices(m_skeleton)))
    {
        for (CgalMesh_EPICK::Vertex_index vd : m_skeleton[v].vertices)
        {
            if (vd.id() == vertex.id())
            {
                target_vertex = v;
                vertex_found = true;
            }
        }
    }
    if (vertex_found == false)
    {
        spdlog::error("Target vertex not found!");
    }
    return target_vertex;
}

std::vector<CgalMesh_EPICK::Vertex_index> Skeleton::getNeighourVertices(const CgalMesh_EPICK::Vertex_index &vertex)
{
    Skeleton_EPICK::vertex_descriptor target_vertex = this->findSkeletonVertex(vertex);

    std::vector<CgalMesh_EPICK::Vertex_index> result;
    for (CgalMesh_EPICK::Vertex_index vd : m_skeleton[target_vertex].vertices)
    {
        result.emplace_back(vd);
    }
    return result;
}

std::vector<Skeleton_EPICK::vertex_descriptor> Skeleton::getEndPoints()
{
    std::vector<SkeletonNode *> nodes_has_no_children_0;
    skeleton_root->findNodesHaveNoChildren(nodes_has_no_children_0);

    std::vector<Skeleton_EPICK::vertex_descriptor> result;
    for (auto end : nodes_has_no_children_0)
    {
        result.emplace_back(end->m_point);
    }
    return result;
}

std::vector<CgalMesh_EPICK::Vertex_index> Skeleton::findEndVertices()
{
    std::vector<CgalMesh_EPICK::Vertex_index> result;

    auto end_points = getEndPoints();

    for (auto end: end_points)
    {
        for (CgalMesh_EPICK::Vertex_index vd : m_skeleton[Skeleton_EPICK::vertex_descriptor(end)].vertices)
        {
            result.emplace_back(vd);
        }
    }
    return result;
}
