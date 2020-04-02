#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the given targets
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
    // TODO: Error handling if there are no nodes?
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();
    for (RouteModel::Node *node : current_node->neighbors)
    {
        if (!node->visited)
        {
            node->parent = current_node;
            // TODO: How to pass node pointer as const?
            node->h_value = CalculateHValue(node);
            node->g_value = current_node->g_value + node->distance(*current_node);
            // TODO: Why does this complain? Adding a Node* to a vector of Node*
            open_list.push_back(node);
            node->visited = true;
        }
    }
}

RouteModel::Node *RoutePlanner::NextNode()
{
    sort(
        open_list.begin(),
        open_list.end(),
        [](RouteModel::Node *a, RouteModel::Node *b) -> bool {
            return a->g_value + a->h_value > b->g_value + b->h_value;
        });
    RouteModel::Node *nextNode = open_list.back();
    open_list.pop_back();
    return nextNode;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    // Create path_found vector
    distance = 0.0f; // Is this necessary? Isn't the g_value what we need?
    std::vector<RouteModel::Node> path_found;

    while (current_node != nullptr)
    {
        path_found.push_back(*current_node);
        if (current_node->parent != nullptr)
        {
            distance += current_node->distance(*current_node->parent);
        }
        current_node = current_node->parent;
    }
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = nullptr;

    open_list.push_back(start_node);
    while (open_list.size() > 0)
    {
        current_node = NextNode();
        if (current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
}