#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find closest start and end nodes to coordinates given
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Determine what options we have for our next node by opening the neighbors of the current node and adding them to the open list
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for(auto node : current_node->neighbors) {
        if (node->visited) {
            continue; // If we've already visited this node, don't add it to the open list again
        }
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + node->distance(*current_node); // is this right too?
        node->visited = true;
        open_list.push_back(node);
    }
}

bool Compare(const RouteModel::Node *a, const RouteModel::Node *b) {
    return (a->g_value + a->h_value) > (b->g_value + b->h_value);
}


// Determine the next node in our path
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node* next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}


// Take the final node and generate a path based on that node's parents.  Also calculate total distance
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    bool at_start = false;
    while(current_node != start_node) {
        // Could use insert here, but that's less memory efficient, so we're going to use push_back then reverse
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }

    path_found.push_back(*current_node); // Add the starting node to our path
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Initialize the starting node
    current_node = start_node;
    current_node->h_value = CalculateHValue(current_node);
    current_node->g_value = 0;
    current_node->visited = true;
    current_node->parent = current_node;
    open_list.push_back(current_node);

    // Actually do the search
    while(open_list.size() > 0) {
        current_node = NextNode();
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
}
