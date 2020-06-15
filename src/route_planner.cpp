#include "route_planner.h"
#include "route_model.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float DistanceToEnd = node->distance(*end_node);
    return DistanceToEnd;
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(RouteModel::Node* n : current_node->neighbors){
        n->parent = current_node;
        n->g_value = current_node->g_value + n->distance(*current_node);
        n->h_value = CalculateHValue(n);
        open_list.push_back(n);
        n->visited = true;
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node* n1, RouteModel::Node* n2){
        return (n1->g_value + n1->h_value > n2->g_value + n2->h_value);
    });

    RouteModel::Node *pLowFNode = open_list.back();
    open_list.pop_back();
    return pLowFNode;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node node = *current_node;

    while(node.parent){
        path_found.emplace_back(node);
        distance += node.distance(*(node.parent));
        node = *node.parent;
    }
    path_found.push_back(node);

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end());
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    open_list.emplace_back(start_node);
    start_node->visited = true;

    while(!open_list.empty()){
        current_node = NextNode();

        if(current_node == end_node){
            m_Model.path = ConstructFinalPath(end_node);
            return;
        }

        AddNeighbors(current_node);
    }
}