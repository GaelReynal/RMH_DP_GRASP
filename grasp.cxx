
#include <algorithm>
#include <chrono>
#include <iostream>
#include <random>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "vrpsdSpp/grasp.h"
#include "vrpsdSpp/key.h"

GRASP_VRPSD::GRASP_VRPSD(int n_clients, int cap, std::vector<int> demands, 
                    std::vector<std::vector<int>> distances, 
                    std::unordered_map<Key, double, KeyHash> costs, 
                    int seed) 
            : n_clients(n_clients), cap(cap), demands(demands), 
            distances(distances), costs(costs), rng(seed) {}

double GRASP_VRPSD::get_cost(std::vector<int> route){
    double cost = 0;
    int load = 0;
    for(size_t i = 1; i < route.size(); i++){
        try {
            cost += costs.at({route[i-1], route[i], load});
        } catch (const std::out_of_range& e) {
            costs[{route[i-1], route[i], load}] = 1e10;
            cost += costs.at({route[i-1], route[i], load});
        }
        load += demands[route[i]];
    }
    return cost;
}

double GRASP_VRPSD::get_sol_cost(std::vector<std::vector<int>> solution){
    double cost = 0;
    for(std::vector<int> route : solution) cost += get_cost(route);
    return cost;
}

std::vector<int> GRASP_VRPSD::build_tsp_rnn(std::unordered_set<int> visit) {
    std::unordered_set<int> to_visit;
    for (int i = 1; i < n_clients; ++i) to_visit.insert(i);
    for (int i : visit) to_visit.erase(i);
    std::vector<int> tour = {0};
    
    while (!to_visit.empty()) {
        int last = tour.back();
        std::vector<std::pair<int, int>> sorted_neighbors;
        
        for (int client : to_visit) sorted_neighbors.emplace_back(distances[last][client], client);
        int k_max = std::min(3, static_cast<int>(to_visit.size()));
        std::uniform_int_distribution<int> dist(1, k_max);
        int k = dist(rng) - 1;
        std::nth_element(sorted_neighbors.begin(), sorted_neighbors.begin() + k, sorted_neighbors.end());
        int chosen_client = sorted_neighbors[k].second;
        
        tour.push_back(chosen_client);
        to_visit.erase(chosen_client);
    } 
    return tour;
}

std::vector<int> GRASP_VRPSD::build_tsp_rni(std::unordered_set<int> visit) {
    std::set<int> to_visit;
    for (int i = 1; i < n_clients; ++i)to_visit.insert(i);
    for (int i : visit) to_visit.erase(i);

    std::uniform_int_distribution<int> dist(0, to_visit.size() - 1);
    int random_index = dist(rng);
    auto it = to_visit.begin();
    std::advance(it, random_index);
    int first_client = *it;
    to_visit.erase(first_client);
    
    std::vector<int> tour = {0, first_client, 0};
    
    while (!to_visit.empty()) {
        int k_max = std::min(6, static_cast<int>(to_visit.size()));
        std::uniform_int_distribution<int> dist(1, k_max);
        int chosen_index = dist(rng) - 1;
        
        std::vector<std::pair<int, int>> closest_clients;
        for (int client : to_visit) {
            int min_dist = std::numeric_limits<int>::max();
            for (int visited : tour) min_dist = std::min(min_dist, distances[visited][client]);
            closest_clients.emplace_back(min_dist, client);
        }
        
        std::nth_element(closest_clients.begin(), closest_clients.begin() + chosen_index, closest_clients.end());
        int chosen_client = closest_clients[chosen_index].second;
        
        int best_position = 1;
        int best_cost_increase = std::numeric_limits<int>::max();
        for (size_t i = 1; i < tour.size(); ++i) {
            int cost_increase = distances[tour[i - 1]][chosen_client] + distances[chosen_client][tour[i]] - distances[tour[i - 1]][tour[i]];
            if (cost_increase < best_cost_increase) {
                best_cost_increase = cost_increase;
                best_position = i;
            }
        }
        
        tour.insert(tour.begin() + best_position, chosen_client);
        to_visit.erase(chosen_client);
    }
    
    tour.pop_back();
    return tour;
}

std::vector<int> GRASP_VRPSD::build_tsp_rfi(std::unordered_set<int> visit) {
    std::set<int> to_visit;
    for (int i = 1; i < n_clients; ++i)to_visit.insert(i);
    for (int i : visit) to_visit.erase(i);

    std::uniform_int_distribution<int> dist(0, to_visit.size() - 1);
    int random_index = dist(rng);
    auto it = to_visit.begin();
    std::advance(it, random_index);
    int first_client = *it;
    to_visit.erase(first_client);
    
    std::vector<int> tour = {0, first_client, 0};
    
    while (!to_visit.empty()) {
        int k_max = std::min(6, static_cast<int>(to_visit.size()));
        std::uniform_int_distribution<int> dist(1, k_max);
        int chosen_index = dist(rng) - 1;
        
        std::vector<std::pair<int, int>> closest_clients;
        for (int client : to_visit) {
            int min_dist = std::numeric_limits<int>::min();
            for (int visited : tour) min_dist = std::max(min_dist, distances[visited][client]);
            closest_clients.emplace_back(min_dist, client);
        }
        
        std::nth_element(closest_clients.begin(), closest_clients.begin() + chosen_index, closest_clients.end());
        int chosen_client = closest_clients[chosen_index].second;
        
        int best_position = 1;
        int best_cost_increase = std::numeric_limits<int>::max();
        for (size_t i = 1; i < tour.size(); ++i) {
            int cost_increase = distances[tour[i - 1]][chosen_client] + distances[chosen_client][tour[i]] - distances[tour[i - 1]][tour[i]];
            if (cost_increase < best_cost_increase) {
                best_cost_increase = cost_increase;
                best_position = i;
            }
        }
        
        tour.insert(tour.begin() + best_position, chosen_client);
        to_visit.erase(chosen_client);
    }
    
    tour.pop_back();
    return tour;
}

std::vector<int> GRASP_VRPSD::build_tsp_rbi(std::unordered_set<int> visit) {
    std::set<int> to_visit;
    for (int i = 1; i < n_clients; ++i)to_visit.insert(i);
    for (int i : visit) to_visit.erase(i);

    std::uniform_int_distribution<int> dist(0, to_visit.size() - 1);
    int random_index = dist(rng);
    auto it = to_visit.begin();
    std::advance(it, random_index);
    int first_client = *it;
    to_visit.erase(first_client);
    
    std::vector<int> tour = {0, first_client, 0};
    
    while (!to_visit.empty()) {
        int k_max = std::min(6, static_cast<int>(to_visit.size()));
        std::uniform_int_distribution<int> dist(1, k_max);
        int chosen_index = dist(rng) - 1;
        
        std::vector<std::tuple<int, int, size_t>> best_insertions;
        for (int client : to_visit) {
            int best_position = 1;
            int best_cost_increase = std::numeric_limits<int>::max();
            for (size_t i = 1; i < tour.size(); ++i) {
                int cost_increase = distances[tour[i - 1]][client] + distances[client][tour[i]] - distances[tour[i - 1]][tour[i]];
                if (cost_increase < best_cost_increase) {
                    best_cost_increase = cost_increase;
                    best_position = i;
                }
            }
            best_insertions.emplace_back(best_cost_increase, client, best_position);
        }
        
        std::nth_element(best_insertions.begin(), best_insertions.begin() + chosen_index, best_insertions.end());
        int chosen_client = std::get<1>(best_insertions[chosen_index]);
        size_t best_position = std::get<2>(best_insertions[chosen_index]);
        
        tour.insert(tour.begin() + best_position, chosen_client);
        to_visit.erase(chosen_client);
    }
    
    tour.pop_back();
    return tour;
}

std::vector<std::vector<int>> GRASP_VRPSD::build_tsp_all(std::unordered_set<int> visit) {
    return {build_tsp_rnn(visit), build_tsp_rni(visit), build_tsp_rfi(visit), build_tsp_rbi(visit)};
}

std::vector<std::vector<int>> GRASP_VRPSD::solve_clustering(const std::vector<int>& tsp) {
    std::unordered_map<int, std::vector<std::pair<int, float>>> graph;
    int n = tsp.size();

    for (int k = 0; k < n; ++k) {
        int loc = tsp[k];
        std::vector<int> route = {0};
        int load = 0;
        int l = k + 1;
        float cost = 0;

        while (l < n && load + demands[tsp[l]] <= cap) {
            route.push_back(tsp[l]);
            cost += costs.at({route[route.size() - 2], route.back(), load});
            load += demands[tsp[l]];
            graph[loc].emplace_back(tsp[l], cost + distances[0][tsp[l]]);
            ++l;
        }
    }

    std::unordered_map<int, float> distances_map;
    std::unordered_map<int, int> previous;
    std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> pq;

    for (int i = 0; i < n_clients; ++i) {
        distances_map[i] = std::numeric_limits<float>::infinity();
    }
    distances_map[0] = 0;
    pq.emplace(0, 0);

    while (!pq.empty()) {
        auto [current_distance, current_node] = pq.top();
        pq.pop();

        if (current_distance > distances_map[current_node]) continue;

        for (auto& [neighbor, weight] : graph[current_node]) {
            float distance = current_distance + weight;
            if (distance < distances_map[neighbor]) {
                distances_map[neighbor] = distance;
                previous[neighbor] = current_node;
                pq.emplace(distance, neighbor);
            }
        }
    }

    std::vector<int> path;
    int node = tsp.back();
    while (node != 0 || previous.count(node)) {
        path.push_back(node);
        node = previous[node];
    }
    std::reverse(path.begin(), path.end());

    std::vector<std::vector<int>> sol;
    int j = 1;
    for (size_t i = 0; i < path.size(); ++i) {
        std::vector<int> route = {0};
        while (tsp[j] != path[i]) {
            route.push_back(tsp[j]);
            ++j;
        }
        route.push_back(tsp[j]);
        route.push_back(0);
        sol.push_back(route);
        ++j;
    }

    
    return sol;
}

std::pair<std::vector<std::vector<int>>, bool> GRASP_VRPSD::relocate(std::vector<std::vector<int>> solution) {
    bool improved = false;
    for (size_t r1 = 0; r1 < solution.size(); r1++) {
        for (size_t r2 = 0; r2 < solution.size(); r2++) {
            if (r1 == r2) continue;

            int load_r2 = 0;
            for (int j : solution[r2]) load_r2 += demands[j];

            for (size_t i = 1; i < solution[r1].size() - 1; i++) {
                int client = solution[r1][i];

                double base_cost = get_sol_cost(solution);
                std::vector< std::vector<int>> new_solution = solution;

                for (size_t j = 1; j < solution[r2].size(); j++) {
                    new_solution[r1].erase(new_solution[r1].begin() + i);
                    new_solution[r2].insert(new_solution[r2].begin() + j, client);
                    double new_cost = get_sol_cost(new_solution);
                    
                    if (new_cost < base_cost) {
                        solution[r1].erase(solution[r1].begin() + i);
                        solution[r2].insert(solution[r2].begin() + j, client);
                        if (solution[r1].size() == 2) solution.erase(solution.begin()+r1);
                        improved = true;
                        return {solution, improved};
                    }
                    
                    new_solution[r1].insert(new_solution[r1].begin() + i, client);
                    new_solution[r2].erase(new_solution[r2].begin() + j);
                }
            }
        }
    }
    return {solution, improved};
}

std::pair<std::vector<std::vector<int>>, bool> GRASP_VRPSD::opt2(std::vector<std::vector<int>> solution) {
    bool improved = false;

    for (size_t r = 0; r < solution.size(); r++) {
        size_t route_size = solution[r].size();

        if (route_size < 4) continue;

        for (size_t i = 1; i < route_size - 2; i++) {
            for (size_t j = i + 1; j < route_size - 1; j++) {
                double base_cost = get_cost(solution[r]);

                std::vector<int> new_route = solution[r];
                std::reverse(new_route.begin() + i, new_route.begin() + j + 1);

                double new_cost = get_cost(new_route);

                if (new_cost < base_cost) {
                    solution[r] = new_route;
                    improved = true;
                    return {solution, improved};
                }
            }
        }
    }
    return {solution, improved};
}

std::vector<std::vector<int>> GRASP_VRPSD::vnd(std::vector<std::vector<int>> solution) {
    bool improved = true;
    while (improved) std::tie(solution, improved) = relocate(solution);
    improved = true;
    while (improved) std::tie(solution, improved) = opt2(solution); 
    return solution;
}

std::vector<std::vector<int>> GRASP_VRPSD::grasp(std::unordered_set<int> visit, int n_runs){
    std::vector<std::vector<int>> best_solution;
    std::vector<std::vector<int>> def_solution;
    double best_cost = std::numeric_limits<double>::max();

    for(int i=0; i<n_runs; i++){
        std::vector<std::vector<int>> tsps = build_tsp_all(visit);
        for (std::vector<int> tsp : tsps){
            std::vector<std::vector<int>> sol;
            sol = solve_clustering(tsp);
            sol = vnd(sol);
            double cost = get_sol_cost(sol);
            if(cost < best_cost){
                best_cost = cost;
                best_solution = sol;
            }
        }
    }
    if (best_cost > 1e10) return def_solution;
    return best_solution;
}