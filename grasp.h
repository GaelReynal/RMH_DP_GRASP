#ifndef GRASP_VRPSD_H
#define GRASP_VRPSD_H

#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <tuple>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include "key.h"

class GRASP_VRPSD {
public:
    int n_veh;
    int n_clients;
    int cap;
    std::vector<int> demands;
    std::vector<std::vector<int>> distances;
    std::unordered_map<Key, double, KeyHash> costs;
    std::mt19937 rng;

    GRASP_VRPSD(int n_clients, int cap, std::vector<int> demands, 
                std::vector<std::vector<int>> distances, 
                std::unordered_map<Key, double, KeyHash> costs, 
                int seed = 0);

    double get_cost(std::vector<int> route);
    double get_sol_cost(std::vector<std::vector<int>> solution);

    std::vector<int> build_tsp_rnn(std::unordered_set<int> visit);
    std::vector<int> build_tsp_rni(std::unordered_set<int> visit);
    std::vector<int> build_tsp_rfi(std::unordered_set<int> visit);
    std::vector<int> build_tsp_rbi(std::unordered_set<int> visit);
    std::vector<std::vector<int>> build_tsp_all(std::unordered_set<int> visit);

    std::vector<std::vector<int>> solve_clustering(const std::vector<int>& tsp);

    std::pair<std::vector<std::vector<int>>, bool> relocate(std::vector<std::vector<int>> solution);
    std::pair<std::vector<std::vector<int>>, bool> opt2(std::vector<std::vector<int>> solution);
    std::vector<std::vector<int>> vnd(std::vector<std::vector<int>> solution);

    std::vector<std::vector<int>> grasp(std::unordered_set<int> visit={}, int n_runs=100);
};

#endif // GRASP_VRPSD_H