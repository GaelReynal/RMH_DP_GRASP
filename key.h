#ifndef VRPSD_KEY_H
#define VRPSD_KEY_H

// Structures to represent the costs in an unordered map (used in the GRASP)
struct Key {
    int i, j, k;

    bool operator==(const Key& other) const {
        return i == other.i && j == other.j && k == other.k;
    }
};

// Fonction de hachage optimisée
struct KeyHash {
    std::size_t operator()(const Key& key) const {
        return static_cast<std::size_t>(key.i) * 73856093 ^
               static_cast<std::size_t>(key.j) * 19349663 ^
               static_cast<std::size_t>(key.k) * 83492791;
    }
};

#endif