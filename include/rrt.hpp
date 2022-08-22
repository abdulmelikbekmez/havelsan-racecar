#pragma once
#include "node.hpp"

struct Route
{
    Vector pos;
    DirState dir;
};

class RRT
{
private:
    static const float BIAS = 0.05;
    static const int MAX_ITER_COUNT = 1000;
    static const int MARGIN = 2;

    static generate_random_point_from_ellipse(float a, float b, float angle, const Vector pos) const;

    Node *m_head;
    Node *m_finded_node;
    Node *get_closest_node(const Vector to) const;
    void set_finded_node(const Node *new_node);
    ReversedState<float> refactor_angle(float angle) const;
    ReversedState<Vector> get_closest_point_from_node_with_max_angle(const Node *closest_node, const Vector random_point) const;
    Vector generate_random_point;
    ReversedState<Node *> get_best_parent(const Vector closest_point, const Node *current_node, bool reversed) const;
    void rewire();

    /* data */
public:
    RRT(const Vector head_pos, float angle);
};
