#pragma once
#include "new_vector.hpp"
#include "vector"

enum DirState
{
    FORWARD,
    BACKWARD
};

template <typename T>
struct ReversedState
{
    T state;
    bool reversed;
};

class Node
{
private:
    static int ID;
    static const float MARGIN_MAX = 0.25;
    static const float MARGIN_MIN = 0.15;
    static const float MAX_ANGLE = M_PI / 10;
    static const int NEIHGBOUR_MAX = 0.30;

    DirState m_dir;
    int m_id;

    float get_angle_from_child(const Vector child) const;

public:
    Vector pos;
    float angle;
    float cost;
    Node *parent;
    std::vector<Node *> childs;
    Node(Vector pos, float angle, float cost, DirState dir, Node *parent = nullptr);
    ~Node();

    Vector direction() const;
    float refactor_angle(float length) const;
    bool can_be_child(const Node *other) const;
    ReversedState<bool> is_in_range(const Vector point) const;
    bool is_close_enough(const Vector goal_pos) const;
    void update_childs_cost();
    void update_parent(Node *new_parent, float new_cost);
};
