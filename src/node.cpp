#include "node.hpp"
#include <cmath>
#include <vector>
#include <algorithm>

// static elements initialize
int Node::ID = 0;

Node::Node(Vector pos, float angle, float cost, DirState dir, Node *parent = nullptr) : pos{pos}, m_dir{dir}, angle{angle}, parent{parent}, cost{cost}
{
    m_id = ID;
    ID++;
    childs = std::vector<Node *>();
}

float Node::get_angle_from_child(const Vector child) const
{
    return (child - pos).angle();
}

Vector Node::direction() const
{
    return Vector{cos(angle), sin(angle), 0};
}

float Node::refactor_angle(float length) const
{
    if (length > MARGIN_MAX)
    {
        return MARGIN_MAX;
    }
    if (length < MARGIN_MIN)
    {
        return MARGIN_MIN;
    }
    return length;
}

bool Node::can_be_child(const Node *other) const
{
    Vector direction = other->pos - pos;
    if (direction.length() > NEIHGBOUR_MAX)
    {
        return false;
    }
    float angle = this->direction().angle_to(direction);
    float angle_abs = abs(angle);

    if (angle_abs > M_PI && !(M_PI * 2 - angle_abs > M_PI * 2 - MARGIN_MAX / 2))
    {
        return false;
    }

    if (!angle_abs < MAX_ANGLE / 2)
    {
        return false;
    }

    float new_angle = get_angle_from_child(other->pos);
    for (auto child : other->childs)
    {
        float dif = abs(child->angle - new_angle);
        if (dif > MAX_ANGLE / 2)
            return false;
    }

    return true;
}

ReversedState<bool> Node::is_in_range(const Vector point) const
{
    Vector direction = point - pos;
    if (direction.length() > NEIHGBOUR_MAX)
        return ReversedState<bool>{false, false};

    float angle = abs(this->direction().angle_to(direction));

    if (angle > M_PI)
    {
        bool is_in_range = M_PI * 2 - angle > M_PI * 2 - MARGIN_MAX / 2;
        return ReversedState<bool>{is_in_range, true};
    }

    return ReversedState<bool>{angle < MAX_ANGLE / 2, false};
}

bool Node::is_close_enough(const Vector goal_pos) const
{
    return (pos - goal_pos).length() < MARGIN_MAX;
}

void Node::update_childs_cost()
{
    for (auto child : childs)
    {
        float new_cost = (pos - child->pos).length();
        child->cost = cost + new_cost;
        child->update_childs_cost();
    }
}

void Node::update_parent(Node *new_parent, float new_cost)
{
    if (parent)
    {
        auto it = std::find(parent->childs.begin(), parent->childs.end(), this);
        if (it != parent->childs.end())
            parent->childs.erase(it);
    }

    parent = new_parent;
    parent->childs.push_back(this);

    angle = parent->get_angle_from_child(pos);
    cost = new_cost;
    update_childs_cost();
}