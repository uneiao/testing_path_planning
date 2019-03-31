#include "rrt.h"
#include <iostream>
#include <map>
#include <cstdlib>
#include <ctime>
#include "Eigen/Dense"

namespace motion_plannning{

template <class STATE_DATA_TYPE>
void RRT<STATE_DATA_TYPE>::Init()
{
    //m_state_dim = state_dim;
    m_nodes.clear();
    m_edges.clear();
    std::srand(std::time(nullptr));
}

template <class STATE_DATA_TYPE>
void RRT<STATE_DATA_TYPE>::StartSearching(STATE_DATA_TYPE start_state)
{
    m_nodes.push_back(start_state);
}

template <class STATE_DATA_TYPE>
void RRT<STATE_DATA_TYPE>::Run(const RRTInput& input, RRTOutput* output)
{
    m_nodes.clear();
    m_edges.clear();
    STATE_DATA_TYPE start_state(input.start_state[0], input.start_state[1]);
    StartSearching(start_state);

    for (int i = 0; i < input.num_expansion; i++) {
        int goal_state_index = -1;
        if (ReachGoalState(input.goal_state, input.delta_t, &goal_state_index)) {
            ConstructPath(goal_state_index, &(output->path));
            return;
        }

        STATE_DATA_TYPE random_state = GenRandomState(*input.map);

        int nearest_ind = 0;
        STATE_DATA_TYPE nearest;
        GetNearestNeighbor(random_state, &nearest, &nearest_ind);

        STATE_DATA_TYPE new_state = Steer(
            random_state, nearest, input.delta_t);
        ChecknAddstate(*input.map, new_state, nearest, nearest_ind);
    }
    //AfterSearching();
}

template <class STATE_DATA_TYPE>
STATE_DATA_TYPE RRT<STATE_DATA_TYPE>::GenRandomState(const Map& map)
{
    // TODO
    std::vector<double> sizes;
    sizes.clear();
    map.GetMapSize(&sizes);

    int size0 = static_cast<int>(sizes[0]);
    int size1 = static_cast<int>(sizes[1]);
    int position = std::rand() % (size0 * size1);
    int row = position / size1;
    int col = position % size1;

    STATE_DATA_TYPE state(row, col);
    //std::cout << state << std::endl;
    return state;
}

template <class STATE_DATA_TYPE>
void RRT<STATE_DATA_TYPE>::GetNearestNeighbor(
        STATE_DATA_TYPE cur_state, STATE_DATA_TYPE* nearest, int* ind)
{
    bool init_flag = false;
    double min_dist = -1;
    for (int i = 0; i < m_nodes.size(); ++i) {
        double dist = Distance(m_nodes[i], cur_state);
        if (!init_flag || dist < min_dist) {
            *ind = i;
            *nearest = m_nodes[i];
            min_dist = dist;
            init_flag = true;
        }
    }
}

template <class STATE_DATA_TYPE>
STATE_DATA_TYPE RRT<STATE_DATA_TYPE>::Steer(
    const STATE_DATA_TYPE& candidate, const STATE_DATA_TYPE& nearest,
    double delta_t)
{
    double dist = Distance(candidate, nearest);
    //std::cout << "nodes: " << candidate << " "<< nearest << " dist:" << dist << std::endl;

    if (dist > delta_t) {
        STATE_DATA_TYPE adjusted = nearest + (candidate - nearest) * \
            (delta_t / dist);
        return adjusted;
    } else {
        return candidate;
    }
}

template <class STATE_DATA_TYPE>
double RRT<STATE_DATA_TYPE>::Distance(
        STATE_DATA_TYPE node_a, STATE_DATA_TYPE node_b)
{
    double dist = (node_a - node_b).norm();
    return dist;
}

template <class STATE_DATA_TYPE>
bool RRT<STATE_DATA_TYPE>::CollisionFree(
        const Map& map, STATE_DATA_TYPE nearest, STATE_DATA_TYPE new_state)
{
    CheckMapInput input;
    CheckMapOutput output;
    input.start = std::vector<double>(2);
    input.start[0] = nearest[0];
    input.start[1] = nearest[1];

    input.end = std::vector<double>(2);
    input.end[0] = new_state[0];
    input.end[1] = new_state[1];

    map.CheckCollision(input, &output);
    return output.is_collision_free;
}

template <class STATE_DATA_TYPE>
int RRT<STATE_DATA_TYPE>::AddState(
    STATE_DATA_TYPE new_state, STATE_DATA_TYPE nearest, int nearest_index)
{
    m_nodes.push_back(new_state);
    int new_index = m_nodes.size() - 1;
    m_edges[new_index] = nearest_index;
    return new_index;
}


template <class STATE_DATA_TYPE>
bool RRT<STATE_DATA_TYPE>::ReachGoalState(
        std::vector<double> goal, double delta_t, int* goal_index)
{
    STATE_DATA_TYPE goal_state(goal[0], goal[1]);

    int nearest_ind = 0;
    STATE_DATA_TYPE nearest;
    GetNearestNeighbor(goal_state, &nearest, &nearest_ind);

    double dist = Distance(goal_state, nearest);

    if (dist < delta_t) {
        *goal_index = AddState(goal_state, nearest, nearest_ind);
        return true;
    }
    return false;
}

template <class STATE_DATA_TYPE>
void RRT<STATE_DATA_TYPE>::ConstructPath(
        int goal_state_index, std::vector<std::vector<double> >* path)
{
    std::map<int, int>& edge_map = m_edges;
    path->clear();
    int prev = goal_state_index;
    while (prev != 0) {
        STATE_DATA_TYPE cur = m_nodes[prev];
        std::vector<double> node(2);
        node[0] = cur[0];
        node[1] = cur[1];
        path->insert(path->begin(), node);
        prev = edge_map[prev];
    }

    STATE_DATA_TYPE cur = m_nodes[prev];
    std::vector<double> node(2);
    node[0] = cur[0];
    node[1] = cur[1];
    path->insert(path->begin(), node);
}

template <class STATE_DATA_TYPE>
int RRT<STATE_DATA_TYPE>::ChecknAddstate(
    const Map& map, STATE_DATA_TYPE new_state, STATE_DATA_TYPE nearest,
    int nearest_index)
{
    if (CollisionFree(map, nearest, new_state)) {
        return AddState(new_state, nearest, nearest_index);
    }
    return -1;

}


template <class STATE_DATA_TYPE>
void RRTStar<STATE_DATA_TYPE>::Init()
{
    this->m_nodes.clear();
    this->m_edges.clear();
    this->m_costs.clear();
    std::srand(std::time(nullptr));
    m_radius = 10;
}

template <class STATE_DATA_TYPE>
void RRTStar<STATE_DATA_TYPE>::StartSearching(STATE_DATA_TYPE start_state)
{
    this->m_nodes.push_back(start_state);
    m_costs[0] = 0;
}

template <class STATE_DATA_TYPE>
int RRTStar<STATE_DATA_TYPE>::ChecknAddstate(
    const Map& map, STATE_DATA_TYPE new_state, STATE_DATA_TYPE nearest,
    int nearest_index)
{
    if (!this->CollisionFree(map, nearest, new_state)) {
        return -1;
    }
    //TODO
    double near_threshold = m_radius;

    STATE_DATA_TYPE min_state = nearest;
    int min_index = nearest_index;
    double min_cost = m_costs[nearest_index] + \
        this->Distance(new_state, this->m_nodes[nearest_index]);

    std::vector<size_t> near_indices;
    near_indices.clear();

    for (size_t i = 0; i < this->m_nodes.size(); ++i) {
        if (this->Distance(new_state, this->m_nodes[i]) < near_threshold) {
            near_indices.push_back(i);
            if (this->CollisionFree(map, this->m_nodes[i], new_state)) {

                double cost = m_costs[i] + this->Distance(
                        new_state, this->m_nodes[i]);

                if (cost < min_cost) {
                    min_cost = cost;
                    min_index = i;
                    min_state = this->m_nodes[i];
                }
            }
        }
    }

    int new_index = AddState(new_state, min_state, min_index);

    for (size_t j = 0; j < near_indices.size(); ++j) {
        int near_index = near_indices[j];
        STATE_DATA_TYPE near_state = this->m_nodes[near_index];

        if (this->CollisionFree(map, near_state, new_state)) {
            double cost = m_costs[new_index] + \
                          this->Distance(new_state, near_state);

            if (cost < m_costs[near_index]) {
                this->m_edges[near_index] = new_index;
                m_costs[near_index] = cost;
            }
        }
    }

    return new_index;
}

template <class STATE_DATA_TYPE>
int RRTStar<STATE_DATA_TYPE>::AddState(
    STATE_DATA_TYPE new_state, STATE_DATA_TYPE prev, int prev_index)
{
    this->m_nodes.push_back(new_state);
    int new_index = this->m_nodes.size() - 1;

    this->m_edges[new_index] = prev_index;

    double dist = this->Distance(new_state, prev);
    m_costs[new_index] = m_costs[prev_index] + dist;

    return new_index;
}


template class RRT<Eigen::Vector2d>;
template class RRTStar<Eigen::Vector2d>;

}
