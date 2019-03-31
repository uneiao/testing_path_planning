#include "map.h"

namespace motion_plannning {

Map::Map(
        const std::vector<double>& sizes,
        const std::vector<std::vector<double> >& obstacles)
{
    m_map_size = sizes;
    m_obstacles = obstacles;
}

void Map::GetMapSize(std::vector<double>* sizes) const
{
    *sizes = m_map_size;
}

void Map::CheckCollision(const CheckMapInput& input, CheckMapOutput* output) const
{
    output->is_collision_free = true;

    for (int i = 0; i < m_obstacles.size(); ++i) {
        if (1) {
            output->is_collision_free = false;
        }
    }
}

}
