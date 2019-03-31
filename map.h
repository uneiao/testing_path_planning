#ifndef MP_MAP_H
#define MP_MAP_H

#include <stdio.h>
#include <vector>
//#include <Eigen/Dense>

namespace motion_plannning {

struct CheckMapInput {
    std::vector<double> start;
    std::vector<double> end;
};

struct CheckMapOutput {
    bool is_collision_free;
};

class Map
{
    public:
        Map(){};
        ~Map(){};

        Map(const std::vector<double>& sizes,
            const std::vector<std::vector<double> >& obstacles);

    public:
        void GetMapSize(std::vector<double>*) const;
        void CheckCollision(const CheckMapInput& input, CheckMapOutput* output) const;

    private:
        std::vector<double> m_map_size;
        std::vector<std::vector<double> > m_obstacles;

};

}
#endif
