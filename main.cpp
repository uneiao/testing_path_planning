#include <iostream>
#include "rrt.h"
#include "map.h"
#include "Eigen/Dense"


int main() {
    std::vector<double> map_size(2);
    map_size[0] = 300;
    map_size[1] = 400;
    std::vector<std::vector<double> > obs;
    motion_plannning::Map map(map_size, obs);
    motion_plannning::RRTInput inp;
    inp.num_expansion = 500;
    inp.delta_t = 5;
    inp.map = &map;
    inp.start_state = std::vector<double>(2);
    inp.start_state[0] = 10;
    inp.start_state[1] = 10;
    inp.goal_state = std::vector<double>(2);
    inp.goal_state[0] = 100;
    inp.goal_state[1] = 100;

    motion_plannning::RRTOutput outp;

    motion_plannning::RRT<Eigen::Vector2d> rrt;
    rrt.Init();
    rrt.Run(inp, &outp);

    std::vector<std::vector<double> >& path = outp.path;
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i][0] << " " << path[i][1] << std::endl;
    }

    std::cout << std::endl;


    motion_plannning::RRTStar<Eigen::Vector2d> rrs;
    rrs.Init();
    rrs.Run(inp, &outp);

    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << path[i][0] << " " << path[i][1] << std::endl;
    }

    std::cout << std::endl;


    return 0;
}
