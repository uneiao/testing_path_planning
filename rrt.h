#ifndef MP_RRT_H
#define MP_RRT_H

#include <stdio.h>
#include <utility>
#include <vector>
#include <map>
#include "map.h"
//#include <Eigen/Dense>

namespace motion_plannning{

struct RRTInput {
    int num_expansion;
    double delta_t;
    const Map* map;
    std::vector<double> start_state;
    std::vector<double> goal_state;
};

struct RRTOutput {
    std::vector<std::vector<double> > path;

};

template <class STATE_DATA_TYPE>
class RRT
{
    public:
        RRT() {};
        ~RRT() {};

    public:
        virtual void Init(void);
        virtual void Run(const RRTInput&, RRTOutput*);

    protected:
        virtual void StartSearching(STATE_DATA_TYPE);
        virtual bool CollisionFree(const Map&, STATE_DATA_TYPE, STATE_DATA_TYPE);
        virtual double Distance(STATE_DATA_TYPE, STATE_DATA_TYPE);
        virtual int ChecknAddstate(const Map&, STATE_DATA_TYPE, STATE_DATA_TYPE, int);
        virtual int AddState(STATE_DATA_TYPE, STATE_DATA_TYPE, int);

    private:
        STATE_DATA_TYPE GenRandomState(const Map& map);
        void GetNearestNeighbor(STATE_DATA_TYPE, STATE_DATA_TYPE*, int*);
        STATE_DATA_TYPE Steer(
                const STATE_DATA_TYPE&, const STATE_DATA_TYPE&, double);
        bool ReachGoalState(std::vector<double>, double, int*);
        void ConstructPath(int, std::vector<std::vector<double> >*);

        void BeforeSearching();
        void AfterSearching();

    protected:
        int m_state_dim;
        std::vector<STATE_DATA_TYPE> m_nodes;
        std::map<int, int> m_edges;
};

template <class STATE_DATA_TYPE>
class RRTStar: public RRT<STATE_DATA_TYPE> {
    public:
        RRTStar() {};
        ~RRTStar() {};

    public:
        void Init(void);

    protected:
        virtual void StartSearching(STATE_DATA_TYPE);
        virtual int ChecknAddstate(const Map&, STATE_DATA_TYPE, STATE_DATA_TYPE, int);
        virtual int AddState(STATE_DATA_TYPE, STATE_DATA_TYPE, int);

    private:
        std::map<int, double> m_costs;
        double m_radius;
};

}
#endif
