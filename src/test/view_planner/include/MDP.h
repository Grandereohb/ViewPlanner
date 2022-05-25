#ifndef MDP_H
#define MDP_H
#include "ViewPlan.h"

struct Action{
    int id;  // 下一个视点状态id
    Action(int id_) : id(id_){};
    Action() : id(0){};
};

class State{
public:
    State(ViewPoint view_point_);
    State(){}
    ~State();

    bool applyAction(const Action &action, vector<ViewPoint> candidates);
    int getNum() const;
    ViewPoint getVP();

private:
    ViewPoint view_point;
};

#endif