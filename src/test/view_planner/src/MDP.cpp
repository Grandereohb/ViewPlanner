#include "MDP.h"

using namespace std;

State::State(ViewPoint view_point_): view_point(view_point_) {}
State::~State(){}
bool State::applyAction(const Action& action, vector<ViewPoint> candidates){
    if(action.id >= candidates.size() || action.id < 0){
        cout << "action执行失败:无法找到下一个状态位置" << endl;
        return false;
    }
    view_point = candidates[action.id];
    return true;
}
int State::getNum() const { return view_point.num; }
ViewPoint State::getVP() { return view_point; }