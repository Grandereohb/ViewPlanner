#ifndef MCST_H
#define MCST_H
#include "ViewPlan.h"
#include "MDP.h"

class MCST{
public:
    MCST(double coverage_rate_, vector<ViewPoint> candidates_, Graph *graph_, vector<vector<int>> visibility_matrix_);
    // ~MSCT();

    vector<ViewPoint> solveMCST();

private:
    // 可按需求修改参数
    const int max_iteration = 200;
    const double epsilon1 = 50;
    const double epsilon2 = 50;

    // 不可修改参数
    double coverage_rate;
    vector<ViewPoint> candidates;
    Graph *graph;
    vector<vector<int>> visibility_matrix;

    int selectStartIndex(const vector<ViewPoint> &candidates);
    TreeNode treePolicy(TreeNode &root, vector<ViewPoint> &select_vp);
    double simulation(TreeNode &node, vector<ViewPoint> &select_vp);
    bool isMostCovered(const vector<ViewPoint> &select_vp);
    Action greedyRollout(const TreeNode &node, vector<ViewPoint> &select_vp, double &once_cost);
    // int updateByModel(TreeNode &node, Action a, const vector<ViewPoint> &candidates);
    double getTravelCost(int start, int end);
    void backPropagation(TreeNode &node, double cost);
};

class TreeNode{
public:
    State state;                  // 节点的状态
    Action action;                // 指向该节点状态的动作
    TreeNode *parent;             // 父节点

    TreeNode(const State &state, TreeNode *parent = NULL);
    bool initializeNode(const vector<ViewPoint> &candidates);
    TreeNode expand(int id);
    int isFullyExpanded();
    bool eraseChild(int id);
    TreeNode *randomChild();
    TreeNode *bestChild();
    void addVisitNum();
    void addCost(double once_cost);
    void checkMinCost(double once_cost);

private:
    // 可按需求修改参数
    

    // 不可修改参数
    int num_visits;               // 该节点被访问的次数
    double cost;                  // 该节点的cost
    double min_cost;              // 该节点的最小cost
    int depth;

    vector<TreeNode> children;    // 当前的子节点
    vector<Action> actions;       // 当前状态所有可能采取的动作
};

#endif