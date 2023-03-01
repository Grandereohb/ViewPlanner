#ifndef MCST_H
#define MCST_H
#include "ViewPlan.h"
#include "MDP.h"

class TreeNode{
public:
    State state;                  // 节点的状态
    Action action;                // 指向该节点状态的动作
    TreeNode *parent;             // 父节点
    bool has_child;               // 记录有无已访问的子节点
    vector<TreeNode> children;    // 当前的子节点

    TreeNode(const State &state, TreeNode *parent);
    TreeNode(const State &state);
    TreeNode(const TreeNode* node);
    // TreeNode(){}
    ~TreeNode(){}

    bool initializeNode(const vector<ViewPoint> &candidates);  // 初始化节点
    TreeNode *expand(int id);                                  // 扩展子节点
    int isFullyExpanded();                                     // 检查是否完全扩展
    TreeNode *randomChild();                                   // 选择随机子节点
    TreeNode *weightedBestChild();                             // 选择加权最佳子节点
    TreeNode *bestChild();                                     // 选择最佳子节点
    TreeNode *applyAction(const Action &action);               // 执行action
    double getTravelCost(int start, int end, Graph *g);        // 计算运动成本
    void addVisitNum();                                        // 增加节点的num_visits
    void addCost(double once_cost);                            // 增加节点的cost
    void updateMinCost(double once_cost);                      // 更新min_cost
    void setUnreachable(int id);                               // 将子节点设置为无法到达

private:
    int num_visits;               // 该节点被访问的次数
    double cost;                  // 该节点的cost
    double min_cost;              // 该节点的最小cost
    int depth;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
    vector<Action> actions;       // 当前状态所有可能采取的动作
};

class MCST{
public:
 MCST(const double& coverage_rate_,
      const vector<ViewPoint>& candidates_,
      const vector<vector<ViewPoint>>& graph_,
      const vector<vector<int>>& visibility_matrix_,
      ros::NodeHandle nh);
 // ~MSCT();

 vector<ViewPoint> solveMCST();

private:
    // 可按需求修改参数
    int max_iteration = 45000;  // 最大迭代次数
    double epsilon1 = 50;       // TreePolicy参数1
    double epsilon2 = 60;       // TreePolicy参数2

    // 不可修改参数
    const double coverage_rate;
    const vector<ViewPoint> candidates;
    const vector<vector<ViewPoint>> graph;
    const vector<vector<int>> visibility_matrix;

    int selectStartIndex(const vector<ViewPoint> &candidates);                                    // 选择初始视点
    TreeNode *treePolicy(TreeNode &root, vector<ViewPoint> &select_vp);                           // 选择与扩展搜索树
    double simulation(const TreeNode *node, vector<ViewPoint> &select_vp);                        // 模拟
    bool isMostCovered(const vector<ViewPoint> &select_vp);                                       // 判断是否满足覆盖
    Action greedyRollout(const TreeNode *node, vector<ViewPoint> &select_vp, double &once_cost);  // 贪心搜索
    // int updateByModel(TreeNode &node, Action a, const vector<ViewPoint> &candidates);
    double getTravelCost(int start, int end);                                                     // 计算运动成本
    void backPropagation(TreeNode *node, double cost);                                            // 反向更新
};

#endif