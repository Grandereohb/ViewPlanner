#include "MCST.h"

using namespace std;

MCST::MCST(double coverage_rate_, vector<ViewPoint> candidates_, Graph *graph_, vector<vector<int>> visibility_matrix_):
coverage_rate(coverage_rate_), candidates(candidates_), graph(graph_), visibility_matrix(visibility_matrix_) {}

vector<ViewPoint> MCST::solveMCST(){
    int start_index = selectStartIndex(candidates);
    if(start_index == -1){
        cout << "候选视点集为空，无法寻找初始视点！" << endl;
        return;
    }
    State state(candidates[start_index]);
    TreeNode root(state);
    root.initializeNode(candidates);

    for (int i = 0; i < max_iteration; ++i){
        vector<ViewPoint> select_vp;
        select_vp.push_back(root.state.getVP());
        TreeNode node = treePolicy(root, select_vp);
        select_vp.push_back(node.state.getVP());
        double cost = simulation(node, select_vp);
        backPropagation(node, cost);
    }

    vector<ViewPoint> res;
    TreeNode *cur = &root;
    while(cur){
        res.push_back(cur->state.getVP());
        cur = cur->bestChild();
    }
    delete cur;
    return res;
}
int MCST::selectStartIndex(const vector<ViewPoint> &candidates){
    if(candidates.size() == 0){
        return -1;
    }
    int max_vis_area = -1;
    int start_index = 0;
    for (int i = 0; i < candidates.size(); ++i){
        if(candidates[i].vis_area > max_vis_area){
            max_vis_area = candidates[i].vis_area;
            start_index = i;
        }
    }
    return start_index;
}
TreeNode MCST::treePolicy(TreeNode &root, vector<ViewPoint> &select_vp){
    int iteration = 50;
    TreeNode *node = &root;
    while (iteration--){
        int expand_id = node->isFullyExpanded();
        int random_num = rand() % 100;
        if (expand_id >= 0 && random_num < epsilon1)
            return node->expand(expand_id);
        else if(random_num < epsilon2)
            node = node->randomChild();
        else
            node = node->bestChild();
        if(iteration != 0)
            select_vp.push_back(node->state.getVP());
    }
    return *node;
}
double MCST::simulation(TreeNode &node, vector<ViewPoint> &select_vp){
    double cost = 0;
    do{
        double once_cost = 0;
        Action action = greedyRollout(node, select_vp, once_cost);
        node.state.applyAction(action, candidates);
        cost += once_cost;
    } while (!isMostCovered(select_vp));
    return cost;
}
bool MCST::isMostCovered(const vector<ViewPoint> &select_vp){
    vector<vector<int>> tempVM;
    for(int i = 0; i < select_vp.size(); i++){
        tempVM.push_back(visibility_matrix[select_vp[i].num]);
    }
    double visible_num = 0;
    for (int i = 0; i < tempVM[0].size(); i++){
        double visible_tmp = 0;
        for (int j = 0; j < tempVM.size(); j++){
            visible_tmp += tempVM[j][i];
			if(visible_tmp == 2)  // 建议设为2
				break;
        }
        visible_num += visible_tmp / 2;
	}
    if (visible_num >= coverage_rate * tempVM[0].size())
    {
        return 1;
    }
    else
        return 0;
}
Action MCST::greedyRollout(const TreeNode &node, vector<ViewPoint> &select_vp, double &once_cost){
    vector<int> uncovered_patch;
    vector<int> select_vp_index;
    for (int i = 0; i < select_vp.size(); ++i){
        select_vp_index.push_back(select_vp[i].num);
    }
    sort(select_vp_index.begin(), select_vp_index.end());
    for (int i = 0; i < visibility_matrix[0].size(); ++i){
        bool flag = false;
        for (int j = 0; j < select_vp.size(); ++j){
            if (visibility_matrix[select_vp[j].num][i] == 1){
                flag = true;
                break;
            }
        }
        if (flag == false)
            uncovered_patch.push_back(i);
    }
    int vp_index = 0;
    double max_value = DBL_MIN;
    int max_index = 0;
    for (int i = 0; i < visibility_matrix.size(); ++i){
        if(i == select_vp_index[vp_index]){
            ++vp_index;
            continue;
        }
        double delta_coverage = 0;
        for (int j = 0; j < uncovered_patch.size(); ++j){
            delta_coverage += visibility_matrix[i][uncovered_patch[j]];
        }
        double travel_cost = getTravelCost(node.state.getNum(), i);
        double value = delta_coverage / travel_cost;
        if(value > max_value){
            max_value = value;
            max_index = i;
            once_cost = travel_cost;
        }
    }
    Action next_action(max_index);
    return next_action;
}
// int MCST::updateByModel(TreeNode &node, Action a, const vector<ViewPoint> &candidates){
//     node.state.applyAction(a, candidates);
// }
double MCST::getTravelCost(int start, int end){
    ViewPoint *curr = graph->edges[start];
    while (curr->num != end){
        curr = curr->next;
    }
    delete curr;
    return curr->cost;
}
void MCST::backPropagation(TreeNode &node, double cost){
    TreeNode *cur = &node;
    while(cur->parent){
        cur->addVisitNum();
        cur->addCost(cost);
        cur->checkMinCost(cost);
        cost += getTravelCost(cur->parent->state.getNum(), cur->state.getNum());
        cur = cur->parent;
    }
    delete cur;
}

// ---------------------------------------------
TreeNode::TreeNode(const State &state_, TreeNode *parent_ = NULL) : state(state_), action(), parent(parent_),
                                                                    num_visits(0), cost(0), min_cost(DBL_MAX), depth(parent ? parent->depth + 1 : 0) {
    action.id = state.getNum();
}
bool TreeNode::initializeNode(const vector<ViewPoint> &candidates){
    if(candidates.size() == 0)
        return false;
    for (int i = 0; i < candidates.size(); ++i){
        if(candidates[i].num != state.getNum()){
            State sub_state(candidates[i]);
            TreeNode son(sub_state, this);
            Action action(candidates[i].num);

            children.push_back(son);
            actions.push_back(action);
        }
    }
    return true;
}
int TreeNode::isFullyExpanded(){
    for (int i = 0; i < children.size(); ++i){
        if(children[i].num_visits == 0)
            return i;
    }
    return -1;
}
TreeNode TreeNode::expand(int id){
    // children[id].num_visits++;
    children[id].parent = this;

    for (int i = 0; i < children.size(); ++i){
        if(i == id)
            continue;
        TreeNode node(children[i].state, &children[id]);
        children[id].children.push_back(node);
    }
    // children[id].children = children;
    // children[id].eraseChild(id);
    return children[id];
}
bool TreeNode::eraseChild(int id){
    children.erase(children.begin() + id);
}
TreeNode *TreeNode::randomChild(){
    vector<int> visited_child_index;
    for (int i = 0; i < children.size(); ++i){
        if(children[i].num_visits != 0)
            visited_child_index.push_back(i);
    }
    random_shuffle(visited_child_index.begin(), visited_child_index.end());
    return &children[visited_child_index[0]];
}
TreeNode *TreeNode::bestChild(){
    double min_value = DBL_MAX;
    int min_index;
    double alpha = 0.3;
    for (int i = 0; i < children.size(); ++i){
        if(children[i].num_visits != 0){
            double value = (1 - alpha) * children[i].min_cost + 
                            alpha * children[i].cost / children[i].num_visits;
            if(value < min_value){
                min_value = value;
                min_index = i;
            }
        }
    }
    return &children[min_index];
}
void TreeNode::addVisitNum(){
    ++num_visits;
}
void TreeNode::addCost(double once_cost){
    cost += once_cost;
}
void TreeNode::checkMinCost(double once_cost){
    min_cost = min(min_cost, once_cost);
}