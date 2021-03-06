#include "MCST.h"

using namespace std;

MCST::MCST(const double &coverage_rate_, const vector<ViewPoint> &candidates_, const vector<vector<ViewPoint>> &graph_, const vector<vector<int>> &visibility_matrix_):
coverage_rate(coverage_rate_), candidates(candidates_), graph(graph_), visibility_matrix(visibility_matrix_) {}

vector<ViewPoint> MCST::solveMCST(){
    vector<ViewPoint> res;
    int start_index = selectStartIndex(candidates);
    if(start_index == -1){
        cout << "候选视点集为空，无法寻找初始视点!" << endl;
        return res;
    }
    State state(candidates[start_index]);
    TreeNode root(state);
    root.initializeNode(candidates);

    for (int i = 0; i < max_iteration; ++i){
        vector<ViewPoint> select_vp;
        select_vp.push_back(root.state.getVP());

        TreeNode *node = treePolicy(root, select_vp);
        select_vp.push_back(node->state.getVP());

        double cost = simulation(node, select_vp);
        
        if(i % 100 == 0)
            cout << i << " " << cost << endl;
        backPropagation(node, cost);
    }

    TreeNode *cur = &root;
    while(cur){
        res.push_back(cur->state.getVP());
        cur = cur->bestChild();
    }
    // delete cur;
    return res;
}
int MCST::selectStartIndex(const vector<ViewPoint> &candidates){
    if(candidates.size() == 0){
        return -1;
    }
    int max_vis_area = -1;
    int start_index = 0;

    // 遍历视点，寻找可见面片最大的视点
    for (int i = 0; i < candidates.size(); ++i){
        if(candidates[i].vis_area > max_vis_area){
            max_vis_area = candidates[i].vis_area;
            start_index = i;
        }
    }
    return start_index;
}
TreeNode *MCST::treePolicy(TreeNode &root, vector<ViewPoint> &select_vp){
    // 从root节点开始对搜索树进行选择或扩展，返回simulation的开始节点

    int iteration = 40;  // 树搜索次数上限
    TreeNode *node = &root;
    while (iteration--){
        int random_num = rand() % 100;
        // 判断是否存在未被访问过的子节点，存在则返回子节点序号，不存在返回-1
        int expand_id = node->isFullyExpanded(); 

        // 如果所有子节点均未被访问，或者存在未被访问的子节点且小于epsilon1 greedy，则扩展子节点 
        if (!node->has_child || (expand_id >= 0 && random_num < epsilon1)){
            if(getTravelCost(node->state.getNum(), node->children[expand_id].state.getNum()) < 1000){
                node->has_child = true;
                return node->expand(expand_id);
            }
            else{
                node->setUnreachable(expand_id);
                ++iteration;
                continue;
            }
        }

        TreeNode *child_node;
        // 不存在未被访问的子节点，直接选择下一个节点
        if(expand_id == -1){
            if(random_num < epsilon2)
                child_node = node->weightedBestChild();
            else
                child_node = node->randomChild();
        } 
        // 存在访问与未被访问过的节点，且大于epsilon1 greedy
        else{
            if(random_num < epsilon2)
                child_node = node->randomChild();
            else
                child_node = node->weightedBestChild();
        }
        // 检查父节点与子节点之间能否到达
        if(getTravelCost(node->state.getNum(), child_node->state.getNum()) > 1000){
            ++iteration;
            continue;
        }
        node = child_node;
        // 选择子节点之后需要将子节点存入select_vp
        if(iteration != 0)
            select_vp.push_back(node->state.getVP());
    }
    return node;
}
double MCST::simulation(const TreeNode *node, vector<ViewPoint> &select_vp){
    // 模拟。从给定节点开始按照贪心策略向下搜索，直到得到一条满足覆盖率要求的轨迹（视点集），返回轨迹的总运动成本
    TreeNode *sim_node = new TreeNode(node);
    double cost = 0; // 总运动成本
    do{
        double once_cost = 0;  // 单次运动成本

        // 贪心搜索
        Action action = greedyRollout(sim_node, select_vp, once_cost);
       
        // 执行action，得到下一状态（节点）
        sim_node = sim_node->applyAction(action);

        select_vp.push_back(sim_node->state.getVP());
        cost += once_cost;
    } while (!isMostCovered(select_vp));
    // delete sim_node;
    return cost;
}
bool MCST::isMostCovered(const vector<ViewPoint> &select_vp){
    // 同RKGA，判断是否满足覆盖率要求
    vector<vector<int>> tempVM;
    for(int i = 0; i < select_vp.size(); i++){
        tempVM.push_back(visibility_matrix[select_vp[i].num]);
    }
    double visible_num = 0;
    for (int i = 0; i < tempVM[0].size(); i++){
        double visible_tmp = 0;
        for (int j = 0; j < tempVM.size(); j++){
            visible_tmp += tempVM[j][i];
			if(visible_tmp == 1)  // 建议设为2
				break;
        }
        visible_num += visible_tmp;  // 设为2时此处应当 / 2
	}
    if (visible_num >= coverage_rate * tempVM[0].size())
    {
        return 1;
    }
    else
        return 0;
}
Action MCST::greedyRollout(const TreeNode *node, vector<ViewPoint> &select_vp, double &once_cost){
    // 根据 新增覆盖面积最大 + 运动成本最短 的原则进行贪心搜索

    // 记录simulation时已经被选择的节点
    vector<int> select_vp_index;  
    for (int i = 0; i < select_vp.size(); ++i){
        select_vp_index.push_back(select_vp[i].num);
    }
    sort(select_vp_index.begin(), select_vp_index.end());   // 升序排列

    // 记录未被覆盖的面片
    vector<int> uncovered_patch;
    for (int i = 0; i < visibility_matrix[0].size(); ++i){  // 遍历面片
        bool flag = false;
        for (int j = 0; j < select_vp.size(); ++j){         // 遍历视点
            if (visibility_matrix[select_vp[j].num][i] == 1){  
                flag = true;
                break;
            }
        }
        if (flag == false)
            uncovered_patch.push_back(i);
    }

    // 贪心搜索，寻找value最大的视点
    int vp_index = 0;
    double max_value = DBL_MIN;
    int max_index = -1;
    // 遍历视点
    for (int i = 0; i < visibility_matrix.size(); ++i){
        // 视点在simulation前已被选择，不能重复选择，跳过
        if(i == select_vp_index[vp_index]){
            ++vp_index;
            continue;
        }
        // 计算simulation起点到该视点的运动成本
        double travel_cost = getTravelCost(node->state.getNum(), i);
        if(travel_cost > 1000)
            continue;
        // 计算该视点能新覆盖的面片数
        double delta_coverage = 0;
        for (int j = 0; j < uncovered_patch.size(); ++j){
            delta_coverage += visibility_matrix[i][uncovered_patch[j]];
        }
        // value用来计算和表示能看到更多面片且更近的视点
        double value = delta_coverage / travel_cost;
        if(value > max_value){
            max_value = value;
            max_index = i;
            once_cost = travel_cost;
        }
    }
    if(max_index == -1){
        cout << "202 GreedyRollout ERROR!" << endl;
        abort();
    }
    // 返回指向下一个状态需要执行的action
    Action next_action(max_index);
    return next_action;
}
// int MCST::updateByModel(TreeNode &node, Action a, const vector<ViewPoint> &candidates){
//     node.state.applyAction(a, candidates);
// }
double MCST::getTravelCost(int start, int end){
    // 遍历图，从起点和终点的边中读取cost
    // ViewPoint *curr = graph->edges[start];
    // while (curr->num != end){
    //     curr = curr->next;
    // }
    return graph[start][end].cost;
}
void MCST::backPropagation(TreeNode *node, double cost){
    // 反向传播，更新从simulation起点到root之间每个节点的参数
    TreeNode *cur = node;
    while(cur->parent){
        cur->addVisitNum();        // 增加访问次数
        cur->addCost(cost);        // 增加总cost
        cur->updateMinCost(cost);  // 更新最小cost
        // 父节点cost = 子节点cost + 父节点到子节点cost
        cost += getTravelCost(cur->parent->state.getNum(), cur->state.getNum());
        cur = cur->parent;
    }
    // delete cur;
}

// ---------------------------------------------
TreeNode::TreeNode(const State &state_, TreeNode *parent_) : state(state_), parent(parent_), has_child(false),
                                                             num_visits(0), cost(0), min_cost(DBL_MAX), depth(parent ? parent->depth + 1 : 0) {
    action.id = state.getNum();
}
TreeNode::TreeNode(const State &state_) : state(state_), parent(nullptr), has_child(false),
                                          num_visits(0), cost(0), min_cost(DBL_MAX), depth(parent ? parent->depth + 1 : 0) {
    action.id = state.getNum();
}
TreeNode::TreeNode(const TreeNode* node){
    state = node->state;
    action = node->action;
    parent = node->parent;
    children = node->children;
}

bool TreeNode::initializeNode(const vector<ViewPoint> &candidates){
    // 初始化节点
    // 未输入候选视点，报错
    if(candidates.size() == 0)
        return false;
    for (int i = 0; i < candidates.size(); ++i){
        // 将候选视点中初该节点以外的视点存入children
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
    // 判断是否存在未被访问过的子节点，存在则返回子节点序号，不存在返回-1
    for (int i = 0; i < children.size(); ++i){
        if(children[i].num_visits == 0)
            return i;
    }
    return -1;
}
TreeNode *TreeNode::expand(int id){
    // 扩展children中的指定子节点id
    if(children[id].children.size() != 0){
        cout << "266 expand ERROR!" << endl;
        abort();
        return nullptr;
    }
    // 子节点的父节点指向this
    children[id].parent = this;
    // 定义子节点id的子节点
    for (int i = 0; i < children.size(); ++i){
        if(i == id)
            continue;
        TreeNode node(children[i].state, &children[id]);
        children[id].children.push_back(node);
    }
    // 返回子节点id的引用
    return &children[id];
}
// bool TreeNode::eraseChild(int id){
//     children.erase(children.begin() + id);
// }
TreeNode *TreeNode::randomChild(){
    // 随机选取子节点
    vector<int> visited_child_index;
    for (int i = 0; i < children.size(); ++i){
        if(children[i].num_visits > 0)
            visited_child_index.push_back(i);
    }
    // 随机随机排序
    random_shuffle(visited_child_index.begin(), visited_child_index.end());
    return &children[visited_child_index[0]];
}
TreeNode *TreeNode::weightedBestChild(){
    // 选择加权最佳子节点
    double min_value = DBL_MAX;
    int min_index;
    double alpha = 0.3;
    for (int i = 0; i < children.size(); ++i){
        if(children[i].num_visits > 0){
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
TreeNode *TreeNode::bestChild(){
    // 选择最佳子节点（cost最小）
    double min_cost = DBL_MAX;
    int min_index = 0;
    for (int i = 0; i < children.size(); ++i){
        if(children[i].num_visits <= 0)
            continue;
        if(min_cost > children[i].min_cost){
            min_cost = children[i].min_cost;
            min_index = i;
        }
    }
    return &children[min_index];
}
TreeNode *TreeNode::applyAction(const Action &action){
    // 获取执行action后得到的节点状态
    for (int i = 0; i < children.size(); ++i){
        if(children[i].state.getNum() == action.id){
            return expand(i);
        }
            // return &children[i];
    }
    cout << "无法获取执行action后的结果 当前节点:" << state.getNum() << ", 目标id:" << action.id << endl;
    abort();
    return nullptr;
}
double TreeNode::getTravelCost(int start, int end, Graph *g){
    // 遍历图，从起点和终点的边中读取cost
    // ViewPoint *curr = g->edges[start];
    // while (curr->num != end){
    //     curr = curr->next;
    // }
    if(start == end){
        cout<<"MCTS getTravelCost ERROR!"<<endl;
        abort();
    }
    return g->graph[start][end].cost;
}

void TreeNode::addVisitNum(){
    if(num_visits >= 0)
        ++num_visits;
    else
        cout << "355 add num_visits ERROR!" << endl;
}
void TreeNode::addCost(double once_cost){
    cost += once_cost;
}
void TreeNode::updateMinCost(double once_cost){
    min_cost = min(min_cost, once_cost);
}
void TreeNode::setUnreachable(int id){
    children[id].num_visits = -1;
}