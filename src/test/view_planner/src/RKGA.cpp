#include "RKGA.h"

using namespace std;

Population::Population(const Population& pop):
    population(pop.population),
    fitness(pop.fitness){ }
Population::Population(const int p):
    population(p), 
    fitness(p){ }
Population::~Population() { }
void Population::setFitness(int i, double fit){
    fitness[i].first = fit;
    fitness[i].second = i;
}
void Population::sortFitness(){
    sort(fitness.begin(), fitness.end(), less<pair<double, int>>());
}
ViewPoint& Population::operator()(unsigned chromosome, unsigned allele) {
	return population[chromosome][allele];
}
vector< ViewPoint >& Population::operator()(unsigned chromosome) {
	return population[chromosome];
}

RKGA::RKGA(int _p, double _pe, double _pm, double _rhoe, double _coverage_rate, vector<ViewPoint> _candVP, Graph *_graph, vector<vector<int>> _visibility_matrix) throw(std::range_error) :
p(_p), pe(_pe*p), pm(_pm*p), rhoe(_rhoe), coverage_rate(_coverage_rate), candVP(_candVP), graph(_graph), visibility_matrix(_visibility_matrix) {
    // Error check:
    using std::range_error;
	//if(n == 0) { throw range_error("Chromosome size equals zero."); }
	if(p == 0) { throw range_error("Population size equals zero."); }
	if(pe == 0) { throw range_error("Elite-set size equals zero."); }
	if(pe > p) { throw range_error("Elite-set size greater than population size (pe > p)."); }
	if(pm > p) { throw range_error("Mutant-set size (pm) greater than population size (p)."); }
	if(pe + pm > p) { throw range_error("elite + mutant sets greater than population size (p)."); }
}
vector<ViewPoint> RKGA::solveRKGA(int maxGen){
    unsigned long iteration = 0;  // 记录迭代次数
	unsigned long lastUpdate = 0;  // 记录上次更新最优解的次数
	double bestFitness = numeric_limits< double >::max();
    // 初始化种群
    current = new Population(p);
    initialize(*current);
    previous = new Population(*current);
    
    // 进化种群
    bool run = true;
    while (run){
        cout << "49 第" << iteration << "代";
        evolve(*current, *previous);
        swap(current, previous);

        if(getBestFitness()<bestFitness){
            bestFitness = getBestFitness();
            lastUpdate = iteration;
        }

        if(iteration >= maxGen)
            run = false;
        if(bestFitness <= minCost)
            run = false;
        if(iteration - lastUpdate >= 0.2 * maxGen)
            run = false;

        ++iteration;
    }
    
    return current->population[current->fitness[0].second];
}
void RKGA::initialize(Population& curr){
    for (int j = 0; j < p; j++){
        // cout << "开始初始化种群" << j << "：" << endl;
        vector<pair<double, int>> RK_index;
        double cost = 0;
        setRandomKey(candVP.size(), RK_index);
        sortRK(RK_index);
        // cout << " 77 sorted RK_index: " << endl;
        // for (int i = 0; i < RK_index.size(); i++){
        //     cout << "(" << RK_index[i].first << ", " << RK_index[i].second << "), ";
        // }
        // cout << endl;
        // 按key从小到大的顺序依次将视点存入初始种群的个体，满足覆盖率要求则停止存入
        int k = 0;
        do
        {
            curr.population[j].push_back(candVP[RK_index[k++].second]);
            // cout << "86 k: " << k <<", vp: "<<candVP[RK_index[k-1].second].num<< endl;
        } while (!isMostCovered(curr.population[j]));
        calcMotionCost(curr.population[j], cost);
        curr.setFitness(j, cost);
    }
    curr.sortFitness();
    cout << "93 sorted fitness: " << endl;
    for (int i = 0; i < curr.fitness.size(); i++){
        cout <<"("<< curr.fitness[i].first << ", " << curr.fitness[i].second<<"), ";
    }
    cout << endl;
}
void RKGA::setRandomKey(int candSize, vector<pair<double, int>> &RK_index){
    for (int i = 0; i < candSize; i++){
        pair<double, int> temp;  // <key， 序号>
        temp.first = (double)rand() / RAND_MAX;
        while (temp.first == 1.0) {
            temp.first = ((double) rand() / RAND_MAX);
        }
        temp.second = i;
        RK_index.push_back(temp);
        //cout << "104 setRK: " << RK_index[i].first << ", " << RK_index[i].second << endl;
    }
}
void RKGA::sortRK(vector<pair<double, int>> &RK_index){
    sort(RK_index.begin(), RK_index.end(), less<pair<double, int>>());
}
bool RKGA::isMostCovered(vector<ViewPoint> single){
    vector<vector<int>> tempVM;
    // cout << "116 tempVM: ";
    for(int i = 0; i < single.size(); i++){
        // cout << single[i].num << " ";
        tempVM.push_back(visibility_matrix[single[i].num]);
    }
    // cout << endl;
    double visible_num = 0;
    for (int i = 0; i < tempVM[0].size(); i++){
        double visible_tmp = 0;
        for (int j = 0; j < tempVM.size(); j++){
            visible_tmp += tempVM[j][i];
			if(visible_tmp == 1)  // 建议设为2
				break;
        }
        visible_num += visible_tmp;
	}
    // cout << "131 visible num: " << visible_num<<" / "<< coverage_rate * tempVM[0].size() << endl;
    if (visible_num >= coverage_rate * tempVM[0].size())
    {
        // cout << "135 true" << endl;
        return 1;
    }
    else
        return 0;
}
void RKGA::calcMotionCost(vector<ViewPoint> single, double &cost){
    // cout << "142 开始计算运动成本" << endl;
    // 起始和最终位置的运动成本为与机器人home位置的角度差
    for (int i = 0; i < 6; i++){
        cost += abs(single[0].joint_state[i]) + abs(single[single.size() - 1].joint_state[i]);
    }
    // cout << "147 " <<cost<< endl;
    int k = 0;
    while(k < (single.size()-1)){
        ViewPoint *curr = graph->edges[single[k].num];
        while (curr->num != single[k + 1].num){
            curr = curr->next;
        }
        cost += curr->cost;
        k++;
    }

}
void RKGA::evolve(Population &curr, Population &next){
    cout << "进化开始：" << endl;
    unsigned i = 0;
    // cout << "159, i = " << i << endl;
    // 父母为精英，后代直接复制
    while (i < pe){
        next.population[i].clear();
        // cout << "162 curr.pop.size = " << curr.population[curr.fitness[i].second].size() <<", next: "<<next.population[i].size()<< endl;
        for (int j = 0; j < curr.population[curr.fitness[i].second].size(); j++){
            // cout << "164 pn: " << curr.population[curr.fitness[i].second][j].num << ", j = "<< j << endl;
            next.population[i].push_back(curr.population[curr.fitness[i].second][j]);
            // cout << "166 next pop size: " << next.population[i].size() << endl;
        }
        // cout << "169 ";
        next.fitness[i].first = curr.fitness[i].first;
        next.fitness[i].second = i;
        ++i;
    }
    // cout << "175 i=" << i << ", p=" << p << ", pm=" << pm << endl;
    while (i < p - pm){
        // cout << "177 ";
        const int eliteParent = rand() % (int)pe;
        const int noneliteParent = pe + rand() % (int)(p - pe);
        int j = 0;
        double cost = 0;
        next.population[i].clear();
        // cout << "183 ep="<<eliteParent<<", nep="<<noneliteParent<<", i="<<i<<endl;
        do{
            bool isRepeated = false;
            int sourceParent = ((rand() % 101 < rhoe) ? eliteParent : noneliteParent);
            int nonParent = ((sourceParent == eliteParent) ? noneliteParent : eliteParent);
           
            if(j >= curr.population[curr.fitness[sourceParent].second].size())
                break;

            // for (int l = 0; l < j; l++){
            //     if(next.population[i][l].num == curr(curr.fitness[sourceParent].second, j).num){
            //         if(j >= curr.population[curr.fitness[nonParent].second].size()){
            //             isRepeated = true;
            //             break;
            //         }
            //         for (int m = 0; m < j; m++){
            //             if(next.population[i][m].num == curr(curr.fitness[nonParent].second, j).num){
            //                 isRepeated = true;
            //                 break;
            //             }
            //         }
            //         if(!isRepeated)
            //             sourceParent = nonParent;
            //         break;
            //     }
            // }
            if(!ifRepeated(j, curr, next.population[i], sourceParent, nonParent)){
                next.population[i].push_back(candVP[curr(curr.fitness[sourceParent].second, j).num]);
                // cout <<" ("<< j << ", "<<curr(curr.fitness[sourceParent].second, j).num<<") "<<endl;
            }
            ++j;
        } while (!isMostCovered(next.population[i]));
        calcMotionCost(next.population[i], cost);
        // cout << "218 cost=" << cost << endl;
        next.setFitness(i, cost);
        ++i;
    }
    // cout << "199 ";
    while (i < p){
        next.population[i].clear();
        vector<pair<double, int>> temp_RK_index;
        setRandomKey(candVP.size(), temp_RK_index);
        sortRK(temp_RK_index);
        for (int j = 0; j < curr.population[i].size(); j++){
            next.population[i].push_back(candVP[temp_RK_index[j].second]);
        }
        double cost = 0;
        calcMotionCost(next.population[i], cost);
        next.setFitness(i, cost);
        ++i;
    }
    next.sortFitness();
    cout << "207 sorted fitness: " << endl;
    for (int i = 0; i < next.fitness.size(); i++){
        cout <<"("<< next.fitness[i].first << ", " << next.fitness[i].second<<"), ";
    }
    cout << endl;
}
double RKGA::getBestFitness() const{
    return current->fitness[0].first;
}
bool RKGA::ifRepeated(int j, const Population &curr, const vector<ViewPoint> &next, int &src, int nsrc){
    for (int l = 0; l < j; l++){
        if(next[l].num == curr.population[curr.fitness[src].second][j].num){
            if(j >= curr.population[curr.fitness[nsrc].second].size()){
                return true;
            }
            for (int m = 0; m < j; m++){
                if(next[m].num == curr.population[curr.fitness[src].second][j].num)
                    return true;
            }
            src = nsrc;
            return false;
        }
     }
}
RKGA::~RKGA(){
    delete current;
    delete previous;
}