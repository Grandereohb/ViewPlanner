#include "RKGA.h"

using namespace std;

Population::Population(const Population& pop):
    population(pop.population),
    fitness(pop.fitness){ }

Population::Population(const int p):
    population(p), 
    fitness(p){ }

void Population::setFitness(int i, double fit){
    fitness[i].first = fit;
    fitness[i].second = i;
}

void Population::sortFitness(){
    sort(fitness.begin(), fitness.end(), greater<pair<double, int>>());
}

RKGA::RKGA(int _p, double _pe, double _pm, double _rhoe, int _K, int MAX) throw(std::range_error) :
p(_p), pe(_pe*p), pm(_pm*p), rhoe(_rhoe), K(_K), MAX_THREAD(MAX), previous(K, 0), current(K, 0) {
    // Error check:
    using std::range_error;
	//if(n == 0) { throw range_error("Chromosome size equals zero."); }
	if(p == 0) { throw range_error("Population size equals zero."); }
	if(pe == 0) { throw range_error("Elite-set size equals zero."); }
	if(pe > p) { throw range_error("Elite-set size greater than population size (pe > p)."); }
	if(pm > p) { throw range_error("Mutant-set size (pm) greater than population size (p)."); }
	if(pe + pm > p) { throw range_error("elite + mutant sets greater than population size (p)."); }
	if(K == 0) { throw range_error("Number of parallel populations cannot be zero."); }

}

vector<ViewPoint> RKGA::solveRKGA(vector<ViewPoint> candVP, Graph *graph, vector<vector<int>> visibility_matrix, double coverage_rate, int maxGen){
    // 初始化种群
    for(int i=0; i<K; i++){
        current[i] = new Population(p);
        
        initialize(*current[i], candVP, graph, visibility_matrix, coverage_rate);

        previous[i] = new Population(*current[i]);
    }

    // 进化种群
    for (int i = 0; i < maxGen; i++){
        for (int j = 0; j < K; j++){
            evolve(*current[j], *previous[j]);
        }
    }
}

void RKGA::initialize(Population& curr, vector<ViewPoint> candVP, Graph *graph, vector<vector<int>> visibility_matrix, double coverage_rate){
    for (int j = 0; j < p; j++){
        vector<pair<double, int>> RK_index;
        setRandomKey(candVP.size(), RK_index);
        sortRK(RK_index);
        do{
            int k = 0;
            double cost = 0;
            curr.population[j].push_back(candVP[RK_index[k++].second]);
            calcMotionCost(graph, curr.population[j], cost);
            curr.setFitness(j, cost);
        } while (!isMostCovered(visibility_matrix, curr.population[j], coverage_rate));
        curr.sortFitness();
    }
}

void RKGA::setRandomKey(int candSize, vector<pair<double, int>> &RK_index){
    for (int i = 0; i < candSize; i++){
        pair<double, int> temp;
        temp.first = (double)rand() / RAND_MAX;
        while (temp.first == 1.0) {
            temp.first = ((double) rand() / RAND_MAX);
        }
        temp.second = i;
        RK_index.push_back(temp);
    }
}

void RKGA::sortRK(vector<pair<double, int>> &RK_index){
    for (int i = RK_index.size() - 2; i < 0; i--){
        for (int j = i; j > RK_index.size() - 2; j++){
            if (RK_index[j].first > RK_index[j + 1].first)
                swap(RK_index[j], RK_index[j + 1]);
        }
    }
}

bool RKGA::isMostCovered(vector<vector<int>> visibility_matrix, vector<ViewPoint> single, double coverage_rate){
    vector<vector<int>> tempVM;
    for(int i = 0; i < single.size(); i++){
        tempVM.push_back(visibility_matrix[single[i].num]);
    }
	double visible_num = 0;
    for (int i = 0; i < tempVM[0].size(); i++){
        double visible_tmp = 0;
        for (int j = 0; j < tempVM.size(); j++){
            visible_tmp += visibility_matrix[j][i];
			if(visible_tmp == 1)  // 建议设为2
				break;
        }
        visible_num += visible_tmp;
	}
	if(visible_num >= coverage_rate * tempVM[0].size()){
		return 1;
	}
    else
        return 0;
}

void RKGA::calcMotionCost(Graph *g, vector<ViewPoint> single, double &cost){
    for (int i = 0; i < 6; i++){
        cost += abs(single[0].joint_state[i]) + abs(single[single.size() - 1].joint_state[i]);
    }
    int k = 0;
    while(k < (single.size()-1)){
        ViewPoint *curr = g->edges[single[k].num];
        while (curr->num != single[k + 1].num){
            curr = curr->next;
        }
        cost += curr->cost;
        k++;
    }

}

void RKGA::evolve(Population &curr, Population &next){

}
