#ifndef RKGA_H
#define RKGA_H
#include "ViewPlan.h"

using namespace std;

class Population{
    friend class RKGA;
public:

private:
    Population(const Population& other);
    Population(int p);
    ~Population();

    vector<vector<ViewPoint>> population;
    vector<pair<double, int>> fitness;
    
    void setFitness(int i, double fit);
    void sortFitness();

    vector< ViewPoint >& operator()(unsigned i);	// Direct access to chromosome i
	ViewPoint& operator()(unsigned i, unsigned j);		// Direct access to allele j of chromosome i
};

class RKGA{
public:
    RKGA(int p, double pe, double pm, double rhoe, double coverage_rate, vector<ViewPoint> candVP, Graph *graph, vector<vector<int>> visibility_matrix);
    ~RKGA();

	vector<ViewPoint> solveRKGA(int maxGen);  // RKGA求解最优视点和测量路径

private:
	const int p;	    // 每一代种群中的个体数
	const double pe;	// 每一代种群中的精英个体比例
	const double pm;	// 每一代种群中正常交叉繁衍的个体比例
    const double rhoe;	// 遗传自精英父母的比例
    const double coverage_rate;
    const Graph *graph;
    const vector<ViewPoint> candVP;
    const vector<vector<int>> visibility_matrix;
    const double minCost = 10;

    Population* previous;
    Population* current;

    void initialize(Population& curr);                                     // 初始化种群
    void setRandomKey(int candSize, vector<pair<double, int>>& RK_index);  // 给每个视点设置随机Key值
    void sortRK(vector<pair<double,int>>& RK_index);                       // 将Key从小到大排序
    bool isMostCovered(vector<ViewPoint> single);                          // 个体是否满足覆盖度要求
    void calcMotionCost(vector<ViewPoint> single, double &cost);           // 计算该个体的运动成本
    void evolve(Population curr, Population &next);
    double getBestFitness() const;
    bool isRepeated(int j, const Population &curr, const vector<ViewPoint> &next, int &src, int nsrc);
    bool isRepeated(const ViewPoint &curr, const vector<ViewPoint> &next);
};
#endif