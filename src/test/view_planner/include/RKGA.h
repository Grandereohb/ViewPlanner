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
    /*
	 * Default constructor
	 * Required hyperparameters:
	 * - n: number of genes in each chromosome
	 * - p: number of elements in each population
	 * - pe: pct of elite items into each population
	 * - pm: pct of mutants introduced at each generation into the population
	 * - rhoe: probability that an offspring inherits the allele of its elite parent
	 *
	 * Optional parameters:
	 * - K: number of independent Populations
	 * - MAX_THREADS: number of threads to perform parallel decoding
	 *                WARNING: Decoder::decode() MUST be thread-safe; safe if implemented as
	 *                + double Decoder::decode(std::vector< double >& chromosome) const
	 */
    RKGA(int p, double pe, double pm, double rhoe, int MAX_THREADS, double coverage_rate, vector<ViewPoint> candVP, Graph *graph, vector<vector<int>> visibility_matrix) throw(std::range_error);
    ~RKGA();

	vector<ViewPoint> solveRKGA(int maxGen);  // RKGA求解最优视点和测量路径

private:
    //const int n;	// number of genes in the chromosome
	const int p;	// number of elements i n the population
	const double pe;	// number of elite items in the population
	const double pm;	// number of mutants introduced at each generation into the population
    const double rhoe;	// probability that an offspring inherits the allele of its elite parent
    const int MAX_THREAD;
    const double coverage_rate;
    const Graph *graph;
    const vector<ViewPoint> candVP;
    const vector<vector<int>> visibility_matrix;

    Population* previous;
    Population* current;

    void initialize(Population& curr);
    void setRandomKey(int candSize, vector<pair<double, int>>& RK_index);
    void sortRK(vector<pair<double,int>>& RK_index);
    void calcMotionCost(vector<ViewPoint> single, double &cost);
    bool isMostCovered(vector<ViewPoint> single);
    void evolve(Population &curr, Population &next);
    double getBestFitness() const;
};
#endif