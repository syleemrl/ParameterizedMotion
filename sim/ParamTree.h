#ifndef __SIM_PARAM_TREE_H__
#define __SIM_PARAM_TREE_H__
#include <vector>
#include <tuple>
#include <Eigen/Dense>
#include <queue>

namespace SIM
{

class ParamNode
{
public:
	ParamNode(Eigen::VectorXd _pn, std::vector<Eigen::VectorXd> _d,
			  double _f, int _u);
	ParamNode* getLeft() { return mLeft; }
	ParamNode* getRight() { return mRight; }
	void setLeft(ParamNode* _l) { mLeft = _l; }
	void setRight(ParamNode* _r) { mRight = _r; }
	void decreaseNumUpdated() { mUpdated -= 1; }	
	void copy(ParamNode* _p);
	Eigen::VectorXd getParamNormalized() { return mParamNormalized; }
	std::vector<Eigen::VectorXd> getDisplacement() { return mDisplacement; }
	double getFitness() { return mFitness; }
	int getNumUpdated() { return mUpdated; }
private:
	ParamNode* mLeft;
	ParamNode* mRight;

	Eigen::VectorXd mParamNormalized;
	std::vector<Eigen::VectorXd> mDisplacement;
	double mFitness;
	int mUpdated;
};
struct distComp
{ 
	bool operator()(std::pair<double, ParamNode*> a, std::pair<double, ParamNode*> b) {
	    return a.first < b.first;
	}
};
class ParamTree
{
public:
	ParamTree() {}
	ParamTree(ParamNode* _p, Eigen::VectorXd _unit);
	void insertNode(ParamNode* _new);
	void deleteNode(ParamNode* _old);
	std::vector<std::pair<double, ParamNode*>> getNeighborNodes(Eigen::VectorXd _pn, double _radius);
	std::vector<std::pair<double, ParamNode*>> getKNearestNodes(Eigen::VectorXd _pn, int _n);
	std::vector<ParamNode*> traverse();
	std::vector<ParamNode*> traverseAndRebuild();
	void build(std::vector<ParamNode*> _params);
	double getDistance(Eigen::VectorXd _p0, Eigen::VectorXd _p1);

private:
	void insertRecursive(ParamNode* _root, ParamNode* _new, int _depth);
	ParamNode* deleteRecursive(ParamNode* _root, ParamNode* _old, int _depth);
	ParamNode* findMinRecursive(ParamNode* _root, int _idx, int _depth);
	void getNeighborNodesRecursive(ParamNode* _root, Eigen::VectorXd _pn, double _radius, int _depth, std::vector<std::pair<double, ParamNode*>>& _results);
	void getKNearestNodesRecursive(ParamNode* _root, Eigen::VectorXd _pn, int _n, int _depth, 
								   std::priority_queue<std::pair<double, ParamNode*>, std::vector<std::pair<double, ParamNode*>>, distComp>& _heap);
	void traverseRecursive(ParamNode* _root, std::vector<ParamNode*>& _results);
	ParamNode* buildRecursive(int _depth, std::vector<ParamNode*> _params);
	bool isCorrectTree(ParamNode* _root, int _depth);

	Eigen::VectorXd mUnit;
	ParamNode* mRoot;	
	int mNumParam;
};
}
#endif