#include "ParamTree.h"
#include "Functions.h"
#include <iterator>
#include <algorithm>
#include <iostream>
namespace SIM
{
ParamNode::
ParamNode(Eigen::VectorXd _pn, std::vector<Eigen::VectorXd> _d,
		  double _f, int _u) :mParamNormalized(_pn), mDisplacement(_d), mFitness(_f), mUpdated(_u),
							  mLeft(0), mRight(0)
{}
void 
ParamNode::
copy(ParamNode* _p)
{
	mParamNormalized = _p->getParamNormalized();
	mDisplacement = _p->getDisplacement();
	mFitness = _p->getFitness();
	mUpdated = _p->getNumUpdated();
}	
ParamTree::
ParamTree(ParamNode* _p, Eigen::VectorXd _unit) : mRoot(_p), mUnit(_unit)
{
	mNumParam = _p->getParamNormalized().rows();
}
void
ParamTree::
insertNode(ParamNode* _new)
{
	insertRecursive(mRoot, _new, 0);
}
void
ParamTree::
insertRecursive(ParamNode* _root, ParamNode* _new, int _depth) {
	int idx = _depth % mNumParam;
	if(_root->getParamNormalized()[idx] > _new->getParamNormalized()[idx]) {
		if(!_root->getLeft()) 
			_root->setLeft(_new);
		else
			insertRecursive(_root->getLeft(), _new, _depth+1);
	} else {
		if(!_root->getRight()) 
			_root->setRight(_new);
		else
			insertRecursive(_root->getRight(), _new, _depth+1);
	}
}
void
ParamTree::
deleteNode(ParamNode* _old)
{
	mRoot = deleteRecursive(mRoot, _old, 0);
}
bool
ParamTree::
isCorrectTree(ParamNode* _root, int _depth)
{
	if (_root == 0)
		return true;
	int idx = _depth % mNumParam;

	if(_root->getLeft() && _root->getParamNormalized()[idx] <= _root->getLeft()->getParamNormalized()[idx] )
		return false;
	else if(_root->getRight() && _root->getParamNormalized()[idx] > _root->getRight()->getParamNormalized()[idx])
		return false;

	return isCorrectTree(_root->getLeft(), _depth+1) && isCorrectTree(_root->getRight(), _depth+1);
}
ParamNode*
ParamTree::
deleteRecursive(ParamNode* _root, ParamNode* _old, int _depth) {
	if(_root == 0) {
		std::cout << "DELETE FAIL" << std::endl;
		return 0;
	}
	int idx = _depth % mNumParam;

	if(isSame(_root->getParamNormalized(), _old->getParamNormalized())) {
		if(_root->getRight() != 0) {
			ParamNode* min = findMinRecursive(_root->getRight(), idx, _depth+1);
			_root->copy(min);
			_root->setRight(deleteRecursive(_root->getRight(), min, _depth+1));
		} else if(_root->getLeft() != 0) {
			ParamNode* min = findMinRecursive(_root->getLeft(), idx, _depth+1);

			_root->copy(min);
			_root->setRight(deleteRecursive(_root->getLeft(), min, _depth+1));
			_root->setLeft(0);
		} else {
			delete _root;
			return 0;
		}
	} else {
		if(_root->getParamNormalized()[idx] > _old->getParamNormalized()[idx])
			_root->setLeft(deleteRecursive(_root->getLeft(), _old, _depth+1));
		else
			_root->setRight(deleteRecursive(_root->getRight(), _old, _depth+1));
	}
	return _root;
}
ParamNode*
ParamTree::
findMinRecursive(ParamNode* _root, int _idx, int _depth)
{
	if(_root == 0) {
		return 0;
	}

	int idx = _depth % mNumParam;
	if(idx == _idx) {
		if(!_root->getLeft()) {
			return _root;
		}
		else {
			return findMinRecursive(_root->getLeft(), _idx, _depth+1); 
		}
	}
		
	ParamNode* minLeft = findMinRecursive(_root->getLeft(), _idx, _depth+1);
	ParamNode* minRight = findMinRecursive(_root->getRight(), _idx, _depth+1);
	
	if(!minLeft && !minRight) {
		return _root;
	}

	else if(!minLeft) {
		if(_root->getParamNormalized()[_idx] <= minRight->getParamNormalized()[_idx]) {
			return _root;
		}
		
		return minRight;

	} else if(!minRight) {
		if(minLeft->getParamNormalized()[_idx] <= _root->getParamNormalized()[_idx])
			return minLeft;
		
		return _root;
	} else {
		if(minLeft->getParamNormalized()[_idx] <= _root->getParamNormalized()[_idx] &&
		   minLeft->getParamNormalized()[_idx] <= minRight->getParamNormalized()[_idx])
			return minLeft;

		if(_root->getParamNormalized()[_idx] <= minLeft->getParamNormalized()[_idx] &&
		   _root->getParamNormalized()[_idx] <= minRight->getParamNormalized()[_idx])
			return _root;

		return minRight;
	}
}
std::vector<std::pair<double, ParamNode*>>
ParamTree::
getNeighborNodes(Eigen::VectorXd _pn, double _radius)
{
	std::vector<std::pair<double, ParamNode*>> results;
	getNeighborNodesRecursive(mRoot, _pn, _radius, 0, results);
	return results;
}
void
ParamTree::
getNeighborNodesRecursive(ParamNode* _root, Eigen::VectorXd _pn, double _radius, int _depth, 
						  std::vector<std::pair<double, ParamNode*>>& _results)
{
	if (_root == 0)
		return;

	int idx = _depth % mNumParam;

	double dist = getDistance(_root->getParamNormalized(), _pn);
	if(dist <= _radius) {
		_results.push_back(std::pair<double, ParamNode*>(dist, _root));
	}

	Eigen::VectorXd projected = _pn;
	projected[idx] = _root->getParamNormalized()[idx];
	dist = getDistance(projected, _pn);
	if(dist <= _radius || _root->getParamNormalized()[idx] <= _pn[idx]) {
		getNeighborNodesRecursive(_root->getRight(), _pn, _radius, _depth+1, _results);
	}
	if(dist <= _radius || _root->getParamNormalized()[idx] > _pn[idx]) {
		getNeighborNodesRecursive(_root->getLeft(), _pn, _radius, _depth+1, _results);
	}
}
std::vector<std::pair<double, ParamNode*>>
ParamTree::
getKNearestNodes(Eigen::VectorXd _pn, int _n)
{
	std::priority_queue<std::pair<double, ParamNode*>, std::vector<std::pair<double, ParamNode*>>, distComp> heap;
	getKNearestNodesRecursive(mRoot, _pn, _n, 0, heap);

	std::vector<std::pair<double, ParamNode*>> results;
	while(heap.size() != 0) {
		results.push_back(heap.top());
		heap.pop();
	}
	return results;
}
void 
ParamTree::
getKNearestNodesRecursive(ParamNode* _root, Eigen::VectorXd _pn, int _n, int _depth, 
							   std::priority_queue<std::pair<double, ParamNode*>, std::vector<std::pair<double, ParamNode*>>, distComp>& _heap)
{
	if(_root == 0)
		return;
	double dist = getDistance(_root->getParamNormalized(), _pn);

	if(_heap.size() < _n)
		_heap.push(std::pair<double, ParamNode*>(dist, _root));	
	else {
		if(dist < _heap.top().first) {
			_heap.pop();
			_heap.push(std::pair<double, ParamNode*>(dist, _root));	
		}
	}

	int idx = _depth % mNumParam;
	Eigen::VectorXd projected = _pn;
	projected[idx] = _root->getParamNormalized()[idx];
	if(_root->getParamNormalized()[idx] <= _pn[idx]) {
		getKNearestNodesRecursive(_root->getRight(), _pn, _n, _depth+1, _heap);
		if(_heap.size() < _n || getDistance(projected, _pn) < _heap.top().first) {
			getKNearestNodesRecursive(_root->getLeft(), _pn, _n, _depth+1, _heap);
		}
	} else {
		getKNearestNodesRecursive(_root->getLeft(), _pn, _n, _depth+1, _heap);
		if(_heap.size() < _n || getDistance(projected, _pn) < _heap.top().first) {
			getKNearestNodesRecursive(_root->getRight(), _pn, _n, _depth+1, _heap);
		}
	}

}
std::vector<ParamNode*> 
ParamTree::
traverse()
{
	std::vector<ParamNode*> params;
	traverseRecursive(mRoot, params);
	return params;
}
void 
ParamTree::
traverseRecursive(ParamNode* _root, std::vector<ParamNode*>& _results)
{
	if (_root == 0)
		return;

	traverseRecursive(_root->getLeft(), _results);
	_results.push_back(_root);
	traverseRecursive(_root->getRight(), _results);

}
double 
ParamTree::
getDistance(Eigen::VectorXd _p0, Eigen::VectorXd _p1)
{
	double r = 0;
	for(int i = 0; i < mNumParam; i++) {
		r += pow((_p0(i) - _p1(i)), 2) * pow(mUnit(i), 2);
	}
	return std::sqrt(r);
}
std::vector<ParamNode*> 
ParamTree::
traverseAndRebuild()
{
	std::vector<ParamNode*> params;
	traverseRecursive(mRoot, params);

	mRoot = buildRecursive(0, params);
	return params;
}
void 
ParamTree::
build(std::vector<ParamNode*> _params)
{
	mRoot = buildRecursive(0, _params);
}
ParamNode* 
ParamTree::
buildRecursive(int _depth, std::vector<ParamNode*> _params)
{
	if(_params.size() == 0)
		return 0;

	int idx = _depth % mNumParam;
	std::stable_sort(_params.begin(), _params.end(), [&idx] (ParamNode* i,ParamNode* j) { 
		return (i->getParamNormalized()[idx] < j->getParamNormalized()[idx]); 
	});

	int middle = _params.size()/2;
	for(int i = middle; i >= 0; i--) {
		if(i != middle && _params[middle]->getParamNormalized()[idx] == _params[i]->getParamNormalized()[idx]) 
			middle = i;
		else if(_params[middle]->getParamNormalized()[idx] > _params[i]->getParamNormalized()[idx])
			break;
	}
	ParamNode* root = _params[middle];
	std::vector<ParamNode*> leftHalf(_params.begin(), _params.begin() + middle);
	std::vector<ParamNode*> rightHalf(_params.begin() + (middle + 1), _params.end());

	ParamNode* left = buildRecursive(_depth + 1, leftHalf);
	ParamNode* right = buildRecursive(_depth + 1, rightHalf);

	root->setLeft(left);
	root->setRight(right);

	return root; 
}
}
