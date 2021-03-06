#include "RigidBodyManipulator.h"
#include <iostream>

using namespace Eigen;
using namespace std;


// Computes surface tangent vectors for a single normal vector
// INPUTS:
//   normal: (3 x 1) normal vector in world coordinates
// OUTPUTS:
//   d: (3 x k) matrix where the k columns contain the surface tangents
// NOTE:
//  k = BASIS_VECTOR_HALF_COUNT is defined as a preprocessor directive so that 
//      Eigen templates can be optimized at compile time
void surfaceTangentsSingle(Vector3d const & normal, Matrix3kd & d)
{
  Vector3d t1,t2;
  double theta;
  
  if (1.0 - normal(2) < EPSILON) { // handle the unit-normal case (since it's unit length, just check z)
    t1 << 1.0, 0.0, 0.0;
  } else if (1 + normal(2) < EPSILON) {
    t1 << -1.0, 0.0, 0.0;  //same for the reflected case
  } else {// now the general case
    t1 << normal(1), -normal(0), 0.0;
    t1 /= sqrt(normal(1)*normal(1) + normal(0)*normal(0));
  }
  
  t2 = t1.cross(normal);

  for (int k = 0 ; k < BASIS_VECTOR_HALF_COUNT ; k++) {
    theta = k*M_PI/BASIS_VECTOR_HALF_COUNT;
    d.col(k)=cos(theta)*t1 + sin(theta)*t2;
  }
}


// Consolidates unique body indexes from idXA and idxB into a sorted set
// INPUTS:
//   idxA mx1 vector of body indexes for body A in m possible contacts
//   idxB mx1 vector of bodyh indexes for body B in m possible contacts
// OUTPUTS:
//   bodyIndsSorted a set of unique, sorted(ascending) body indexes participating in contact pairs
void getUniqueBodiesSorted(VectorXi const & idxA, VectorXi const & idxB, std::vector<int> & bodyIndsSorted)
{
  size_t m = idxA.size();
  std::set<int> bodyInds;
  
  for (int i = 0 ; i < m ; i++) {  
    bodyInds.insert(idxA[i]);
    bodyInds.insert(idxB[i]);
  }
  
  bodyIndsSorted.clear();

  for (std::set<int>::const_iterator citer = bodyInds.begin() ; citer != bodyInds.end() ; citer++) {
    if ( *citer > 1 ) {
      bodyIndsSorted.push_back(*citer);
    }
  }
  
  sort(bodyIndsSorted.begin(), bodyIndsSorted.end());
}


//Finds the n occurences of the body index in the original list of m contact indexes
// INPUTS:
//   idxList: (m x 1) Indexes of m bodies for possible contact pairs
//   bodyIdx: the body index to search for
// OUTPUTS:
//   contactIdx: the list of n indexes into idxList where bodyIdx occurred
void findContactIndexes(VectorXi const & idxList, const int bodyIdx, std::vector<int> & contactIdx)
{
  int m = idxList.size();
  contactIdx.clear();
  for (int i = 0 ; i < m ; i++) {
    if (idxList[i] == bodyIdx) {
      contactIdx.push_back(i); //zero-based index 
    }
  }
}

// Gets a concatenated list of contact points corresponding to a single body regardless of whether it is body A or body B
//  This allows all the points to be transformed into world space using a single forwardKin call.
// INPUTS:
//   cindA: list of indexes into the original list of m contact pairs where the body appeared as body A
//   cindB: list of iindexes into the original list of m contact pairs where the body appeared as body B
//   xA: (3 x m) matrix where each column represents a contact point on body A
//   xB: (3 x m) matrix where each column represents a contact point on body B
// OUTPUTS:
//   bodyPoints: (4 x n) matrix of contact points containing occurences of body A contact points followed
//     by body B contactpoints where n = size(cindA) + size(cindB)
// NOTE: the output is a matrix of 4-vector columns in homogeneous coordinates (x,y,z,1)'
void getBodyPoints(std::vector<int> const & cindA, std::vector<int> const & cindB, Matrix3xd const & xA, Matrix3xd const & xB, MatrixXd & bodyPoints)
{
  int i = 0;
  int numPtsA = cindA.size();
  int numPtsB = cindB.size();

  bodyPoints.resize(4, numPtsA + numPtsB);

  for (i = 0 ; i < numPtsA ; i++ ) {
    bodyPoints.col(i) << xA.col(cindA[i]), 1.0; //homogeneous coordinates
  }

  for (i = 0 ; i < numPtsB ; i++ ) {
    bodyPoints.col(numPtsA + i) << xB.col(cindB[i]), 1.0;
  }
}

// Builds the part of the contact Jacobian matrix corresponding to bodyInd
//  by accumulating positive contributions  from points on body A and negative 
//  contributions from the points on body B.
//  This is because the normal vectors(n = dphi/dq) are defined to be pointing from B to A
// INPUTS
//   bodyInd: the index of the body currently being processed
//   bodyPoints: (3 x n) matrix where each column is a point on the body
//   cindA: indexes into the original set of m contact pairs where the body appears as Body A
//   cindB: indexes into the original set of m contact pairs where the body appears as Body B
// NOTE 
//   cindA and cindB are gotten by calling findContactIndexes in drakeContactConstraintsUtil
//   cols(bodyPoints) = size(cindA) + size(cindB)
// OUTPUTS:
//   J: (3m x nq) The partial contact Jacobian matrix
// NOTE
//  After one call to the function, the n rows of the Jacobian matrix corresponding to bodyInd will be completed
//  This function must be called with all bodyInds to finish the total accumulation of the contact Jacobian

void RigidBodyManipulator::accumulateContactJacobian(const int bodyInd, MatrixXd const & bodyPoints, std::vector<int> const & cindA, std::vector<int> const & cindB, MatrixXd & J)
{
  const int nq = J.cols();
  const int numPts = bodyPoints.cols();
  const size_t numCA = cindA.size();
  const size_t numCB = cindB.size();
  const size_t offset = 3*numCA;

  MatrixXd J_tmp(3*numPts, nq);
  forwardJac(bodyInd - 1, bodyPoints, 0, J_tmp);

  //add contributions from points in xA
  for (int x = 0 ; x < numCA ; x++) {
    J.block(3*cindA[x], 0, 3, nq) += J_tmp.block(3*x, 0, 3, nq);
  }

  //subtract contributions from points in xB
  for (int x = 0 ; x < numCB ; x++) {
    J.block(3*cindB[x], 0, 3, nq) -= J_tmp.block(offset + 3*x, 0, 3, nq);
  }
}

// Same as above but computes the second order contact Jacobian
// OUTPUTS:
//  dJ: (3m x nq^2) Second order contact Jacobian
void RigidBodyManipulator::accumulateSecondOrderContactJacobian(const int bodyInd, MatrixXd const & bodyPoints, std::vector<int> const & cindA, std::vector<int> const & cindB, MatrixXd & dJ)
{
  const int dJCols = dJ.cols(); //nq^2 instead of nq
  const int numPts = bodyPoints.cols();
  const size_t numCA = cindA.size();
  const size_t numCB = cindB.size();
  const size_t offset = 3*numCA;
  MatrixXd dJ_tmp(3*numPts, dJCols);
  forwarddJac(bodyInd - 1, bodyPoints, dJ_tmp); //dJac instead of Jac

  //add contributions from points in xA
  for (int x = 0 ; x < numCA ; x++) {
    dJ.block(3*cindA[x], 0, 3, dJCols) += dJ_tmp.block(3*x, 0, 3, dJCols);
  }

  //subtract contributions from points in xB
  for (int x = 0 ; x < numCB ; x++) {
    dJ.block(3*cindB[x], 0, 3, dJCols) -= dJ_tmp.block(offset + 3*x, 0, 3, dJCols);
  }  
}

// Computes the full contact Jacobian and, optionally, the second order contact Jacobian.  
//  This can be used to compute the contact normals in joint coordinates (n = dphi/dq), the surface tangents
//  in joint coordinates (D), and their respective second derivatives with respect to q (dn, dD)
// INPUTS
//   idxA: (m x 1) an integer list of body indexes of body A for m possible contact pairs
//   idxB: (m x 1) an integeer list of body indexes of body B for m possible contact pairs
//   xA: (3 x m) each column of the matrix is a contact point in the body A frame for that contact pair
//   xB: (3 x m) each column of the matrix is a contact point in the body B frame for that contact pair
//   compute_second_derivatives: boolean flag to indicate that the second order contact Jacobian should also be computed
// OUTPUTS
//   J: (3m x nq)
//  (optional outputs if compute_second_derivatives is true)
//  dJ: (3m x nq^2) Second order contact Jacobian

void RigidBodyManipulator::computeContactJacobians(Map<VectorXi> const & idxA, Map<VectorXi> const & idxB, Map<Matrix3xd> const & xA, Map<Matrix3xd> const & xB, const bool compute_second_derivatives, MatrixXd & J, MatrixXd & dJ)
{
  std::vector<int> bodyInds;
  const int nq = num_positions;
  const int numContactPairs = xA.cols();

  J = MatrixXd::Zero(3*numContactPairs, nq);
  dJ = MatrixXd::Zero(3*numContactPairs, nq*nq);
  
  getUniqueBodiesSorted(idxA, idxB, bodyInds);
  
  const int numUniqueBodies = bodyInds.size();

  for (int i = 0; i < numUniqueBodies ; i++) {
    const int bodyInd = bodyInds[i];
    vector<int> cindA, cindB;
    MatrixXd bodyPoints;
    findContactIndexes(idxA, bodyInd, cindA);
    findContactIndexes(idxB, bodyInd, cindB);
    getBodyPoints(cindA, cindB, xA, xB, bodyPoints);
    accumulateContactJacobian(bodyInd, bodyPoints, cindA, cindB, J);
    if (compute_second_derivatives) {
      accumulateSecondOrderContactJacobian(bodyInd, bodyPoints, cindA, cindB, dJ);
    }
  } 
}

// Computes surface tangent vectors for many normal vectors at once
// INPUTS:
//   normals: (3 x m) matrix where each column is a normal vector in world coordinates
// OUTPUTS:
//   tangents: list of k (3 x m) matrices where each column contains one of the k tangent vectors
//              for the corresponding normal.
// NOTE:
//  k = BASIS_VECTOR_HALF_COUNT is defined as a preprocessor directive so that 
//      Eigen templates can be optimized at compile time
void RigidBodyManipulator::surfaceTangents(Map<Matrix3xd> const & normals, std::vector< Map<Matrix3xd> > & tangents)
{
  const int numContactPairs = normals.cols();
  for (int curNormal = 0 ; curNormal < numContactPairs; curNormal++) {
    Matrix3kd d;
    surfaceTangentsSingle(normals.col(curNormal), d);
    for (int k = 0 ; k < BASIS_VECTOR_HALF_COUNT ; k++) {
      tangents[k].col(curNormal) = d.col(k);
    }
  }
}
