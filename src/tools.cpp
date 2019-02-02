#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  //Create the rmse vetor and initialize it with zeros
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  //Check to make sure the estimations vector size is not zero
  if(estimations.size() == 0){
    cout << "Estimation vector has zero size";
    return rmse;
  }
  
  //Check to make sure that the estimation vector and the ground truth vector have the same size
  if(estimations.size() != ground_truth.size()){
    cout << "Estimation vector and Ground Truth vector are not the same size"; 
    return rmse;
  }
  
  for (int i=0; i < estimations.size(); ++i){
    //Get the difference between the estimation and the ground truth and save as residual
    VectorXd residual = estimations[i] - ground_truth[i];
    //residual is now a vector so to get R^2 we need to multiply together
    residual = residual.array()*residual.array();
    //accumulate this into the rmse variable
    rmse += residual;    
  }
  
  //Calculate the mean of rmse
  rmse = rmse/estimations.size();
  //Calculate the square root
  rmse = rmse.array().sqrt();
  //return the result
  return rmse;  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  //Create the 3x4 matrix
  MatrixXd Hj(3,4);
  //Get state parameters from the x_state vector that is passed in
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  //Use the precompute that was used in the class lesson. Allows for cleaner code
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  
  //Check to make sure that we are not going to divide by zero (this is the px*px+py*py)
  if (fabs(c1) < 0.0001){
   cout << "CalculateJacobian function created divide by zero error";
    return Hj;
  }
  
  //Now that we know we will not have a division by zero error we can compute the Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
  		-(py/c1), (px/c1), 0, 0,
  		py*(vx*py-vy*px)/c3, px*(px*vy-py*vx)/c3, px/c2, py/c2;
  
  return Hj;  
}
