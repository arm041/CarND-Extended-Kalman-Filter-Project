#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if (estimations.size() == 0){
	    cout << "error happened size of measurement is 0!!!!" << endl;
	    return rmse;

	}

    if (estimations.size() != ground_truth.size()){
        cout << "Error the dimensions don't fit!!!"<< endl;
        return rmse;

    }

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd temp(4);
		temp = ((estimations[i] - ground_truth[i]).array() )* ((estimations[i] - ground_truth[i]).array());
	    rmse += temp;

	}

	//calculate the mean
	// ... your code here

    rmse = rmse / estimations.size();
	//calculate the squared root
	// ... your code here
    rmse = rmse.array().sqrt();
	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE
    float px2 = px * px;
    float py2 = py * py;



    Hj << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

	//check division by zero
	if (px2 + py2 < 0.0001){

	    cout << "Error division by 0" << endl;
	    return Hj;
	}

	//compute the Jacobian matrix

    Hj (0,0) = px / sqrt(px2 + py2);
    Hj (0,1) = py / sqrt(px2 + py2);
    Hj (1,0) = -py / (px2 + py2);
    Hj (1,1) = px / (px2 + py2);
    Hj (2,0) = py * (vx*py - vy*px) / sqrt((px2 + py2)* (px2 + py2) * (px2 + py2));
    Hj (2,1) = px * (vy*px - vx*py) / sqrt((px2 + py2)* (px2 + py2) * (px2 + py2));;
    Hj (2,2) = px / sqrt(px2 + py2);
    Hj (2,3) = py / sqrt(px2 + py2);

	return Hj;
}

