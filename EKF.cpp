#include "EKF.h"



void EKF::Update(State &X, VectorXd &M)
{
    // Takes the current state X and a new measurement M and predicts the state and updates according to the EKF algorithm
    MatrixXd J = X.DF(X,dt);
    
    //predict next state

    X.f(X,dt); 
    P = J*P*J.transpose() + Q;

    // update

    K = P*H.transpose()*(H*P*H.transpose() + R).inverse();
    X.x = X.x + K*(M-H*X.x);
    P = (I - K*H)*P;
}