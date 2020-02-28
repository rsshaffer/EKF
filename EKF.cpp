#include "EKF.h"

void f(State &x, double dt)
{
    // Euler discretized dynamics of a dubins vehicle
    // xdot = v cos(x)
    // ydot = v sin(y)
    // thetadot = v/r  <-- produces a circle of radius r
    float v = p[0], r = p[1];
    x[0] = x[0] + dt*(v*cos(x[2]));
    x[1] = x[1] + dt*(v*sin(x[2]));
    x[2] = x[2] + dt*(v/r);

}

MatrixXd DF(State &x, double dt)
{
    // Returns the jacobian of the system f()
    int n = x.n;
    float v = p[0], r = p[1];
    MatrixXd J(n,n);

    J << 1, 0, -dt*v*sin(x[2]), 0, 1, dt*v*cos(x[2]), 0, 0, 1;

    return J;
}

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