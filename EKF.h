#include <Eigen/Dense>
#include <math.h>


using namespace std;
using namespace Eigen;

// This class represents the dynamic system with governing equations and jacobian.
class State
{
public:
    VectorXd x; //states
    VectorXd p; //parameters

    State(VectorXd x, VectorXd p) : x(x), p(p) {};
    void f(double dt);
    MatrixXd DF(double dt);
    int GetN() {return this->x.size();}
};


class EKF 
{
public:
    double dt;
    int n;
    MatrixXd Q, P, R, H, K, I;
    
    

    EKF(MatrixXd Q, MatrixXd P, MatrixXd R, MatrixXd H, double dt):Q(Q), P(P), R(R),H(H), n(Q.rows()), dt(dt), I(n,n)
    {
        I.setIdentity();
    };


    void Update(State &X, VectorXd &M);
    
};



