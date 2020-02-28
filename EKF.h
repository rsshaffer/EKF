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
    int n;
    function<void(State &x, double dt)> f;
    function<MatrixXd(State &x, double dt) DF;

    State(VectorXd x, VectorXd p, function<void(State x, double dt)> f,function<MatrixXd(State x, double dt) DF) 
    : x(x), p(p), n(x.size()), f(f), DF(DF) 
    {};

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



