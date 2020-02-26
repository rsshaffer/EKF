#include "EKF.h"
#include <fstream>
#include <iostream>





int main()
{


    int n = 3, m = 2; // n -states m -measurements
    double dt = 0.1; // timestep

    MatrixXd Q(n,n), P(n,n), R(m,m), H(m,n); //

    // Process noise covariance
    Q << .000001, 0, 0, 0, .0001, 0, 0, 0, 0; 
    // Sensor noise covariance
    R << 2, 0,0, 2;
    // State covariance
    P << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    // Measurement model (Linear)
    H << 1, 0, 0, 0, 1, 0;


    // Initialize an EKF 
    EKF ekf(Q,P,R,H,dt);


    double tfinal = 50, t = 0;

    VectorXd x0(n), M(m), p(2);

    // Initial state
    x0 << 50, 50, 0; 
    // Model parameters (velocity, radius)
    p << 5, 40;
    
    State X(x0,p);

    ifstream measurements;
    ofstream Output;
    Output.open("Filtered.txt");
    measurements.open("m.txt");
    
    while (t <= tfinal)
    {    
        //read in the measurements from the txt file
        measurements >> M[0];
        measurements >> M[1];
        // EKF predict and update step. X is updated.
        ekf.Update(X,M);
        
        Output << to_string(X.x[0]) + "," + to_string(X.x[1]) + "," + to_string(X.x[2]) + "\n";
        t += dt;
    }
    
    Output.close();
    measurements.close();

    return 0;

    
}


