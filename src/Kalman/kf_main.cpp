/**
 * Kalman Filter
 * Skeleton code for teaching
 * B3M33MKR
 * Czech Technical University
 * Faculty of Electrical Engineering
 * Intelligent and Mobile Robotics group
 *
 * Authors: Zdeněk Kasl, Karel Košnar kosnar@labe.felk.cvut.cz, Gaël Écorchard
 *
 * this code is inspired by tutorial on Kalman Filter by Greg Czerniak
 *
 * Licence: MIT (see LICENSE file)
 **/

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>

#include "gui/gui.h"
#include "systemSimulator/system.h"

using namespace imr;
using namespace gui;
using namespace Eigen;

Point KalmanFilter(const Point & measuredPosition);
Point prevMeas (0,0);

MatrixXf A(4,4);
MatrixXf B(4,1);
MatrixXf Q(2,2); // Covariance of the process noise.
MatrixXf R(4,4);  // Covariance of the observation noise
MatrixXf C(2,4);  // Observation matrix. Sometimes called H.
MatrixXf S(2,2);
MatrixXf P(4,4);
MatrixXf P_est(4,4);
MatrixXf K(4,2);
MatrixXf I(4,4);
VectorXf cur_state(4);  // State vector. Sometimes called X.
VectorXf x_predict(4);
VectorXf cur_meas(2);  // State vector. Sometimes called X.
VectorXf innovation(2);
float dt = 0.1;
float g = 9.81;


void help(char** argv)
{
    std::cout << "\nUsage of the program " << argv[0]+2 << std::endl
              << "Parameter [-h or -H] displays this message." <<std::endl
              << " Parametr [-n or -N] number of simulation steps."
              << std::endl;
}

int main(int argc, char** argv)
{
    int nSteps = 1000;
    char *dataFile;
    // Parse all console parameters
    for (int i = 0; i < argc; i++)
    {
        if (argv[i][0] == '-')
        {
            switch(argv[i][1])
            {
                //> HELP
                case 'H' : case 'h' :
                    help(argv);
                    exit(EXIT_SUCCESS);
                case 'N': case 'n':
                    assert(i + 1 < argc);
                    assert(atoi(argv[i + 1]) > 1);
                    nSteps = atoi(argv[i + 1]);
                    break;
                default :
                    std::cout << "Parameter \033[1;31m" << argv[i] << "\033[0m is not valid!\n"
                              << "Use parameter -h or -H for help." << std::endl;
                    break;
            }
        }
    }
    // All parameters parsed
    A << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;
    float tmp_helper = -0.5*dt*dt;
    B << 0, tmp_helper, 0, -dt;
    C << MatrixXf::Identity(2, 4);
    C(0,0) = 1;
    C(1,1) = 1;

// """ MEASUREMENT UNCERTAINTY """
    Q << MatrixXf::Identity(2, 2);
    Q(0,0) = 5;
    Q(1,1) = 5;
//    Q(2,2) = 4;
//    Q(3,3) = 4;
// """ PREDICTION UNCERTAINTY """
    R << MatrixXf::Identity(4, 4);
    R(0,0) = 0.1;
    R(1,1) = 0.1;
    R(2,2) = 0.1;
    R(3,3) = 0.1;


    I << MatrixXf::Identity(4,4);
    P << MatrixXf::Zero(4, 4);
    P_est << MatrixXf::Zero(4, 4);


    cur_state << 0, 0, 10, 10;
    cur_meas << 0, 0;
    innovation << 0, 0;

    Gui gui;
    System system;

    //> comment line below in order to let the program
    //> continue right awayprintf("predicted");
    gui.startInteractor();

    Point measurement;
    Point truth;
    Point kfPosition;
    for (int i = 1; i < nSteps; i++)
    {
        system.makeStep();
        truth = system.getTruthPosition();
        measurement = system.getMeasurement();
        kfPosition = KalmanFilter(measurement);
        gui.setPoints(truth, measurement, kfPosition);
        //> comment line below in order to let the program
        //> continue right away
        if (i % 20 == 0)
        {
            gui.startInteractor();
        }

    }
    gui.startInteractor();

    return EXIT_SUCCESS;
}

/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//                               implement Kalman Filter here
/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


Point  KalmanFilter(const Point & measuredPosition)
{

  cur_meas(0) = measuredPosition.x;
  cur_meas(1) = measuredPosition.y;
  //cur_meas(2) = 0.01*(measuredPosition.x - cur_state(2))/(abs(measuredPosition.x - cur_state(2))*dt);
  //cur_meas(3) = 0.01*(measuredPosition.y - cur_state(3))/(abs(measuredPosition.x - cur_state(2))*dt);
// prediction
  x_predict =  A * cur_state + B*g;
  P_est = A * P * A.transpose() + R;
// correction
  S = C * P_est * C.transpose() + Q;
  K = P_est * C.transpose() * S.inverse();
  innovation = cur_meas - C * x_predict;
  cur_state = x_predict + K * innovation;
// update covariance
  P = (I - K*C)*P_est;
// prevMeas = measuredPosition;
  return Point(cur_state(0), cur_state(1));
}
