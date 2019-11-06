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
MatrixXf Sigma;  // Error covariance matrix. Sometimes called P.
MatrixXf Q(4,4); // Covariance of the process noise.
MatrixXf R(4,4);  // Covariance of the observation noise
MatrixXf C(4,4);  // Observation matrix. Sometimes called H.
MatrixXf S(4,4);
MatrixXf P(4,4);
MatrixXf K(4,4);
VectorXf xu(4);  // State vector. Sometimes called X.
VectorXf xn(4);
VectorXf zu(4);  // State vector. Sometimes called X.
VectorXf y_t(4);  // State vector. Sometimes called X.
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
    printf("A\n");
    A << 1, 0, dt, 0,
         0, 1, 0, -dt,
         0, 0, 1, 0,
         0, 0, 0, 1;

    std::cout << A;

    printf("\n********************\n");
    printf("B\n");
    float tmp_helper = -0.5*dt*dt;
    B << 0, tmp_helper, 0, -dt;
    std::cout << B;

    printf("\n******************\n");
    printf("C\n");
    C << MatrixXf::Identity(4, 4);
    std::cout << C;
    printf("\n******************\n");
    printf("R\n");
    R << MatrixXf::Identity(4, 4);
    R(0,0) = 30*30;
    R(1,1) = 30*30;
    R(2,2) = 400*400;
    R(3,3) = 400*400;
    std::cout << R;
    printf("\n******************\n");
    printf("Q\n");
    Q << MatrixXf::Identity(4, 4);
    Q(0,0) = 30*30;
    Q(1,1) = 30*30;
    Q(2,2) = 400*400;
    Q(3,3) = 400*400;
    std::cout << Q;
    printf("\n******************\n");
    printf("P\n");
    P << MatrixXf::Zero(4, 4);

    std::cout << P;
    printf("\n******************\n");


    xu << 0, 0, 0, 0;
    xn << 0, 0, 0, 0;
    zu << 0, 0, 0, 0;
    y_t << 0, 0, 0, 0;

    Gui gui;
    System system;

    //> comment line below in order to let the program
    //> continue right away
    gui.startInteractor();

    Point measurement;
    Point truth;
    Point kfPosition;

    for (int i = 1; i < nSteps; i++)
    {
        system.makeStep();
        truth = system.getTruthPosition();
        measurement = system.getMeasurement();
        //printf("%d %f %f\n", i, -(prevMeas.x-truth.x)/dt, -(prevMeas.y-truth.y)/dt);

        kfPosition = KalmanFilter(measurement);
        gui.setPoints(truth, measurement, kfPosition);
        printf("********************");
        printf("\ni = %d\t %.2f %.2f --- %.2f %.2f\nXn:\n", i, kfPosition.x, kfPosition.y, truth.x, truth.y);
        std::cout << xn;
        printf("\n");
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

  zu(2) = -(measuredPosition.x - zu(2))/dt;
  zu(3) = -(measuredPosition.y - zu(3))/dt;
  zu(0) = measuredPosition.x;
  zu(1) = measuredPosition.y;

  static int init_checker = 0;

  xu =  A * xn + B*g;

  P = A * P * A.transpose() + Q;
  y_t = zu - C * xu;
  S = C * P * C.transpose() + R;
  K = P * C.transpose() * S.inverse();
  printf("\n*****XU******\n");
  std::cout << xu;
  printf("\n*****K ******\n");
  std::cout << (K);
  xn = xu + K * y_t;
  prevMeas = measuredPosition;

  init_checker++;

  return Point(xn(0), xn(1));
}
