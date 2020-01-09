/**
 * Particule filter localization
 * Skeleton code for teaching
 * AE3M33MKR/B3M33MKR
 * Czech Technical University
 * Faculty of Electrical Engineering
 * Intelligent and Mobile Robotics Group
 *
 * Authors:
 * - Miroslav Kulich <miroslav.kulich@cvut.cz>,
 * - Zdeněk Kasl,
 * - Karel Košnar <karel.kosnar@cvut.cz>,
 * - Gaël Écorchard <gael.ecorchard@cvut.cz>
 *
 * Licence: MIT (see LICENSE file)
 **/

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <random>
#include <map>
#include <vtkPNGReader.h>
#include <math.h>
#include <utility>
#include "gui/gui.h"
#include "dataLoader/laserDataLoader.h"
#include "laserSimulator/lasersimulator.h"
#include "typedefs.h"

using namespace imr;
using namespace gui;
using namespace laserDataLoader;
using steady_clock = std::chrono::steady_clock;

std::random_device g_rd;
std::default_random_engine g_eng(g_rd());
std::uniform_real_distribution<double> g_uniform_distribution(0, 1);
std::normal_distribution<double> g_normal_distribution(0, 1);
std::pair <double, Particle> max_weight (0, {{0,0,0},0});

// GLOBAL parameters

// max range is defined in laser simulator script
double z_max = 30.0;
// custom sensor model parameter
double n = 1.0;

RobotPosition prev_pos;
LaserScan scan;
PointList scanPoints;

/* Return one sample of a uniform distribution between min and max
 */
double uniformSample(double min=0, double max=1)
{
    return  min + g_uniform_distribution(g_eng) * (max - min);
}

/* Return one sample of a normal distribution with the given mean and standard deviation
 */
double normalSample(double mean=0, double sigma=1)
{
    return mean + g_normal_distribution(g_eng) * sigma;
}

inline double toRadians(double alpha);

// Convert RobotPosition to Point structure (for drawing).
Point robotPosition2point(const RobotPosition& rp);

// Convert the laser scan into vector of points in Cartesian coordinate system
// using the odometry from the Measurement structure.
PointList calculateRawPoints(const Measurement& m);

// Convert the laser scan into vector of points in Cartesian coordinate system
// using given robot position.
PointList calculateRawPoints(const Measurement& m, const RobotPosition& p);

// Convert point in polar coordinate system into Cartesian coordinate system.
Point polar2cartesian(double alpha, double r, const RobotPosition& p0);

void help(char** argv)
{
    std::cout << "\nUsage of the program " << argv[0] << ":\n" <<
       "Parameter [-h or -H] displays this message.\n" <<
       "Parameter [-f or -F] specifies path to data.\n" <<
       "Parameter [-m or -M] specifies number of measurements taken, defaults to 2." <<
       std::endl;
}

// -------------------------------------------------------------------------------------
//  SENSOR MODEL OF PARTICLES
double normalize_angle(double angle){
    // reduction of angle if it exceeds a value bigger than 2 pi
    if (fabs(angle)>2*M_PI) {
      angle = remainder(angle,(2*M_PI));
    }
    return angle;
}
double distribution(double z,double z_star){
    // standard Gaussian distribution
    double stddev = 0.7;
    double function;
    function = (1/(stddev*sqrt(2*M_PI)))*exp(-(z-z_star)*(z-z_star)/(2*stddev*stddev));
    return function;
}
//---------------- Gaussian ---------------------------
double prob_hit(double z, double z_star){
        double function_prob;
        if (z >= 0.0 && z<=z_max)
        {
            function_prob = n*distribution(z,z_star);
        }
        else
        {
            function_prob = 0.0;
        }

        return function_prob;
}
//---------------- Exponential ---------------------------
double prob_short(double z, double z_star){
        double function_prob,lambda;
        lambda = 0.3;

        if (z >= 0.0 && z<=z_star)
        {
            function_prob = n*lambda*exp(-lambda*z);
        }
        else
        {
            function_prob = 0.0;
        }

        return function_prob;
}
//---------------- Flat non-zero ---------------------------
double prob_rand(double z){
        double function_prob;
        function_prob = 0.0;

        if (z >= 0.0 && z<=z_max)
        {
            function_prob = 1/z_max;
        }
        else
        {
            function_prob = 0.0;
        }

        return function_prob;
}
//---------------- Column max range ---------------------------
double prob_max(double z){
        double function_prob;
        function_prob = 0.0;

        if (z==z_max)
        {
            function_prob = 1;
        }
        else
        {
            function_prob = 0.0;
        }

        return function_prob;
}


ParticleVector weightUpdate(ParticleVector init, LaserSimulator simul, LaserScan scanTest){
    double alpha_hit = 0.9; //0.9
    double alpha_short = 1; //1
    double alpha_rand = 1; //1
    double alpha_max = 0.5; //0.5
    LaserScan z, z_star;
    double prob_beam;

    // get 36 beams from robot data
    for (int i = 0; i < 36; i++){
      z_star.push_back(scanTest[i*10]);
    }
    // for each particle
    for (auto &a:init)
    {
      // scan particle
      z = simul.getScan(a.pos);
      // calculate probability of each beam and multiply
      prob_beam = 1;
      for (int i=0;i<z_star.size();i++){
        // I have changed the order of z_star and z because of consistency - TU
        prob_beam = prob_beam*(alpha_hit*prob_hit(z[i],z_star[i]) + alpha_short*prob_short(z[i],z_star[i])+alpha_rand*prob_rand(z[i])+alpha_max*prob_max(z[i]));
      }
      // update weight of each particle
      a.weight = prob_beam;
      // calculate maximal weight
      if (a.weight > max_weight.first)
      {
        max_weight.first = a.weight;
        max_weight.second = a;
      }
    }
    // checking of most probable particle's position
    //printf("%.4f %.4f %.4f %.12f (best) %d %d\n", max_weight.second.pos.x, max_weight.second.pos.y, max_weight.second.pos.phi, max_weight.first, z.size(), z_star.size());
    return init;
}

/// -----------------------------------------------------------------------------------
//  MOTION MODEL OF PARTICLE MOVEMENTS - TIMUR UZAKOV
ParticleVector moveParticles(ParticleVector init, double delta_rot1, double delta_rot2,double delta_trans, LaserSimulator simul){
    double delta_hat_rot1, delta_hat_rot2, delta_hat_trans;
    double alpha1,alpha2,alpha3,alpha4;

    // from videos of motion model --- MM3
    //angle coefficients
    alpha1 = 0.3; // angle
    alpha2 = 0.04; // distance
    //distance coeffcitients
    alpha3 = 0.25; // distance
    alpha4 = 0.04; // two angles

    //calculate new randomized deltas based on previous deltas


    //update each particle with respect to the common new deltas(<delta_hat>s)

    for (auto &a:init)
    {
      //check if the point is on map
      RobotPosition tmp;
      Particle p;
      // do{
          delta_hat_rot1 = normalSample(delta_rot1, alpha1*fabs(delta_rot1)+alpha2*fabs(delta_trans));
          delta_hat_rot2 = normalSample(delta_rot2, alpha1*fabs(delta_rot2)+alpha2*fabs(delta_trans));
          delta_hat_trans = normalSample(delta_trans, alpha3*fabs(delta_trans)+alpha4*(fabs(delta_rot1)+fabs(delta_rot2)));
          //printf("%.2f\t", a.pos.x);
          tmp.x = a.pos.x + delta_hat_trans*cos(a.pos.phi + delta_hat_rot1);
          //printf("%.2f\n", a.pos.x);
          tmp.y = a.pos.y + delta_hat_trans*sin(a.pos.phi + delta_hat_rot1);
          tmp.phi = a.pos.phi + delta_hat_rot1 + delta_hat_rot2;
          //if (tmp.x <= -16.96 || tmp.x >= 19.7243) continue;
          //if (tmp.y <= -43.25 || tmp.y >= 55.0255) continue;
          if (!simul.isFeasible(tmp)){
              do {
                // random particle
                double x,y,phi;
                x = uniformSample(-16.96, 19.7243);
                y = uniformSample(-43.25, 55.0255);
                phi = uniformSample(-M_PI, M_PI);
                p.pos = RobotPosition(x, y, phi);
              }while(!simul.isFeasible(p.pos));
              a.pos = p.pos;
          } else a.pos = tmp;

    }
    return init;
}
// --------------------------------------------------------------------------
// ROULETTE SAMPLER

ParticleVector rouletteSampler(const ParticleVector init, LaserSimulator simul){
    std::map <double, int> hashTable;
    double weightAdder = 0;
    ParticleVector result;
    // create the table of weights
    for (int i = 0; i < init.size(); i++)
    {
        weightAdder += init[i].weight;
        hashTable[weightAdder] = i;
    }

    int cntMaxW = 0;
    double meanWeight = 0.0;

    // resample 85 % of particles
    for (int i = 0; i < (0.95*init.size()); i++)
    {
        //for (int i = 0; i < 10; i++){

        //take random number from 0 to sum of weights
        double tmp = uniformSample(0, weightAdder);

        //printf("%.12f\n", tmp);

        //find particle according to the table
        Particle won = init[hashTable.lower_bound(tmp)->second];
        result.push_back(won);

        // counting how many times the most probable particle was selected
        if (won.pos.x == max_weight.second.pos.x &&  won.pos.y == max_weight.second.pos.y && won.pos.phi == max_weight.second.pos.phi){
          cntMaxW++;
        }
        // add weight of the selected particle ---- can be changed to won.weight
        meanWeight += init[hashTable.lower_bound(tmp)->second].weight;

    }

    //printf("MW won %d\n", cntMaxW);
    //printf("%.4f %.4f %.4f\n", max_weight.second.pos.x, max_weight.second.pos.y, max_weight.second.pos.phi);


    // set 15 % of all particles to random position and mean weight
    meanWeight /= result.size();

    double x;
    double y;
    double phi;

    while (result.size() != init.size())
    {
         // random particle
         x = uniformSample(-16.96, 19.7243);
         y = uniformSample(-43.25, 55.0255);
         phi = uniformSample(-M_PI, M_PI);
         Particle p;
         p.pos = RobotPosition(x, y, phi);

         // check if on map
         if (simul.isFeasible(p.pos))
         {
              p.weight = meanWeight;
              result.push_back(p);
         }
    }

    return result;
}
// ----------------------------------------------------------------

// MAIN
int main(int argc, char** argv)
{
    unsigned int nMeasurements = 10;
    char *dataFile;

    // argument count must be greater than three
    // >>> at least source file must be specified
    if (argc < 3)
    {
        help(argv);
        return EXIT_FAILURE;
    }

    // Parse all console parameters
    for (int i = 0; i < argc; i++)
    {
        if (argv[i][0] == '-')
        {
            switch(argv[i][1])
            {
                //> HELP
                case 'H' :
                case 'h' :
                    help(argv);
                    break;

                //> Source file
                case 'F' :
                case 'f' :
                    assert(i + 1 < argc);
                    dataFile = argv[i + 1];
                    break;

                //> Number of Measurements
                case 'M' :
                case 'm' :
                    assert(i + 1 < argc);
                    assert(atoi(argv[i + 1]) > 1);
                    nMeasurements = static_cast<unsigned int>(atoi(argv[i + 1]));
                    break;

                default :
                    std::cout << "Parameter \033[1;31m" << argv[i] << "\033[0m is not valid!\n" <<
                       "Use parameter -h or -H for help." << std::endl;
                    break;
            }
        }
    }
    // All parameters parsed.

    std::cout << "Max. number of measurements taken: " << nMeasurements << "\n"
        << "Source file: " << dataFile
        << std::endl;

    // Load data.
    LaserDataLoader loader(dataFile, nMeasurements, "FLASER");

    // Exit if no data could be loaded.
    if (loader.empty())
    {
        std::cerr << "No measurement read" << std::endl;
        exit(EXIT_FAILURE);
    }


    Measurement measurement;
    RobotPosition pos;

    // Load the map as a grid.
    vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
    reader->SetFileName("../data/belgioioso-map5b.png");

    //simul(reader)
    imr::LaserSimulator simul(reader);

    Gui gui(reader);
    std::cout << "X: " << simul.grid2realX(0) << " " << simul.grid2realX(1872) << std::endl;
    std::cout << "Y: " << simul.grid2realY(0) << " " << simul.grid2realY(5015) << std::endl;

    // Generate a set of random particles.
    ParticleVector particles;
    double x;
    double y;
    double phi;

    // Motion model variables

    double delta_x, delta_y, delta_phi,theta;
    double delta_rot1,delta_rot2,delta_trans;

    for (size_t i = 0; i < 2000;)
    {
       x = uniformSample(-16.96, 19.7243);
       y = uniformSample(-43.25, 55.0255);
       phi = uniformSample(-M_PI, M_PI);
       Particle p;
       p.pos = RobotPosition(x, y, phi);
       if (simul.isFeasible(p.pos))
       {
          p.weight = 1.0 / 2000.0;
          particles.push_back(p);
          i++;
       }
    }

    /* Example of use of the probability map, to test the sensor model */
    WeightedPointList probabilities;
    for (double x = -16.5; x <= 19.5; x += 0.5)
    {
       for (double y = -43.0; y <= 55.0; y += 0.5)
       {
          double w = (x + 16.5) / (19.5 + 16.5);
          probabilities.push_back(WeightedPoint(x, y, w));
       }
    }
    gui.setProbabilityMap(probabilities, true, 2);
    gui.startInteractor();
    gui.clearProbabilityMap();

    auto begin = steady_clock::now();


    for (size_t i = 0; i < nMeasurements; i++)
    {
         pos = loader[i].position;
         LaserScan scanTest = loader[i].scan; //0th, 10th, 20th
         scan = simul.getScan(pos); //36
         scanPoints = simul.getRawPoints();
         if (i > 0)
         {
           //printf("%.4f %.4f %.4f (real)\n", pos.x, pos.y, pos.phi);
           // MEASUREMENTS AND RELATED CALCULATIONS FOR MOTION MODEL
           // --------------------------------------------------------
           // calculate step position differences from odometry
           delta_x = pos.x - prev_pos.x;
           delta_y = pos.y - prev_pos.y;
           delta_phi = pos.phi - prev_pos.phi;
           // calculate orientation of movement line
           theta = atan2(pos.y-prev_pos.y,pos.x-prev_pos.x); //radians
           // previous state angle delta
           delta_rot1 = theta-prev_pos.phi;
           delta_rot1 = normalize_angle(delta_rot1);


           // current state angle delta
           delta_rot2 = delta_phi - delta_rot1;
           delta_rot2 = normalize_angle(delta_rot2);
           // change in position aka length of step
           delta_trans = sqrt(pow(delta_x,2)+pow(delta_y,2));

           if (fabs(delta_trans) < 0.3 &&  fabs(delta_phi) < 0.15){
             continue;
           }

           particles = rouletteSampler(particles, simul);
           // MAIN PARTICLE FILTER ALGORITHM
           // ----------------------------------------------------------
           //update motion model

           //update sensor model

            particles = moveParticles(particles, delta_rot1,delta_rot2,delta_trans, simul);// please check
            particles = weightUpdate(particles, simul, scanTest); //use scanTest instead - done by MI
            prev_pos = pos;
           // }
           //resample
           //
           // ----------------------------------------------------------
           //printf("\n");
           max_weight.first = 0;
         }

         // storing position data



         //DO WE NEED THIS? - TU
         // --------------------
         /*
         printf("\nPARTICLES:\n");
         for (auto &a:particles){
           printf("%.2f %.2f %.2f %.4f\n", a.pos.x, a.pos.y, a.pos.phi, a.weight);

         }


         printf("\n%d\n", scanPoints.size());

         for (auto &a:scanPoints){
           printf("%.2f\t", a);
         }

         printf("\n%d\n", scan.size());

         for (auto &a:scan){
           printf("%.2f\t", a);
         }

         printf("\n%d\n", scanTest.size());

         for (auto &a:scanTest){
           printf("%.2f\t", a);
         }
         */
         // ------------------

         // printing particles on map
         // gui.clearPositionPoints();
         gui.setPosition(robotPosition2point(pos));
         gui.clearMapPoints();
         gui.setPointsToMap(scanPoints, robotPosition2point(pos));
         gui.setParticlePoints(particles, true);
         gui.startInteractor();
    }

    auto end = steady_clock::now();
    std::chrono::duration<double> elapsed_secs = end - begin;
    std::cout << "ELAPSED TIME: " << elapsed_secs.count() << " s" << std::endl;
    gui.startInteractor();
    return EXIT_SUCCESS;
}

/// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

Point robotPosition2point(const RobotPosition& rp)
{
   return Point(rp.x, rp.y);
}

PointList calculateRawPoints(const Measurement& m)
{
    PointList rp = calculateRawPoints(m, m.position);
    return rp;
}

PointList calculateRawPoints(const Measurement& m, const RobotPosition& pos)
{
    PointList rp;
    const double laserResolution = 0.5; // deg
    const double laserShift = -90.0;
    for (size_t j = 0; j < m.scan.size(); j++)
    {
        if (m.scan[j] > 50.0)
        {
            /* A value greater than 50 indicates a failed acquisition. */
            continue;
        }
        const Point p = polar2cartesian(toRadians(j * laserResolution + laserShift), m.scan[j], pos);
        rp.push_back(p);
    }
    return rp;
}

Point polar2cartesian(double alpha, double r, const RobotPosition& p0)
{
    Point p;
    p.x = p0.x + r * cos(alpha + p0.phi);
    p.y = p0.y + r * sin(alpha + p0.phi);
    return p;
}

inline double toRadians(double alpha)
{
   return (alpha * M_PI) / 180.0;
}
