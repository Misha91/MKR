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


// GLOBAL parameters
double z_max = 5.0;
double n = 1.0;
RobotPosition real_pos,prev_real_pos;
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

double normalSampleMove (double mean, double sigma) {
    static std::default_random_engine eng;
    std::normal_distribution<> randomGenerator(mean, sigma);
    return randomGenerator(eng);
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
// TU
double distribution(double z,double z_star){
    double stddev = 0.4;
    double function;
    function = (1/(stddev*sqrt(2*M_PI)))*exp(-(z-z_star)*(z-z_star)/(2*stddev*stddev));
    return function;
}

double prob_hit(double z_star, double z){
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

double prob_short(double z_star, double z){
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


ParticleVector weightUpdate(ParticleVector init, LaserSimulator simul){
  double alpha_hit = 0.9;
  double alpha_short = 1; //1
  double alpha_rand = 1; //1
  double alpha_max = 0.5; //0.5
  LaserScan z, z_star;
  double prob_beam;
  z_star = simul.getScan(real_pos);
  for (auto &a:init)
  {
    prob_beam = 1;
    z = simul.getScan(a.pos);
    for (int i=0;i<z_star.size();i++){
      prob_beam = prob_beam*(alpha_hit*prob_hit(z_star[i],z[i]) + alpha_short*prob_short(z_star[i],z[i])+alpha_rand*prob_rand(z[i])+alpha_max*prob_max(z[i]));
      //printf("%.10f\t", prob_beam);
    }
    a.weight = prob_beam;

    //printf("%.2f %.2f %.2f %.4f\n", a.pos.x, a.pos.y, a.pos.phi, a.weight);
  }
  /*
  double maxW_found = 0;
  Particle maxW_Particle;

  for (auto &a:init){
    if (a.weight > maxW_found)
    {
      maxW_found = a.weight;
      maxW_Particle = a;
    }
    if (a.weight > 0){
     printf("%.2f %.2f %.2f %.10f\n",a.pos.x, a.pos.y, a.pos.phi, a.weight);
    }
  }
  printf("\n----------------------\n");
  printf("%.2f %.2f %.2f %.10f\n",maxW_Particle.pos.x, maxW_Particle.pos.y,  maxW_Particle.pos.phi, maxW_Particle.weight);
  printf("%.2f %.2f %.2f\n",real_pos.x,real_pos.y, real_pos.phi);
  z = simul.getScan(maxW_Particle.pos);
  for (int i=0;i<z_star.size();i++)
  {
    printf("%.5f %.5f %.5f %.5f\n",prob_hit(z_star[i],z[i]), prob_short(z_star[i],z[i]), prob_rand(z[i]), prob_max(z[i]));
  }
  printf("\n----------------------\n");
  ParticleVector dummy;
  dummy.push_back(maxW_Particle);
  */
  return init;
}

/// -----------------------------------------------------------------------------------
//Move Particle

RobotPosition moveParticle (RobotPosition const &r) {
  RobotPosition result;
  result.x = normalSampleMove(r.x,0.1);
  result.y = normalSampleMove(r.y,0.1);
  result.phi = normalSampleMove(r.phi,0.1);
  //std::cout << "P: " << result.x << " " << result.y << std::endl;
  return result;
}

//Move particles
ParticleVector moveParticles(ParticleVector init, double delta_x, double delta_y,double delta_phi){
  RobotPosition r;
  for (auto &a:init)
  {
    r.x = a.pos.x + delta_x;
    r.y = a.pos.y + delta_y;
    r.phi = a.pos.phi + delta_phi;
    a.pos = moveParticle(r);
  }
  return init;
}
// --------------------------------------------------------------------------


ParticleVector rouletteSampler(const ParticleVector init, LaserSimulator simul){
  std::map <double, int> hashTable;
  double weightAdder = 0;
  ParticleVector result;

  for (int i = 0; i < init.size(); i++)
  {
    weightAdder += init[i].weight;
    hashTable[weightAdder] = i;
  }

  double meanWeight = 0.0;
  for (int i = 0; i < (0.9*init.size()); i++)
  {
  //for (int i = 0; i < 10; i++){
    double tmp = uniformSample(0, weightAdder);
    result.push_back(init[hashTable.lower_bound(tmp)->second]);
    meanWeight += init[hashTable.lower_bound(tmp)->second].weight;
  }
  //printf("%d %d\n", init.size(), result.size());

  meanWeight /= result.size();
  double x;
  double y;
  double phi;

  while (result.size() != init.size())
  {
     x = uniformSample(-16.96, 19.7243);
     y = uniformSample(-43.25, 55.0255);
     phi = uniformSample(-M_PI, M_PI);
     Particle p;
     p.pos = RobotPosition(x, y, phi);
     if (simul.isFeasible(p.pos))
     {
        p.weight = meanWeight;
        result.push_back(p);
     }
  }

  return result;
}

int main(int argc, char** argv)
{
    unsigned int nMeasurements = 5;
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
    for (size_t i = 0; i < 1000;)
    {
       x = uniformSample(-16.96, 19.7243);
       y = uniformSample(-43.25, 55.0255);
       phi = uniformSample(-M_PI, M_PI);
       Particle p;
       p.pos = RobotPosition(x, y, phi);
       if (simul.isFeasible(p.pos))
       {
          p.weight = 1.0 / 1000.0;
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
         LaserScan scanTest = loader[i].scan; //361
         double delta_x, delta_y, delta_phi;
         real_pos = pos;
         scan = simul.getScan(pos); //36
         scanPoints = simul.getRawPoints();
         if (i > 0)
         {
           delta_x = real_pos.x - prev_real_pos.x;
           delta_y = real_pos.y - prev_real_pos.y;
           delta_phi = real_pos.phi - prev_real_pos.phi;
           particles = moveParticles(particles, delta_x,delta_y,delta_phi);
           particles = weightUpdate(particles, simul);
           particles = rouletteSampler(particles, simul);
         }

         prev_real_pos = real_pos;
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

         //
         // gui.clearPositionPoints();
         gui.setPosition(robotPosition2point(pos));
         gui.clearMapPoints();
         gui.setPointsToMap(scanPoints, robotPosition2point(pos));
         gui.setParticlePoints(particles);
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
