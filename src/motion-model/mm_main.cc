/**
 * Model model for a particle filter
 * Skeleton code for B3M33MKR labs
 * Czech Technical University
 * Czech Institute of Informatics, Robotics, and Cybernetics
 * Intelligent and Mobile Robotics group
 *
 * Author: Miroslav Kulich,  kulich@cvut.cz
 **/

#include "gui.h"
#include <unistd.h>
#include <random>
#include <sstream>
#include <iomanip>


using namespace imr;
using namespace imr::gui;

std::default_random_engine re;


double to_angle (double x) {
    return std::atan2 (std::sin (x), std::cos (x));
}

double normalSample (double mean, double sigma) {
    static std::default_random_engine eng;
    std::normal_distribution<> randomGenerator(mean, sigma);
    return randomGenerator(eng);
}





RobotPosition moveParticle (RobotPosition const &pos, RobotPosition const &prev,RobotPosition const &r) {
  RobotPosition result;
  result.x = normalSample(r.x,0.1);   
  result.y = normalSample(r.y,0.1);   
  result.phi = normalSample(r.phi,0.1);   
  std::cout << "P: " << result.x << " " << result.y << std::endl;
  return result;
}


const bool DRAW_ARROWS = true;
const int NUM_PARTICLES = 100;

int main(int argc, char **argv) {
  Gui gui (DRAW_ARROWS);
  RobotPosition pos, next;
  ParticleVector particles;
  Particle p;

  std::cout << "partNum " << NUM_PARTICLES << std::endl;


  pos = RobotPosition(0, 0, 0);
  next = RobotPosition(1, 0, 0);

  for (size_t i = 0; i < NUM_PARTICLES; i++) {
    p.pos = moveParticle(pos,pos,next);
    p.weight = 1.0 / NUM_PARTICLES;
    particles.push_back(p);
  }

  gui.setPosition(pos);
  gui.setParticlePoints(particles);
  gui.startInteractor();

  return 0;
}
