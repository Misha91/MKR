#include "system.h"
#include <chrono>
using namespace imr;


System::System(float x , float y , float angle, float v, float g, float mu, float sigma) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
      generator = new std::default_random_engine (seed);

      distribution = new std::normal_distribution<double>  (mu,sigma);
   _x = x;
   _y = y;
   _vx = v*cos(angle /180*M_PI);
   _vy = v*sin(angle /180*M_PI);
   _g = g;
   _dt = 0.1;
} 

gui::Point System::getMeasurement() {
   return _measurement;
}

gui::Point System::getTruthPosition() {
   gui::Point ret;
   ret.x = _x;
   ret.y = _y;
   return ret;
}
void System::makeStep() {
   _vy = _vy - _g * _dt;   
   _x += _vx * _dt; 
   _y += _vy * _dt - 0.5*_g*_dt*_dt; 
   float error_x = (*distribution)(*generator);
   float error_y = (*distribution)(*generator);
   _measurement.x = _x + error_x;
   _measurement.y = _y + error_y;
   
}

