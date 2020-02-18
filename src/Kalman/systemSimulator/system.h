#include <gui/gui.h>
#include <random>
namespace imr {
   class System {
      private:
	 float _x, _y, _vx, _vy, _g;
	 gui::Point _measurement;
	 float _dt;

	 std::default_random_engine* generator;
	 std::normal_distribution<double>* distribution;
      public:
	 System(float x = 0, float y = 0, float angle = 45, float v = 150, float g=9.81, float mu=0.0, float sigma=25.0); 
	 gui::Point getMeasurement();
	 gui::Point getTruthPosition();
	 void makeStep();
   };
} //namespace imr
