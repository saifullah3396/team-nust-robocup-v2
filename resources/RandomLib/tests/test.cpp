#include "RandomLib/include/Random.hpp"
#include "RandomLib/include/RandomSelect.hpp"
#include "RandomLib/include/NormalDistribution.hpp"
#include <iostream>

using namespace std;
using namespace RandomLib;

int main()
{
  //! Random  library object
  Random random;
  NormalDistribution<double> dist;
  NormalDistribution<double> dist1;
  NormalDistribution<double> dist2;
  cout << "dist: " << dist(random, 0.0, 0.01) << endl;
  cout << "dist: " << dist(random, 0.0, 0.02) << endl;
  cout << "dist: " << dist(random, 0.0, 0.03) << endl;
  cout << "dist1: " << dist1(random, 0.0, 0.01) << endl;
  cout << "dist1: " << dist1(random, 0.0, 0.02) << endl;
  cout << "dist1: " << dist1(random, 0.0, 0.03) << endl;
  cout << "dist2: " << dist2(random, 0.0, 0.01) << endl;
  cout << "dist2: " << dist2(random, 0.0, 0.02) << endl;
  cout << "dist2: " << dist2(random, 0.0, 0.03) << endl;
}
