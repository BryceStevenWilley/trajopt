#include <gtest/gtest.h>
#include <openrave-core.h>
#include <openrave/openrave.h>

#include "trajopt/common.hpp"
#include "trajopt_test_utils.hpp"
#include "utils/config.hpp"
#include "utils/eigen_conversions.hpp"
#include "trajopt/trajectory_costs.hpp"
#include "sco/modeling.hpp"

trajopt::VarArray makeArray(sco::OptProb& o, int waypoints, int dof)
{
  std::vector<std::string> names;
  for (int i = 0; i < waypoints; i++)
  {
      for (int j = 0; j < dof; j++)
      {
        names.push_back( (boost::format("j_%i_%i")%i%j).str());
      }
  }
  o.createVariables(names);
  sco::VarVector trajvarvec = o.getVars();
  return trajopt::VarArray(waypoints, dof, trajvarvec.data());
}

std::vector<double> random(int waypoints, int dof)
{
  std::vector<double> x;
  for (int i = 0; i < waypoints; i++)
  {
    for (int j = 0; j < dof; j++) {
      x.push_back(((double) rand() /RAND_MAX) * 5.0);
    }   
  }
  return x;
}

std::vector<double> straightLine(int waypoints, int dof, double *start, double *end)
{
  std::vector<double> x;
  for (int i = 0; i < waypoints; i++) {
    for (int j = 0; j < dof; j++) {
      x.push_back(start[j] + (end[j] - start[j]) * i / (waypoints - 1));
    }
  }
  return x;
}

void run_test(int WAYPOINTS, int DOF, std::vector<double> x)
{
  // Make a TrajOpt VarArray for a straight line path.
  sco::OptProb o; 
  trajopt::VarArray vars = makeArray(o, WAYPOINTS, DOF);
  
  std::vector<double> coeffs;
  for (int i = 0; i < DOF; i++){
    coeffs.push_back(1);
  } 
  trajopt::JointVelCost cost(vars, trajopt::toVectorXd(coeffs));
  double res = cost.value(x);
  std::cout << res << std::endl;
  
  
  // Get and print the joint vel cost of this path.
  // Do the same for a path with one waypoint (handcode maybe).
  // Copy this test into OMPL and compare the exact answers. If
  // different, find out why
  // If same, then this was a dead end?
}


TEST(joint_vel, straight)
{
  double start[] = {0, 0, 0, 0, 0, 0};
  double end[] = {1.0, 2.0, 4.0, 3.0, 1.0, 2.0};
  std::vector<double> x = straightLine(10, 6, start, end); 
  run_test(10, 6, x);

  double realEnd[] = {-1.0, -0.5, 0, 2, 2.0, 3.0};
  std::vector<double> y = straightLine(10, 6, end, realEnd);
  x.insert(x.end(), y.begin(), y.end());
  run_test(20, 6, x);

  srand(10);
  for (int i = 0; i < 10; i++) {
    x = random(10, 6);
    run_test(10, 6, x);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  //RaveInitialize(false);
  int result = RUN_ALL_TESTS();
  return result;
}
