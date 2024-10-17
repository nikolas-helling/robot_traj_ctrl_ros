#include "car_simulator/test_step.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  test_step test_step_node;
   
  test_step_node.Prepare();
  
  test_step_node.RunPeriodically(test_step_node.RunPeriod);
  
  test_step_node.Shutdown();
  
  return (0);
}

