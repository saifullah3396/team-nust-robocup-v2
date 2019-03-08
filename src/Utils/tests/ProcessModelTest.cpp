#include "Utils/include/Filters/ProcessModel.h"

int main()
{
  ProcessModel<float, 3, 1, 1> model;
  cout << "StateMatrix:\n" << model.getStateMatrix() << endl;
  cout << "InputMatrix:\n" << model.getInputMatrix() << endl;
  cout << "OutputMatrix:\n" << model.getOutputMatrix() << endl;
  cout << "State:\n" << model.getState() << endl;
  cout << "Input:\n" << model.getInput() << endl;
  cout << "Output:\n" << model.getOutput() << endl;
  return 0;
}
