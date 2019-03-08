/**
 * @file src/main.cpp
 *
 * The main file that starts the simulator interface to run in the 
 * background
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 Jun 2017
 */
 
#include "ArgParser.h"
#include "Interface.h"

int main(int argc, char* argv[])
{
	Interface* interface;
  if(ArgParser::cmdOptionExists(argv, argv+argc, "--use-cameras"))
    interface = new Interface(true);
  else
    interface = new Interface(false);
  interface->init();
	while (true) 
    interface->update();
	return 0;
}
