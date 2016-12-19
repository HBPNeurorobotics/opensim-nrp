#include <iostream>

#include "OpenSimSimulation.h"
#include <boost/program_options.hpp>

int main(int argc, char** argv)
{
  // Declare the supported options.
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
      ("help", "Show help message")
      ("scene_file", boost::program_options::value<std::string>(), "OpenSim scene file to load")
  ;

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);

  if (vm.count("help"))
  {
      std::cout << desc << "\n";
      return 1;
  }

  if (vm.count("scene_file"))
  {
      std::cout << "Scene file to load: " << vm["scene_file"].as<std::string>() << std::endl;

      OpenSimSimulation* simulation = new OpenSimSimulation("OpenSim", vm["scene_file"].as<std::string>());

      simulation->Init();

      simulation->Fini();
      delete simulation;
  }
  else
  {
      std::cout << "No scene file to use has been provided!" << std::endl;
      return 1;
  }
  return 0;
}
