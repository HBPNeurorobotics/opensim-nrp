#include <iostream>

#include "OpenSimSimulation.h"
#include <boost/program_options.hpp>

int main(int argc, char** argv)
{
  // Declare the supported options.
  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
      ("help", "Show help message")
      ("iterations", boost::program_options::value<unsigned int>(), "Number of iterations to simulate")
      ("timestep", boost::program_options::value<double>(), "Timestep to use")
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

  unsigned int sim_iterations = 1000;
  double sim_timestep = 0.01;

  if (vm.count("timestep"))
  {
    sim_timestep = vm["timestep"].as<double>();
  }

  if (vm.count("iterations"))
  {
    sim_iterations = vm["iterations"].as<unsigned int>();
  }

  if (vm.count("scene_file"))
  {
    std::cout << "Scene file to load: " << vm["scene_file"].as<std::string>() << std::endl;
    std::cout << "Iterations to simulate: " << sim_iterations << std::endl;
    std::cout << "Timestep: " << sim_timestep << std::endl;

    OpenSimSimulation* simulation = new OpenSimSimulation("OpenSim", vm["scene_file"].as<std::string>());

    simulation->Init();

    simulation->SetTimeStep(sim_timestep);

    for (unsigned int k = 0; k < sim_iterations; ++k)
    {
      simulation->Step();
    }

    simulation->Fini();

    delete simulation;
    simulation = NULL;
  }
  else
  {
    std::cout << "No scene file to use has been provided!" << std::endl;
    return 1;
  }
  return 0;
}
