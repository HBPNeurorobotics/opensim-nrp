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
      ("use_visualization", boost::program_options::value<bool>(), "Use Simbody's OpenGL visualization (might not work with all OpenSim models)")
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

  bool use_visualization = false;

  if (vm.count("timestep"))
  {
    sim_timestep = vm["timestep"].as<double>();
  }

  if (vm.count("iterations"))
  {
    sim_iterations = vm["iterations"].as<unsigned int>();
  }

  if (vm.count("use_visualization"))
  {
    use_visualization = vm["use_visualization"].as<bool>();
  }

  if (vm.count("scene_file"))
  {
    std::cout << "Scene file to load: " << vm["scene_file"].as<std::string>() << std::endl;
    std::cout << "Iterations to simulate: " << sim_iterations << std::endl;
    std::cout << "Timestep: " << sim_timestep << std::endl;
    std::cout << "Use visualization: " << use_visualization << std::endl;

    OpenSimSimulation* simulation = new OpenSimSimulation("OpenSim", vm["scene_file"].as<std::string>());

    simulation->SetTimeStep(sim_timestep);
    simulation->SetUseVisualization(use_visualization);

    simulation->Init();

    int milliseconds = 1000;
    for (unsigned int k = 0; k < sim_iterations; ++k)
    {
      simulation->Step();
      struct timespec ts;
      ts.tv_sec = milliseconds / 1000;
      ts.tv_nsec = (milliseconds % 1000) * 1000000;
      nanosleep(&ts, NULL);
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
