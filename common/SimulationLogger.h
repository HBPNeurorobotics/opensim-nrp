#ifndef SIMULATION_LOGGER_H
#define SIMULATION_LOGGER_H

#include <string>
#include <map>

class SimulationLogger
{
public:
	SimulationLogger(const std::string& logDir);
	~SimulationLogger();
	
	void newLogger(const std::string& objectToLog, const std::string& filename, const std::string& dataDescription);
	void logData(const std::string& objectToLog, const std::string& logString);
	
protected:
	std::map<std::string, std::string> logFiles;
	std::string logDirectory;
};

#endif //SIMULATION_LOGGER_H
