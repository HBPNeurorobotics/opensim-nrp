#include "SimulationLogger.h"

#include <boost/filesystem.hpp> // ?
#include <boost/filesystem/path.hpp> // ?

#include <fstream>
#include <iostream>
#include <ios>

SimulationLogger::SimulationLogger(const std::string& logDir):
	logDirectory(logDir)
{
}

SimulationLogger::~SimulationLogger()
{
	
}

void SimulationLogger::newLogger(const std::string& objectToLog, const std::string& filename, const std::string& dataDescription)
{
	std::string log_file_name;
	if (logFiles.find(objectToLog) == logFiles.end())
	{
		log_file_name = this->logDirectory + "/" + filename + ".log";
		logFiles.insert(std::make_pair(objectToLog, log_file_name));
	}
	else
	{
		std::cerr << "WARNING: Log file for " << objectToLog << " already exists. Overwriting existing file." << std::endl;
		log_file_name = logFiles[objectToLog];
	}
	
	std::ofstream log_stream;
	log_stream.open(log_file_name.c_str(), std::ios::out);
	if (log_stream.is_open())
	{
		log_stream << dataDescription;
	}
	log_stream.close();
}

void SimulationLogger::logData(const std::string& objectToLog, const std::string& logString)
{
	std::string log_file_name;
	
	if (logFiles.find(objectToLog) == logFiles.end())
	{
		newLogger(objectToLog, objectToLog + "_logFile", "no data description provided (use the newLogger method)\n");
	}
	log_file_name = logFiles[objectToLog];
	
	std::ofstream log_stream;
	log_stream.open(log_file_name.c_str(), std::ios::out | std::ios::app | std::ios::ate);
	
	if (log_stream.is_open())
	{
		log_stream << logString;
	}
	
	log_stream.close();
}
