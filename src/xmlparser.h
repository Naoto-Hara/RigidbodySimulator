#ifndef xmlparser_h
#define xmlparser_h

#include <Eigen/Core>
#include <rapidxml.hpp>
#include "settings.h"

bool openXMLFile( const std::string& filename, SimulatorSettings& setting );

#endif /* xmlparser_h */
