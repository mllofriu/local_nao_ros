/// <summary>
/// Module to save current values to a file in each DCM iteration (each 10 ms)
/// </summary>

#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <fstream>

using namespace std;

namespace AL
{
  class ALBroker;
  class ALMemoryFastAccess;
  class DCMProxy;
}

/// <summary>
/// Class to save values each 10 ms
/// </summary>
class Publisher : public AL::ALModule
{
public:
    Publisher(boost::shared_ptr<AL::ALBroker> pBroker,
                        const std::string &pName);

    virtual ~Publisher();

    void startPublishing(const std::string &sensorFileName);

    void stopPublishing();

    //void setFile(std::string file);

private:

  // Initialisation of ALMemory/DCM link
  void init(const std::string &sensorFileName);

  // ALMemory fast access
  void initFastAccess(const std::string &sensorFileName);

  //  Connect callback to the DCM post proccess
  void connectToDCMloop();

  // Callback called by the DCM every 10ms
  void synchronisedDCMcallback();

  // Used for postprocess sync with the DCM
  ProcessSignalConnection fDCMPostProcessConnection;

  // Used for fast memory access
  boost::shared_ptr<AL::ALMemoryFastAccess> fMemoryFastAccess;

  // Store sensor values.
  std::vector<float> sensorValues;
  boost::shared_ptr<AL::DCMProxy> dcmProxy;
};

#endif // PUBLISHER_H
