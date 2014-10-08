/// <summary>
/// Module to save current values to a file in each DCM iteration (each 10 ms)
/// </summary>

#include "publisher.h"
#include <boost/shared_ptr.hpp>
#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include <alerror/alerror.h>

// Use DCM proxy
#include <alproxies/dcmproxy.h>

// Used to read values of ALMemory directly in RAM
#include <almemoryfastaccess/almemoryfastaccess.h>

#include <boost/bind.hpp>
#include <iostream>
#include <fstream>

using namespace std;

/// <summary>
/// Module to save current values to a file in each DCM iteration (each 10 ms)
/// </summary>
/// <param name="broker"> A smart pointer to the broker.</param>
/// <param name="name">   The name of the module. </param>
Publisher::Publisher(boost::shared_ptr<AL::ALBroker> broker,
                                         const std::string &name)
    : AL::ALModule(broker, name )
    , fMemoryFastAccess(boost::shared_ptr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess()))
{
    setModuleDescription( "Module to save current values to a file in each DCM iteration (each 10 ms)." );

    functionName("startPublishing", getName() , "start");
    addParam("fileName", "file name to get the sensors to record from");
    BIND_METHOD(Publisher::startPublishing);

    functionName("stopPublishing", getName() , "stop");
    BIND_METHOD(Publisher::stopPublishing);

    // hack para evitar excepcion de boost
    // TODO: averiguar como sacar esto
    //    startAcquiring("~/currentValues.data");
    //    stopAcquiring();
}

Publisher::~Publisher()
{
    stopPublishing();
}

// Start the example
void Publisher::startPublishing(const std::string &sensorFileName)
{
    signed long isDCMRunning;

    try
    {
        // Get the DCM proxy
        dcmProxy = getParentBroker()->getDcmProxy();
    }
    catch (AL::ALError& e)
    {
        throw ALERROR(getName(), "startLoop()", "Impossible to create DCM Proxy : " + e.toString());
    }

    // Is the DCM running ?
    try
    {
        isDCMRunning = getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", std::string("DCM"));
    }
    catch (AL::ALError& e)
    {
        throw ALERROR(getName(), "startLoop()", "Error when connecting to DCM : " + e.toString());
    }

    if (!isDCMRunning)
    {
        throw ALERROR(getName(), "startLoop()", "Error no DCM running ");
    }

    init(sensorFileName);
    connectToDCMloop();
}

// Stop the example
void Publisher::stopPublishing()
{
    // Remove the postProcess call back connection
    fDCMPostProcessConnection.disconnect();
}

// Initialisation of ALmemory fast access, DCM commands, Alias, stiffness, ...
void Publisher::init(const std::string &sensorFileName)
{
    initFastAccess(sensorFileName);
}



// ALMemory fast access
void Publisher::initFastAccess(const std::string &sensorFileName)
{
    // Read sensor keys from a file
    std::vector<std::string> fSensorKeys;
    ifstream sensorFile;
    sensorFile.open(sensorFileName.c_str());
    string line;
    if (sensorFile.is_open())
    {
        while ( sensorFile.good() )
        {
            getline (sensorFile,line);
            fSensorKeys.push_back(line);
        }
    } else cout << "Publisher: Unable to open file";
    sensorFile.close();


    // Create the fast memory access
    fMemoryFastAccess->ConnectToVariables(getParentBroker(), fSensorKeys, false);
}

void Publisher::connectToDCMloop()
{
    // Get all values from ALMemory using fastaccess
    fMemoryFastAccess->GetValues(sensorValues);

    // Connect callback to the DCM post proccess
    try
    {
        fDCMPostProcessConnection =
                getParentBroker()->getProxy("DCM")->getModule()->atPostProcess(boost::bind(&Publisher::synchronisedDCMcallback, this));
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "connectToDCMloop()", "Error when connecting to DCM postProccess: " + e.toString());
    }
}


/**
 *  WARNING
 *
 *  Once this method is connected to DCM postprocess
 *  it will be called in Real Time every 10 milliseconds from DCM thread
 *  Dynamic allocation and system call are strictly forbidden in this method
 *  Computation time in this section must remain as short as possible to prevent
 *  erratic move or joint getting loose.
 *
 */
//  Send the new values of all joints to the DCM.
//  Note : a joint could be ignore unsing a NAN value.
//  Note : do not forget to set stiffness
void Publisher::synchronisedDCMcallback()
{
    int DCMtime;

    try
    {
        // Get absolute time, at 0 ms in the future ( i.e. now )
        DCMtime = dcmProxy->getTime(0);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "synchronisedDCMcallback()", "Error on DCM getTime : " + e.toString());
    }

    fMemoryFastAccess->GetValues(sensorValues);

    //dumpFile << DCMtime;
    //for (std::vector<float>::iterator it = sensorValues.begin() ; it != sensorValues.end(); ++it)
    //    dumpFile << ' ' << *it;
    //dumpFile << '\n';
    // Publish values obtained
}
