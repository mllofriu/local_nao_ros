/// <summary>
/// Module to save current values to a file in each DCM iteration (each 10 ms)
/// </summary>

#include "getvalorescorriente.h"
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

std::string sensorsToRecord[] = {"Device/SubDeviceList/RHipPitch/Position/Sensor/Value",
                                 "Device/SubDeviceList/LKneePitch/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/RKneePitch/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/RAnkleRoll/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/LAnkleRoll/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/RAnklePitch/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/LAnklePitch/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/RHipPitch/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/LHipPitch/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/RHipRoll/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/LHipRoll/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/RHipYawPitch/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/LHipYawPitch/ElectricCurrent/Sensor/Value",
                                 "Device/SubDeviceList/LKneePitch/Position/Sensor/Value",
                                 "Device/SubDeviceList/RKneePitch/Position/Sensor/Value",
                                 "Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value",
                                 "Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value",
                                 "Device/SubDeviceList/RAnklePitch/Position/Sensor/Value",
                                 "Device/SubDeviceList/LAnklePitch/Position/Sensor/Value",
                                 "Device/SubDeviceList/RHipPitch/Position/Sensor/Value",
                                 "Device/SubDeviceList/LHipPitch/Position/Sensor/Value",
                                 "Device/SubDeviceList/RHipRoll/Position/Sensor/Value",
                                 "Device/SubDeviceList/LHipRoll/Position/Sensor/Value",
                                 "Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value",
                                 "Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value",
                                 "Device/SubDeviceList/InertialSensor/AccX/Sensor/Value",
                                 "Device/SubDeviceList/InertialSensor/AccY/Sensor/Value",
                                 "Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value",
                                 "Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value",
                                 "Device/SubDeviceList/InertialSensor/GyrZ/Sensor/Value",
                                 "Device/SubDeviceList/Battery/Charge/Sensor/CellVoltageMin",
                                 "Device/SubDeviceList/Battery/Current/Sensor/Value"
                                };

/// <summary>
/// Module to save current values to a file in each DCM iteration (each 10 ms)
/// </summary>
/// <param name="broker"> A smart pointer to the broker.</param>
/// <param name="name">   The name of the module. </param>
GetValoresCorriente::GetValoresCorriente(boost::shared_ptr<AL::ALBroker> broker,
                                         const std::string &name)
    : AL::ALModule(broker, name )
    , fMemoryFastAccess(boost::shared_ptr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess()))
{
    setModuleDescription( "Module to save current values to a file in each DCM iteration (each 10 ms)." );

    functionName("startAcquiring", getName() , "start");
    addParam("fileName", "file name to dump records");
    BIND_METHOD(GetValoresCorriente::startAcquiring);

    functionName("stopAcquiring", getName() , "stop");
    BIND_METHOD(GetValoresCorriente::stopAcquiring);

    // hack para evitar excepcion de boost
    // TODO: averiguar como sacar esto
    //    startAcquiring("~/currentValues.data");
    //    stopAcquiring();
}

GetValoresCorriente::~GetValoresCorriente()
{
    stopAcquiring();
}

// Start the example
void GetValoresCorriente::startAcquiring(const std::string &sensorFileName, const std::string &dumpFileName)
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

    init(sensorFileName, dumpFileName);
    connectToDCMloop();
}

// Stop the example
void GetValoresCorriente::stopAcquiring()
{
    //setStiffness(0.0f);
    // Close file
    if (dumpFile.is_open())
        dumpFile.close();

    // Remove the postProcess call back connection
    fDCMPostProcessConnection.disconnect();
}

// Initialisation of ALmemory fast access, DCM commands, Alias, stiffness, ...
void GetValoresCorriente::init(const std::string &sensorFileName, const std::string &dumpFileName)
{
    dumpFile.open(dumpFileName.c_str());

    initFastAccess(sensorFileName);
    createPositionActuatorAlias();
    createHardnessActuatorAlias();
    //setStiffness(0.2f); // Set to 1.0 for maximum stiffness, but only after a test
}



// ALMemory fast access
void GetValoresCorriente::initFastAccess(const std::string &sensorFileName)
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
    } else cout << "GetValoresCorriente: Unable to open file";
    sensorFile.close();


    // Create the fast memory access
    fMemoryFastAccess->ConnectToVariables(getParentBroker(), fSensorKeys, false);
}

void GetValoresCorriente::setStiffness(const float &stiffnessValue)
{
    AL::ALValue stiffnessCommands;
    int DCMtime;
    // increase stiffness with the "jointStiffness" Alias created at initialisation
    try
    {
        // Get time : return the time in 1 seconde
        DCMtime = dcmProxy->getTime(1000);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "setStiffness()", "Error on DCM getTime : " + e.toString());
    }

    // Prepare one dcm command:
    // it will linearly "Merge" all joint stiffness
    // from last value to "stiffnessValue" in 1 seconde
    stiffnessCommands.arraySetSize(3);
    stiffnessCommands[0] = std::string("jointStiffness");
    stiffnessCommands[1] = std::string("Merge");
    stiffnessCommands[2].arraySetSize(1);
    stiffnessCommands[2][0].arraySetSize(2);
    stiffnessCommands[2][0][0] = stiffnessValue;
    stiffnessCommands[2][0][1] = DCMtime;
    try
    {
        dcmProxy->set(stiffnessCommands);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "setStiffness()", "Error when sending stiffness to DCM : " + e.toString());
    }
}

void GetValoresCorriente::connectToDCMloop()
{
    // Get all values from ALMemory using fastaccess
    fMemoryFastAccess->GetValues(sensorValues);

    // Connect callback to the DCM post proccess
    try
    {
        fDCMPostProcessConnection =
                getParentBroker()->getProxy("DCM")->getModule()->atPostProcess(boost::bind(&GetValoresCorriente::synchronisedDCMcallback, this));
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
void GetValoresCorriente::synchronisedDCMcallback()
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

    dumpFile << DCMtime;
    for (std::vector<float>::iterator it = sensorValues.begin() ; it != sensorValues.end(); ++it)
        dumpFile << ' ' << *it;
    dumpFile << '\n';
}

void GetValoresCorriente::createPositionActuatorAlias()
{
    AL::ALValue jointAliasses;

    jointAliasses.arraySetSize(2);
    jointAliasses[0] = std::string("jointActuator"); // Alias for all 25 joint actuators
    jointAliasses[1].arraySetSize(25);

    // Joints actuator list

    jointAliasses[1][HEAD_PITCH]       = std::string("Device/SubDeviceList/HeadPitch/Position/Actuator/Value");
    jointAliasses[1][HEAD_YAW]         = std::string("Device/SubDeviceList/HeadYaw/Position/Actuator/Value");
    jointAliasses[1][L_ANKLE_PITCH]    = std::string("Device/SubDeviceList/LAnklePitch/Position/Actuator/Value");
    jointAliasses[1][L_ANKLE_ROLL]     = std::string("Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value");
    jointAliasses[1][L_ELBOW_ROLL]     = std::string("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value");
    jointAliasses[1][L_ELBOW_YAW]      = std::string("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value");
    jointAliasses[1][L_HAND]           = std::string("Device/SubDeviceList/LHand/Position/Actuator/Value");
    jointAliasses[1][L_HIP_PITCH]      = std::string("Device/SubDeviceList/LHipPitch/Position/Actuator/Value");
    jointAliasses[1][L_HIP_ROLL]       = std::string("Device/SubDeviceList/LHipRoll/Position/Actuator/Value");
    jointAliasses[1][L_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value");
    jointAliasses[1][L_KNEE_PITCH]     = std::string("Device/SubDeviceList/LKneePitch/Position/Actuator/Value");
    jointAliasses[1][L_SHOULDER_PITCH] = std::string("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value");
    jointAliasses[1][L_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value");
    jointAliasses[1][L_WRIST_YAW]      = std::string("Device/SubDeviceList/LWristYaw/Position/Actuator/Value");
    jointAliasses[1][R_ANKLE_PITCH]    = std::string("Device/SubDeviceList/RAnklePitch/Position/Actuator/Value");
    jointAliasses[1][R_ANKLE_ROLL]     = std::string("Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value");
    jointAliasses[1][R_ELBOW_ROLL]     = std::string("Device/SubDeviceList/RElbowRoll/Position/Actuator/Value");
    jointAliasses[1][R_ELBOW_YAW]      = std::string("Device/SubDeviceList/RElbowYaw/Position/Actuator/Value");
    jointAliasses[1][R_HAND]           = std::string("Device/SubDeviceList/RHand/Position/Actuator/Value");
    jointAliasses[1][R_HIP_PITCH]      = std::string("Device/SubDeviceList/RHipPitch/Position/Actuator/Value");
    jointAliasses[1][R_HIP_ROLL]       = std::string("Device/SubDeviceList/RHipRoll/Position/Actuator/Value");
    jointAliasses[1][R_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value");
    jointAliasses[1][R_SHOULDER_PITCH] = std::string("Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value");
    jointAliasses[1][R_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value");
    jointAliasses[1][R_WRIST_YAW]      = std::string("Device/SubDeviceList/RWristYaw/Position/Actuator/Value");

    // Create alias
    try
    {
        dcmProxy->createAlias(jointAliasses);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "createPositionActuatorAlias()", "Error when creating Alias : " + e.toString());
    }
}

void GetValoresCorriente::createHardnessActuatorAlias()
{
    AL::ALValue jointAliasses;
    // Alias for all joint stiffness
    jointAliasses.clear();
    jointAliasses.arraySetSize(2);
    jointAliasses[0] = std::string("jointStiffness"); // Alias for all 25 actuators
    jointAliasses[1].arraySetSize(25);

    // stiffness list
    jointAliasses[1][HEAD_PITCH]        = std::string("Device/SubDeviceList/HeadPitch/Hardness/Actuator/Value");
    jointAliasses[1][HEAD_YAW]          = std::string("Device/SubDeviceList/HeadYaw/Hardness/Actuator/Value");
    jointAliasses[1][L_ANKLE_PITCH]     = std::string("Device/SubDeviceList/LAnklePitch/Hardness/Actuator/Value");
    jointAliasses[1][L_ANKLE_ROLL]      = std::string("Device/SubDeviceList/LAnkleRoll/Hardness/Actuator/Value");
    jointAliasses[1][L_ELBOW_ROLL]      = std::string("Device/SubDeviceList/LElbowRoll/Hardness/Actuator/Value");
    jointAliasses[1][L_ELBOW_YAW]       = std::string("Device/SubDeviceList/LElbowYaw/Hardness/Actuator/Value");
    jointAliasses[1][L_HAND]            = std::string("Device/SubDeviceList/LHand/Hardness/Actuator/Value");
    jointAliasses[1][L_HIP_PITCH]       = std::string("Device/SubDeviceList/LHipPitch/Hardness/Actuator/Value");
    jointAliasses[1][L_HIP_ROLL]        = std::string("Device/SubDeviceList/LHipRoll/Hardness/Actuator/Value");
    jointAliasses[1][L_HIP_YAW_PITCH]   = std::string("Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value");
    jointAliasses[1][L_KNEE_PITCH]      = std::string("Device/SubDeviceList/LKneePitch/Hardness/Actuator/Value");
    jointAliasses[1][L_SHOULDER_PITCH]  = std::string("Device/SubDeviceList/LShoulderPitch/Hardness/Actuator/Value");
    jointAliasses[1][L_SHOULDER_ROLL]   = std::string("Device/SubDeviceList/LShoulderRoll/Hardness/Actuator/Value");
    jointAliasses[1][L_WRIST_YAW]       = std::string("Device/SubDeviceList/LWristYaw/Hardness/Actuator/Value");
    jointAliasses[1][R_ANKLE_PITCH]     = std::string("Device/SubDeviceList/RAnklePitch/Hardness/Actuator/Value");
    jointAliasses[1][R_ANKLE_ROLL]      = std::string("Device/SubDeviceList/RAnkleRoll/Hardness/Actuator/Value");
    jointAliasses[1][R_ELBOW_ROLL]      = std::string("Device/SubDeviceList/RElbowRoll/Hardness/Actuator/Value");
    jointAliasses[1][R_ELBOW_YAW]       = std::string("Device/SubDeviceList/RElbowYaw/Hardness/Actuator/Value");
    jointAliasses[1][R_HAND]            = std::string("Device/SubDeviceList/RHand/Hardness/Actuator/Value");
    jointAliasses[1][R_HIP_PITCH]       = std::string("Device/SubDeviceList/RHipPitch/Hardness/Actuator/Value");
    jointAliasses[1][R_HIP_ROLL]        = std::string("Device/SubDeviceList/RHipRoll/Hardness/Actuator/Value");
    jointAliasses[1][R_KNEE_PITCH]      = std::string("Device/SubDeviceList/RKneePitch/Hardness/Actuator/Value");
    jointAliasses[1][R_SHOULDER_PITCH]  = std::string("Device/SubDeviceList/RShoulderPitch/Hardness/Actuator/Value");
    jointAliasses[1][R_SHOULDER_ROLL]   = std::string("Device/SubDeviceList/RShoulderRoll/Hardness/Actuator/Value");
    jointAliasses[1][R_WRIST_YAW]       = std::string("Device/SubDeviceList/RWristYaw/Hardness/Actuator/Value");

    // Create alias
    try
    {
        dcmProxy->createAlias(jointAliasses);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "createHardnessActuatorAlias()", "Error when creating Alias : " + e.toString());
    }
}
