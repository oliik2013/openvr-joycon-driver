#ifndef _JOYCON_DRIVER_H_
#define _JOYCON_DRIVER_H_

#include <openvr_driver.h>
#include <string>

#include <JSL/JoyShockLibrary.h>

class JoyconDriver : public vr::ITrackedDeviceServerDriver
{
public:
    JoyconDriver(vr::ETrackedControllerRole controllerRole, std::string serialNumber);
    virtual ~JoyconDriver();

    virtual vr::EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId);
    virtual void Deactivate();
    virtual void EnterStandby();

    virtual void PowerOff();
    virtual void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize);
    virtual vr::DriverPose_t GetPose();

    void RunFrame();
    void ProcessEvent(const vr::VREvent_t &vrEvent);
    void *GetComponent(const char *pchComponentNameAndVersion);
    std::string GetSerialNumber() const { return m_serialNumber; }

    void processInput(JOY_SHOCK_STATE state, IMU_STATE imu, float dt);

private:
    vr::TrackedDeviceIndex_t m_controllerId;
    vr::PropertyContainerHandle_t m_containerHandle;
    vr::ETrackedControllerRole m_controllerRole;

    vr::DriverPose_t m_pose;
    std::string m_serialNumber;

    vr::VRInputComponentHandle_t m_compA, m_compB, m_compX, m_compY;
    vr::VRInputComponentHandle_t m_compUp, m_compDown, m_compLeft, m_compRight;
    vr::VRInputComponentHandle_t m_compPlus, m_compMinus;
    vr::VRInputComponentHandle_t m_compL, m_compR, m_compZL, m_compZR;
    vr::VRInputComponentHandle_t m_compSL, m_compSR;
    vr::VRInputComponentHandle_t m_compHome, m_compCapture;
    vr::VRInputComponentHandle_t m_compLStickClick, m_compRStickClick;
    vr::VRInputComponentHandle_t m_compLStickX, m_compLStickY;
    vr::VRInputComponentHandle_t m_compRStickX, m_compRStickY;

    int m_jslHandle;
};

#endif
