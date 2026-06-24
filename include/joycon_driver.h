#ifndef _JOYCON_DRIVER_H_
#define _JOYCON_DRIVER_H_

#include <openvr_driver.h>
#include <string>
#include <mutex>

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

    vr::VRInputComponentHandle_t m_compSystem;
    vr::VRInputComponentHandle_t m_compAppMenu;
    vr::VRInputComponentHandle_t m_compGrip, m_compGripTouch;
    vr::VRInputComponentHandle_t m_compTrigger;
    vr::VRInputComponentHandle_t m_compTrackpadX, m_compTrackpadY;
    vr::VRInputComponentHandle_t m_compTrackpadClick;

    int m_jslHandle;
    int m_prevButtons;
    float m_frozenRel[3];
    vr::HmdQuaternion_t m_qOffset;
    vr::HmdQuaternion_t m_rawQuat;
    bool m_systemSuppressed;
    double m_systemHoldStart;
    bool m_pendingRecenter;
    vr::VRInputComponentHandle_t m_compHaptic;
    double m_rumbleEndTime;
    mutable std::mutex m_quatMutex;
};

#endif
