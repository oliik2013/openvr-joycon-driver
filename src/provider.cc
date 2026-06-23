#include "provider.h"
#include "driverlog.h"
#include "jsl_glue.h"

extern "C" void JslSetAutomaticCalibration(int deviceId, bool enabled);

vr::EVRInitError Provider::Init(vr::IVRDriverContext *pDriverContext)
{
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    
    InitDriverLog(vr::VRDriverLog());

    DriverLog("Provider::Init - Initializing JoyShockLibrary");
    JSLGlue::instance.init();

    DriverLog("Provider::Init - Initializing driver");
    
    vr::EVRInitError eError = vr::VRInitError_None;

    m_LeftController = new JoyconDriver(vr::TrackedControllerRole_LeftHand, "joycon_left");
    m_RightController = new JoyconDriver(vr::TrackedControllerRole_RightHand, "joycon_right");

    JSLGlue::instance.setDriver(false, m_LeftController);
    JSLGlue::instance.setDriver(true, m_RightController);

    vr::VRServerDriverHost()->TrackedDeviceAdded("joycon_left", vr::TrackedDeviceClass_Controller, m_LeftController);
    vr::VRServerDriverHost()->TrackedDeviceAdded("joycon_right", vr::TrackedDeviceClass_Controller, m_RightController);

    return eError;
}

void Provider::Cleanup()
{
    CleanupDriverLog();
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void Provider::RunFrame()
{
    if (m_bIsFirstRunFrame)
    {
        m_bIsFirstRunFrame = false;
        JSLGlue::instance.enable_callback();
        int handles[16];
        int count = JslGetConnectedDeviceHandles(handles, 16);
        for (int i = 0; i < count; i++)
        {
            JslSetAutomaticCalibration(handles[i], true);
            JslResetContinuousCalibration(handles[i]);
            DriverLog("Enabled auto-calibration for device handle %d", handles[i]);
        }
    }

    if (m_LeftController)
        m_LeftController->RunFrame();
    if (m_RightController)
        m_RightController->RunFrame();
}
