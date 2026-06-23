#include "joycon_driver.h"
#include "driverlog.h"
#include "jsl_glue.h"
#include <cmath>

JoyconDriver::JoyconDriver(
    vr::ETrackedControllerRole controllerRole,
    std::string serialNumber
) :
    m_controllerId(vr::k_unTrackedDeviceIndexInvalid),
    m_containerHandle(vr::k_ulInvalidPropertyContainer),
    m_controllerRole(controllerRole),
    m_serialNumber(serialNumber),
    m_jslHandle(-1)
{
}

JoyconDriver::~JoyconDriver()
{
}

vr::EVRInitError JoyconDriver::Activate(vr::TrackedDeviceIndex_t unObjectId) 
{
    m_controllerId = unObjectId;
    m_containerHandle = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_controllerId);

    vr::VRProperties()->SetStringProperty(m_containerHandle, vr::Prop_SerialNumber_String, m_serialNumber.c_str());
    vr::VRProperties()->SetStringProperty(m_containerHandle, vr::Prop_TrackingSystemName_String, "joycon");
    vr::VRProperties()->SetStringProperty(m_containerHandle, vr::Prop_ModelNumber_String, "JoyCon");
    vr::VRProperties()->SetStringProperty(m_containerHandle, vr::Prop_ManufacturerName_String, "Nintendo");
    vr::VRProperties()->SetStringProperty(m_containerHandle, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");
    vr::VRProperties()->SetStringProperty(m_containerHandle, vr::Prop_ControllerType_String, "joycon_controller");
    vr::VRProperties()->SetStringProperty(m_containerHandle, vr::Prop_RegisteredDeviceType_String, "joycon");
    vr::VRProperties()->SetStringProperty(m_containerHandle, vr::Prop_InputProfilePath_String, "{joycon}/resources/input/joycon_profile.json");

    vr::VRProperties()->SetBoolProperty(m_containerHandle, vr::Prop_NeverTracked_Bool, false);
    vr::VRProperties()->SetBoolProperty(m_containerHandle, vr::Prop_WillDriftInYaw_Bool, false);
    vr::VRProperties()->SetBoolProperty(m_containerHandle, vr::Prop_DeviceIsWireless_Bool, true);
    vr::VRProperties()->SetBoolProperty(m_containerHandle, vr::Prop_Identifiable_Bool, false);

    vr::VRProperties()->SetUint64Property(m_containerHandle, vr::Prop_CurrentUniverseId_Uint64, 2);
    vr::VRProperties()->SetInt32Property(m_containerHandle, vr::Prop_ControllerRoleHint_Int32, m_controllerRole);
    vr::VRProperties()->SetInt32Property(m_containerHandle, vr::Prop_DeviceClass_Int32, (int32_t)vr::TrackedDeviceClass_Controller);

    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/a/click", &m_compA);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/b/click", &m_compB);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/x/click", &m_compX);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/y/click", &m_compY);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/dpad/up", &m_compUp);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/dpad/down", &m_compDown);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/dpad/left", &m_compLeft);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/dpad/right", &m_compRight);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/trigger_left/click", &m_compZL);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/trigger_right/click", &m_compZR);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/shoulder_left/click", &m_compL);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/shoulder_right/click", &m_compR);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/grip/click", &m_compSL);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/grip/touch", &m_compSR);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/start/click", &m_compPlus);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/back/click", &m_compMinus);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/guide/click", &m_compHome);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/capture/click", &m_compCapture);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/joystick_left/click", &m_compLStickClick);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/joystick_right/click", &m_compRStickClick);
    vr::VRDriverInput()->CreateScalarComponent(m_containerHandle, "/input/joystick_left/x", &m_compLStickX, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    vr::VRDriverInput()->CreateScalarComponent(m_containerHandle, "/input/joystick_left/y", &m_compLStickY, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    vr::VRDriverInput()->CreateScalarComponent(m_containerHandle, "/input/joystick_right/x", &m_compRStickX, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    vr::VRDriverInput()->CreateScalarComponent(m_containerHandle, "/input/joystick_right/y", &m_compRStickY, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);

    m_pose = {};
    m_pose.poseIsValid = true;
    m_pose.result = vr::TrackingResult_Running_OK;
    m_pose.deviceIsConnected = true;
    m_pose.shouldApplyHeadModel = false;

    m_pose.qRotation = { 1, 0, 0, 0 };
    m_pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
    m_pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };

    float side = (m_controllerRole == vr::TrackedControllerRole_RightHand) ? 0.25f : -0.25f;
    m_pose.vecPosition[0] = side;
    m_pose.vecPosition[1] = 1.0f;
    m_pose.vecPosition[2] = -0.5f;

    if (m_controllerRole == vr::TrackedControllerRole_LeftHand)
    {
        m_jslHandle = JSLGlue::instance.leftHandle;
    }
    else
    {
        m_jslHandle = JSLGlue::instance.rightHandle;
    }

    return vr::VRInitError_None;
}

void JoyconDriver::Deactivate()
{
    m_controllerId = vr::k_unTrackedDeviceIndexInvalid;
}

void JoyconDriver::EnterStandby()
{
}

void JoyconDriver::PowerOff()
{
}

void JoyconDriver::DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

void JoyconDriver::processInput(JOY_SHOCK_STATE state, IMU_STATE imu, float dt)
{
    (void)imu;
    (void)dt;

    if (m_jslHandle >= 0)
    {
        MOTION_STATE motion = JslGetMotionState(m_jslHandle);
        float qw = motion.quatW;
        float qx = motion.quatX;
        float qy = motion.quatY;
        float qz = motion.quatZ;

        float n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
        if (n > 0.0f)
        {
            qw /= n; qx /= n; qy /= n; qz /= n;
        }

        bool isRight = (m_controllerRole == vr::TrackedControllerRole_RightHand);

        m_pose.qRotation.w = qw;
        m_pose.qRotation.x = -qx;
        m_pose.qRotation.y = qy;
        m_pose.qRotation.z = -qz;
    }

    int buttons = state.buttons;
    bool isRight = (m_controllerRole == vr::TrackedControllerRole_RightHand);

    if (isRight)
    {
        vr::VRDriverInput()->UpdateBooleanComponent(m_compA, buttons & JSMASK_E, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compB, buttons & JSMASK_S, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compX, buttons & JSMASK_W, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compY, buttons & JSMASK_N, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compR, buttons & JSMASK_R, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compZR, buttons & JSMASK_ZR, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compSL, buttons & JSMASK_SL, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compSR, buttons & JSMASK_SR, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compPlus, buttons & JSMASK_PLUS, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compHome, buttons & JSMASK_HOME, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compRStickClick, buttons & JSMASK_RCLICK, false);
        vr::VRDriverInput()->UpdateScalarComponent(m_compRStickX, state.stickRX, false);
        vr::VRDriverInput()->UpdateScalarComponent(m_compRStickY, -state.stickRY, false);
    }
    else
    {
        vr::VRDriverInput()->UpdateBooleanComponent(m_compUp, buttons & JSMASK_UP, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compDown, buttons & JSMASK_DOWN, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compLeft, buttons & JSMASK_LEFT, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compRight, buttons & JSMASK_RIGHT, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compL, buttons & JSMASK_L, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compZL, buttons & JSMASK_ZL, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compSL, buttons & JSMASK_SL, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compSR, buttons & JSMASK_SR, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compMinus, buttons & JSMASK_MINUS, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compCapture, buttons & JSMASK_CAPTURE, false);
        vr::VRDriverInput()->UpdateBooleanComponent(m_compLStickClick, buttons & JSMASK_LCLICK, false);
        vr::VRDriverInput()->UpdateScalarComponent(m_compLStickX, state.stickLX, false);
        vr::VRDriverInput()->UpdateScalarComponent(m_compLStickY, -state.stickLY, false);
    }
}

vr::DriverPose_t JoyconDriver::GetPose()
{
    return m_pose;
}

void JoyconDriver::RunFrame()
{
    if (m_controllerId == vr::k_unTrackedDeviceIndexInvalid)
        return;

    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_controllerId, GetPose(), sizeof(vr::DriverPose_t));
}

void JoyconDriver::ProcessEvent(const vr::VREvent_t &vrEvent)
{
}

void *JoyconDriver::GetComponent(const char *pchComponentNameAndVersion)
{
    return nullptr;
}
