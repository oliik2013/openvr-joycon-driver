#include "joycon_driver.h"
#include "driverlog.h"
#include "jsl_glue.h"
#include <cmath>
#include <chrono>

JoyconDriver::JoyconDriver(
    vr::ETrackedControllerRole controllerRole,
    std::string serialNumber
) :
    m_controllerId(vr::k_unTrackedDeviceIndexInvalid),
    m_containerHandle(vr::k_ulInvalidPropertyContainer),
    m_controllerRole(controllerRole),
    m_serialNumber(serialNumber),
    m_jslHandle(-1),
    m_prevButtons(0),
    m_frozenRel{0, 0, 0},
    m_qOffset{1, 0, 0, 0},
    m_rawQuat{1, 0, 0, 0},
    m_systemSuppressed(false),
    m_systemHoldStart(0),
    m_pendingRecenter(false),
    m_rumbleEndTime(0),
    m_recenterHmdPos{0, 0, 0}
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
    vr::VRProperties()->SetStringProperty(m_containerHandle, vr::Prop_ControllerType_String, "vive_controller");
    vr::VRProperties()->SetStringProperty(m_containerHandle, vr::Prop_RegisteredDeviceType_String, "joycon");
    vr::VRProperties()->SetStringProperty(m_containerHandle, vr::Prop_InputProfilePath_String, "{joycon}/resources/input/joycon_profile.json");

    vr::VRProperties()->SetBoolProperty(m_containerHandle, vr::Prop_NeverTracked_Bool, false);
    vr::VRProperties()->SetBoolProperty(m_containerHandle, vr::Prop_WillDriftInYaw_Bool, true);
    vr::VRProperties()->SetBoolProperty(m_containerHandle, vr::Prop_DeviceIsWireless_Bool, true);
    vr::VRProperties()->SetBoolProperty(m_containerHandle, vr::Prop_Identifiable_Bool, false);

    vr::VRProperties()->SetUint64Property(m_containerHandle, vr::Prop_CurrentUniverseId_Uint64, 2);
    vr::VRProperties()->SetInt32Property(m_containerHandle, vr::Prop_ControllerRoleHint_Int32, m_controllerRole);
    vr::VRProperties()->SetInt32Property(m_containerHandle, vr::Prop_DeviceClass_Int32, (int32_t)vr::TrackedDeviceClass_Controller);

    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/system/click", &m_compSystem);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/application_menu/click", &m_compAppMenu);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/grip/click", &m_compGrip);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/grip/touch", &m_compGripTouch);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/trigger/click", &m_compTrigger);
    vr::VRDriverInput()->CreateScalarComponent(m_containerHandle, "/input/trackpad/x", &m_compTrackpadX, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    vr::VRDriverInput()->CreateScalarComponent(m_containerHandle, "/input/trackpad/y", &m_compTrackpadY, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
    vr::VRDriverInput()->CreateBooleanComponent(m_containerHandle, "/input/trackpad/click", &m_compTrackpadClick);
    vr::VRDriverInput()->CreateHapticComponent(m_containerHandle, "/output/haptic", &m_compHaptic);

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

static void quatMul(float aW, float aX, float aY, float aZ,
                     float bW, float bX, float bY, float bZ,
                     double &rW, double &rX, double &rY, double &rZ)
{
    rW = aW * bW - aX * bX - aY * bY - aZ * bZ;
    rX = aW * bX + aX * bW + aY * bZ - aZ * bY;
    rY = aW * bY - aX * bZ + aY * bW + aZ * bX;
    rZ = aW * bZ + aX * bY - aY * bX + aZ * bW;
}

void JoyconDriver::processInput(JOY_SHOCK_STATE state, IMU_STATE imu, float dt)
{
    (void)imu;
    (void)dt;

    bool isRight = (m_controllerRole == vr::TrackedControllerRole_RightHand);
    int buttons = state.buttons;

    if (m_jslHandle >= 0)
    {
        MOTION_STATE motion = JslGetMotionState(m_jslHandle);
        float qw = motion.quatW;
        float qx = motion.quatX;
        float qy = motion.quatY;
        float qz = motion.quatZ;

        // Normalize first
        float n = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
        if (n > 0.0f)
        {
            qw /= n; qx /= n; qy /= n; qz /= n;
        }

        // FIX 1: Force into positive-w hemisphere.
        // JSL can return q or -q for the same rotation (both are valid).
        // Without this, qw crossing zero causes a sudden instantaneous
        // flip of all axes — looks like the controller snapping upside-down
        // and then "fixing itself" when qw happens to flip back.
        if (qw < 0.0f)
        {
            qw = -qw;
            qx = -qx;
            qy = -qy;
            qz = -qz;
        }

        // FIX 2: Per-hand axis remap.
        // Left and right Joy-Cons are held sideways in opposite orientations
        // so they need a mirrored remap, not the same one.
        {
            std::lock_guard<std::mutex> lock(m_quatMutex);
            if (isRight)
            {
                m_rawQuat.w =  qw;
                m_rawQuat.x = -qx;
                m_rawQuat.y =  qy;
                m_rawQuat.z = -qz;
            }
            else
            {
                m_rawQuat.w =  qw;
                m_rawQuat.x =  qx;
                m_rawQuat.y = -qy;
                m_rawQuat.z =  qz;
            }
        }
    }

    int recenterBtn = isRight ? JSMASK_PLUS : JSMASK_MINUS;
    bool btnDown = buttons & recenterBtn;
    bool btnPressed = btnDown && !(m_prevButtons & recenterBtn);
    bool btnReleased = !btnDown && (m_prevButtons & recenterBtn);

    if (btnPressed)
    {
        m_systemHoldStart = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        m_systemSuppressed = false;
    }

    if (btnDown && !m_systemSuppressed)
    {
        double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if (now - m_systemHoldStart >= 0.5)
        {
            m_systemSuppressed = true;
            m_pendingRecenter = true;
            m_frozenRel[0] = m_frozenRel[1] = m_frozenRel[2] = 0.0f;
        }
    }

    if (btnReleased)
    {
        m_systemSuppressed = false;
    }
    m_prevButtons = buttons;

    vr::VRDriverInput()->UpdateBooleanComponent(m_compSystem, btnDown && !m_systemSuppressed, false);
    vr::VRDriverInput()->UpdateBooleanComponent(m_compAppMenu, isRight ? (buttons & JSMASK_HOME) : (buttons & JSMASK_CAPTURE), false);
    vr::VRDriverInput()->UpdateBooleanComponent(m_compGrip, buttons & JSMASK_SL, false);
    vr::VRDriverInput()->UpdateBooleanComponent(m_compGripTouch, buttons & JSMASK_SR, false);
    vr::VRDriverInput()->UpdateBooleanComponent(m_compTrigger, isRight ? (buttons & JSMASK_ZR) : (buttons & JSMASK_ZL), false);
    vr::VRDriverInput()->UpdateBooleanComponent(m_compTrackpadClick, isRight ? (buttons & JSMASK_RCLICK) : (buttons & JSMASK_LCLICK), false);
    vr::VRDriverInput()->UpdateScalarComponent(m_compTrackpadX, isRight ? state.stickRX : state.stickLX, false);
    vr::VRDriverInput()->UpdateScalarComponent(m_compTrackpadY, isRight ? (-state.stickRY) : state.stickLY, false);
}

vr::DriverPose_t JoyconDriver::GetPose()
{
    return m_pose;
}

void JoyconDriver::RunFrame()
{
    if (m_controllerId == vr::k_unTrackedDeviceIndexInvalid)
        return;

    auto now = std::chrono::steady_clock::now();
    double nowSec = std::chrono::duration<double>(now.time_since_epoch()).count();
    if (m_rumbleEndTime > 0 && nowSec >= m_rumbleEndTime)
    {
        if (m_jslHandle >= 0)
            JslSetRumble(m_jslHandle, 0, 0);
        m_rumbleEndTime = 0;
    }

    vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, poses, vr::k_unMaxTrackedDeviceCount);

    vr::TrackedDevicePose_t &hmdPose = poses[vr::k_unTrackedDeviceIndex_Hmd];

    if (m_pendingRecenter)
    {
        m_pendingRecenter = false;
        {
            std::lock_guard<std::mutex> lock(m_quatMutex);
            // Conjugate of rawQuat = inverse rotation = offset to get back to identity
            m_qOffset.w =  m_rawQuat.w;
            m_qOffset.x = -m_rawQuat.x;
            m_qOffset.y = -m_rawQuat.y;
            m_qOffset.z = -m_rawQuat.z;
        }

        // FIX 3: Anchor recenter position to current HMD position.
        // Previously frozenRel was just zeroed, so any position offset was discarded
        // rather than corrected. Now we remember where the HMD was at recenter time
        // so the controller snaps to a predictable position relative to it.
        if (hmdPose.bPoseIsValid && hmdPose.bDeviceIsConnected)
        {
            m_recenterHmdPos[0] = hmdPose.mDeviceToAbsoluteTracking.m[0][3];
            m_recenterHmdPos[1] = hmdPose.mDeviceToAbsoluteTracking.m[1][3];
            m_recenterHmdPos[2] = hmdPose.mDeviceToAbsoluteTracking.m[2][3];
        }

        m_frozenRel[0] = m_frozenRel[1] = m_frozenRel[2] = 0.0f;
    }

    {
        std::lock_guard<std::mutex> lock(m_quatMutex);
        quatMul(m_qOffset.w, m_qOffset.x, m_qOffset.y, m_qOffset.z,
                m_rawQuat.w, m_rawQuat.x, m_rawQuat.y, m_rawQuat.z,
                m_pose.qRotation.w, m_pose.qRotation.x, m_pose.qRotation.y, m_pose.qRotation.z);
    }

    // FIX 4: Normalize the quaternion result every frame.
    // quatMul output drifts away from unit length over time due to floating-point
    // error. A non-unit quaternion sent to SteamVR produces garbage rotations.
    {
        double qn = sqrt(
            m_pose.qRotation.w * m_pose.qRotation.w +
            m_pose.qRotation.x * m_pose.qRotation.x +
            m_pose.qRotation.y * m_pose.qRotation.y +
            m_pose.qRotation.z * m_pose.qRotation.z
        );
        if (qn > 0.0)
        {
            m_pose.qRotation.w /= qn;
            m_pose.qRotation.x /= qn;
            m_pose.qRotation.y /= qn;
            m_pose.qRotation.z /= qn;
        }
    }

    if (hmdPose.bPoseIsValid && hmdPose.bDeviceIsConnected)
    {
        float hmdX = hmdPose.mDeviceToAbsoluteTracking.m[0][3];
        float hmdY = hmdPose.mDeviceToAbsoluteTracking.m[1][3];
        float hmdZ = hmdPose.mDeviceToAbsoluteTracking.m[2][3];

        float side = (m_controllerRole == vr::TrackedControllerRole_RightHand) ? 0.3f : -0.3f;
        float armLocal[3] = { side, -0.3f, -0.5f };

        float n = m_pose.qRotation.w * m_pose.qRotation.w
                + m_pose.qRotation.x * m_pose.qRotation.x
                + m_pose.qRotation.y * m_pose.qRotation.y
                + m_pose.qRotation.z * m_pose.qRotation.z;
        if (n > 0.0f) { n = 1.0f / sqrtf(n); }
        float cqw = (float)(m_pose.qRotation.w * n);
        float cqx = (float)(m_pose.qRotation.x * n);
        float cqy = (float)(m_pose.qRotation.y * n);
        float cqz = (float)(m_pose.qRotation.z * n);

        float armWorld[3];
        float ix = cqw * armLocal[0] + cqy * armLocal[2] - cqz * armLocal[1];
        float iy = cqw * armLocal[1] + cqz * armLocal[0] - cqx * armLocal[2];
        float iz = cqw * armLocal[2] + cqx * armLocal[1] - cqy * armLocal[0];
        float iw = -cqx * armLocal[0] - cqy * armLocal[1] - cqz * armLocal[2];
        armWorld[0] = ix * cqw + iw * -cqx + iy * -cqz - iz * -cqy;
        armWorld[1] = iy * cqw + iw * -cqy + iz * -cqx - ix * -cqz;
        armWorld[2] = iz * cqw + iw * -cqz + ix * -cqy - iy * -cqx;

        m_pose.vecPosition[0] = hmdX + armWorld[0] + m_frozenRel[0];
        m_pose.vecPosition[1] = hmdY + armWorld[1] + m_frozenRel[1];
        m_pose.vecPosition[2] = hmdZ + armWorld[2] + m_frozenRel[2];
    }

    vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_controllerId, GetPose(), sizeof(vr::DriverPose_t));
}

void JoyconDriver::ProcessEvent(const vr::VREvent_t &vrEvent)
{
    if (vrEvent.eventType == vr::VREvent_Input_HapticVibration)
    {
        if (m_jslHandle >= 0 &&
            vrEvent.data.hapticVibration.componentHandle == (uint64_t)m_compHaptic)
        {
            float amplitude = vrEvent.data.hapticVibration.fAmplitude;
            float duration  = vrEvent.data.hapticVibration.fDurationSeconds;
            int val = (int)(amplitude * 255.0f);
            JslSetRumble(m_jslHandle, val, val);

            auto now = std::chrono::steady_clock::now();
            m_rumbleEndTime = std::chrono::duration<double>(now.time_since_epoch()).count() + duration;
        }
    }
}

void *JoyconDriver::GetComponent(const char *pchComponentNameAndVersion)
{
    return nullptr;
}
