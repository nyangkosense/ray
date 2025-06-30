#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

// Missile guidance data refs
static XPLMDataRef gMissileX[25];
static XPLMDataRef gMissileY[25];
static XPLMDataRef gMissileZ[25];
static XPLMDataRef gMissileVX[25];
static XPLMDataRef gMissileVY[25];
static XPLMDataRef gMissileVZ[25];
static XPLMDataRef gMissileQ1[25]; // Quaternion components
static XPLMDataRef gMissileQ2[25];
static XPLMDataRef gMissileQ3[25];
static XPLMDataRef gMissileQ4[25];
static XPLMDataRef gMissileThrustRat[25];
static XPLMDataRef gMissileTargetLat[25];
static XPLMDataRef gMissileTargetLon[25];
static XPLMDataRef gMissileTargetH[25];
static XPLMDataRef gMissileType[25];
static XPLMDataRef gWeaponCount;

// Aircraft data refs
static XPLMDataRef gPlaneLatitude;
static XPLMDataRef gPlaneLongitude;
static XPLMDataRef gPlaneElevation;

// Target coordinates from JTAC system
struct TargetCoords {
    double latitude;
    double longitude;
    double elevation;
    float localX, localY, localZ;
    bool valid;
};

static TargetCoords gGuidanceTarget = {0};
static bool gGuidanceActive = false;
static int gActiveMissileIndex = -1;

// Quaternion utility functions
void NormalizeQuaternion(float* q1, float* q2, float* q3, float* q4) {
    float magnitude = sqrt(*q1 * *q1 + *q2 * *q2 + *q3 * *q3 + *q4 * *q4);
    if (magnitude > 0.0001f) {
        *q1 /= magnitude;
        *q2 /= magnitude;
        *q3 /= magnitude;
        *q4 /= magnitude;
    }
}

void QuaternionFromDirectionVector(float dirX, float dirY, float dirZ, 
                                   float* q1, float* q2, float* q3, float* q4) {
    // Create quaternion that rotates missile to point in direction vector
    // This is a simplified version - assumes missile forward is along Z axis
    
    // Normalize direction vector
    float length = sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
    if (length < 0.0001f) return;
    
    dirX /= length;
    dirY /= length;
    dirZ /= length;
    
    // Calculate rotation from (0,0,1) to direction vector
    float dot = dirZ; // dot product with (0,0,1)
    
    if (dot > 0.9999f) {
        // Already pointing forward
        *q1 = 1.0f; *q2 = 0.0f; *q3 = 0.0f; *q4 = 0.0f;
        return;
    }
    
    if (dot < -0.9999f) {
        // Pointing backward - rotate 180 degrees around X axis
        *q1 = 0.0f; *q2 = 1.0f; *q3 = 0.0f; *q4 = 0.0f;
        return;
    }
    
    // Cross product to get rotation axis
    float axisX = 0.0f - dirY; // (0,0,1) x (dirX,dirY,dirZ)
    float axisY = dirX - 0.0f;
    float axisZ = 0.0f - 0.0f;
    
    float axisLength = sqrt(axisX * axisX + axisY * axisY + axisZ * axisZ);
    if (axisLength > 0.0001f) {
        axisX /= axisLength;
        axisY /= axisLength;
        axisZ /= axisLength;
    }
    
    float angle = acos(dot);
    float sinHalfAngle = sin(angle * 0.5f);
    float cosHalfAngle = cos(angle * 0.5f);
    
    *q1 = cosHalfAngle;
    *q2 = axisX * sinHalfAngle;
    *q3 = axisY * sinHalfAngle;
    *q4 = axisZ * sinHalfAngle;
    
    NormalizeQuaternion(q1, q2, q3, q4);
}

// Flight callback
float MissileGuidanceCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, 
                             int inCounter, void* inRefcon);

// Initialize missile guidance system
bool InitializeMissileGuidance() {
    // Get weapon count
    gWeaponCount = XPLMFindDataRef("sim/weapons/weapon_count");
    if (!gWeaponCount) {
        XPLMDebugString("Missile Guidance: Could not find weapon_count dataref\n");
        return false;
    }
    
    // Initialize missile data refs for all 25 possible weapons
    for (int i = 0; i < 25; i++) {
        char datarefName[256];
        
        // Position data refs
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/x[%d]", i);
        gMissileX[i] = XPLMFindDataRef(datarefName);
        
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/y[%d]", i);
        gMissileY[i] = XPLMFindDataRef(datarefName);
        
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/z[%d]", i);
        gMissileZ[i] = XPLMFindDataRef(datarefName);
        
        // Velocity data refs
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/vx[%d]", i);
        gMissileVX[i] = XPLMFindDataRef(datarefName);
        
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/vy[%d]", i);
        gMissileVY[i] = XPLMFindDataRef(datarefName);
        
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/vz[%d]", i);
        gMissileVZ[i] = XPLMFindDataRef(datarefName);
        
        // Quaternion orientation data refs
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/q1[%d]", i);
        gMissileQ1[i] = XPLMFindDataRef(datarefName);
        
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/q2[%d]", i);
        gMissileQ2[i] = XPLMFindDataRef(datarefName);
        
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/q3[%d]", i);
        gMissileQ3[i] = XPLMFindDataRef(datarefName);
        
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/q4[%d]", i);
        gMissileQ4[i] = XPLMFindDataRef(datarefName);
        
        // Thrust control
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/shell/thrust_rat[%d]", i);
        gMissileThrustRat[i] = XPLMFindDataRef(datarefName);
        
        // Target coordinates
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/targ_lat[%d]", i);
        gMissileTargetLat[i] = XPLMFindDataRef(datarefName);
        
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/targ_lon[%d]", i);
        gMissileTargetLon[i] = XPLMFindDataRef(datarefName);
        
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/targ_h[%d]", i);
        gMissileTargetH[i] = XPLMFindDataRef(datarefName);
        
        // Weapon type
        snprintf(datarefName, sizeof(datarefName), "sim/weapons/type[%d]", i);
        gMissileType[i] = XPLMFindDataRef(datarefName);
    }
    
    // Aircraft position
    gPlaneLatitude = XPLMFindDataRef("sim/flightmodel/position/latitude");
    gPlaneLongitude = XPLMFindDataRef("sim/flightmodel/position/longitude");
    gPlaneElevation = XPLMFindDataRef("sim/flightmodel/position/elevation");
    
    // Register flight loop callback for guidance
    XPLMRegisterFlightLoopCallback(MissileGuidanceCallback, 0.1f, NULL);
    
    XPLMDebugString("Missile Guidance: Initialized successfully\n");
    return true;
}


// Set target coordinates for missile guidance
void SetMissileTarget(const TargetCoords& target) {
    gGuidanceTarget = target;
    gGuidanceActive = target.valid;
    
    if (gGuidanceActive) {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), 
                "Missile Guidance: Target set to %.6f, %.6f, %.0fm\n", 
                target.latitude, target.longitude, target.elevation);
        XPLMDebugString(buffer);
    }
}

// Find the next available missile to guide
int FindAvailableMissile() {
    if (!gWeaponCount) return -1;
    
    int weaponCount = XPLMGetDatai(gWeaponCount);
    
    for (int i = 0; i < weaponCount && i < 25; i++) {
        if (gMissileType[i] && gMissileX[i] && gMissileY[i] && gMissileZ[i]) {
            int weaponType = XPLMGetDatai(gMissileType[i]);
            // Check if it's a missile type (you may need to adjust this based on X-Plane weapon types)
            if (weaponType > 0) {
                return i;
            }
        }
    }
    
    return -1;
}

// Calculate proportional navigation guidance
void CalculateProportionalNavigation(int missileIndex, float deltaTime) {
    if (!gGuidanceTarget.valid || missileIndex < 0 || missileIndex >= 25) return;
    
    // Get missile current position
    float missileX = XPLMGetDataf(gMissileX[missileIndex]);
    float missileY = XPLMGetDataf(gMissileY[missileIndex]);
    float missileZ = XPLMGetDataf(gMissileZ[missileIndex]);
    
    // Get missile current velocity
    float missileVX = XPLMGetDataf(gMissileVX[missileIndex]);
    float missileVY = XPLMGetDataf(gMissileVY[missileIndex]);
    float missileVZ = XPLMGetDataf(gMissileVZ[missileIndex]);
    
    // Calculate relative position to target
    float relativeX = gGuidanceTarget.localX - missileX;
    float relativeY = gGuidanceTarget.localY - missileY;
    float relativeZ = gGuidanceTarget.localZ - missileZ;
    
    // Calculate distance to target
    float distance = sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);
    
    if (distance < 10.0f) {
        // Close to target, reduce thrust
        XPLMSetDataf(gMissileThrustRat[missileIndex], 0.1f);
        return;
    }
    
    // Normalize relative position vector (line of sight)
    float losX = relativeX / distance;
    float losY = relativeY / distance;
    float losZ = relativeZ / distance;
    
    // Calculate missile speed
    float speed = sqrt(missileVX * missileVX + missileVY * missileVY + missileVZ * missileVZ);
    if (speed < 1.0f) speed = 1.0f; // Avoid division by zero
    
    // Calculate closing velocity
    float closingVelocity = -(missileVX * losX + missileVY * losY + missileVZ * losZ);
    
    // Time to intercept
    float timeToIntercept = distance / (closingVelocity > 0 ? closingVelocity : 1.0f);
    
    // Proportional navigation constant (typically 3-5)
    float navigationConstant = 3.0f;
    
    // Calculate desired velocity direction
    float desiredVX = losX;
    float desiredVY = losY; 
    float desiredVZ = losZ;
    
    // Apply proportional navigation correction
    if (timeToIntercept > 0.1f) {
        // Calculate angular velocity of line of sight
        static float prevLosX = losX, prevLosY = losY, prevLosZ = losZ;
        float angularVelX = (losX - prevLosX) / deltaTime;
        float angularVelY = (losY - prevLosY) / deltaTime;
        float angularVelZ = (losZ - prevLosZ) / deltaTime;
        
        // Apply proportional navigation
        desiredVX += navigationConstant * speed * angularVelX;
        desiredVY += navigationConstant * speed * angularVelY;
        desiredVZ += navigationConstant * speed * angularVelZ;
        
        prevLosX = losX;
        prevLosY = losY;
        prevLosZ = losZ;
    }
    
    // Normalize desired velocity
    float desiredSpeed = sqrt(desiredVX * desiredVX + desiredVY * desiredVY + desiredVZ * desiredVZ);
    if (desiredSpeed > 0.1f) {
        desiredVX = (desiredVX / desiredSpeed) * speed;
        desiredVY = (desiredVY / desiredSpeed) * speed;
        desiredVZ = (desiredVZ / desiredSpeed) * speed;
    }
    
    // Calculate velocity correction
    float deltaVX = desiredVX - missileVX;
    float deltaVY = desiredVY - missileVY;
    float deltaVZ = desiredVZ - missileVZ;
    
    // Apply velocity correction (simple proportional control)
    float correctionGain = 0.1f;
    XPLMSetDataf(gMissileVX[missileIndex], missileVX + deltaVX * correctionGain);
    XPLMSetDataf(gMissileVY[missileIndex], missileVY + deltaVY * correctionGain);
    XPLMSetDataf(gMissileVZ[missileIndex], missileVZ + deltaVZ * correctionGain);
    
    // Update missile orientation quaternion to point toward target
    if (gMissileQ1[missileIndex] && gMissileQ2[missileIndex] && 
        gMissileQ3[missileIndex] && gMissileQ4[missileIndex]) {
        
        float q1, q2, q3, q4;
        QuaternionFromDirectionVector(desiredVX, desiredVY, desiredVZ, &q1, &q2, &q3, &q4);
        
        // Smoothly interpolate toward desired orientation
        float currentQ1 = XPLMGetDataf(gMissileQ1[missileIndex]);
        float currentQ2 = XPLMGetDataf(gMissileQ2[missileIndex]);
        float currentQ3 = XPLMGetDataf(gMissileQ3[missileIndex]);
        float currentQ4 = XPLMGetDataf(gMissileQ4[missileIndex]);
        
        float orientationGain = 0.2f; // Smooth orientation changes
        float newQ1 = currentQ1 + (q1 - currentQ1) * orientationGain;
        float newQ2 = currentQ2 + (q2 - currentQ2) * orientationGain;
        float newQ3 = currentQ3 + (q3 - currentQ3) * orientationGain;
        float newQ4 = currentQ4 + (q4 - currentQ4) * orientationGain;
        
        // Normalize and set new orientation
        NormalizeQuaternion(&newQ1, &newQ2, &newQ3, &newQ4);
        XPLMSetDataf(gMissileQ1[missileIndex], newQ1);
        XPLMSetDataf(gMissileQ2[missileIndex], newQ2);
        XPLMSetDataf(gMissileQ3[missileIndex], newQ3);
        XPLMSetDataf(gMissileQ4[missileIndex], newQ4);
    }
    
    // Set thrust based on distance
    float thrustRatio = 1.0f;
    if (distance < 1000.0f) {
        thrustRatio = 0.5f; // Reduce thrust when close
    } else if (distance > 5000.0f) {
        thrustRatio = 1.0f; // Full thrust when far
    } else {
        thrustRatio = 0.5f + 0.5f * (distance - 1000.0f) / 4000.0f;
    }
    
    XPLMSetDataf(gMissileThrustRat[missileIndex], thrustRatio);
}

// Flight loop callback for missile guidance
float MissileGuidanceCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, 
                             int inCounter, void* inRefcon) {
    
    if (!gGuidanceActive) {
        return 0.1f; // Call again in 0.1 seconds
    }
    
    // Find active missile if we don't have one
    if (gActiveMissileIndex < 0) {
        gActiveMissileIndex = FindAvailableMissile();
        if (gActiveMissileIndex < 0) {
            return 0.1f; // No missiles available
        }
        
        // Set target coordinates in weapon system
        if (gMissileTargetLat[gActiveMissileIndex] && gMissileTargetLon[gActiveMissileIndex] && gMissileTargetH[gActiveMissileIndex]) {
            XPLMSetDataf(gMissileTargetLat[gActiveMissileIndex], (float)gGuidanceTarget.latitude);
            XPLMSetDataf(gMissileTargetLon[gActiveMissileIndex], (float)gGuidanceTarget.longitude);
            XPLMSetDataf(gMissileTargetH[gActiveMissileIndex], (float)gGuidanceTarget.elevation);
        }
        
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Missile Guidance: Guiding missile %d to target\n", gActiveMissileIndex);
        XPLMDebugString(buffer);
    }
    
    // Apply proportional navigation guidance
    CalculateProportionalNavigation(gActiveMissileIndex, inElapsedSinceLastCall);
    
    return 0.05f; // Call again in 0.05 seconds for responsive guidance
}

// External interface for JTAC system
extern "C" {
    bool InitMissileGuidance() {
        return InitializeMissileGuidance();
    }
    
    void ShutdownMissileGuidance() {
        XPLMUnregisterFlightLoopCallback(MissileGuidanceCallback, NULL);
    }
    
    void SetJTACTarget(double latitude, double longitude, double elevation, 
                       float localX, float localY, float localZ) {
        TargetCoords target;
        target.latitude = latitude;
        target.longitude = longitude;
        target.elevation = elevation;
        target.localX = localX;
        target.localY = localY;
        target.localZ = localZ;
        target.valid = true;
        
        SetMissileTarget(target);
        gActiveMissileIndex = -1; // Reset to find new missile
    }
    
    void ClearMissileTarget() {
        gGuidanceActive = false;
        gActiveMissileIndex = -1;
        XPLMDebugString("Missile Guidance: Target cleared\n");
    }
}