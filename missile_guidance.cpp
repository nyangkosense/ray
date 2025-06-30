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

// Missile guidance data refs (array datarefs)
static XPLMDataRef gMissileX;
static XPLMDataRef gMissileY;
static XPLMDataRef gMissileZ;
static XPLMDataRef gMissileVX;
static XPLMDataRef gMissileVY;
static XPLMDataRef gMissileVZ;
static XPLMDataRef gMissileQ1; // Quaternion components
static XPLMDataRef gMissileQ2;
static XPLMDataRef gMissileQ3;
static XPLMDataRef gMissileQ4;
static XPLMDataRef gMissileThrustRat;
static XPLMDataRef gMissileTargetLat;
static XPLMDataRef gMissileTargetLon;
static XPLMDataRef gMissileTargetH;
static XPLMDataRef gMissileType;
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
    
    // Initialize missile data refs (array datarefs)
    gMissileX = XPLMFindDataRef("sim/weapons/x");
    gMissileY = XPLMFindDataRef("sim/weapons/y");
    gMissileZ = XPLMFindDataRef("sim/weapons/z");
    gMissileVX = XPLMFindDataRef("sim/weapons/vx");
    gMissileVY = XPLMFindDataRef("sim/weapons/vy");
    gMissileVZ = XPLMFindDataRef("sim/weapons/vz");
    gMissileQ1 = XPLMFindDataRef("sim/weapons/q1");
    gMissileQ2 = XPLMFindDataRef("sim/weapons/q2");
    gMissileQ3 = XPLMFindDataRef("sim/weapons/q3");
    gMissileQ4 = XPLMFindDataRef("sim/weapons/q4");
    gMissileThrustRat = XPLMFindDataRef("sim/weapons/shell/thrust_rat");
    gMissileTargetLat = XPLMFindDataRef("sim/weapons/targ_lat");
    gMissileTargetLon = XPLMFindDataRef("sim/weapons/targ_lon");
    gMissileTargetH = XPLMFindDataRef("sim/weapons/targ_h");
    gMissileType = XPLMFindDataRef("sim/weapons/type");
    
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
    if (!gWeaponCount || !gMissileType) return -1;
    
    int weaponCount = XPLMGetDatai(gWeaponCount);
    
    // Get weapon types array
    int weaponTypes[25];
    int numTypes = XPLMGetDatavi(gMissileType, weaponTypes, 0, weaponCount < 25 ? weaponCount : 25);
    
    for (int i = 0; i < numTypes; i++) {
        // Check if it's a missile type (you may need to adjust this based on X-Plane weapon types)
        if (weaponTypes[i] > 0) {
            return i;
        }
    }
    
    return -1;
}

// Calculate proportional navigation guidance
void CalculateProportionalNavigation(int missileIndex, float deltaTime) {
    if (!gGuidanceTarget.valid || missileIndex < 0 || missileIndex >= 25) return;
    
    // Get missile current position using array access
    float missilePos[3];
    XPLMGetDatavf(gMissileX, &missilePos[0], missileIndex, 1);
    XPLMGetDatavf(gMissileY, &missilePos[1], missileIndex, 1);
    XPLMGetDatavf(gMissileZ, &missilePos[2], missileIndex, 1);
    float missileX = missilePos[0];
    float missileY = missilePos[1];
    float missileZ = missilePos[2];
    
    // Get missile current velocity using array access
    float missileVel[3];
    XPLMGetDatavf(gMissileVX, &missileVel[0], missileIndex, 1);
    XPLMGetDatavf(gMissileVY, &missileVel[1], missileIndex, 1);
    XPLMGetDatavf(gMissileVZ, &missileVel[2], missileIndex, 1);
    float missileVX = missileVel[0];
    float missileVY = missileVel[1];
    float missileVZ = missileVel[2];
    
    // Calculate relative position to target
    float relativeX = gGuidanceTarget.localX - missileX;
    float relativeY = gGuidanceTarget.localY - missileY;
    float relativeZ = gGuidanceTarget.localZ - missileZ;
    
    // Calculate distance to target
    float distance = sqrt(relativeX * relativeX + relativeY * relativeY + relativeZ * relativeZ);
    
    if (distance < 10.0f) {
        // Close to target, reduce thrust
        float thrustVal = 0.1f;
        XPLMSetDatavf(gMissileThrustRat, &thrustVal, missileIndex, 1);
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
    
    // Apply velocity correction (simple proportional control) using array access
    float correctionGain = 0.1f;
    float newVX = missileVX + deltaVX * correctionGain;
    float newVY = missileVY + deltaVY * correctionGain;
    float newVZ = missileVZ + deltaVZ * correctionGain;
    
    XPLMSetDatavf(gMissileVX, &newVX, missileIndex, 1);
    XPLMSetDatavf(gMissileVY, &newVY, missileIndex, 1);
    XPLMSetDatavf(gMissileVZ, &newVZ, missileIndex, 1);
    
    // Update missile orientation quaternion to point toward target
    if (gMissileQ1 && gMissileQ2 && gMissileQ3 && gMissileQ4) {
        
        float q1, q2, q3, q4;
        QuaternionFromDirectionVector(desiredVX, desiredVY, desiredVZ, &q1, &q2, &q3, &q4);
        
        // Get current quaternion using array access
        float currentQ[4];
        XPLMGetDatavf(gMissileQ1, &currentQ[0], missileIndex, 1);
        XPLMGetDatavf(gMissileQ2, &currentQ[1], missileIndex, 1);
        XPLMGetDatavf(gMissileQ3, &currentQ[2], missileIndex, 1);
        XPLMGetDatavf(gMissileQ4, &currentQ[3], missileIndex, 1);
        
        // Smoothly interpolate toward desired orientation
        float orientationGain = 0.2f; // Smooth orientation changes
        float newQ1 = currentQ[0] + (q1 - currentQ[0]) * orientationGain;
        float newQ2 = currentQ[1] + (q2 - currentQ[1]) * orientationGain;
        float newQ3 = currentQ[2] + (q3 - currentQ[2]) * orientationGain;
        float newQ4 = currentQ[3] + (q4 - currentQ[3]) * orientationGain;
        
        // Normalize and set new orientation using array access
        NormalizeQuaternion(&newQ1, &newQ2, &newQ3, &newQ4);
        XPLMSetDatavf(gMissileQ1, &newQ1, missileIndex, 1);
        XPLMSetDatavf(gMissileQ2, &newQ2, missileIndex, 1);
        XPLMSetDatavf(gMissileQ3, &newQ3, missileIndex, 1);
        XPLMSetDatavf(gMissileQ4, &newQ4, missileIndex, 1);
    }
    
    // Set thrust based on distance using array access
    float thrustRatio = 1.0f;
    if (distance < 1000.0f) {
        thrustRatio = 0.5f; // Reduce thrust when close
    } else if (distance > 5000.0f) {
        thrustRatio = 1.0f; // Full thrust when far
    } else {
        thrustRatio = 0.5f + 0.5f * (distance - 1000.0f) / 4000.0f;
    }
    
    XPLMSetDatavf(gMissileThrustRat, &thrustRatio, missileIndex, 1);
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
        
        // Set target coordinates in weapon system using array access
        if (gMissileTargetLat && gMissileTargetLon && gMissileTargetH) {
            float targetLat = (float)gGuidanceTarget.latitude;
            float targetLon = (float)gGuidanceTarget.longitude;
            float targetElev = (float)gGuidanceTarget.elevation;
            
            XPLMSetDatavf(gMissileTargetLat, &targetLat, gActiveMissileIndex, 1);
            XPLMSetDatavf(gMissileTargetLon, &targetLon, gActiveMissileIndex, 1);
            XPLMSetDatavf(gMissileTargetH, &targetElev, gActiveMissileIndex, 1);
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