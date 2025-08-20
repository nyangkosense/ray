/*
 * MIT License
 *
 * Copyright (c) sebastian <sebastian@eingabeausgabe.io>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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

// X-Plane weapon datarefs are array-based to support multiple missiles
static XPLMDataRef gMissileX;
static XPLMDataRef gMissileY;
static XPLMDataRef gMissileZ;
static XPLMDataRef gMissileVX;
static XPLMDataRef gMissileVY;
static XPLMDataRef gMissileVZ;
static XPLMDataRef gMissileQ1;
static XPLMDataRef gMissileQ2;
static XPLMDataRef gMissileQ3;
static XPLMDataRef gMissileQ4;
static XPLMDataRef gMissileThrustRat;
static XPLMDataRef gMissileTargetLat;
static XPLMDataRef gMissileTargetLon;
static XPLMDataRef gMissileTargetH;
static XPLMDataRef gMissileType;
static XPLMDataRef gWeaponCount;

// Aircraft position needed for relative calculations
static XPLMDataRef gPlaneLatitude;
static XPLMDataRef gPlaneLongitude;
static XPLMDataRef gPlaneElevation;

// Target coordinates storage - maintains both world and local coordinates
struct TargetCoords {
    double latitude;
    double longitude;
    double elevation;
    float localX, localY, localZ;
    bool valid;
};

static TargetCoords gGuidanceTarget = {0};
static bool gGuidanceActive = false;

// X-Plane supports up to 25 simultaneous weapons
#define MAX_MISSILES 25
struct ActiveMissile {
    int index;
    bool hasTarget;
    float lastSeenTime;
};
static ActiveMissile gActiveMissiles[MAX_MISSILES];
static int gNumActiveMissiles = 0;

// Quaternion math for missile orientation - needed because X-Plane uses quaternions internally
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
    // Converts direction vector to quaternion for missile orientation
    // Assumes X-Plane missile model has forward axis along +Z
    
    // Normalize direction vector
    float length = sqrt(dirX * dirX + dirY * dirY + dirZ * dirZ);
    if (length < 0.0001f) return;
    
    dirX /= length;
    dirY /= length;
    dirZ /= length;
    
    // Calculate rotation from (0,0,1) to direction vector
    float dot = dirZ; // dot product with (0,0,1)
    
    if (dot > 0.9999f) {
        *q1 = 1.0f; *q2 = 0.0f; *q3 = 0.0f; *q4 = 0.0f;
        return;
    }
    
    if (dot < -0.9999f) {
        *q1 = 0.0f; *q2 = 1.0f; *q3 = 0.0f; *q4 = 0.0f;
        return;
    }
    
    // Cross product (0,0,1) x direction gives rotation axis
    float axisX = 0.0f - dirY;
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

// Forward declaration for X-Plane flight loop callback
float MissileGuidanceCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, 
                             int inCounter, void* inRefcon);

// Finds X-Plane weapon datarefs and registers flight loop callback
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


// Internal function to update guidance target
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

// Prevents duplicate tracking of same missile index
bool IsMissileTracked(int missileIndex) {
    for (int i = 0; i < gNumActiveMissiles; i++) {
        if (gActiveMissiles[i].index == missileIndex) {
            return true;
        }
    }
    return false;
}

// Starts guidance for newly detected missile
void AddActiveMissile(int missileIndex) {
    if (gNumActiveMissiles >= MAX_MISSILES || IsMissileTracked(missileIndex)) {
        return;
    }
    
    gActiveMissiles[gNumActiveMissiles].index = missileIndex;
    gActiveMissiles[gNumActiveMissiles].hasTarget = true;
    gActiveMissiles[gNumActiveMissiles].lastSeenTime = 0.0f;
    gNumActiveMissiles++;
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "Missile Guidance: Added missile %d to tracking (total: %d)\n", 
             missileIndex, gNumActiveMissiles);
    XPLMDebugString(buffer);
}

// Cleans up missiles that have exploded or disappeared
void CleanupInactiveMissiles() {
    if (!gWeaponCount || !gMissileType) return;
    
    int weaponCount = XPLMGetDatai(gWeaponCount);
    int weaponTypes[MAX_MISSILES];
    int numTypes = XPLMGetDatavi(gMissileType, weaponTypes, 0, weaponCount < MAX_MISSILES ? weaponCount : MAX_MISSILES);
    
    // Remove missiles that are no longer active
    for (int i = gNumActiveMissiles - 1; i >= 0; i--) {
        int missileIndex = gActiveMissiles[i].index;
        
        // Check if missile still exists and is active
        bool stillActive = false;
        if (missileIndex < numTypes && weaponTypes[missileIndex] > 0) {
            stillActive = true;
        }
        
        if (!stillActive) {
            // Remove this missile from tracking
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Missile Guidance: Removing inactive missile %d from tracking\n", missileIndex);
            XPLMDebugString(buffer);
            
            // Shift remaining missiles down
            for (int j = i; j < gNumActiveMissiles - 1; j++) {
                gActiveMissiles[j] = gActiveMissiles[j + 1];
            }
            gNumActiveMissiles--;
        }
    }
}

// Detects newly fired missiles and adds them to guidance
void FindNewMissiles() {
    if (!gWeaponCount || !gMissileType) return;
    
    int weaponCount = XPLMGetDatai(gWeaponCount);
    int weaponTypes[MAX_MISSILES];
    int numTypes = XPLMGetDatavi(gMissileType, weaponTypes, 0, weaponCount < MAX_MISSILES ? weaponCount : MAX_MISSILES);
    
    for (int i = 0; i < numTypes; i++) {
        // Check if it's a missile type and not already tracked
        if (weaponTypes[i] > 0 && !IsMissileTracked(i)) {
            AddActiveMissile(i);
        }
    }
}

// Per-missile state tracking for smooth guidance
struct MissileState {
    float prevPosX, prevPosY, prevPosZ;
    float prevVelX, prevVelY, prevVelZ;
    float targetDirX, targetDirY, targetDirZ;
    float lastUpdateTime;
    bool initialized;
};

static MissileState gMissileStates[25] = {0};

// Proportional navigation guidance - preserves missile speed, only adjusts direction
void CalculateSmoothGuidance(int missileIndex, float deltaTime) {
    if (!gGuidanceTarget.valid || missileIndex < 0 || missileIndex >= 25) return;
    
    // Get missile current position
    float missilePos[3];
    XPLMGetDatavf(gMissileX, &missilePos[0], missileIndex, 1);
    XPLMGetDatavf(gMissileY, &missilePos[1], missileIndex, 1);
    XPLMGetDatavf(gMissileZ, &missilePos[2], missileIndex, 1);
    float missileX = missilePos[0];
    float missileY = missilePos[1];
    float missileZ = missilePos[2];
    
    // Get missile current velocity
    float missileVel[3];
    XPLMGetDatavf(gMissileVX, &missileVel[0], missileIndex, 1);
    XPLMGetDatavf(gMissileVY, &missileVel[1], missileIndex, 1);
    XPLMGetDatavf(gMissileVZ, &missileVel[2], missileIndex, 1);
    float missileVX = missileVel[0];
    float missileVY = missileVel[1];
    float missileVZ = missileVel[2];
    
    // Calculate vector to target
    float toTargetX = gGuidanceTarget.localX - missileX;
    float toTargetY = gGuidanceTarget.localY - missileY;
    float toTargetZ = gGuidanceTarget.localZ - missileZ;
    float distanceToTarget = sqrt(toTargetX * toTargetX + toTargetY * toTargetY + toTargetZ * toTargetZ);
    
    // Check if very close to target
    if (distanceToTarget < 20.0f) {
        // Stop thrust when very close
        float noThrust = 0.0f;
        XPLMSetDatavf(gMissileThrustRat, &noThrust, missileIndex, 1);
        return;
    }
    
    // Get current speed (magnitude)
    float currentSpeed = sqrt(missileVX * missileVX + missileVY * missileVY + missileVZ * missileVZ);
    
    // Don't modify speed - just preserve it
    if (currentSpeed < 1.0f) currentSpeed = 50.0f; // Fallback minimum speed
    
    // Calculate desired direction (normalized vector to target)
    float desiredDirX = toTargetX / distanceToTarget;
    float desiredDirY = toTargetY / distanceToTarget;
    float desiredDirZ = toTargetZ / distanceToTarget;
    
    // Current direction (normalized current velocity)
    float currentDirX = missileVX / currentSpeed;
    float currentDirY = missileVY / currentSpeed;
    float currentDirZ = missileVZ / currentSpeed;
    
    // Low gain prevents oscillation and maintains stable flight
    float steeringGain = 0.05f;
    
    // Smooth interpolation prevents violent course corrections
    float newDirX = currentDirX + (desiredDirX - currentDirX) * steeringGain;
    float newDirY = currentDirY + (desiredDirY - currentDirY) * steeringGain;
    float newDirZ = currentDirZ + (desiredDirZ - currentDirZ) * steeringGain;
    
    // Normalize new direction
    float newDirLength = sqrt(newDirX * newDirX + newDirY * newDirY + newDirZ * newDirZ);
    if (newDirLength > 0.001f) {
        newDirX /= newDirLength;
        newDirY /= newDirLength;
        newDirZ /= newDirLength;
    }
    
    // Preserve original missile speed to maintain X-Plane physics
    float newVX = newDirX * currentSpeed;
    float newVY = newDirY * currentSpeed;
    float newVZ = newDirZ * currentSpeed;
    
    // Only modify direction vector, not magnitude
    XPLMSetDatavf(gMissileVX, &newVX, missileIndex, 1);
    XPLMSetDatavf(gMissileVY, &newVY, missileIndex, 1);
    XPLMSetDatavf(gMissileVZ, &newVZ, missileIndex, 1);
    
    // Maintain consistent thrust for predictable flight
    float steadyThrust = 0.8f;
    XPLMSetDatavf(gMissileThrustRat, &steadyThrust, missileIndex, 1);
    
    // Gradually align missile body with flight direction
    if (gMissileQ1 && gMissileQ2 && gMissileQ3 && gMissileQ4) {
        float q1, q2, q3, q4;
        QuaternionFromDirectionVector(newDirX, newDirY, newDirZ, &q1, &q2, &q3, &q4);
        
        // Get current quaternion
        float currentQ[4];
        XPLMGetDatavf(gMissileQ1, &currentQ[0], missileIndex, 1);
        XPLMGetDatavf(gMissileQ2, &currentQ[1], missileIndex, 1);
        XPLMGetDatavf(gMissileQ3, &currentQ[2], missileIndex, 1);
        XPLMGetDatavf(gMissileQ4, &currentQ[3], missileIndex, 1);
        
        // Even lower gain for orientation to prevent visual jittering
        float orientationGain = 0.02f;
        float newQ1 = currentQ[0] + (q1 - currentQ[0]) * orientationGain;
        float newQ2 = currentQ[1] + (q2 - currentQ[1]) * orientationGain;
        float newQ3 = currentQ[2] + (q3 - currentQ[2]) * orientationGain;
        float newQ4 = currentQ[3] + (q4 - currentQ[3]) * orientationGain;
        
        NormalizeQuaternion(&newQ1, &newQ2, &newQ3, &newQ4);
        XPLMSetDatavf(gMissileQ1, &newQ1, missileIndex, 1);
        XPLMSetDatavf(gMissileQ2, &newQ2, missileIndex, 1);
        XPLMSetDatavf(gMissileQ3, &newQ3, missileIndex, 1);
        XPLMSetDatavf(gMissileQ4, &newQ4, missileIndex, 1);
    }
}

// Called by X-Plane every 0.05 seconds during flight
float MissileGuidanceCallback(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, 
                             int inCounter, void* inRefcon) {
    
    if (!gGuidanceActive) {
        return 0.1f;
    }
    
    // Maintain active missile list
    CleanupInactiveMissiles();
    
    // Track newly fired missiles
    FindNewMissiles();
    
    // Apply guidance to each tracked missile
    for (int i = 0; i < gNumActiveMissiles; i++) {
        int missileIndex = gActiveMissiles[i].index;
        
        // Update X-Plane's internal target datarefs
        if (gActiveMissiles[i].hasTarget && gMissileTargetLat && gMissileTargetLon && gMissileTargetH) {
            float targetLat = (float)gGuidanceTarget.latitude;
            float targetLon = (float)gGuidanceTarget.longitude;
            float targetElev = (float)gGuidanceTarget.elevation;
            
            XPLMSetDatavf(gMissileTargetLat, &targetLat, missileIndex, 1);
            XPLMSetDatavf(gMissileTargetLon, &targetLon, missileIndex, 1);
            XPLMSetDatavf(gMissileTargetH, &targetElev, missileIndex, 1);
        }
        
        // Calculate and apply course corrections
        if (gActiveMissiles[i].hasTarget) {
            CalculateSmoothGuidance(missileIndex, inElapsedSinceLastCall);
        }
    }
    
    return 0.05f; // 20Hz update rate for smooth guidance
}

// C interface for integration with main plugin
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
        
        // Set target for all currently tracked missiles
        for (int i = 0; i < gNumActiveMissiles; i++) {
            gActiveMissiles[i].hasTarget = true;
        }
    }
    
    void ClearMissileTarget() {
        gGuidanceActive = false;
        
        // Clear all tracked missiles
        gNumActiveMissiles = 0;
        
        XPLMDebugString("Missile Guidance: Target cleared, all missiles released\n");
    }
}
