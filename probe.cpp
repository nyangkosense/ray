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

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "XPLMScenery.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMPlugin.h"
#include "XPLMDisplay.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "missile_guidance.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// OpenGL needed for coordinate transformations
#if IBM
#include <GL/gl.h>
#elif APL
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

// Aircraft datarefs for position calculations
static XPLMDataRef gPlaneLatitude = NULL;
static XPLMDataRef gPlaneLongitude = NULL;
static XPLMDataRef gPlaneElevation = NULL;
static XPLMDataRef gPlaneHeading = NULL;
static XPLMDataRef gPlanePitch = NULL;
static XPLMDataRef gPlaneRoll = NULL;

// View system datarefs for ray casting
static XPLMDataRef gViewHeading = NULL;
static XPLMDataRef gViewPitch = NULL;

// Hotkey IDs for JTAC controls
static XPLMHotKeyID gLaserHotkey = NULL;
static XPLMHotKeyID gClearTargetHotkey = NULL;

// Plugin menu integration
static XPLMMenuID gMenu = NULL;
static int gMenuItemToggleWindow = 0;

// Forward declarations
void LaserDesignationHotkey(void* refcon);
void ClearTargetHotkey(void* refcon);
void JTACMenuHandler(void* inMenuRef, void* inItemRef);

// X-Plane terrain intersection probe
static XPLMProbeRef gTerrainProbe = NULL;

// JTAC display window
static XPLMWindowID gWindow = NULL;

// Stores both world and local coordinates for efficiency
struct TargetCoords {
    double latitude;
    double longitude;
    double elevation;
    float localX, localY, localZ;
    bool valid;
};

static TargetCoords gLastTarget = {0};

// Mathematical constants
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

// WGS84 Earth radius for distance calculations
#define EARTH_RADIUS 6378137.0

// X-Plane plugin initialization
PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    strcpy(outName, "JTAC Coordinate System");
    strcpy(outSig, "com.example.jtac_coords");
    strcpy(outDesc, "Laser designation coordinate extraction system");
    
    // Locate X-Plane datarefs
    gPlaneLatitude = XPLMFindDataRef("sim/flightmodel/position/latitude");
    gPlaneLongitude = XPLMFindDataRef("sim/flightmodel/position/longitude");
    gPlaneElevation = XPLMFindDataRef("sim/flightmodel/position/elevation");
    gPlaneHeading = XPLMFindDataRef("sim/flightmodel/position/psi");
    gPlanePitch = XPLMFindDataRef("sim/flightmodel/position/theta");
    gPlaneRoll = XPLMFindDataRef("sim/flightmodel/position/phi");
    
    gViewHeading = XPLMFindDataRef("sim/graphics/view/view_heading");
    gViewPitch = XPLMFindDataRef("sim/graphics/view/view_pitch");
    
    // Y-probe for terrain height queries
    gTerrainProbe = XPLMCreateProbe(xplm_ProbeY);
    
    // Ctrl+L for crosshair targeting
    gLaserHotkey = XPLMRegisterHotKey(XPLM_VK_L, xplm_DownFlag | xplm_ControlFlag, "Laser Target Designation", LaserDesignationHotkey, NULL);
    
    // Ctrl+C to clear guidance target
    gClearTargetHotkey = XPLMRegisterHotKey(XPLM_VK_C, xplm_DownFlag | xplm_ControlFlag, "Clear Missile Target", ClearTargetHotkey, NULL);
    
    // Add plugin menu entry
    int pluginMenuIndex = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "JTAC Coordinate System", NULL, 1);
    gMenu = XPLMCreateMenu("JTAC Coordinate System", XPLMFindPluginsMenu(), pluginMenuIndex, JTACMenuHandler, NULL);
    gMenuItemToggleWindow = XPLMAppendMenuItem(gMenu, "Toggle Window", (void*)1, 1);
    
    // Start missile guidance subsystem
    if (!InitMissileGuidance()) {
        XPLMDebugString("JTAC: Warning - Missile guidance system failed to initialize\n");
    }
    
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    ShutdownMissileGuidance();
    if (gLaserHotkey) {
        XPLMUnregisterHotKey(gLaserHotkey);
    }
    if (gClearTargetHotkey) {
        XPLMUnregisterHotKey(gClearTargetHotkey);
    }
    if (gMenu) {
        XPLMDestroyMenu(gMenu);
    }
    if (gTerrainProbe) {
        XPLMDestroyProbe(gTerrainProbe);
    }
}

PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API int XPluginEnable(void) { return 1; }

// 4x4 matrix operations for coordinate transformations
void MultiplyMatrix4x4(const float* a, const float* b, float* result) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[i * 4 + j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                result[i * 4 + j] += a[i * 4 + k] * b[k * 4 + j];
            }
        }
    }
}

// Matrix inversion using analytical method - unused but kept for reference
// Current implementation uses direct trigonometric approach instead
void InvertMatrix4x4(const float* m, float* invOut) {
    float inv[16], det;
    int i;

    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
    
    if (det == 0) return;

    det = 1.0f / det;
    for (i = 0; i < 16; i++) {
        invOut[i] = inv[i] * det;
    }
}

// Projects 2D screen coordinates to 3D world ray using camera parameters
bool ScreenToWorldRay(int screenX, int screenY, 
                     float* rayStartX, float* rayStartY, float* rayStartZ,
                     float* rayDirX, float* rayDirY, float* rayDirZ) {
    
    // Use actual camera position for accurate ray origin
    XPLMDataRef camXRef = XPLMFindDataRef("sim/graphics/view/view_x");
    XPLMDataRef camYRef = XPLMFindDataRef("sim/graphics/view/view_y");
    XPLMDataRef camZRef = XPLMFindDataRef("sim/graphics/view/view_z");
    
    if (!camXRef || !camYRef || !camZRef) {
        // Fallback when camera datarefs unavailable
        double aircraftLat = XPLMGetDatad(gPlaneLatitude);
        double aircraftLon = XPLMGetDatad(gPlaneLongitude);
        double aircraftElev = XPLMGetDatad(gPlaneElevation);
        
        double localX, localY, localZ;
        XPLMWorldToLocal(aircraftLat, aircraftLon, aircraftElev, &localX, &localY, &localZ);
        
        *rayStartX = (float)localX;
        *rayStartY = (float)localY + 2.0f;
        *rayStartZ = (float)localZ;
    } else {
        *rayStartX = XPLMGetDataf(camXRef);
        *rayStartY = XPLMGetDataf(camYRef);
        *rayStartZ = XPLMGetDataf(camZRef);
    }
    
    // Screen size needed for normalized coordinates
    XPLMDataRef screenWidthRef = XPLMFindDataRef("sim/graphics/view/window_width");
    XPLMDataRef screenHeightRef = XPLMFindDataRef("sim/graphics/view/window_height");
    int screenWidth = XPLMGetDatai(screenWidthRef);
    int screenHeight = XPLMGetDatai(screenHeightRef);
    
    // OpenGL normalized device coordinates
    float normalX = (2.0f * screenX / (float)screenWidth) - 1.0f;
    float normalY = 1.0f - (2.0f * screenY / (float)screenHeight);
    
    // Current view orientation
    float heading = XPLMGetDataf(gViewHeading) * DEG_TO_RAD;
    float pitch = XPLMGetDataf(gViewPitch) * DEG_TO_RAD;
    
    // FOV determines ray spread angle
    XPLMDataRef fovRef = XPLMFindDataRef("sim/graphics/view/field_of_view_deg");
    float fov = fovRef ? XPLMGetDataf(fovRef) * DEG_TO_RAD : (45.0f * DEG_TO_RAD);
    float aspect = (float)screenWidth / (float)screenHeight;
    
    // Transform screen position to world direction vector
    float tanHalfFov = tan(fov * 0.5f);
    
    // Calculate direction in camera space
    float viewX = normalX * tanHalfFov * aspect;
    float viewY = normalY * tanHalfFov;
    float viewZ = -1.0f;
    
    // Apply view orientation to ray direction
    float cosHeading = cos(heading);
    float sinHeading = sin(heading);
    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);
    
    // Pitch rotation first
    float tempY = viewY * cosPitch - viewZ * sinPitch;
    float tempZ = viewY * sinPitch + viewZ * cosPitch;
    viewY = tempY;
    viewZ = tempZ;
    
    // Then heading rotation
    *rayDirX = viewX * cosHeading - viewZ * sinHeading;
    *rayDirY = viewY;
    *rayDirZ = viewX * sinHeading + viewZ * cosHeading;
    
    // Ensure unit vector for consistent scaling
    float length = sqrt(*rayDirX * *rayDirX + *rayDirY * *rayDirY + *rayDirZ * *rayDirZ);
    if (length > 0.0f) {
        *rayDirX /= length;
        *rayDirY /= length;
        *rayDirZ /= length;
        return true;
    }
    
    return false;
}

// Main targeting function - converts screen position to terrain coordinates
bool DesignateLaser(int screenX, int screenY, TargetCoords* target) {
    // Convert screen coordinates to 3D ray
    float rayStartX, rayStartY, rayStartZ;
    float rayDirX, rayDirY, rayDirZ;
    
    if (!ScreenToWorldRay(screenX, screenY,
                         &rayStartX, &rayStartY, &rayStartZ,
                         &rayDirX, &rayDirY, &rayDirZ)) {
        return false;
    }
    
    // Binary search for precise terrain intersection point
    // More efficient than linear stepping and avoids missing terrain
    
    float minDistance = 1.0f;
    float maxDistance = 30000.0f;
    float currentDistance = maxDistance;
    
    XPLMProbeInfo_t probeInfo;
    probeInfo.structSize = sizeof(XPLMProbeInfo_t);
    
    // Iterative refinement to 1-meter precision
    int iterations = 0;
    const int maxIterations = 50;
    
    while ((maxDistance - minDistance) > 1.0f && iterations < maxIterations) {
        currentDistance = (minDistance + maxDistance) * 0.5f;
        
        // Test point at current distance
        float testX = rayStartX + rayDirX * currentDistance;
        float testY = rayStartY + rayDirY * currentDistance;
        float testZ = rayStartZ + rayDirZ * currentDistance;
        
        // Query terrain height at test point
        XPLMProbeResult result = XPLMProbeTerrainXYZ(gTerrainProbe, testX, testY, testZ, &probeInfo);
        
        if (result == xplm_ProbeHitTerrain) {
            if (testY < probeInfo.locationY) {
                // Ray below terrain - intersection is closer
                maxDistance = currentDistance;
            } else {
                // Ray above terrain - intersection is further
                minDistance = currentDistance;
            }
        } else {
            // No terrain data available at this location
            maxDistance = currentDistance;
        }
        
        iterations++;
    }
    
    // Final intersection calculation
    currentDistance = (minDistance + maxDistance) * 0.5f;
    float finalX = rayStartX + rayDirX * currentDistance;
    float finalY = rayStartY + rayDirY * currentDistance;
    float finalZ = rayStartZ + rayDirZ * currentDistance;
    
    XPLMProbeResult finalResult = XPLMProbeTerrainXYZ(gTerrainProbe, finalX, finalY, finalZ, &probeInfo);
    
    if (finalResult == xplm_ProbeHitTerrain) {
        target->localX = probeInfo.locationX;
        target->localY = probeInfo.locationY;
        target->localZ = probeInfo.locationZ;
        
        // Transform to lat/lon/elevation
        XPLMLocalToWorld(probeInfo.locationX, probeInfo.locationY, probeInfo.locationZ,
                       &target->latitude, &target->longitude, &target->elevation);
        
        target->valid = true;
        return true;
    }
    
    target->valid = false;
    return false;
}

// Navigation calculations for target relative to aircraft
void CalculateTargetInfo(const TargetCoords* target, float* bearing, float* distance, float* elevation) {
    double aircraftLat = XPLMGetDatad(gPlaneLatitude);
    double aircraftLon = XPLMGetDatad(gPlaneLongitude);
    double aircraftElev = XPLMGetDatad(gPlaneElevation);
    
    // True bearing using spherical trigonometry
    double dLon = (target->longitude - aircraftLon) * DEG_TO_RAD;
    double lat1 = aircraftLat * DEG_TO_RAD;
    double lat2 = target->latitude * DEG_TO_RAD;
    
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    
    *bearing = atan2(y, x) * RAD_TO_DEG;
    if (*bearing < 0) *bearing += 360.0f;
    
    // Great circle distance on Earth's surface
    double dLat = (target->latitude - aircraftLat) * DEG_TO_RAD;
    double a = sin(dLat/2) * sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    *distance = (float)(EARTH_RADIUS * c);
    
    // Height difference for angle calculations
    *elevation = (float)(target->elevation - aircraftElev);
}

// Convert decimal degrees to military coordinate format
void FormatMGRS(double latitude, double longitude, char* output, size_t outputSize) {
    // Simplified format - full MGRS requires UTM grid calculations
    int latDeg = (int)latitude;
    int latMin = (int)((latitude - latDeg) * 60);
    float latSec = (float)(((latitude - latDeg) * 60 - latMin) * 60);
    
    int lonDeg = (int)longitude;
    int lonMin = (int)((longitude - lonDeg) * 60);
    float lonSec = (float)(((longitude - lonDeg) * 60 - lonMin) * 60);
    
    snprintf(output, outputSize, "%02d°%02d'%05.2f\"N %03d°%02d'%05.2f\"W",
             abs(latDeg), latMin, latSec, abs(lonDeg), lonMin, lonSec);
}

// Ctrl+L handler - designates target at screen center
void LaserDesignationHotkey(void* refcon) {
    // Target at crosshair (screen center)
    XPLMDataRef screenWidthRef = XPLMFindDataRef("sim/graphics/view/window_width");
    XPLMDataRef screenHeightRef = XPLMFindDataRef("sim/graphics/view/window_height");
    
    if (screenWidthRef && screenHeightRef) {
        int screenWidth = XPLMGetDatai(screenWidthRef);
        int screenHeight = XPLMGetDatai(screenHeightRef);
        
        // Calculate screen center coordinates
        int centerX = screenWidth / 2;
        int centerY = screenHeight / 2;
        
        // Execute targeting at crosshair
        if (DesignateLaser(centerX, centerY, &gLastTarget)) {
            XPLMDebugString("JTAC: Target designated successfully at crosshair\n");
            
            // Activate missile guidance
            SetJTACTarget(gLastTarget.latitude, gLastTarget.longitude, gLastTarget.elevation,
                         gLastTarget.localX, gLastTarget.localY, gLastTarget.localZ);
        } else {
            XPLMDebugString("JTAC: No terrain intersection found at crosshair\n");
        }
    } else {
        XPLMDebugString("JTAC: Could not get screen dimensions\n");
    }
}

// Ctrl+C handler - clears all guidance targets
void ClearTargetHotkey(void* refcon) {
    ClearMissileTarget();
    XPLMDebugString("JTAC: Missile target cleared\n");
}

// Plugin menu item handler
void JTACMenuHandler(void* inMenuRef, void* inItemRef) {
    if ((intptr_t)inItemRef == 1) {
        if (gWindow) {
            int isVisible = XPLMGetWindowIsVisible(gWindow);
            XPLMSetWindowIsVisible(gWindow, !isVisible);
            XPLMDebugString(isVisible ? "JTAC: Window hidden\n" : "JTAC: Window shown\n");
        }
    }
}

// Renders JTAC coordinate display window
void DrawWindow(XPLMWindowID inWindowID, void* inRefcon) {
    int left, top, right, bottom;
    XPLMGetWindowGeometry(inWindowID, &left, &top, &right, &bottom);
    
    // Semi-transparent background
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);
    
    char buffer[512];
    int line = 0;
    
    // Show current aircraft location
    double aircraftLat = XPLMGetDatad(gPlaneLatitude);
    double aircraftLon = XPLMGetDatad(gPlaneLongitude);
    double aircraftElev = XPLMGetDatad(gPlaneElevation);
    
    snprintf(buffer, sizeof(buffer), "JTAC Coordinate System");
    float white[] = {1.0f, 1.0f, 1.0f};
    XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
    
    snprintf(buffer, sizeof(buffer), "Aircraft: %.6f, %.6f, %.0fm", aircraftLat, aircraftLon, aircraftElev);
    XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
    
    // Show targeting data when available
    if (gLastTarget.valid) {
        line++;
        
        snprintf(buffer, sizeof(buffer), "TARGET COORDINATES:");
        float yellow[] = {1.0f, 1.0f, 0.0f};
        XPLMDrawString(yellow, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        // Military grid format
        char mgrs[128];
        FormatMGRS(gLastTarget.latitude, gLastTarget.longitude, mgrs, sizeof(mgrs));
        snprintf(buffer, sizeof(buffer), "MGRS: %s", mgrs);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        // High precision coordinates
        snprintf(buffer, sizeof(buffer), "LAT/LON: %.6f, %.6f", gLastTarget.latitude, gLastTarget.longitude);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        // Target altitude
        snprintf(buffer, sizeof(buffer), "ELEVATION: %.0fm MSL", gLastTarget.elevation);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        // Navigation information
        float bearing, distance, elevDiff;
        CalculateTargetInfo(&gLastTarget, &bearing, &distance, &elevDiff);
        
        snprintf(buffer, sizeof(buffer), "BEARING: %.1f°", bearing);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        snprintf(buffer, sizeof(buffer), "DISTANCE: %.0fm", distance);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        snprintf(buffer, sizeof(buffer), "ELEV DIFF: %+.0fm", elevDiff);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
    }
    
    // Usage instructions
    line++;
    snprintf(buffer, sizeof(buffer), "Press Ctrl+L to designate target at crosshair");
    float gray[] = {0.78f, 0.78f, 0.78f};
    XPLMDrawString(gray, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
    
    snprintf(buffer, sizeof(buffer), "Press Ctrl+C to clear missile target");
    XPLMDrawString(gray, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
    
    snprintf(buffer, sizeof(buffer), "Use Plugins > JTAC > Toggle Window to show/hide");
    XPLMDrawString(gray, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
}

// Initialize JTAC display window
void CreateJTACWindow() {
    XPLMCreateWindow_t params;
    params.structSize = sizeof(params);
    params.left = 50;
    params.top = 600;
    params.right = 400;
    params.bottom = 400;
    params.visible = 0;
    params.drawWindowFunc = DrawWindow;
    params.handleMouseClickFunc = NULL;
    params.handleKeyFunc = NULL;
    params.handleCursorFunc = NULL;
    params.handleMouseWheelFunc = NULL;
    params.refcon = NULL;
    params.decorateAsFloatingWindow = xplm_WindowDecorationRoundRectangle;
    params.layer = xplm_WindowLayerFloatingWindows;
    params.handleRightClickFunc = NULL;
    
    gWindow = XPLMCreateWindowEx(&params);
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void* inParam) {
    if (inMsg == XPLM_MSG_PLANE_LOADED) {
        if (!gWindow) {
            CreateJTACWindow();
        }
    }
}
