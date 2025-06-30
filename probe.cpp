#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "XPLMScenery.h"
#include "XPLMGraphics.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMPlugin.h"
#include "XPLMDisplay.h"
#include "XPLMProcessing.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// OpenGL includes
#if IBM
#include <GL/gl.h>
#elif APL
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

// Data refs for aircraft position and orientation
static XPLMDataRef gPlaneLatitude = NULL;
static XPLMDataRef gPlaneLongitude = NULL;
static XPLMDataRef gPlaneElevation = NULL;
static XPLMDataRef gPlaneHeading = NULL;
static XPLMDataRef gPlanePitch = NULL;
static XPLMDataRef gPlaneRoll = NULL;

// Data refs for view/camera direction
static XPLMDataRef gViewHeading = NULL;
static XPLMDataRef gViewPitch = NULL;

// Hotkey handling
static XPLMHotKeyID gLaserHotkey = NULL;

// Function declarations
void LaserDesignationHotkey(void* refcon);

// Terrain probe handle
static XPLMProbeRef gTerrainProbe = NULL;

// Window for display
static XPLMWindowID gWindow = NULL;

// Target coordinates storage
struct TargetCoords {
    double latitude;
    double longitude;
    double elevation;
    float localX, localY, localZ;
    bool valid;
};

static TargetCoords gLastTarget = {0};

// Convert degrees to radians
#define DEG_TO_RAD (M_PI / 180.0)
#define RAD_TO_DEG (180.0 / M_PI)

// Earth radius in meters
#define EARTH_RADIUS 6378137.0

// Plugin lifecycle
PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    strcpy(outName, "JTAC Coordinate System");
    strcpy(outSig, "com.example.jtac_coords");
    strcpy(outDesc, "Laser designation coordinate extraction system");
    
    // Get data refs
    gPlaneLatitude = XPLMFindDataRef("sim/flightmodel/position/latitude");
    gPlaneLongitude = XPLMFindDataRef("sim/flightmodel/position/longitude");
    gPlaneElevation = XPLMFindDataRef("sim/flightmodel/position/elevation");
    gPlaneHeading = XPLMFindDataRef("sim/flightmodel/position/psi");
    gPlanePitch = XPLMFindDataRef("sim/flightmodel/position/theta");
    gPlaneRoll = XPLMFindDataRef("sim/flightmodel/position/phi");
    
    gViewHeading = XPLMFindDataRef("sim/graphics/view/view_heading");
    gViewPitch = XPLMFindDataRef("sim/graphics/view/view_pitch");
    
    // Create terrain probe
    gTerrainProbe = XPLMCreateProbe(xplm_ProbeY);
    
    // Register hotkey for laser designation (Ctrl+L)
    gLaserHotkey = XPLMRegisterHotKey(XPLM_VK_L, xplm_DownFlag | xplm_ControlFlag, "Laser Target Designation", LaserDesignationHotkey, NULL);
    
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    if (gLaserHotkey) {
        XPLMUnregisterHotKey(gLaserHotkey);
    }
    if (gTerrainProbe) {
        XPLMDestroyProbe(gTerrainProbe);
    }
}

PLUGIN_API void XPluginDisable(void) {}
PLUGIN_API int XPluginEnable(void) { return 1; }

// Matrix multiplication helper functions
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

// Convert screen coordinates to world ray using X-Plane's view system
bool ScreenToWorldRay(int screenX, int screenY, 
                     float* rayStartX, float* rayStartY, float* rayStartZ,
                     float* rayDirX, float* rayDirY, float* rayDirZ) {
    
    // Get camera/view position (not aircraft position)
    XPLMDataRef camXRef = XPLMFindDataRef("sim/graphics/view/view_x");
    XPLMDataRef camYRef = XPLMFindDataRef("sim/graphics/view/view_y");
    XPLMDataRef camZRef = XPLMFindDataRef("sim/graphics/view/view_z");
    
    if (!camXRef || !camYRef || !camZRef) {
        // Fallback to aircraft position if camera position not available
        double aircraftLat = XPLMGetDatad(gPlaneLatitude);
        double aircraftLon = XPLMGetDatad(gPlaneLongitude);
        double aircraftElev = XPLMGetDatad(gPlaneElevation);
        
        double localX, localY, localZ;
        XPLMWorldToLocal(aircraftLat, aircraftLon, aircraftElev, &localX, &localY, &localZ);
        
        *rayStartX = (float)localX;
        *rayStartY = (float)localY + 2.0f; // Add 2m height for eye level
        *rayStartZ = (float)localZ;
    } else {
        // Use actual camera position
        *rayStartX = XPLMGetDataf(camXRef);
        *rayStartY = XPLMGetDataf(camYRef);
        *rayStartZ = XPLMGetDataf(camZRef);
    }
    
    // Get current screen dimensions
    XPLMDataRef screenWidthRef = XPLMFindDataRef("sim/graphics/view/window_width");
    XPLMDataRef screenHeightRef = XPLMFindDataRef("sim/graphics/view/window_height");
    int screenWidth = XPLMGetDatai(screenWidthRef);
    int screenHeight = XPLMGetDatai(screenHeightRef);
    
    // Convert to normalized coordinates (-1 to 1)
    float normalX = (2.0f * screenX / (float)screenWidth) - 1.0f;
    float normalY = 1.0f - (2.0f * screenY / (float)screenHeight);
    
    // Get view parameters
    float heading = XPLMGetDataf(gViewHeading) * DEG_TO_RAD;
    float pitch = XPLMGetDataf(gViewPitch) * DEG_TO_RAD;
    
    // Get actual field of view
    XPLMDataRef fovRef = XPLMFindDataRef("sim/graphics/view/field_of_view_deg");
    float fov = fovRef ? XPLMGetDataf(fovRef) * DEG_TO_RAD : (45.0f * DEG_TO_RAD);
    float aspect = (float)screenWidth / (float)screenHeight;
    
    // Calculate ray direction based on screen position and view angles
    float tanHalfFov = tan(fov * 0.5f);
    
    // Ray direction in view space (camera relative)
    float viewX = normalX * tanHalfFov * aspect;
    float viewY = normalY * tanHalfFov;
    float viewZ = -1.0f; // Forward direction (negative Z in OpenGL)
    
    // Transform ray direction by view rotation
    float cosHeading = cos(heading);
    float sinHeading = sin(heading);
    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);
    
    // Apply pitch rotation (around X axis)
    float tempY = viewY * cosPitch - viewZ * sinPitch;
    float tempZ = viewY * sinPitch + viewZ * cosPitch;
    viewY = tempY;
    viewZ = tempZ;
    
    // Apply heading rotation (around Y axis) 
    *rayDirX = viewX * cosHeading - viewZ * sinHeading;
    *rayDirY = viewY;
    *rayDirZ = viewX * sinHeading + viewZ * cosHeading;
    
    // Normalize direction vector
    float length = sqrt(*rayDirX * *rayDirX + *rayDirY * *rayDirY + *rayDirZ * *rayDirZ);
    if (length > 0.0f) {
        *rayDirX /= length;
        *rayDirY /= length;
        *rayDirZ /= length;
        return true;
    }
    
    return false;
}

// Perform laser designation at screen coordinates
bool DesignateLaser(int screenX, int screenY, TargetCoords* target) {
    // Get ray start and direction
    // double aircraftLat = XPLMGetDatad(gPlaneLatitude);
    // double aircraftLon = XPLMGetDatad(gPlaneLongitude);
    // double aircraftElev = XPLMGetDatad(gPlaneElevation);
    
    // Get ray start and direction using OpenGL matrices
    float rayStartX, rayStartY, rayStartZ;
    float rayDirX, rayDirY, rayDirZ;
    
    if (!ScreenToWorldRay(screenX, screenY,
                         &rayStartX, &rayStartY, &rayStartZ,
                         &rayDirX, &rayDirY, &rayDirZ)) {
        return false;
    }
    
    // Perform terrain intersection using ray marching
    const int maxSteps = 1000;
    const float stepSize = 10.0f; // 10 meters per step
    
    XPLMProbeInfo_t probeInfo;
    probeInfo.structSize = sizeof(XPLMProbeInfo_t);
    
    for (int i = 1; i <= maxSteps; i++) {
        float testX = rayStartX + rayDirX * stepSize * i;
        float testY = rayStartY + rayDirY * stepSize * i;
        float testZ = rayStartZ + rayDirZ * stepSize * i;
        
        // Probe terrain at this point
        XPLMProbeResult result = XPLMProbeTerrainXYZ(gTerrainProbe, testX, testY, testZ, &probeInfo);
        
        if (result == xplm_ProbeHitTerrain) {
            // Found intersection - store target coordinates
            target->localX = probeInfo.locationX;
            target->localY = probeInfo.locationY;
            target->localZ = probeInfo.locationZ;
            
            // Convert back to world coordinates
            XPLMLocalToWorld(probeInfo.locationX, probeInfo.locationY, probeInfo.locationZ,
                           &target->latitude, &target->longitude, &target->elevation);
            
            target->valid = true;
            return true;
        }
        
        // Check if we've gone too far (beyond reasonable range)
        if (stepSize * i > 50000.0f) { // 50km max range
            break;
        }
    }
    
    target->valid = false;
    return false;
}

// Calculate bearing and distance to target
void CalculateTargetInfo(const TargetCoords* target, float* bearing, float* distance, float* elevation) {
    double aircraftLat = XPLMGetDatad(gPlaneLatitude);
    double aircraftLon = XPLMGetDatad(gPlaneLongitude);
    double aircraftElev = XPLMGetDatad(gPlaneElevation);
    
    // Calculate bearing using great circle formula
    double dLon = (target->longitude - aircraftLon) * DEG_TO_RAD;
    double lat1 = aircraftLat * DEG_TO_RAD;
    double lat2 = target->latitude * DEG_TO_RAD;
    
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    
    *bearing = atan2(y, x) * RAD_TO_DEG;
    if (*bearing < 0) *bearing += 360.0f;
    
    // Calculate distance using haversine formula
    double dLat = (target->latitude - aircraftLat) * DEG_TO_RAD;
    double a = sin(dLat/2) * sin(dLat/2) + cos(lat1) * cos(lat2) * sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    *distance = (float)(EARTH_RADIUS * c);
    
    // Calculate elevation difference
    *elevation = (float)(target->elevation - aircraftElev);
}

// Format coordinates for military grid reference (simplified MGRS)
void FormatMGRS(double latitude, double longitude, char* output, size_t outputSize) {
    // This is a simplified MGRS format - full implementation would require UTM conversion
    int latDeg = (int)latitude;
    int latMin = (int)((latitude - latDeg) * 60);
    float latSec = (float)(((latitude - latDeg) * 60 - latMin) * 60);
    
    int lonDeg = (int)longitude;
    int lonMin = (int)((longitude - lonDeg) * 60);
    float lonSec = (float)(((longitude - lonDeg) * 60 - lonMin) * 60);
    
    snprintf(output, outputSize, "%02d°%02d'%05.2f\"N %03d°%02d'%05.2f\"W",
             abs(latDeg), latMin, latSec, abs(lonDeg), lonMin, lonSec);
}

// Hotkey callback for laser designation (Ctrl+L)
void LaserDesignationHotkey(void* refcon) {
    // Use screen center as target point (crosshair designation)
    XPLMDataRef screenWidthRef = XPLMFindDataRef("sim/graphics/view/window_width");
    XPLMDataRef screenHeightRef = XPLMFindDataRef("sim/graphics/view/window_height");
    
    if (screenWidthRef && screenHeightRef) {
        int screenWidth = XPLMGetDatai(screenWidthRef);
        int screenHeight = XPLMGetDatai(screenHeightRef);
        
        // Use screen center (crosshair/center of view)
        int centerX = screenWidth / 2;
        int centerY = screenHeight / 2;
        
        // Perform laser designation at screen center
        if (DesignateLaser(centerX, centerY, &gLastTarget)) {
            XPLMDebugString("JTAC: Target designated successfully at crosshair\n");
        } else {
            XPLMDebugString("JTAC: No terrain intersection found at crosshair\n");
        }
    } else {
        XPLMDebugString("JTAC: Could not get screen dimensions\n");
    }
}

// Display window draw function
void DrawWindow(XPLMWindowID inWindowID, void* inRefcon) {
    int left, top, right, bottom;
    XPLMGetWindowGeometry(inWindowID, &left, &top, &right, &bottom);
    
    // Draw background
    XPLMDrawTranslucentDarkBox(left, top, right, bottom);
    
    char buffer[512];
    int line = 0;
    
    // Display aircraft position
    double aircraftLat = XPLMGetDatad(gPlaneLatitude);
    double aircraftLon = XPLMGetDatad(gPlaneLongitude);
    double aircraftElev = XPLMGetDatad(gPlaneElevation);
    
    snprintf(buffer, sizeof(buffer), "JTAC Coordinate System");
    float white[] = {1.0f, 1.0f, 1.0f};
    XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
    
    snprintf(buffer, sizeof(buffer), "Aircraft: %.6f, %.6f, %.0fm", aircraftLat, aircraftLon, aircraftElev);
    XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
    
    // Display target information if valid
    if (gLastTarget.valid) {
        line++; // Skip a line
        
        snprintf(buffer, sizeof(buffer), "TARGET COORDINATES:");
        float yellow[] = {1.0f, 1.0f, 0.0f};
        XPLMDrawString(yellow, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        // MGRS format
        char mgrs[128];
        FormatMGRS(gLastTarget.latitude, gLastTarget.longitude, mgrs, sizeof(mgrs));
        snprintf(buffer, sizeof(buffer), "MGRS: %s", mgrs);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        // Decimal degrees
        snprintf(buffer, sizeof(buffer), "LAT/LON: %.6f, %.6f", gLastTarget.latitude, gLastTarget.longitude);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        // Elevation
        snprintf(buffer, sizeof(buffer), "ELEVATION: %.0fm MSL", gLastTarget.elevation);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        // Bearing, distance, elevation difference
        float bearing, distance, elevDiff;
        CalculateTargetInfo(&gLastTarget, &bearing, &distance, &elevDiff);
        
        snprintf(buffer, sizeof(buffer), "BEARING: %.1f°", bearing);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        snprintf(buffer, sizeof(buffer), "DISTANCE: %.0fm", distance);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
        
        snprintf(buffer, sizeof(buffer), "ELEV DIFF: %+.0fm", elevDiff);
        XPLMDrawString(white, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
    }
    
    // Instructions
    line++;
    snprintf(buffer, sizeof(buffer), "Press Ctrl+L to designate target at crosshair");
    float gray[] = {0.78f, 0.78f, 0.78f};
    XPLMDrawString(gray, left + 10, top - 20 - (line++ * 15), buffer, NULL, xplmFont_Proportional);
}

// Create window
void CreateJTACWindow() {
    XPLMCreateWindow_t params;
    params.structSize = sizeof(params);
    params.left = 50;
    params.top = 600;
    params.right = 400;
    params.bottom = 400;
    params.visible = 1;
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