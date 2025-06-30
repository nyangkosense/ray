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

// Mouse tracking
static XPLMDataRef gMouseX = NULL;
static XPLMDataRef gMouseY = NULL;
static XPLMDataRef gScreenWidth = NULL;
static XPLMDataRef gScreenHeight = NULL;

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
    
    gMouseX = XPLMFindDataRef("sim/graphics/view/click_3d_x");
    gMouseY = XPLMFindDataRef("sim/graphics/view/click_3d_y");
    gScreenWidth = XPLMFindDataRef("sim/graphics/view/window_width");
    gScreenHeight = XPLMFindDataRef("sim/graphics/view/window_height");
    
    // Create terrain probe
    gTerrainProbe = XPLMCreateProbe(xplm_ProbeY);
    
    return 1;
}

PLUGIN_API void XPluginStop(void) {
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

// Convert screen coordinates to local ray direction using OpenGL matrices
bool ScreenToLocalRay(int screenX, int screenY, double aircraftLat, double aircraftLon, double aircraftElev,
                      float* rayStartX, float* rayStartY, float* rayStartZ,
                      float* rayDirX, float* rayDirY, float* rayDirZ) {
    
    // Convert aircraft position to local coordinates for ray start
    double localX, localY, localZ;
    XPLMWorldToLocal(aircraftLat, aircraftLon, aircraftElev, &localX, &localY, &localZ);
    
    *rayStartX = (float)localX;
    *rayStartY = (float)localY;
    *rayStartZ = (float)localZ;
    
    // Get OpenGL matrices
    GLfloat modelMatrix[16];
    GLfloat projMatrix[16];
    GLint viewport[4];
    
    glGetFloatv(GL_MODELVIEW_MATRIX, modelMatrix);
    glGetFloatv(GL_PROJECTION_MATRIX, projMatrix);
    glGetIntegerv(GL_VIEWPORT, viewport);
    
    // Calculate combined model-view-projection matrix
    float mvpMatrix[16];
    MultiplyMatrix4x4(projMatrix, modelMatrix, mvpMatrix);
    
    // Invert the MVP matrix
    float invMvpMatrix[16];
    InvertMatrix4x4(mvpMatrix, invMvpMatrix);
    
    // Convert screen coordinates to normalized device coordinates
    float ndcX = (2.0f * (screenX - viewport[0])) / viewport[2] - 1.0f;
    float ndcY = 1.0f - (2.0f * (screenY - viewport[1])) / viewport[3];
    
    // Create two points in NDC space (near and far)
    float nearPoint[4] = {ndcX, ndcY, -1.0f, 1.0f}; // Near plane
    float farPoint[4] = {ndcX, ndcY, 1.0f, 1.0f};   // Far plane
    
    // Transform to OpenGL world space
    float worldNear[4], worldFar[4];
    
    // Transform near point
    worldNear[0] = invMvpMatrix[0] * nearPoint[0] + invMvpMatrix[4] * nearPoint[1] + 
                   invMvpMatrix[8] * nearPoint[2] + invMvpMatrix[12] * nearPoint[3];
    worldNear[1] = invMvpMatrix[1] * nearPoint[0] + invMvpMatrix[5] * nearPoint[1] + 
                   invMvpMatrix[9] * nearPoint[2] + invMvpMatrix[13] * nearPoint[3];
    worldNear[2] = invMvpMatrix[2] * nearPoint[0] + invMvpMatrix[6] * nearPoint[1] + 
                   invMvpMatrix[10] * nearPoint[2] + invMvpMatrix[14] * nearPoint[3];
    worldNear[3] = invMvpMatrix[3] * nearPoint[0] + invMvpMatrix[7] * nearPoint[1] + 
                   invMvpMatrix[11] * nearPoint[2] + invMvpMatrix[15] * nearPoint[3];
    
    // Transform far point
    worldFar[0] = invMvpMatrix[0] * farPoint[0] + invMvpMatrix[4] * farPoint[1] + 
                  invMvpMatrix[8] * farPoint[2] + invMvpMatrix[12] * farPoint[3];
    worldFar[1] = invMvpMatrix[1] * farPoint[0] + invMvpMatrix[5] * farPoint[1] + 
                  invMvpMatrix[9] * farPoint[2] + invMvpMatrix[13] * farPoint[3];
    worldFar[2] = invMvpMatrix[2] * farPoint[0] + invMvpMatrix[6] * farPoint[1] + 
                  invMvpMatrix[10] * farPoint[2] + invMvpMatrix[14] * farPoint[3];
    worldFar[3] = invMvpMatrix[3] * farPoint[0] + invMvpMatrix[7] * farPoint[1] + 
                  invMvpMatrix[11] * farPoint[2] + invMvpMatrix[15] * farPoint[3];
    
    // Perspective divide
    if (worldNear[3] != 0.0f) {
        worldNear[0] /= worldNear[3];
        worldNear[1] /= worldNear[3];
        worldNear[2] /= worldNear[3];
    }
    
    if (worldFar[3] != 0.0f) {
        worldFar[0] /= worldFar[3];
        worldFar[1] /= worldFar[3];
        worldFar[2] /= worldFar[3];
    }
    
    // The OpenGL unprojection gives us OpenGL world coordinates
    // We need to convert these to X-Plane local coordinates
    // X-Plane's OpenGL coordinate system is already in local space relative to the world origin
    
    // Calculate ray direction in local space
    *rayDirX = worldFar[0] - worldNear[0];
    *rayDirY = worldFar[1] - worldNear[1];
    *rayDirZ = worldFar[2] - worldNear[2];
    
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
    // Get aircraft position in world coordinates
    double aircraftLat = XPLMGetDatad(gPlaneLatitude);
    double aircraftLon = XPLMGetDatad(gPlaneLongitude);
    double aircraftElev = XPLMGetDatad(gPlaneElevation);
    
    // Get ray start and direction using OpenGL matrices
    float rayStartX, rayStartY, rayStartZ;
    float rayDirX, rayDirY, rayDirZ;
    
    if (!ScreenToLocalRay(screenX, screenY, aircraftLat, aircraftLon, aircraftElev,
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

// Mouse click handler
int MouseClickHandler(XPLMWindowID inWindowID, int x, int y, int mouse, void* inRefcon) {
    if (mouse == xplm_MouseDown) {
        // Convert window coordinates to screen coordinates
        int left, top, right, bottom;
        XPLMGetWindowGeometry(inWindowID, &left, &top, &right, &bottom);
        
        int screenX = x;
        int screenY = y;
        
        // Perform laser designation
        if (DesignateLaser(screenX, screenY, &gLastTarget)) {
            XPLMDebugString("JTAC: Target designated successfully\n");
        } else {
            XPLMDebugString("JTAC: No terrain intersection found\n");
        }
    }
    
    return 1;
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
    snprintf(buffer, sizeof(buffer), "Click to designate laser target");
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
    params.handleMouseClickFunc = MouseClickHandler;
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