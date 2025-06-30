#ifndef MISSILE_GUIDANCE_H
#define MISSILE_GUIDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

// Initialize missile guidance system
bool InitMissileGuidance();

// Shutdown missile guidance system
void ShutdownMissileGuidance();

// Set target from JTAC system
void SetJTACTarget(double latitude, double longitude, double elevation, 
                   float localX, float localY, float localZ);

// Clear current missile target
void ClearMissileTarget();

#ifdef __cplusplus
}
#endif

#endif // MISSILE_GUIDANCE_H