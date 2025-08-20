# X-Plane 12 JTAC Coordinate System Plugin

![JTAC Plugin Screenshot](https://github.com/user-attachments/assets/5d265fbd-f6bc-4b2b-addf-7de1c6cfd4f5)

![20250820102224_1](https://github.com/user-attachments/assets/78986e31-69d4-4c91-a0ea-e5a7c4c61a19)

## Overview

This plugin implements a realistic Joint Terminal Attack Controller (JTAC) coordinate designation system for X-Plane 12. It combines precise terrain probing with missile guidance capabilities, allowing pilots to designate targets and guide missiles to specific coordinates using real-world military procedures.

## Installation

[Download](https://github.com/nyangkosense/ray/releases/download/1.0/JTACCoords.xpl) the Plugin from the Release and place it in \X-Plane 12\Resources\plugins

[Video Demonstration](https://www.youtube.com/watch?v=hPHwOmK8MfQ)

** NOTE: This plugin calculates Target Coordinates from the center view of your screen. By Pressing CTRL + L the target is "selected" and you can arm & fire your missiles. **
** The missiles need to be in "internal radar" mode, can be selected in Planemaker. **

<img width="787" height="390" alt="intrdr" src="https://github.com/user-attachments/assets/e29168df-6e3a-4154-92ba-35ea14b2ed18" />


## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## How It Works

### 1. **Terrain Probing & Ray Casting**

The plugin uses sophisticated 3D ray casting to determine where the pilot is looking and find the exact terrain intersection point:

```cpp
// Convert screen coordinates to world ray using X-Plane's view system
bool ScreenToWorldRay(int screenX, int screenY, 
                     float* rayStartX, float* rayStartY, float* rayStartZ,
                     float* rayDirX, float* rayDirY, float* rayDirZ)
```

**Why This Works:**
- **Camera Position Detection**: Uses X-Plane's view system datarefs (`sim/graphics/view/view_x`, `view_y`, `view_z`) to get the actual camera position, not the aircraft position
- **View Angle Calculation**: Incorporates real field of view (`sim/graphics/view/field_of_view_deg`) and screen aspect ratio for accurate ray direction
- **Coordinate Transformation**: Converts 2D screen coordinates to 3D world rays using proper OpenGL perspective projection math

### 2. **Binary Search Terrain Intersection**

The plugin uses a divide-and-conquer approach to find precise terrain intersection points:

```cpp
// Binary search for terrain intersection
while ((maxDistance - minDistance) > 1.0f && iterations < maxIterations) {
    currentDistance = (minDistance + maxDistance) * 0.5f;
    
    // Calculate test point along ray
    float testX = rayStartX + rayDirX * currentDistance;
    float testY = rayStartY + rayDirY * currentDistance;
    float testZ = rayStartZ + rayDirZ * currentDistance;
    
    // Probe terrain at this point
    XPLMProbeResult result = XPLMProbeTerrainXYZ(gTerrainProbe, testX, testY, testZ, &probeInfo);
    
    if (result == xplm_ProbeHitTerrain) {
        if (testY < probeInfo.locationY) {
            // We're underground - intersection is closer
            maxDistance = currentDistance;
        } else {
            // We're above ground - intersection is further
            minDistance = currentDistance;
        }
    }
}
```

**Why This Method:**
- **Accuracy**: Achieves 1-meter precision through iterative refinement
- **Performance**: Only requires ~50 iterations maximum vs. thousands of linear probes
- **Reliability**: Works with X-Plane's terrain mesh system regardless of terrain complexity
- **Range**: Supports up to 30km targeting range

### 3. **Coordinate System Integration**

The plugin converts between multiple coordinate systems for military compatibility:

```cpp
// Convert back to world coordinates
XPLMLocalToWorld(probeInfo.locationX, probeInfo.locationY, probeInfo.locationZ,
               &target->latitude, &target->longitude, &target->elevation);
```

**Coordinate Transformations:**
- **Local Coordinates**: X-Plane's internal meter-based coordinate system
- **World Coordinates**: Latitude/Longitude/Elevation (WGS84)
- **MGRS Format**: Military Grid Reference System for tactical communication
- **Bearing/Distance**: Great circle calculations for navigation

### 4. **Missile Guidance System**

The plugin implements realistic missile physics with proportional navigation:

```cpp
void CalculateSmoothGuidance(int missileIndex, float deltaTime) {
    // Calculate vector to target
    float toTargetX = gGuidanceTarget.localX - missileX;
    float toTargetY = gGuidanceTarget.localY - missileY;
    float toTargetZ = gGuidanceTarget.localZ - missileZ;
    
    // Very gentle steering - small correction factor
    float steeringGain = 0.05f; // Very small - smooth changes only
    
    // Calculate new direction (interpolate very slowly toward target)
    float newDirX = currentDirX + (desiredDirX - currentDirX) * steeringGain;
    float newDirY = currentDirY + (desiredDirY - currentDirY) * steeringGain;
    float newDirZ = currentDirZ + (desiredDirZ - currentDirZ) * steeringGain;
}
```

**Why This Approach:**
- **Smooth Flight**: Uses gentle steering (0.05 gain) to avoid "vibrating" missiles
- **Physics Preservation**: Doesn't interfere with X-Plane's flight model
- **Realistic Behavior**: Implements proportional navigation used in real missile systems
- **Stability**: Prevents oscillation and erratic behavior

## Technical Architecture

### Plugin Components

1. **probe.cpp** - Main plugin with terrain probing and JTAC functionality
2. **missile_guidance.cpp** - Missile physics and guidance system  
3. **missile_guidance.h** - C interface for missile system integration

### Key Features

#### Real-Time Terrain Analysis
- Uses X-Plane's `XPLMProbeTerrainXYZ` API for accurate terrain data
- Binary search algorithm provides meter-level precision
- Works with all X-Plane scenery including custom terrain

#### Military-Standard Coordinate Display
- **MGRS Format**: `31UCQ1234567890` style grid references
- **Decimal Degrees**: High-precision lat/lon coordinates
- **Bearing/Distance**: Great circle navigation data
- **Elevation**: MSL altitude of target point

#### Weapon System Integration
- Interfaces with X-Plane's weapon datarefs (`sim/weapons/x`, `sim/weapons/vx`, etc.)
- Supports multiple missile types through array-based datarefs
- Real-time guidance updates at 20Hz (0.05 second intervals)

## Controls

- **Ctrl+L**: Designate target at screen crosshair
- **Ctrl+C**: Clear current missile target

## Installation

1. Compile the plugin using the provided Makefile:
   ```bash
   make
   ```

2. Copy the built plugin folder to X-Plane 12:
   ```
   X-Plane 12/Resources/plugins/JTACCoords/
   ```

## Technical Requirements

- **X-Plane 12**: Uses modern X-Plane SDK features
- **MinGW-w64**: For cross-compilation to Windows
- **OpenGL**: For graphics and coordinate transformations
- **C++11**: Modern C++ features for reliability

## Why This System Works

### 1. **Precision Through Mathematics**
The ray casting system uses proper 3D geometry and OpenGL perspective projection to ensure that where you look is exactly where the coordinates are calculated.

### 2. **Performance Through Algorithms**
Binary search reduces computational complexity from O(n) to O(log n), making real-time terrain probing feasible even at long ranges.

### 3. **Realism Through Physics**
The missile guidance system uses actual proportional navigation algorithms while respecting X-Plane's flight model, resulting in realistic missile behavior.

### 4. **Compatibility Through Standards**
All coordinate outputs follow military standards (MGRS) and aviation standards (WGS84), ensuring compatibility with real-world navigation systems.

## Development History

This system evolved from a simple coordinate display to a complete JTAC solution:

1. **Initial Implementation**: Basic terrain probing with aircraft position
2. **Camera Integration**: Fixed to use actual view position instead of aircraft position
3. **Ray Casting Refinement**: Implemented binary search for precision
4. **Missile Integration**: Added weapon system integration
5. **Physics Optimization**: Simplified guidance to prevent oscillation
6. **Final Polish**: Made plugin toggleable and added menu entry for convenience

The key breakthrough was realizing that accurate targeting requires using the camera/view position rather than the aircraft position, combined with proper binary search terrain intersection rather than linear probing.

## Contributign & Future Enhancements

### Contributing

If you want to help making this plugin more "mature" - you are welcome to do so!

### Future Enhancements

- Integration with FLIR camera systems for visual targeting
- Support for moving targets
- Multiple simultaneous target designation
- Advanced missile guidance modes (maybe homing?, etc.)
- Integration with external navigation systems

This plugin demonstrates how precise mathematical modeling combined with X-Plane's robust API can create realistic military simulation capabilities within the flight simulator environment.
