#ifndef ANGLE_UTILS_H
#define ANGLE_UTILS_H

/** Angle helpers: wrap and normalize for shortest-path and [0, 360) range. */
class AngleUtils {
public:
    /** Wrap angle to (-180, 180] for shortest-path error. */
    static float wrap180(float deg) {
        while (deg > 180.0f) deg -= 360.0f;
        while (deg < -180.0f) deg += 360.0f;
        return deg;
    }

    /** Normalize angle to [0, 360). */
    static float normalize360(float deg) {
        while (deg >= 360.0f) deg -= 360.0f;
        while (deg < 0.0f) deg += 360.0f;
        return deg;
    }
};

#endif
