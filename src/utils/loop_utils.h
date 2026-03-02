#ifndef LOOP_UTILS_H
#define LOOP_UTILS_H

/** Loop timing helpers: compute dt and smoothed loop frequency. */
class LoopUtils {
public:
    /**
     * Compute loop period and update smoothed frequency.
     * Call each loop with current micros(); updates lastLoopTimeUs and loopFrequencyHz in place.
     * @param nowUs        Current time from micros()
     * @param lastLoopTimeUs  Last loop timestamp (updated to nowUs on return)
     * @param loopFrequencyHz Smoothed loop frequency in Hz (updated with 0.995/0.005 filter)
     * @return Loop period in seconds (0.001f if no previous sample)
     */
    static float computeLoopDtAndHz(
        unsigned long nowUs,
        unsigned long& lastLoopTimeUs,
        float& loopFrequencyHz
    ) {
        float loopDtSec = 0.001f;
        if (lastLoopTimeUs > 0) {
            unsigned long periodUs = nowUs - lastLoopTimeUs;
            if (periodUs > 0) {
                loopDtSec = (float)periodUs / 1000000.0f;
                float instantHz = 1000000.0f / (float)periodUs;
                loopFrequencyHz = 0.995f * loopFrequencyHz + 0.005f * instantHz;
            }
        }
        lastLoopTimeUs = nowUs;
        return loopDtSec;
    }
};

#endif
