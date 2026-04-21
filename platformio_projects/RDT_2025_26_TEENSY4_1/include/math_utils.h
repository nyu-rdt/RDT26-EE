#pragma once

static inline float slew(float cur, float tgt, float maxDelta) {
    float d = tgt - cur;
    return (d > maxDelta) ? cur + maxDelta : (d < -maxDelta) ? cur - maxDelta : tgt;
}
