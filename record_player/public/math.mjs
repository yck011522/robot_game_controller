/** Shared playback math. Imported by app.mjs and tested with Node's built-in test runner. */

/** Return latest sample at/before time, or -1 for pre-stream/stale data; binary search supports scrubbing. */
export function sampleIndex(times, time, maxAge = .25) {
  let low = 0; // Inclusive lower binary-search bound.
  let high = times.length; // Exclusive upper binary-search bound.
  while (low < high) {
    const middle = (low + high) >>> 1; // Search midpoint.
    if (times[middle] <= time) low = middle + 1; else high = middle;
  }
  const index = low - 1; // Most recent causal sample.
  return index >= 0 && time - times[index] <= maxAge ? index : -1;
}

/** Convert finite radians to degrees, retaining nulls to distinguish absent data from zeros. */
export function degrees(value) { return Number.isFinite(value) ? value * 180 / Math.PI : null; }

/** Signed, unwrapped dial lead in joint degrees; used by live dials and inspection plots. */
export function dialLead(dialDegrees, robotRadians) {
  return Number.isFinite(dialDegrees) && Number.isFinite(robotRadians) ? dialDegrees - degrees(robotRadians) : null;
}

/** Express signed dial velocity in robot joint degrees/s using the recorded gear ratio. */
export function jointDialSpeed(dialRadiansPerSecond, ratio) {
  return Number.isFinite(ratio) && Number.isFinite(dialRadiansPerSecond) ? degrees(dialRadiansPerSecond) * ratio : null;
}

/** Human-friendly elapsed time including tenths; timeline and chart labels call this. */
export function clock(seconds) {
  return `${Math.floor(Math.max(0, seconds) / 60)}:${(Math.max(0, seconds) % 60).toFixed(1).padStart(4, '0')}`;
}
