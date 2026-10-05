# First evaluation session

Allow about 15–20 minutes. Open the local viewer, keep this guide nearby, and note the recording ID and timestamp for anything confusing. The same main questions are available inside the viewer.

## Suggested recordings

These examples were selected from the copied October 3 data. They are contrasts to inspect, not labeled good/bad players.

| Recording | Why inspect it? |
|---|---|
| `2026-10-03/18-37-04` | Full recording with both cameras and all 12 loudness files. Play lead P95 is approximately 2.5° for A versus 26.3° for B. Start here for orientation. |
| `2026-10-03/16-47-21` | About 1.8 seconds total, only 0.43 seconds of Play. Tests whether short-game and missing-media displays are clear. |
| `2026-10-03/17-55-24` | Strong A/B lead contrast: approximately 2.6° versus 81.3°. Investigate what the motion and control flags actually show. |
| `2026-10-03/14-22-43` | Unusually large pooled lead P95 for A (about 767°). Treat as a data/behavior investigation. The viewer deliberately does not wrap away accumulated turns. |

## Questions to answer while using it

1. **Can you orient yourself quickly?** Confirm P1 is on the left for both cameras. Compare the cropped and full camera views. Is skeleton confidence 0.35 sensible, or does another threshold better remove noise without losing hands?
2. **Can you follow one player?** Click P1–P6 and watch signed lead and both velocities. Does a selected player's trace correspond to the joint you expect? Is the lead sign understandable? Would an absolute-lead option also help?
3. **What explains a mismatch?** Seek to motion-limited periods. Does dial speed continue while robot speed falls? Does lead accumulate? Check torque and collision details; force feedback can also move the dial.
4. **Are the plots readable?** Compare Full recording, Play stage, and the 30-second window. Are extreme spikes concealing useful smaller behavior? Would shared A/B vertical scales or a manually adjustable vertical range be more helpful?
5. **How convincing is the timing?** Locate an obvious hand-motion/robot-motion event. Check skeleton alignment. Audio timing is provisional: try small positive/negative offsets, and record whether one offset seems consistent across microphones and games. If not, we may need better source timing metadata.
6. **Where should controller guides go?** On a populated frame, show guides and try the hand-based suggestion. Drag to correct. Are the same six positions plausible across games? Are sound halos following the right hands, or are they confusing? Suggestions may refuse a camera with weak evidence; that is preferable to presenting an unsupported calibration.
7. **Is the 3D scene useful?** Orbit/zoom and toggle geometry. Can you see the bucket tool and scoring area? Which walls should default to hidden? Would matching both camera orbits help compare teams?
8. **Which statistics help you find something interesting?** Sort by duration, travel, lead P95 A/B, and limited time A/B. Do they identify worthwhile moments? Inspect the 767° example before deciding whether high lead represents behavior or corrupted measurements.
9. **What can we remove?** Which labels, overlays, and charts did you actually use? Which additional signal would resolve a question you could not answer?

## Feedback template

```text
Recording ID:
Time / range:
Team and player/joint:
What I expected:
What I saw:
Useful panel / distracting panel:
Controller-guide or audio-offset adjustment:
Next question this raised:
```

Likely next iterations, guided by your feedback: robust per-camera controller calibration, better clock metadata or measured alignment, hand-association diagnostics, selectable trace overlays, coordinated 3D cameras, event bookmarks, and more representative activity/quality metrics. None is required to explore the current prototype.
