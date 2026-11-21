# Field headings and audience direction

In this project's coordinate map, **audience** means the spectator-facing edge
of the field: the bottom edge in the diagram, at **Y = −72 inches**.
The far wall is opposite it, at **Y = +72 inches**.

```text
                 FAR WALL — Y = +72
                 Away from audience
                       90°
                        ↑
        ┌─────────────────────────────┐
        │                             │
        │     RED     │     BLUE      │
        │             │               │
  180° ←│────────── (0,0) ─────────────│→ 0°
        │          Field center       │
        │                             │
        └─────────────────────────────┘
                        ↓
                       270°
                 Toward audience
              AUDIENCE WALL — Y = −72

                    SPECTATORS
```

Headings describe where the robot's front points:

- **0°:** right, toward +X / the blue half.
- **90°:** away from the audience, toward +Y / the far wall.
- **180°:** left, toward −X / the red half.
- **270°:** toward the audience, toward −Y / the audience wall.

The origin `(0, 0)` is the field center. Coordinates are in inches and refer to
the robot center, not its front bumper. This is the project's coordinate
convention; align it with the actual field layout when placing the robot.

For the current Blue autonomous route, the first hive shooting pose is
`(12.75, 40, 270°)`. The opposite-side shooting pose is `(12.75, −46, 90°)`
after the six-inch shooting backoff adjustment.
Both face inward toward the hive from their respective sides.

See [Field locations and start presets](BIOBUZZ_FIELD_LOCATIONS.md) and
[Blue/Red preloaded-hive autonomous](BIOBUZZ_PRELOADED_HIVE_AUTO.md).
