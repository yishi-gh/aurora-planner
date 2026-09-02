# API and Compatibility Policy

This document freezes the current `0.1.0` development baseline so future
changes can be reviewed against an explicit contract. It is an engineering
policy, not a promise that the current development line is ABI-stable.

## Version Scope

- All AURORA packages currently use version `0.1.0` and are released as one workspace baseline.
- The public ROS 2 schema is defined by `aurora_msgs/msg/*.msg` and `aurora_msgs/srv/*.srv`.
- Public C++ headers under each package's `include/` are source-level APIs. ABI stability is not promised before a stable `1.0` line.
- The ROS 2 distribution pair is part of the compatibility target: Ubuntu 22.04 + Humble and Ubuntu 24.04 + Jazzy. A compiled workspace must not mix distributions.

## Change Rules

- Adding, removing, reordering, or changing a ROS message/service field is a schema change. It requires a versioned changelog entry, regenerated interfaces, updated rosbag2 fixtures, and a Humble/Jazzy build check before release.
- Removing or changing the meaning of a topic, service, enum value, timestamp, frame, validation state, or safety action is breaking and requires a new compatibility decision record.
- New parameters may be added with documented units, defaults, ranges, and safety behavior. Changing a default or safety interpretation requires an explicit changelog entry and regression coverage.
- Core algorithm changes must preserve three-dimensional flight semantics, absolute trajectory time, publication safety gates, and emergency-stop behavior unless a user-confirmed decision record changes them.
- A failed, stale, or unvalidated result must never be made wire-compatible by silently publishing it as an executable trajectory.

## Validation Before Release

1. Update `CHANGELOG.md` and the relevant decision record.
2. Rebuild `aurora_msgs` and all dependent packages in both target ROS distributions.
3. Re-run the source SPDX check, full test suite, failure-path tests, and benchmark comparison.
4. Record topic/service schema and parameter changes in the release checklist before creating a tag.

The current source schema and dependency inventory are descriptive artifacts;
they do not replace a clean Humble/Jazzy build or an external flight-controller
acceptance test.
