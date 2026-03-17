# AirGradient Go UI Reference Snapshot

This folder contains a copy of the current website-based AirGradient Go simulator code so firmware developers can inspect the intended UI behavior without needing access to `apps/website`.

Files:

1. `AirGradientGoDeviceSvg.vue`
   1. Main reference for on-device UI behavior
   2. Contains screen structure, menu hierarchy, settings options, snackbar text, lock behavior, and interaction flow
2. `airgradient-go-simulator.vue`
   1. Thin page wrapper around the device component
   2. Useful mainly for the external power toggle behavior

Original source paths in this repo:

1. `apps/website/components/simulators/AirGradientGoDeviceSvg.vue`
2. `apps/website/pages/airgradient-go-simulator.vue`

Usage note:

1. These files are reference snapshots, not firmware source
2. The implementation target for firmware remains the specification in `../AirGradient-Go-Complete-UI-Spec.md`
3. If the simulator changes later, this snapshot should be refreshed intentionally rather than edited independently
