# RealSense D435i preset notes

Recommended testing order for handheld Gaussian Splatting capture:

1. `Medium Density`
2. `High Accuracy`
3. `Default`
4. `High Density`

Use:

```bash
./scripts/set_preset.sh "Medium Density"
```

## Practical interpretation

- `Medium Density`: best starting compromise for indoor rooms and corridors.
- `High Accuracy`: lower fill, stricter confidence; often better when depth is used as geometry supervision.
- `High Density`: more filled depth but more likely to inject uncertain / noisy geometry.
- `Hand`: not meant for room-scale scanning; tuned for close hand/gesture use.

For GS-SLAM, wrong depth is usually worse than missing depth. Prefer holes over floating walls if the reconstruction backend uses depth directly.
