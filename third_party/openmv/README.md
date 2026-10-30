# OpenMV weighted ROI line-following reference

- Repository: https://github.com/openmv/openmv
- Version: `v4.5.9`
- Pinned revision: `f1aa0247d8052d43ba0a9a07a4fbcd867c47d134`
- Source: `scripts/examples/02-Image-Processing/02-Color-Tracking/black_grayscale_line_following.py`
- Source copyright: Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
- License: MIT; exact upstream license text retained in `LICENSE`.
- Exact upstream example: `black_grayscale_line_following.py.reference`.
- Downloaded bytes and SHA256 checksums: `SOURCE.json`.

`line_core.steering_from_rois()` adapts the weighted-centroid and arctangent
bearing calculation. The example's three bands become five configurable bands.
The example's largest blob is replaced by the existing track-associated,
outlier-filtered center-line samples. Only valid bands contribute to the
denominator; insufficient bands request zero yaw. Reference offsets are
evaluated at each band before averaging. The image dimensions are dynamic,
left-positive normalized coordinates are retained, and bearing is kept in
radians before conversion to the project's bounded yaw request. The existing
heading term, acquisition gate, loss stop and UART protocol remain in use.

The upstream example is retained for reference only and is never imported.
Its original camera setup and method-based Blob API are not run on the user's
board. Only `main.py` and `line_core.py` are runtime modules; retain this license
when redistributing the adaptation. This is not evidence of a physical biped
track test, and no upstream motor or gait control is included.

The related biped implementation inspected was
[Hiwonder/TonyPi VisualPatrol.py](https://github.com/Hiwonder/TonyPi/blob/dc452eaac9991516600860f706f4b889f3305d1d/Functions/VisualPatrol.py).
It also uses ROI weighting. No repository-level license was found at that
revision, so no TonyPi source or assets are copied into this repository.
