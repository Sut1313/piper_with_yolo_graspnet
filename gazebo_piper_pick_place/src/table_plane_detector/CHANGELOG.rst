^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package table_plane_detector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2026-10-18)
------------------
* Add constrained RANSAC tabletop detection and SVD plane refinement.
* Add TF-based normal constraints, quality gates and temporal filtering.
* Add a custom ``TablePlane`` message, diagnostics, inlier cloud and marker outputs.
* Add D435i and generic point-cloud launch configurations.
* Add a deterministic Gazebo Harmonic scene and automatic acceptance script.
* Add plane-math, synthetic-cloud and temporal-filter tests.
* Validate tabletop-only tracking with a physical RealSense D435i.
