# Provenance and license findings

## Scope and repository history

This package was prepared from `loco_mjx` branch `blueboat/currents-and-comparison-videos`, commit `05ffc1b9aa0f016253dff255ad37d7882310f36b`, for the owner-requested BlueBoat handoff. The research checkout was read only. The target repository initially contained an empty `test.txt` at main commit `6aa501f21302c4f04e7e61a8d3c8095354fb08d9`; that file is preserved.

The feature branch is `feature/ppo-deployment-package`. No changes are merged into main and no research repository changes are part of this handoff.

## Model and research-derived material

The original trained checkpoint is preserved byte-for-byte, SHA-256 `43213ea3bc2219d93acf991fdb74c02a559c1d2ed77a8af266138474eb1269e4`. It is 7,141,893 bytes, below GitHub's ordinary-Git file-size limits; no LFS pointer or external download is necessary. See [GitHub's large-file documentation](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github). The archive includes its original inert training-completion path metadata; it is not deployment configuration and is never interpreted by the deployed runtime. It was retained to preserve the exact requested checksum. A credential-pattern scan found no credential/private-key matches.

Research `setup.py` declares **MIT** and names **Nico Bohlinger**, but no repository-root LICENSE text was found at the source commit. No blanket MIT or other license grant has been invented for this target. `package.xml` uses `LicenseRef-See-Provenance` to point to this finding. Confirm the applicable copyright/license notice and the model's redistribution terms with the rights holder before presenting this as a generally licensed public package; the user's request authorizes this specific repository handoff but does not supply missing third-party license text.

`policy/checkpoints/provenance.json` lists source hashes for the extracted Gazebo bridge, MJX resistance/wave helpers, plane model and procedural hull. The hull copy only trims a trailing blank line. `dynamics.py` only changes its wave-helper import to work in this standalone directory. The ROS node, calibration/safety wrappers and portable exporter are the new integration. The observation adapter and ONNX graph reproduce the existing trained policy semantics; they do not copy the entire training framework or Flax implementation. Original actor arrays and JAX numerical fixtures preserve the training provenance.

The source's BlueBoat `SOURCES_AND_ASSETS.md` identifies the small hull OBJ as an original procedural approximation (126 vertices, 248 triangles), not manufacturer CAD. That is the only copied hull mesh.

## External dependencies and excluded assets

- ArduPilot/SITL_Models at `25bc38ed8c6c0345840159a8cbc0b02781d52f3c`: no root license or BlueBoat-specific mesh redistribution grant was established in the source investigation. **No upstream model.sdf or CAD meshes are committed here.** `prepare_world.py` operates on the operator's existing external installation and places the generated world in ignored artifacts.
- asv_wave_sim at `ca8629df4e191235753dfae92ef725d30b923364`: installed externally; its upstream LICENSE and LICENSE_THIRDPARTY govern that distribution and dependencies such as CGAL/FFTW. No plugin source/binaries or third-party patches are bundled here.
- ArduPilot and MAVROS source was consulted to document interfaces. Their source is not redistributed as part of this package.
- NumPy, SciPy, ONNX Runtime, ROS 2 and optional JAX/MuJoCo are installed separately under their own licenses. The wheel contains the ROS package/model assets, not those libraries.

Links: [source research repository](https://github.com/nico-bohlinger/loco_mjx), [SITL models](https://github.com/ArduPilot/SITL_Models), [wave simulator](https://github.com/srmainwaring/asv_wave_sim), [ArduPilot](https://github.com/ArduPilot/ardupilot), [MAVROS](https://github.com/mavlink/mavros).
