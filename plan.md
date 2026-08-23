# Task Execution Plan: Install and Verify Isaac ROS (ESS + CuVSLAM + Nvblox) in Docker

## 1. Requirement Analysis
- **Core Goal**: Install NVIDIA Isaac ROS packages (`isaac_ros_ess`, `isaac_ros_visual_slam` [CuVSLAM], and `isaac_ros_nvblox`) and verify their functionality inside the Docker container defined by `Dockerfile`.
- **Target Hardware & Environment**: Jetson Orin Nano (L4T r36.5.0 / JetPack 6.x), ROS 2 Humble.
- **Rules Compliance**:
  - Rule 0: Master Protocol (`using-agents`)
  - Rule 1: Python commands follow `code-execution` logic
  - Rule 2: Compile/run project after updates, follow `systematic-debugging` if issues arise
  - Rule 4: Target platform Jetson Orin Nano
  - Rule 5: Report update process & ask user before executing `docker commit`

## 2. Task Checklist
- [ ] **Phase 1: Environment & Repository Preparation**
    - [ ] Inspect existing `Dockerfile` and ROS 2 setup.
    - [ ] Test NVIDIA Isaac ROS APT repository configuration and package availability inside Docker.
    - [ ] Determine best installation path (APT packages vs building source repos in workspace).
- [ ] **Phase 2: Dockerfile & Container Update**
    - [ ] Update `Dockerfile` to include required dependencies and Isaac ROS components (ESS, CuVSLAM, Nvblox).
    - [ ] Build the updated Docker image using `docker build`.
- [ ] **Phase 3: Package Installation & Workspace Build Verification**
    - [ ] Run container with GPU support (`--runtime nvidia`).
    - [ ] Verify `isaac_ros_visual_slam`, `isaac_ros_ess`, and `isaac_ros_nvblox` ROS 2 nodes / libraries can be loaded and recognized by ROS 2 (`ros2 pkg list`, `ros2 component types`, or node check).
- [ ] **Phase 4: Verification Loop & Final Reporting**
    - [ ] Perform runtime execution test for Visual SLAM, ESS, and Nvblox nodes inside the Docker container.
    - [ ] Clean up temporary workspace artifacts (`plan.md` upon completion).
    - [ ] Report update process to user and explicitly ask before running `docker commit` (Rule 5).

## 3. Risks and Countermeasures
- **NVIDIA GPU Acceleration missing in container** -> Ensure container is executed with `--runtime nvidia` and CUDA environment variables are set.
- **APT package vs Source build mismatch** -> Verify package sources, dependencies via `rosdep` or explicit APT installs.
