# Boat PPO bundle

New PPO code (packet-gap handling, new safety) and the u990 model for the real boat.
Nothing here changes autopilot parameters, arms the boat or edits the live config.

    cd ~/transfer_repo && git pull
    cd ~/blueboat-hardware-deployment && source /opt/ros/humble/setup.bash && source .venv/bin/activate && source ~/blueboat_ws/install/setup.bash
    bash ~/transfer_repo/blueboat_boat_bundle/install.sh
    python3 ~/transfer_repo/blueboat_boat_bundle/boat_config.py

- `install.sh` replaces a file only if it is unchanged from the original; locally modified files are skipped (exit 3) and must be sent for review. Backups go to `~/blueboat_boat_backup_*`.
- `boat_config.py` writes `~/.config/blueboat/alberto/controller.u990.yaml` from the live `controller.yaml` (u990 model, new gap settings); the live file is untouched.

Shadow test (motors never driven):

    ros2 launch blueboat_ppo_controller controller.launch.py config:=$HOME/.config/blueboat/alberto/controller.u990.yaml mode:=shadow deployment_target:=hardware
