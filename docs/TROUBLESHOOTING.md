# Troubleshooting

| Symptom | Fix |
|---|---|
| `Linux user reconcile skipped` at startup | `ros_desktop` isn't running. Run `docker start ros_desktop` |
| 3D view never connects / rosbridge errors | Check `docker exec ros_desktop tail /tmp/rosbridge.log`; confirm port 9091 is published |
| Browser URL shows `0.0.0.0` | Use `localhost` or the LAN IP instead |
| Phone/laptop on hotspot can't connect | Use the host's hotspot IP (e.g. `172.20.10.x`); iPhone hotspot client isolation may block peers |
| UI change not visible | Hard reload (Cmd/Ctrl-Shift-R) |
| Python change not visible | Restart: `./scripts/start.sh` |
| Robot keeps moving after stopping a script | rospy ignores SIGTERM; stop uses `pkill -9` |
| Disk suddenly full | `./scripts/clean_gazebo_logs.sh` |
| Signup says email not configured | Set `ROS_SMTP_USER` / `ROS_SMTP_PASS` in `.env` |
| Client sim stuck "starting" | `docker logs ros_client_<user>`; `docker exec ros_client_<user> tail -50 /tmp/gazebo.log` |
| Max containers reached | `./scripts/stop.sh` or raise `MAX_CONTAINERS` |
