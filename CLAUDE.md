# aurmr-2homa
Robot stack for the UR16e workcell. ROS 2 Jazzy in Docker.
- Layers: demos -> skills + end_effectors -> robot. Perception only through its interfaces. Nothing imports demos.
- Core code has no lab IPs, hostnames or calibration: those live in sites/.
- Failures raise; expected stops return a Result. Force limits are explicit per move.
- Run scripts/pre_push_check.sh before pushing. Never command real robot motion without a person confirmed at the e-stop.
- If you are told you are the dev agent or the test agent, read ~/refresh/phase3/agent_board.md first.
