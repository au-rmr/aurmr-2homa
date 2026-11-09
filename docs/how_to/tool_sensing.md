# How a tool's sensing reaches the arm

There are two paths. They don't depend on each other.

## 1. During a move: stop signals (the safety path)
- The tool's driver publishes a `std_msgs/Bool` topic, for example `/cap_contact` or `/vacuum_seal`.
- `tool.yaml` lists it under `stop_signals` with a short name:
  ```yaml
  stop_signals:
    contact: /cap_contact
    seal: /vacuum_seal
  ```
- A move asks for it by name: `robot.move_straight(..., stop=Stop(contact=True))`.
- motion_server subscribes to the topic itself and stops the arm when it reads true. It does that at its control rate, on the control host. Nothing in the demo's process is involved, so it also stops the arm if the demo hangs.
- If the topic is older than 100 ms when the move starts, the move is refused. If it goes quiet during the move, the move aborts (`SensorStale`).
- A move that a signal can end is capped at 0.03 m/s.
- A signal that is already true when the move starts ends the move at once.

A new signal needs no new robot code: a Bool topic, a `stop_signals` entry, and the name in `Stop(...)`.

## 2. Between moves: the tool's client (decision logic)
- `tool.yaml` names the client class and the capabilities it provides:
  ```yaml
  capabilities: [contact, suction]
  client: capcup_driver.client:CapCup
  ```
- The demo gets the client from the robot's tool, so the tool is named only once:
  ```python
  from aurmr_skills import load_tool
  with Robot.connect("capcup", 50, workcell) as robot, \
       load_tool(robot, needs=["contact", "suction"]) as cup:
      if cup.sealed(): ...
  ```
- `load_tool` refuses at startup, before anything moves, if:
  - the tool lacks a capability you need
  - a capability name isn't defined in `aurmr_skills/capabilities.py`
  - the client lacks the methods of a capability it lists
- Readings must be fresh: the capcup client raises `ToolError` for any reading older than 0.2 s rather than return an old value.
- A passive tool (no sensors) leaves out `client:` and gets a `NoClient`.

## Adding a tool
1. URDF and `tool.yaml` in `end_effectors/<tool>/`: name, `tcp_link`, mass, centre of gravity, `capabilities`, `stop_signals`, `client`.
2. The driver and client in `end_effectors/<tool>_driver/`. Each Bool it publishes for stopping goes in `stop_signals`. The client provides every listed capability's methods, as defined in `skills/aurmr_skills/capabilities.py`. Add a capability there if it's new.
3. A test like `tests/skills/test_load_tool.py::test_capcup_yaml_and_client_agree` for the new pair.

## Later: streaming (S7, not built)
Sensing feeds a planner whose commands go through the streaming gate on the control host. See the S0 design note.
