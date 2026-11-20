# The collision monitor's verdict for the arm's current and predicted states (S7, streaming).
# motion_server's stream session requires a fresh clear status for its scene and stops otherwise.
std_msgs/Header header       # stamp = when this check ran
bool clear                   # true: no collision now or within horizon_s, inputs fresh
string scene                 # the scene checked (the monitor's scene name; a session names the scene it needs)
string reason                # empty if clear, else what collides or why there is no verdict
float64 horizon_s            # how far ahead the states were checked
float64 joint_state_age_s    # age of the joint state the check used (receive time)
