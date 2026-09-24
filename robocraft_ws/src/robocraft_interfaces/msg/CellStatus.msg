# Cell-level status published by the RoboCraft cell interface
std_msgs/Header header
string mode                 # "serial" | "parallel" | "idle"
string[] arms
string[] grasp_states       # per arm: idle | holding:<model>/<link>[:locked] | failed:...
float64 closure_error       # parallel mode: max TCP-to-leg error [m]
float64 min_interference_margin   # smallest distance margin between arms [m]
string active_task
