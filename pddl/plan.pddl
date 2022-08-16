Number of literals: 15
Constructing lookup tables: [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%]
Post filtering unreachable actions:  [10%] [20%] [30%] [40%] [50%] [60%] [70%] [80%] [90%] [100%] [110%] [120%] [130%]
Have identified that bigger values of (hypothesis_complete) are preferable
[01;34mNo analytic limits found, not considering limit effects of goal-only operators[00m
23% of the ground temporal actions in this problem are compression-safe
Initial heuristic = 7.000
b (6.000 | 5.000)b (5.000 | 15.004)b (4.000 | 16.005)b (3.000 | 16.006)b (1.000 | 21.007);;;; Solution Found
; States evaluated: 14
; Cost: 22.008
; Time 0.00
0.000: (leave_home home wp0)  [5.000]
5.001: (add_to_onthology wp0)  [1.000]
5.002: (go_to_waypoint wp0 wp1)  [5.000]
10.003: (add_to_onthology wp1)  [1.000]
10.004: (go_to_waypoint wp1 wp2)  [5.000]
15.005: (add_to_onthology wp2)  [1.000]
15.006: (hypothesis_check sherlock)  [1.000]
16.007: (return_home home wp2)  [5.000]
21.008: (check_oracle home oracle)  [1.000]
