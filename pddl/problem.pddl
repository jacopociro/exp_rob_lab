(define (problem sherlockbot_problem) (:domain sherlockbot)
(:objects 
    wp0, wp1, wp2, wp3 - waypoint
    h0, h1, h2, h3, h4, h5 - hint
    hp0, hp1, hp2, hp3 - hint_pos
    sherlock - robot
    ap - arm_pos
    home - home
    oracle - oracle
)

(:init
    (= (distance_hint ap hp0) 0.5)
    (= (distance_hint ap hp1) 0.5)
    (= (distance_hint ap hp2) 0.5)
    (= (distance_hint ap hp3) 0.5)

    (= (distance_wp wp0 wp1) 4.24)
    (= (distance_wp wp1 wp2) 4.24)
    (= (distance_wp wp2 wp3) 4.24)
    (= (distance_wp wp3 wp0) 4.24)
    (= (distance_wp wp1 wp3) 6)
    (= (distance_wp wp0 wp2) 6)

    (robot_home sherlock home)

    (= (distance_home wp0 home ) 3)
    (= (distance_home wp1 home) 3)
    (= (distance_home wp2 home) 3)
    (= (distance_home wp3 home) 3)



)

(:goal (and
    ;todo: put the goal condition here
    (oracle_called oracle)
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
