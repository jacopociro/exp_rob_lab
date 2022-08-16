(define (problem task)
(:domain sherlockbot)
(:objects
    wp0 wp1 wp2 wp3 - waypoint
    home - home
    oracle - oracle
    sherlock - robot
)
(:init



    (robot_home home)


    (move wp0 wp1)
    (move wp0 wp2)
    (move wp0 wp3)
    (move wp1 wp0)
    (move wp1 wp2)
    (move wp1 wp3)
    (move wp2 wp0)
    (move wp2 wp1)
    (move wp2 wp3)
    (move wp3 wp0)
    (move wp3 wp1)
    (move wp3 wp2)

    (visited wp3)
    (visited wp1)
    (visited wp2)
    (visited wp0)

    (= (hypothesis_complete) 2)

)
(:goal (and
    (oracle_called oracle)
))
)
