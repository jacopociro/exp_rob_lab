(define (domain sherlockbot)
    (:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions  :negative-preconditions :duration-inequalities)

    (:types
        waypoint
        home
        oracle
        robot
    )

    (:predicates
        (robot_at ?wp - waypoint)
        (reached ?wp - waypoint)
        (checked)
        (robot_home ?h - home)
        (oracle_called ?o - oracle)
        (move ?from ?to - waypoint)
        (visited ?wp - waypoint)
    )
;; functions to define the distances on the map and if the hypothesis is complete
    (:functions
        (hypothesis_complete)
    )
;;action to move to a waypoiny
    (:durative-action go_to_waypoint
        :parameters (?from - waypoint ?to - waypoint)
        :duration (= ?duration 5)
        :condition (and 
            (at start (robot_at ?from))
            (at start (move ?from ?to))
            (at start (visited ?to))
        )
        :effect (and 
            (at start (not (robot_at ?from) ))
            (at end ( robot_at ?to))
            (at end (reached ?to))
            (at end (not(visited ?to)))
        )
    )
;;action to reach the hint with the arm   


;;action to add the hint to the onthology
    (:durative-action add_to_onthology
        :parameters (?wp - waypoint)
        :duration (= ?duration 1)
        :condition (and 
            (at start(reached ?wp)) 
            (at start(robot_at ?wp))
            )
        :effect (and (at start (increase (hypothesis_complete) 1))
                (at start(not(reached ?wp)))
                )
    )
;;action to check if the hypothesis is complete
    (:durative-action hypothesis_check
        :parameters (?r - robot)
        :duration (= ?duration 1)
        :condition (and (at start(> (hypothesis_complete) 2) ))
        :effect (and (at end(checked)))
    )
    
;;action to move home
    (:durative-action return_home
        :parameters (?h - home ?wp - waypoint)
        :duration (= ?duration 5)
        :condition (and 
            (at start (checked))
            (at start (robot_at ?wp)
            ))
        :effect (and 
            (at end (and (robot_home ?h) (not(robot_at ?wp))))
            
        )
    )
;; action to move from home
    (:durative-action leave_home
        :parameters (?h - home ?wp - waypoint)
        :duration (= ?duration 5)
        :condition (and 
            (at start (and (robot_home ?h))
            ))
        :effect (and 
            (at end (robot_at ?wp))
            (at end (reached ?wp))
            (at end (not(robot_home ?h)))
            (at end (not(visited ?wp)))
            
        )
    )
;;action that check the oracle
    (:durative-action check_oracle
        :parameters (?h - home ?o - oracle)
        :duration (= ?duration 1)
        :condition (and(at start  (checked))  (at start (robot_home ?h)))
        :effect (and (at end (oracle_called ?o)))
    )   
)