(define (domain sherlockbot)
    (:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions  :negative-preconditions)

    (:types
        waypoint
        robot
        hint
        arm
        order
        hypothesis
        arm_pos
        hint_pos
        home
        oracle
    )

    (:predicates
        (robot_at ?r -robot ?wp - waypoint)
        (reached ?a - arm ?h - hint)
        (add_to_ont ?h - hint)
        (checked ?hy - hypothesis)
        (inspected ?h - hint)
        (robot_home ?r - robot ?h - home)
        (oracle_called ?o - oracle)
    )
;; functions to define the distances on the map and if the hypothesis is complete
    (:functions
        (distance_wp ?a ?b - waypoint)
        (distance_hint ?a - arm_pos ?b - hint_pos)
        (hypothesis_complete ?hy - hypothesis)
        (distance_home ?a - waypoint ?b - home)
    )
;;action to move to a waypoiny
    (:durative-action go_to_waypoint
        :parameters (?r - robot ?from ?to - waypoint)
        :duration (= ?duration (distance_wp ?from ?to))
        :condition (and 
            (at start (robot_at ?r ?from))
        )
        :effect (and 
            (at start (not (robot_at ?r ?from) ))
            (at end ( robot_at ?r ?to))
        )
    )
;;action to reach the hint with the arm   
    (:durative-action reach_hint
        :parameters (?r - robot ?a - arm ?h - hint ?ap - arm_pos ?hp - hint_pos ?at - waypoint)
        :duration (= ?duration (distance_hint ?ap ?hp))
        :condition (and 
            (at start (robot_at ?r ?at))
            )
        :effect (and 
            (at end (reached ?a ?h))
        )
    )
;;action to inspect if the hint is malformed
    (:action hint_inspection
        :parameters (?h - hint ?a - arm)
        :precondition (and (reached ?a ?h))
        :effect (and (inspected ?h))
    )
;;action to add the hint to the onthology
    (:action add_to_onthology
        :parameters (?h - hint ?hy - hypothesis)
        :precondition (and (inspected ?h))
        :effect (and (add_to_ont ?h) (increase (hypothesis_complete ?hy) 1))
    )
;;action to check if the hypothesis is complete
    (:action hypothesis_check
        :parameters (?hy - hypothesis)
        :precondition (and (= (hypothesis_complete ?hy) 3) )
        :effect (and (checked ?hy))
    )
    
;;action to move home
    (:durative-action return_home
        :parameters (?r - robot ?h - home ?wp - waypoint ?hy - hypothesis)
        :duration (= ?duration (distance_home ?wp ?h))
        :condition (and 
            (at start (and (checked ?hy))
            ))
        :effect (and 
            (at end (and (robot_home ?r ?h)))
        )
    )
;;action that check the oracle
    (:action check_oracle
        :parameters (?r - robot ?hy - hypothesis ?h - home ?o - oracle)
        :precondition (and (checked ?hy) (robot_home ?r ?h))
        :effect (and (oracle_called ?o))
    )
    
    
)