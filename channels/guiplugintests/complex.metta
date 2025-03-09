!(bind! &requestid (new-state 0))
!(let $requestid (nartech.gui ((Enter a goal for NARS)
                               (AddGoalEvent (((?1 * ({SELF} & person)) --> perceive) (1.0 0.9)))))
      (change-state! &requestid $requestid))

(= (Step $t $T)
   (let $requestid (get-state &requestid)
        (nartech.gui $requestid)))
