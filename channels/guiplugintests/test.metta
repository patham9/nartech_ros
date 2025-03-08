

!(nartech.gui ((Enter a goal for NARS) 
               (AddGoalEvent (((?1 * ({SELF} & person)) --> perceive) (1.0 0.9)))))

(= (Step $t $T)
   (match &self (nartech.gui.value $x) $x)
    )
