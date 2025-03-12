!(nartech.gui (1 (Enter a goal for NARS)
                 (AddGoalEvent (((?1 * ({SELF} & (partOf /1 kitchen))) --> perceive) (1.0 0.9)))))
(= (narsgoal) (nartech.gui 1))

!(nartech.nars (Command volume=0))
!(nartech.nars (AddBeliefEternal (((fridge * kitchen) --> partOf) (1.0 0.9))))
!(nartech.nars (AddBeliefEternal (((sink * kitchen) --> partOf) (1.0 0.9))))
!(nartech.nars (AddBeliefEternal (((stove * kitchen) --> partOf) (1.0 0.9))))
!(nartech.nars (AddBeliefEternal (((table * kitchen) --> partOf) (1.0 0.9))))
!(nartech.nars (AddBeliefEternal (((chair * kitchen) --> partOf) (1.0 0.9))))
!(nartech.nars (AddBeliefEternal ((((($location * $object) --> perceive) &/
                                   (({SELF} * $location) --> (^ go))) =/>
                                     (($location * ({SELF} & $object)) --> perceive))
                                       (1.0 0.9))))

(= (Step $stepTime $realTime)
   (if (== (nartech.ros.navigation) BUSY)
       (navigation stack busy)
       (let* (($objects  (nartech.ros.objects))
              ($unknowns (nartech.ros.objects.filter unknown $objects)))
             (superpose ((nartech.nars.perceive $objects)
                         (if (or (== () (narsgoal))
                                 (== () (nartech.nars.execute 3 (narsgoal)))) ;3 input circles
                             (let (detection unknown $coordinates) (superpose $unknowns)
                                  (nartech.ros (go $coordinates)))
                             Empty)
                         (nartech.nars (Cycles 20)))))))
