!(nartech.gui (1 (Enter a goal for NARS)
                 (AddGoalEvent (((?1 * ({SELF} & person)) --> perceive) (1.0 0.9)))))
(= (narsgoal) (nartech.gui 1)) ;first request

!(nartech.nars (Command volume=0))
!(nartech.nars (AddBeliefEternal ((((($location * $object) --> perceive) &/
                                   (({SELF} * $location) --> (^ go))) =/>
                                     (($location * ({SELF} & $object)) --> perceive))
                                       (1.0 0.9))))

(= (Step $stepTime $realTime $navstate $objects)
   (if (== $navstate BUSY)
       (navigation stack busy $objects)
       (superpose ((nartech.nars.perceive $objects)
                   (if (or (== () (narsgoal))
                           (== () (nartech.nars.execute (narsgoal))))
                       (let $unknowns (nartech.ros.objects.filter unknown $objects)
                            (let (detection unknown (coordinates $x $y)) (superpose $unknowns)
                                 (nartech.ros (go (coordinates $x $y)))))
                       Empty)
                   (nartech.nars (Cycles 10))))))
