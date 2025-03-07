!(nartech.nars (Command volume=0))
!(nartech.nars (AddBeliefEternal (((fridge * kitchen) --> partOf) (1.0 0.9))))
!(nartech.nars (AddBeliefEternal (((sink * kitchen) --> partOf) (1.0 0.9))))
!(nartech.nars (AddBeliefEternal (((stove * kitchen) --> partOf) (1.0 0.9))))
!(nartech.nars (AddBeliefEternal (((table * kitchen) --> partOf) (1.0 0.9))))
!(nartech.nars (AddBeliefEternal ((((($location * $object) --> perceive) &/
                                   (({SELF} * $location) --> (^ go))) =/>
                                     (($location * ({SELF} & $object)) --> perceive))
                                       (1.0 0.9))))
!(nartech.nars (AddBeliefEvent ((((10 * 10) * table) --> perceive) (1.0 0.9))))
!(nartech.nars.execute (AddGoalEvent (((?1 * ({SELF} & (partOf /1 kitchen))) --> perceive) (1.0 0.9))))
