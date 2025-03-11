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

!(nartech.nars (AddBeliefEvent ((((10 * 10) * table) --> perceive) (1.0 0.9))))
!(nartech.nars.execute 3 (AddGoalEvent (((?1 * ({SELF} & table)) --> perceive) (1.0 0.9))))
