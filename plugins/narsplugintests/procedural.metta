!(nartech.nars (Command volume=0))
!(nartech.nars (AddBeliefEternal ((((($location * $object) --> perceive) &/ (({SELF} * $location) --> (^ go))) =/> (($location * ({SELF} & $object)) --> perceive)) (1.0 0.9))))
!(nartech.nars (AddBeliefEvent ((((10 * 10) * human) --> perceive) (1.0 0.9))))
!(nartech.nars.execute 3 (AddGoalEvent (((?1 * ({SELF} & human)) --> perceive) (1.0 0.9))))
