(= (Step $stepTime $realTime $navstate $objects)
   (if (== $navstate BUSY)
       (navigation stack busy $objects)
       (let $persons (nartech.ros.objects.filter person $objects)
            (if (== $persons ())
                (let $unknowns (nartech.ros.objects.filter unknown $objects)
                     (let (detection unknown (coordinates $x $y)) (superpose $unknowns)
                          (nartech.ros (go (coordinates $x $y)))))
                (let (detection person (coordinates $x $y)) (superpose $persons)
                     (nartech.ros (go (coordinates $x $y))))))))
