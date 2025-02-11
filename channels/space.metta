!(bind! &plan (new-state (left (right (done ())))))

(= (progress)
   (let ($head $rest) (get-state &plan)
        (If (not (== $rest ()))
            (superpose ((case (change-state! &plan $rest) ())
                        (bridgeinput $head)))
            ())))

(= (Step $stepTime $realTime $navstate)
   (if (== $navstate BUSY)
       (busy (get-state &plan))
       (superpose ((progress)))))
