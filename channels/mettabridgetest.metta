!(narsinput (AddBeliefEvent ((bottle --> trash) (1.0 0.9))))
!(let $x (narsinput (AddBeliefEvent ((can --> trash) (1.0 0.9))))
      (add-atom &self (belief $x)))

!(match &self (belief $x) $x)
