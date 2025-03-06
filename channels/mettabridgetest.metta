!(nartech.nars AddBeliefEternal ((bottle --> trash) (1.0 0.9)))
!(nartech.nars AddBeliefEternal ((can --> trash) (1.0 0.8)))
!(nartech.nars AddBeliefEternal ((garbage <-> trash) (1.0 0.8)))
!(nartech.nars AddBeliefEternal ((plasticbag --> garbage) (1.0 0.9)))

!(nartech.nars.space &self (EternalQuestionAboveExpectation 0.8) (?1 --> trash))

!(let $mode (superpose (INPUT ANSWER DERIVATION))
      (match &self
             ($mode (.: (($1 --> trash) ($f $c))))
                    (if (== (get-metatype $1) Symbol)
                        (($1 --> trash) ($f $c))
                        Empty)))
