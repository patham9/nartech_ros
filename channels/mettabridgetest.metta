!(NARS_I AddBeliefEternal ((bottle --> trash) (1.0 0.9)))
!(NARS_I AddBeliefEternal ((can --> trash) (1.0 0.8)))

!(NARS_I AddBeliefEternal ((aba --> trash) (1.0 0.8)))
!(NARS_I AddBeliefEternal ((aca --> trash) (1.0 0.8)))
!(NARS_I AddBeliefEternal ((ada --> trash) (1.0 0.8)))
!(NARS_I AddBeliefEternal ((afa --> trash) (1.0 0.8)))
!(NARS_I AddBeliefEternal ((aea --> trash) (1.0 0.8)))

!(NARS_IOS &self (EternalQuestionMultiple 0.9) (?1 --> trash))

!(let $mode (superpose (INPUT ANSWER DERIVATION))
      (match &self
             ($mode (.: (($1 --> trash) ($f $c))))
                    (($1 --> trash) ($f $c))))
