(= (Store $x)
   (let $y (superpose $x) (add-atom &self $y)))
(= (NARS_I $Command $Content)
   (narsinput ($Command $Content)))
(= (NARS_IO $Command $Content)
   (narsIO ($Command $Content)))
(= (NARS_IOS $Command $Content)
   (Store (narsIO ($Command $Content))))

!(NARS_I AddBeliefEternal ((bottle --> trash) (1.0 0.9)))
!(NARS_I AddBeliefEternal ((can --> trash) (1.0 0.8)))

!(NARS_IOS (EternalQuestionMultiple 0.8) (?1 --> trash))

!(let $mode (superpose (INPUT ANSWER DERIVATION))
      (match &self
             ($mode (.: (($1 --> trash) ($f $c))))
                    (($1 --> trash) ($f $c))))
