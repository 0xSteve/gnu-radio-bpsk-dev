#!/bin/bash
for ((c=1; c<=5; c++))
 do
#rm goff_random_20pc_5km_E_no$c/outputBinary*
  rm -r goff_random_200pc_5km_E_no$c/resultsfilters32POLY
rm -r goff_random_20pc_5km_E_no$c/resultsfilters64POLY

# rm -r logs
done
