#!/bin/bash
for ((c=1; c<=100; c++))
 do
#mkdir goff_random_40pc_5km_E_no$c/filters62POLY
# mv goff_random_40pc_5km_E_no$c/output* goff_random_40pc_5km_E_no$c/filters62POLY
# mv goff_random_40pc_5km_E_no$c/BER goff_random_40pc_5km_E_no$c/filters62Poly
 mv goff_random_80pc_5km_E_no$c/filters62Poly goff_random_80pc_5km_E_no$c/filters64Poly


done
mkdir logs/resultsfilters64POLY
mv logs/myresult* logs/resultsfilters64POLY