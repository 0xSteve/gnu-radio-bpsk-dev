#!/bin/bash
for ((c=1; c<=7; c++))
 do
for ((d=1; d<=7; d++))
 do
	#rm goff_random_20pc_5km_E_no$c/outputBinary*
	  #rm -r seatrial-$c-$d/resultsfilters32POLY
	rm -r seatrial-$c-$d/resultsfilters64POLY
done
done
