#MODIFIED ON SUNDAY 5:30 AM NEEDS TO BE TESTED

#!/bin/bash
#create a directory for the log files
mkdir logs
#run BPSK-Sender to generate outputBinary file (this is performed only once) MAKE SURE BPSK-Sender and BPSK_ReceiverPOLY.py have the same interpolation rate.
arrivalFilename="seatrial-"
receiver_type="resultsfilters64POLY"
mkdir logs/$receiver_type
NoF=1 #7
NoX=1 #7
python2.7 BPSK_Sender-M.py 1000
#y is the number of arrival files you are trying to process (source depths)
#j is the number of receiver depths
for ((y=1; y<=$NoF; y=y+1))
do
for((x=1; x<=$NoX; x=x+1))
do
mkdir "$arrivalFilename$y"'-'"$x"'/'"$receiver_type"
#do the below for each arrival file
for ((SNR=-25; SNR<=10; SNR=SNR+5))
  do
# input of receiver is as follow inputwave_file, outputBinaryfile
  sp="_"
  outputfile="$arrivalFilename$y"'-'"$x"'/'"$receiver_type"'/'"outputBinary$y$sp$SNR"
  inputfile="$arrivalFilename$y"'-'"$x"'/'"$arrivalFilename$y"'-'"$x"'SNR'"$SNR.wav"
  echo $outputfile
  echo $inputfile
  python2.7 BPSK_ReceiverPOLY.py $inputfile $outputfile  & >>logs/logfileBPSKRX
# wait ${!}



#wait ${!}


done #SNR
wait ${!}
done #x
done #y
wait ${!}

#y is the number of arrival files you are trying to process
for ((y=1; y<=$NoF; y=y+1))
do
for ((x=1; x<=$NoX; x=x+1))
do
#do the below for each arrival file
for ((SNR=-25; SNR<=10; SNR=SNR+5))
do
# input of receiver is as follow inputwave_file, outputBinaryfile
sp="_"
outputfile="$arrivalFilename$y"'-'"$x"'/'"$receiver_type"'/'"outputBinary$y$sp$SNR"
inputfile="$arrivalFilename$y'-'$x/$arrivalFilename$y'-'$x"'SNR'"$SNR.wav"
#Starting Correlation and preparing inputfiles
python2.7 RIN.py inputBinary >>logs/logfileRIN
python2.7 ROUT.py $outputfile >>logs/logfileROUT
done
wait ${!}
done
done
wait ${!}



#y is the number of arrival files you are trying to process
for ((y=1; y<=$NoF; y=y+1))
do
for ((x=1; x<=$NoX; x=x+1))do
#do the below for each arrival file
for ((SNR=-25; SNR<=10; SNR=SNR+5))
do
# input of receiver is as follow inputwave_file, outputBinaryfile
sp="_"
outputfile="$arrivalFilename$y"'-'"$x"'/'"$receiver_type"'/'"outputBinary$y$sp$SNR"
inputfile="$arrivalFilename$y'-'$x/$arrivalFilename$y'-'$x"'SNR'"$SNR.wav"

outputBinaryFile=$outputfile"OUT"
#Starting matlab and running Correlation
cat <<EOF | matlab -nodesktop -nosplash -nodisplay />"$arrivalFilename$y"'-'"$x"'/'"$receiver_type"'/'myresult$y$x$SNR.out &
import java.lang.System;
A=CorrelateGnuRadio('inputBinaryIN','$outputBinaryFile')
java.lang.System.exit(0);
exit
EOF
done

#if [ $((y%2)) -eq 0 ];
#then
wait ${!}
done
#fi
done


#y is the number of arrival files you are trying to process
for ((y=1; y<=$NoF; y=y+1))do
for ((x=1; x<=$NoX; x=x+1))do
#do the below for each arrival file
for ((SNR=-25; SNR<=10; SNR=SNR+5))
do
sp="_"
outputfile="$arrivalFilename$y"'-'"$x"'/'"$receiver_type"'/'"outputBinary$y$sp$SNR"
inputfile="$arrivalFilename$y'-'$x/$arrivalFilename$y'-'$x"'SNR'"$SNR.wav"

#extract the new delay
#a=$(grep 'lagDiff=\+' myresult$y$SNR.out | grep  -o "[^=|.]\d\+")
a=$(grep 'lagDiff=\+' "$arrivalFilename$y"'-'"$x"'/'"$receiver_type"'/'myresult$y$x$SNR.out | grep  -o "[-]*[0-9]\+")
echo "THIS IS THE RESULT OF CORRELATION $a"
#calculating BER with the delay
wait ${!}
python2.7 BER.py inputBinary $outputfile $a >>logs/logfile2 &
wait ${!}
echo 'This is the value of '"$SNR"
done
done
done
