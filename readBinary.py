#! python
from sys import argv
import numpy as np
import numpy
import scipy
from bitstring import BitArray
script, filename = argv
numpy.set_printoptions(threshold='nan')
f = scipy.fromfile(open(filename), dtype=scipy.uint8)
print(f)
print(f.__len__())
x=np.average(f)
print(x*10**-6)
#a= BitArray(float=0.34, length=32)
#a.bin=f
#print(a.float)
poo=str(f)
print(poo.find("0 1 0 0 0 0 0 1 0 1 0 1 1 0 1 1 0 0 1 1 1"))
