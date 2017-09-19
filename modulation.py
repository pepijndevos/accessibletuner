import pandas as pd
import numpy as np
import matplotlib.pylab as plt

data = pd.read_csv('plong.txt', skiprows=4, sep='\t', usecols=[0, 1], index_col=0,  parse_dates=True, decimal=',')

scaled = (data*100*256/5+127).round()

ADCFREQ = 1/8E-5
OMEGA   = 2*np.pi/ADCFREQ
note    = (380*OMEGA)


idx = np.arange(data.size)
scaled['sine'] = 128*np.sin(idx*note)+127

#scaled['mod'] = scaled.sine*127
scaled['mod'] = scaled.sine*scaled.Ch0
scaled['env'] = scaled['mod'].rolling(64).max()
scaled['threshold'] = 0x7fff

scaled.plot()

plt.show()
