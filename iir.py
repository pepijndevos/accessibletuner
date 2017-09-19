import matplotlib
matplotlib.use("TkAgg")

from scipy import signal
import numpy as np
import matplotlib.pyplot as plt

Fs = 10000.0

b, a = signal.iirfilter(2, 10/(2*Fs), rp=1, rs=30, btype='lowpass',
                        analog=False, ftype='ellip')

sb = np.round((b*(2**16)))
sa = np.round((a*(2**16)))
print(*sb)
print("-"*80)
print(*sa)

def plot_transfer(b, a):
    w, h = signal.freqz(b, a, 10000)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(w*Fs, 20 * np.log10(abs(h)))
    ax.set_xscale('log')
    ax.set_title('Chebyshev lowpass frequency response')
    ax.set_xlabel('Frequency [Hz]')
    ax.set_ylabel('Amplitude [dB]')
    #ax.axis((10, 1000, -100, 10))
    ax.grid(which='both', axis='both')
    plt.show()

plot_transfer(sb, sa)
