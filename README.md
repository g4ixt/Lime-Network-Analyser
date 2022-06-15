# It is not giving correct readings, I must have introduced a bug.  Investigating.....

# Lime Network Analyser
 
 This uses the example code from Myriad RF's Lime VNA, 
https://github.com/myriadrf/pyLMS7002Soapy.git
 
It is entirely Python and plots Return Loss and Through Loss in real time using 
pyqtgraph.  Two channels can be measured with the Lime-USB (although not 
concurrently).  I have tried to retain the compatibility with Lime-mini but I 
don't have one to test it with.

I made this programme because I wanted to learn Python.  I read various online 
tutorials and official Python documents.  Learnpyqt was particularly helpful and 
easy to understand https://www.learnpyqt.com/.  I used Qt-designer to make the 
GUI and Spyder in Anaconda to write and test my code.  It is probably Python 3.8 
because that's what I have.

I added a lot of comments to the Myriad RF example programme in order to try to 
understand how it works.  These may not be correct but are my best guesses.  I 
rearranged some of the code and moved chunks around.

Although it can measure Phase, and can write the data to a file that is 
compatible with the example 'CaluclateVNA.py', I have not yet figured out how to 
plot vector measurements although I guess it can be done with a matplotlib subplot.

With Vector measurements turned off (unticked) it can plot 50 points in 8 - 25 
sec.  This seems to depend on signal power, and I think it's related to 
calibrating residual DC offset.  It runs slower if you run it in Spyder, and 
about 4 times slower with Vector turned on.

I noticed that the original function 'adjustrxgain' only adjusts PGA gain 
despite returning LNA gain as well, so I added some code that sets LNA gain by 
aiming for RSSI of 50,000 with a fixed value of PGA gain (allowing for some 
headroom).

I am a beginner with Python and object-oriented languages so my coding may be 
poor.  Helpful comments welcome.

I have made measurements at a few frequencies in the low and high bands using 
some high quality (12GHz) RF attenuators and some ancient Telonic calibrated 
mismatches and the results are within about 2dB of expected.

LICENCE

pyLMS7002Soapy and the code taken from the examples is copyright 2019 Lime 
Microsystems and provided under the Apache 2.0 License, so this code also is 
also provided under the Apache 2.0 License, which I believe is the correct way 
to proceed.
