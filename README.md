# Lime_network_analyser
 
 This uses the example code from Myriad RF's Lime VNA, 
https://github.com/myriadrf/pyLMS7002Soapy.git
 
 It is entirely Python and plots Return Loss and Through Loss in real time using 
pyqtgraph.  Two channels can be measured with the Lime-USB (although not 
concurrently).  I have tried to retain the compatibility with Lime-mini but I 
don't have one to test it with and I have not yet done a 'thought experiment' to 
see if it might work properly.

I made this programme because I wanted to learn Python.  I read various online 
tutorials and official Python documents.  Learnpyqt was particularly helpful and 
easy to understand https://www.learnpyqt.com/.  I used Qt-designer to make the 
GUI and Spyder in Anaconda to write and test my code.  It is probably Python 
3.8 because that's what I have.

I added a lot of comments to the Myriad RF example programme in order to try to 
understand how it works.  These may not be correct but are my best guesses.  I 
rearranged some of the code and moved bits around but I don't think I broke it.

Although it can measure Phase, and can write the data to a file that is 
compatible with the example 'CaluclateVNA.py', I have not yet figured out how to 
plot vector measurements.

With Vector measurements turned off (unticked) it can plot 50 points in 8 - 25 
sec.  This seems to depend on signal power, with stronger signals being a lot 
faster.  I haven't figured out why.  It runs slower if you run it in Spyder, and 
about 4 times slower with Vector turned on.

I am a beginner with Python so my coding may be poor.  Helpful comments welcome.
