# experimental KiwiSDR CW decoder
Contained in these folders are utilities and support libraries required to set
up a raspberry pi (3B / 4B) as a CW decoding station. Though tolerant of a 
range of timing variations it is assumed here the recieved
CW is machine keyed.  The algorithm has not designed or tested with manually keyed
CW.

The following basic instructions are illustrative and assume connecting with
the Wellington based KiwiwSDR at  123.255.47.67 port 8073. Small adjustments 
to the frequency may be required when using other KiwiSDR's.

The python script available from
https://github.com/jks-prv/kiwiclient
extracts the I/Q stream that is then passed to KiwiSDR_dcdr

The latter performs the necessary signal conditioning, filtering, demodulation
and resampling (oversampling) of the binary CW. The result of this is sent via
UDP to port 9222 to be decoded by a third utility tc_mrsdcdr.

(There are two further options not explained here ... yet to be detailed here
that allow for graphical realtime display of the spectral signal, as well as the
signal enevelope and CW binary.)

To set up a CW decoder on a headless raspberry pi try the following

Open two terminal windows on the (headless) pi and set the path in each to the cloned
experimental_kiwidcdr directory. we assume the pi is internet connected.

In one window type ./tc_mrsdcdr <ret>

This is where the decoded morse message characters will appear upon receipt.

In the second window set up a shell variable FREQ  to reflect the expected say
7.202MHz say on which the signal will be received This is passed into the kiwi_nc_py
utility. The second utility includes a series of parameters. -r sets the gain
(db) of the narrow band resonant filters, -t sets up a discriminant threshold
(db) margin above noise for extracting the CW binary. The decoder in any case
adjusts to fluctuating noise levels by virtue of its use of NB filters, so the
idea is to raise this value only in so far as is necessary to minimise the
output of spurious text. -b is a balancing coefficient betweena pair of resonant
filters, one which is tuned to the CW frequency and the other 120Hz offset (this
is part of an FSK demodulation facility). -d is a dampaning factor that controls
the Q of the resonant filters. 1.0 is the default value. It may be adjusted
upwards to reduce the Q and thus improve responsiveness to higher word rates. As
one reduces this to say < 1.0, say .8 or .7 the balance -b may need to be
adjusted. This is best doen when a screen is attached to the pi and the spectral
output is vied graphically. The instructions for this will be added later. -w is
the WPM rate... here set to 25WPM, the -c 1 indicates that CW and not FSK is
implied. This disables automatic centre tuning of the lower of the pair of
resonant filter banks.


The KiwiSDR is in fact offset tuned. The CW is then in the USB. The .8KHz is to
allow for the 800Hz Local Oscillator that can be adjusted remotely (UDP
instruction on port 9223. Documented elsewhere.) the 600Hz offset is where the
CW signal tone is assumed to be optimally set. (A second NB filter is ~120Hz
below this, i.e. 480Hz and used as a level reference in the receipt of CW, or in
the case of FSK, the 'space' frequency.) 


FREQ=7202 FREQ=`echo "$FREQ-0.800-0.600 -.035" | bc -l` 

(Note: The .035 adjustment is a 35Hz adjustment specific to using the Wellington KiwiSDR.)


Then type:
./kiwi_nc.py -s 123.255.47.67 -p 8073 -f $FREQ -m iq | ./KiwiSDR_dcdr -r 8 -t 10
-b 1.0 -d 1.2 -w 25 -c 1



Once this is ready to go do arrange a time with me by email
mark.at.tcode@protonmail.com 
and I will set up a series of test transmissions from my Auckland
residential location. In my experience the above settings provide reliable,
repeatable decoding of CW during especially daylight and early evening hours.
 Auckland to Wellington is a distance of some 500km.
Under adverse conditions adjustments may need to be made to parameters.

Not documented here is the fact that KiwiSDR_dcdr outputs signal and CW binary
for graphical display and spectral diagnostic display purposes. Instructions for
this yet to be uploaded. Two real-time scope displays can be optionally used to
fine tune receiver performance under adverse reception conditions.

7 September 2023

