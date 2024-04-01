/****************************************************************************************/
//																						*/
//  program take I/Q input from kiwi_nc.py, e.g.										*/
// ./kiwiclient/kiwi_nc.py -s 123.255.61.168 -p 8073 -f 7010.300 -m iq | ./upsample 	*/
// ./kiwiclient/kiwi_nc.py -s 123.255.61.168 -p 8073 -f 7009.320 -m iq | ./KiwiSDR_dcdr -r 3 -t 6 -w 10 | nc 192.168.1.7 9333	
// gcc -lm -lpthread -ffast-math KiwiSDR_dcdr.c  -o KiwiSDR_dcdr														*/
/****************************************************************************************/
//cc KiwiSDR_dcdr.c -o KiwiSDR_dcdr

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <pthread.h>
#include <wordexp.h>
#include <unistd.h>

#include "getopt.h"


#define GAUSS_TAPS 	(200) //size of gaussian filter

#define	DIFFERENTIAL_TAPS	(31)	// size of the differential filter
//#define	DIFFERENTIAL_TAPS_30	(61)	// size of the differential filter

#define DEFAULT_RESPONSE (.0003) // DC-removam response parameter

#define MAXBUFLEN 1024 /// string processing


#define TRUE 		(1) 
#define FALSE 		(0) 


#define FRST_FLTR 	(8) //first filter 7th order: order Number of terms 1 more than the order
#define denomOne	(8254409552.019)
#define consta 		(54867539664.451/denomOne) 
#define constb 		(156373549966.423/denomOne) 
#define constc 		(247701763446.204/denomOne) 
#define constd 		(235522413677.033/denomOne) 
#define conste 		(134421433690.466/denomOne) 
#define constf 		(42639309193.356/denomOne) 
#define constg 		(5798945459.710/denomOne) 

#define denomHP		(1.193)
#define a 			(7.930/denomHP) 
#define b 			(22.602/denomHP) 
#define c 			(35.802/denomHP) 
#define d 			(34.042/denomHP) 
#define e 			(19.429/denomHP) 
#define f 			(6.163/denomHP) 
#define g 			(0.838/denomHP) 

#define MARK_freq		(0.610) // 0.64
#define SPACE_freq		(0.485)

#define LO				(800) // local oscillator set to 800Hz


#define HDAMP	0.007 //.0068
#define LDAMP	0.007

// upsampling to 48kHz from 11025Hz
//#define UPSAMP (74)
//#define DWNSAMP (17)
//#define HALFPOINT (175)
// upsampling to 48kHz from 12kHz
#define UPSAMP 		(4)
#define DWNSAMP 	(1)
#define HALFPOINT 	(21)


static float mixer, t = 0;



struct control {
	int		In_flg;
	float	increment;
	int		testsignal;
	float	signalVnoise;
	int		cwFlg; // TRUE for CW decoding 
	float	localosc;
	float	localoscsv;
	int		resonantFilterFlag;
	float 	attenuate;
	float 	balance;
	float 	damper;
	int		resampleFlag;
	float	WordsPerMinute;
	float	threshold; //top threshold
	float	response;
	float	ScaleSpectrum;
};

int	rc;
struct control	parameters;
pthread_t 	controls_thread;


// gaussian weighted sinc function e^{-(x/pi)^2} * sin(pi*x)/(pi*x)  
//awk 'BEGIN{for (i=1; i <= 5*4; i++) print i,exp(-(i/4/3.1415926)^2) * sin(3.1415926*i/4)/(3.14159*i/4)}'
//awk '{i++; printf "%f, ",$2; if (i==10) {i=0; printf "\n"}}' /tmp/sinc
// assuming 12kHz stream
float sinc[] = {
0.000000, 0.004817, 0.009090, 0.008495, 0.000000, -0.014438, -0.026287, -0.023750, 0.000000, 0.038039, 
0.067591, 0.059894, 0.000000, -0.094306, -0.168948, -0.153699, 0.000000, 0.283480, 0.620697, 0.894634, 
1.000000, 0.894634, 0.620697, 0.283480, 0.000000, -0.153699, -0.168948, -0.094306, 0.000000, 0.059894, 
0.067591, 0.038039, 0.000000, -0.023750, -0.026287, -0.014438, 0.000000, 0.008495, 0.009090, 0.004817, 
0.000000
};

// assumed an 11025Hz stream
/*float sinc[] = {
0.000000, 0.000218, 0.000443, 0.000674, 0.000912, 0.001156, 0.001405, 0.001659, 0.001919, 0.002182, 
0.002450, 0.002721, 0.002995, 0.003272, 0.003551, 0.003832, 0.004113, 0.004395, 0.004677, 0.004958, 
0.005237, 0.005515, 0.005789, 0.006060, 0.006327, 0.006589, 0.006846, 0.007096, 0.007340, 0.007575, 
0.007802, 0.008020, 0.008228, 0.008425, 0.008610, 0.008784, 0.008944, 0.009090, 0.009222, 0.009339, 
0.009440, 0.009524, 0.009591, 0.009639, 0.009669, 0.009680, 0.009671, 0.009642, 0.009591, 0.009519, 
0.009425, 0.009308, 0.009169, 0.009006, 0.008819, 0.008609, 0.008374, 0.008115, 0.007831, 0.007522, 
0.007189, 0.006831, 0.006447, 0.006039, 0.005607, 0.005150, 0.004669, 0.004164, 0.003635, 0.003084, 
0.002510, 0.001914, 0.001296, 0.000658, 0.000000, -0.000677, -0.001373, -0.002086, -0.002815, -0.003559, 
-0.004318, -0.005089, -0.005872, -0.006666, -0.007468, -0.008278, -0.009095, -0.009916, -0.010739, -0.011564, 
-0.012389, -0.013212, -0.014031, -0.014844, -0.015650, -0.016446, -0.017231, -0.018002, -0.018759, -0.019498, 
-0.020218, -0.020917, -0.021592, -0.022242, -0.022865, -0.023458, -0.024020, -0.024548, -0.025041, -0.025496, 
-0.025912, -0.026287, -0.026619, -0.026905, -0.027145, -0.027336, -0.027477, -0.027565, -0.027601, -0.027581, 
-0.027505, -0.027371, -0.027178, -0.026925, -0.026611, -0.026235, -0.025795, -0.025292, -0.024724, -0.024091, 
-0.023393, -0.022629, -0.021799, -0.020904, -0.019943, -0.018916, -0.017824, -0.016668, -0.015448, -0.014165, 
-0.012820, -0.011414, -0.009949, -0.008426, -0.006846, -0.005211, -0.003524, -0.001786, 0.000000, 0.001832, 
0.003708, 0.005625, 0.007579, 0.009569, 0.011590, 0.013640, 0.015714, 0.017810, 0.019924, 0.022052, 
0.024190, 0.026334, 0.028480, 0.030623, 0.032759, 0.034883, 0.036992, 0.039080, 0.041143, 0.043175, 
0.045173, 0.047131, 0.049044, 0.050908, 0.052716, 0.054465, 0.056150, 0.057764, 0.059305, 0.060765, 
0.062142, 0.063429, 0.064622, 0.065716, 0.066708, 0.067591, 0.068363, 0.069017, 0.069551, 0.069961, 
0.070242, 0.070390, 0.070403, 0.070277, 0.070009, 0.069595, 0.069034, 0.068322, 0.067457, 0.066437, 
0.065262, 0.063928, 0.062435, 0.060781, 0.058967, 0.056992, 0.054856, 0.052559, 0.050101, 0.047484, 
0.044709, 0.041778, 0.038691, 0.035453, 0.032065, 0.028530, 0.024851, 0.021033, 0.017079, 0.012994, 
0.008782, 0.004449, 0.000000, -0.004559, -0.009223, -0.013984, -0.018836, -0.023771, -0.028782, -0.033862, 
-0.039002, -0.044193, -0.049427, -0.054695, -0.059988, -0.065296, -0.070609, -0.075917, -0.081210, -0.086477, 
-0.091707, -0.096891, -0.102016, -0.107071, -0.112045, -0.116926, -0.121704, -0.126365, -0.130899, -0.135293, 
-0.139535, -0.143614, -0.147518, -0.151235, -0.154753, -0.158059, -0.161144, -0.163994, -0.166599, -0.168948, 
-0.171028, -0.172829, -0.174341, -0.175552, -0.176453, -0.177033, -0.177284, -0.177194, -0.176756, -0.175961, 
-0.174800, -0.173265, -0.171349, -0.169044, -0.166345, -0.163244, -0.159736, -0.155815, -0.151478, -0.146719, 
-0.141535, -0.135922, -0.129879, -0.123403, -0.116493, -0.109148, -0.101367, -0.093151, -0.084501, -0.075419, 
-0.065906, -0.055966, -0.045602, -0.034819, -0.023620, -0.012012, 0.000000, 0.012409, 0.025207, 0.038387, 
0.051940, 0.065856, 0.080127, 0.094742, 0.109689, 0.124957, 0.140534, 0.156407, 0.172564, 0.188989, 
0.205670, 0.222591, 0.239736, 0.257091, 0.274639, 0.292363, 0.310247, 0.328272, 0.346422, 0.364677, 
0.383021, 0.401434, 0.419896, 0.438390, 0.456896, 0.475393, 0.493863, 0.512285, 0.530639, 0.548906, 
0.567066, 0.585097, 0.602981, 0.620697, 0.638225, 0.655546, 0.672639, 0.689486, 0.706066, 0.722360, 
0.738350, 0.754018, 0.769343, 0.784310, 0.798900, 0.813095, 0.826879, 0.840236, 0.853149, 0.865603, 
0.877583, 0.889075, 0.900065, 0.910539, 0.920485, 0.929891, 0.938745, 0.947037, 0.954755, 0.961892, 
0.968438, 0.974385, 0.979726, 0.984454, 0.988563, 0.992049, 0.994907, 0.997133, 0.998726, 0.999682, 
1.000000, 0.999682, 0.998726, 0.997133, 0.994907, 0.992049, 0.988563, 0.984454, 0.979726, 0.974385, 
0.968438, 0.961892, 0.954755, 0.947037, 0.938745, 0.929891, 0.920485, 0.910539, 0.900065, 0.889075, 
0.877583, 0.865603, 0.853149, 0.840236, 0.826879, 0.813095, 0.798900, 0.784310, 0.769343, 0.754018, 
0.738350, 0.722360, 0.706066, 0.689486, 0.672639, 0.655546, 0.638225, 0.620697, 0.602981, 0.585097, 
0.567066, 0.548906, 0.530639, 0.512285, 0.493863, 0.475393, 0.456896, 0.438390, 0.419896, 0.401434, 
0.383021, 0.364677, 0.346422, 0.328272, 0.310247, 0.292363, 0.274639, 0.257091, 0.239736, 0.222591, 
0.205670, 0.188989, 0.172564, 0.156407, 0.140534, 0.124957, 0.109689, 0.094742, 0.080127, 0.065856, 
0.051940, 0.038387, 0.025207, 0.012409, 0.000000, -0.012012, -0.023620, -0.034819, -0.045602, -0.055966, 
-0.065906, -0.075419, -0.084501, -0.093151, -0.101367, -0.109148, -0.116493, -0.123403, -0.129879, -0.135922, 
-0.141535, -0.146719, -0.151478, -0.155815, -0.159736, -0.163244, -0.166345, -0.169044, -0.171349, -0.173265, 
-0.174800, -0.175961, -0.176756, -0.177194, -0.177284, -0.177033, -0.176453, -0.175552, -0.174341, -0.172829, 
-0.171028, -0.168948, -0.166599, -0.163994, -0.161144, -0.158059, -0.154753, -0.151235, -0.147518, -0.143614, 
-0.139535, -0.135293, -0.130899, -0.126365, -0.121704, -0.116926, -0.112045, -0.107071, -0.102016, -0.096891, 
-0.091707, -0.086477, -0.081210, -0.075917, -0.070609, -0.065296, -0.059988, -0.054695, -0.049427, -0.044193, 
-0.039002, -0.033862, -0.028782, -0.023771, -0.018836, -0.013984, -0.009223, -0.004559, 0.000000, 0.004449, 
0.008782, 0.012994, 0.017079, 0.021033, 0.024851, 0.028530, 0.032065, 0.035453, 0.038691, 0.041778, 
0.044709, 0.047484, 0.050101, 0.052559, 0.054856, 0.056992, 0.058967, 0.060781, 0.062435, 0.063928, 
0.065262, 0.066437, 0.067457, 0.068322, 0.069034, 0.069595, 0.070009, 0.070277, 0.070403, 0.070390, 
0.070242, 0.069961, 0.069551, 0.069017, 0.068363, 0.067591, 0.066708, 0.065716, 0.064622, 0.063429, 
0.062142, 0.060765, 0.059305, 0.057764, 0.056150, 0.054465, 0.052716, 0.050908, 0.049044, 0.047131, 
0.045173, 0.043175, 0.041143, 0.039080, 0.036992, 0.034883, 0.032759, 0.030623, 0.028480, 0.026334, 
0.024190, 0.022052, 0.019924, 0.017810, 0.015714, 0.013640, 0.011590, 0.009569, 0.007579, 0.005625, 
0.003708, 0.001832, 0.000000, -0.001786, -0.003524, -0.005211, -0.006846, -0.008426, -0.009949, -0.011414, 
-0.012820, -0.014165, -0.015448, -0.016668, -0.017824, -0.018916, -0.019943, -0.020904, -0.021799, -0.022629, 
-0.023393, -0.024091, -0.024724, -0.025292, -0.025795, -0.026235, -0.026611, -0.026925, -0.027178, -0.027371, 
-0.027505, -0.027581, -0.027601, -0.027565, -0.027477, -0.027336, -0.027145, -0.026905, -0.026619, -0.026287, 
-0.025912, -0.025496, -0.025041, -0.024548, -0.024020, -0.023458, -0.022865, -0.022242, -0.021592, -0.020917, 
-0.020218, -0.019498, -0.018759, -0.018002, -0.017231, -0.016446, -0.015650, -0.014844, -0.014031, -0.013212, 
-0.012389, -0.011564, -0.010739, -0.009916, -0.009095, -0.008278, -0.007468, -0.006666, -0.005872, -0.005089, 
-0.004318, -0.003559, -0.002815, -0.002086, -0.001373, -0.000677, 0.000000, 0.000658, 0.001296, 0.001914, 
0.002510, 0.003084, 0.003635, 0.004164, 0.004669, 0.005150, 0.005607, 0.006039, 0.006447, 0.006831, 
0.007189, 0.007522, 0.007831, 0.008115, 0.008374, 0.008609, 0.008819, 0.009006, 0.009169, 0.009308, 
0.009425, 0.009519, 0.009591, 0.009642, 0.009671, 0.009680, 0.009669, 0.009639, 0.009591, 0.009524, 
0.009440, 0.009339, 0.009222, 0.009090, 0.008944, 0.008784, 0.008610, 0.008425, 0.008228, 0.008020, 
0.007802, 0.007575, 0.007340, 0.007096, 0.006846, 0.006589, 0.006327, 0.006060, 0.005789, 0.005515, 
0.005237, 0.004958, 0.004677, 0.004395, 0.004113, 0.003832, 0.003551, 0.003272, 0.002995, 0.002721, 
0.002450, 0.002182, 0.001919, 0.001659, 0.001405, 0.001156, 0.000912, 0.000674, 0.000443, 0.000218, 
0.000000
};
*/

float gaussian[] = {
0.0000276867, 0.0000313372, 0.0000354252, 0.0000399970, 0.0000451024, 0.0000507969, 0.0000571393, 0.0000641942, 0.0000720305, 0.0000807234,
0.0000903534, 0.0001010067, 0.0001127759, 0.0001257607, 0.0001400663, 0.0001558062, 0.0001730996, 0.0001920743, 0.0002128643, 0.0002356122,
0.0002604675, 0.0002875883, 0.0003171365, 0.0003492890, 0.0003842253, 0.0004221278, 0.0004631981, 0.0005076294, 0.0005556344, 0.0006074231,
0.0006632135, 0.0007232295, 0.0007877002, 0.0008568493, 0.0009309116, 0.0010101191, 0.0010947065, 0.0011849031, 0.0012809352, 0.0013830323,
0.0014914123, 0.0016062852, 0.0017278555, 0.0018563163, 0.0019918500, 0.0021346223, 0.0022847824, 0.0024424682, 0.0026077876, 0.0027808234,
0.0029616639, 0.0031503478, 0.0033469028, 0.0035512460, 0.0037634049, 0.0039832968, 0.0042107560, 0.0044456995, 0.0046878789, 0.0049371008,
0.0051931166, 0.0054555949, 0.0057242044, 0.0059985860, 0.0062782701, 0.0065628150, 0.0068516959, 0.0071443880, 0.0074403389, 0.0077388583,
0.0080393386, 0.0083410893, 0.0086433647, 0.0089454468, 0.0092465623, 0.0095458826, 0.0098426069, 0.0101359066, 0.0104249255, 0.0107088076,
0.0109867242, 0.0112577916, 0.0115211814, 0.0117760096, 0.0120214479, 0.0122566952, 0.0124809783, 0.0126934963, 0.0128935035, 0.0130803370,
0.0132533065, 0.0134118320, 0.0135552782, 0.0136831756, 0.0137950272, 0.0138904462, 0.0139690461, 0.0140305507, 0.0140747113, 0.0141013900,
0.0141104762, 0.0141019147, 0.0140757608, 0.0140320973, 0.0139710898, 0.0138929870, 0.0137980375, 0.0136866002, 0.0135590894, 0.0134160022,
0.0132578082, 0.0130851149, 0.0128985023, 0.0126986608, 0.0124862809, 0.0122620807, 0.0120268609, 0.0117813951, 0.0115264840, 0.0112630114,
0.0109917782, 0.0107136406, 0.0104294824, 0.0101401321, 0.0098465010, 0.0095493901, 0.0092496279, 0.0089480981, 0.0086455189, 0.0083427464,
0.0080404709, 0.0077394383, 0.0074403665, 0.0071438908, 0.0068506188, 0.0065611580, 0.0062760607, 0.0059958242, 0.0057209179, 0.0054517837,
0.0051887806, 0.0049322953, 0.0046826039, 0.0044399827, 0.0042046249, 0.0039767791, 0.0037565557, 0.0035440930, 0.0033394184, 0.0031426425,
0.0029537376, 0.0027727038, 0.0025995161, 0.0024340780, 0.0022763065, 0.0021260912, 0.0019832968, 0.0018477686, 0.0017193409, 0.0015978315,
0.0014830441, 0.0013747746, 0.0012728101, 0.0011769243, 0.0010868990, 0.0010024966, 0.0009234880, 0.0008496356, 0.0007807074, 0.0007164715,
0.0006566930, 0.0006011456, 0.0005496055, 0.0005018518, 0.0004576718, 0.0004168556, 0.0003792044, 0.0003445166, 0.0003126100, 0.0002833021,
0.0002564185, 0.0002317937, 0.0002092704, 0.0001886977, 0.0001699335, 0.0001528425, 0.0001372977, 0.0001231784, 0.0001103721, 0.0000987724,
0.0000882807, 0.0000788043, 0.0000702563, 0.0000625568, 0.0000556308, 0.0000494094, 0.0000438284, 0.0000388288, 0.0000343561, 0.0000303607
};

/*
float zcoef30[] = {-2.263856362217020e-02, -1.873536299765810e-02, -1.496447425872265e-02, -1.132589740536392e-02, -7.819632437581867e-03, 
-4.445679355376491e-03, -1.204038158747800e-03, 1.905291152304215e-03, 4.882308577779545e-03, 7.727014117678192e-03, 1.043940777200016e-02, 
1.301948954074544e-02, 1.546725942391405e-02, 1.778271742150597e-02, 1.996586353352121e-02, 2.201669775995977e-02, 2.393522010082165e-02, 
2.572143055610686e-02, 2.737532912581537e-02, 2.889691580994721e-02, 3.028619060850236e-02, 3.154315352148084e-02, 3.266780454888262e-02, 
3.366014369070773e-02, 3.452017094695617e-02, 3.524788631762792e-02, 3.584328980272298e-02, 3.630638140224136e-02, 3.663716111618307e-02, 
3.683562894454809e-02, 3.690178488733643e-02, 3.683562894454808e-02, 3.663716111618307e-02, 3.630638140224136e-02, 3.584328980272298e-02, 
3.524788631762792e-02, 3.452017094695617e-02, 3.366014369070774e-02, 3.266780454888262e-02, 3.154315352148084e-02, 3.028619060850236e-02, 
2.889691580994720e-02, 2.737532912581537e-02, 2.572143055610686e-02, 2.393522010082166e-02, 2.201669775995978e-02, 1.996586353352122e-02, 
1.778271742150598e-02, 1.546725942391405e-02, 1.301948954074545e-02, 1.043940777200016e-02, 7.727014117678192e-03, 4.882308577779545e-03, 
1.905291152304213e-03, -1.204038158747798e-03, -4.445679355376498e-03, -7.819632437581863e-03, -1.132589740536393e-02, -1.496447425872266e-02, 
-1.873536299765809e-02, -2.263856362217019e-02 
};

*/

float zcoef15[] = {-4.105571847507337e-02, -2.639296187683288e-02, -1.274142987157447e-02, -1.011224592982157e-04, 1.152796035999595e-02, 
2.214581858630802e-02, 3.175245221963798e-02, 4.034786125998584e-02, 4.793204570735161e-02, 5.450500556173526e-02, 6.006674082313684e-02, 
6.461725149155630e-02, 6.815653756699364e-02, 7.068459904944890e-02, 7.220143593892205e-02, 7.270704823541310e-02, 7.220143593892205e-02, 
7.068459904944890e-02, 6.815653756699364e-02, 6.461725149155630e-02, 6.006674082313684e-02, 5.450500556173528e-02, 4.793204570735163e-02, 
4.034786125998586e-02, 3.175245221963800e-02, 2.214581858630803e-02, 1.152796035999597e-02, -1.011224592982007e-04, -1.274142987157447e-02, 
-2.639296187683284e-02, -4.105571847507331e-02 
};

/*
float fcoef15[] = { // smoothing differential FIR filter 31 taps
1.805890978092128e-01, 7.811152320165679e-02, -5.654678698256484e-03, -7.204603451250098e-02, -1.223990708630513e-01, -1.580503143718819e-01,
-1.803362916609674e-01, -1.905935293522823e-01, -1.901585540678009e-01, -1.803678924294981e-01, -1.625580710593481e-01, -1.380656165793256e-01,
-1.082270556114050e-01, -7.437891477756088e-02, -3.785772069976773e-02, -2.168078613171118e-17, 3.785772069976766e-02, 7.437891477756081e-02,
1.082270556114050e-01, 1.380656165793255e-01, 1.625580710593481e-01, 1.803678924294981e-01, 1.901585540678010e-01, 1.905935293522822e-01, 
1.803362916609674e-01, 1.580503143718819e-01, 1.223990708630513e-01, 7.204603451250102e-02, 5.654678698256546e-03, -7.811152320165669e-02, -1.805890978092130e-01 
};
*/




	int sockfda,sockfdb, sockfin;
	struct addrinfo hintsa,hintsb, *servinfoa, *servinfob, *porta,*portb;
	int rva,rvb;


	double lpx[FRST_FLTR], lpy[FRST_FLTR], lpu[FRST_FLTR], lpv[FRST_FLTR];
	double hpx[FRST_FLTR], hpy[FRST_FLTR], hpu[FRST_FLTR], hpv[FRST_FLTR];
	int		ii = 0, jj = 0, kk = 0, ll = 0, mm = 0, nn = 0, oo = 0, pp = 0, qq = 0;
	float kdiv = 0.0, kfft = 0.0;


	double 	Omega ;
	double 	Damping; // 0.00455 	

	double	integ_0_in, integ_1_in, integ_2_in, integ_3_in, integ_4_in, integ_5_in, integ_6_in, integ_7_in, integ_8_in;
	double	integ_0_out = 0.0, integ_1_out = 0.0, integ_2_out = 0.0, integ_3_out = 0.0, integ_4_out = 0.0, integ_5_out = 0.0, integ_6_out = 0.0, integ_7_out = MARK_freq; 
	double	integ_8_out = 0.0;
	
	double	integ_9_in, integ_10_in, integ_11_in, integ_12_in, integ_13_in, integ_14_in, integ_15_in, integ_16_in;
	double	integ_9_out = 0.0, integ_10_out = 0.0, integ_11_out = 0.0, integ_12_out = 0.0, integ_13_out = 0.0, integ_14_out = 0.0, integ_15_out = 0.0, integ_16_out = SPACE_freq; 

	double	lug_0 = 0, lug_1 = 0, lug_2 = 0, lug_3 = 0, lug_4 = 0, lug_5 = 0, lug_6 = 0, lug_7 = 0; 
	double	lug_8 = 0, lug_9 = 0, lug_10 = 0, lug_11 = 0, lug_12 = 0, lug_13 = 0, lug_14 = 0, lug_15 = 0; 

	double	COMPARATOR_0_out = -1, COMPARATOR_1_out = 0; 
	
	
	int 	hystFlag = FALSE; 
	float 	resample = 0.0; // allowing for variable rates of sampling

// gaussian filter
	int 	gaussiancalcflag = FALSE;
	int 	dataIndx = 0;
	float 	gaussA = 0.0, gaussB = 0.0, gaussC = 0.0;
	float 	dataA[GAUSS_TAPS], dataB[GAUSS_TAPS], dataC[GAUSS_TAPS];
// smoothing differential filter
	int		commencesmoothingflag = FALSE;	
	int 	sgfilterIndx = 0, sgfilterOut = 0; 
	float 	sgfilter = 0.0;
	float	sgfilterdata[DIFFERENTIAL_TAPS];

//	
	int 	numbytes, hostipFlag = TRUE, hostipCount = 0;
	char 	hostip[20];
	long 	int counter = 0;
	float 	dataAavg = 0.0, dataBavg = 0.0, dataCavg = 0.0;
	int		avgsum = 0;


/****************************************************************************/
/*																			*/
/*   get sockaddr, IPv4 or IPv6:											*/
/*																			*/
/****************************************************************************/


void *get_in_addr(struct sockaddr *sa)
{
	if (sa->sa_family == AF_INET) {
		return &(((struct sockaddr_in*)sa)->sin_addr);
	}

	return &(((struct sockaddr_in6*)sa)->sin6_addr);
}


/****************************************************************************/
/*																			*/
/*  parse incoming UDP controls												*/
/*																			*/
/****************************************************************************/
void *controls_loop(void *threadarg)
{
	struct control *DataPtr;
	int sockfd;
	struct addrinfo hints, *servinfo, *p;
	int rv, opt;
	size_t i;
	int numbytes;
	struct sockaddr_storage their_addr;
	char buf[MAXBUFLEN];
	socklen_t addr_len;
	char s[INET6_ADDRSTRLEN];
	wordexp_t newargv;
	float	tempfloat = 0.0;
	
	DataPtr = (struct control *) threadarg;
	if ( DataPtr->In_flg == TRUE) {
		fprintf(stderr, "Listening on port %s\n", "9223"); fflush(stderr);
	
		memset(&hints, 0, sizeof hints);
		hints.ai_family = AF_UNSPEC; // set to AF_INET to force IPv4
		hints.ai_socktype = SOCK_DGRAM;
		hints.ai_flags = AI_PASSIVE; // use my IP


		if ((rv = getaddrinfo(NULL, "9223", &hints, &servinfo)) != 0) { // set it to 9223 for now
			fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
			exit (1);
		}

		// loop through all the results and bind to the first we can
		for(p = servinfo; p != NULL; p = p->ai_next) {
			if ((sockfd = socket(p->ai_family, p->ai_socktype,
					p->ai_protocol)) == -1) {
				perror("listener: socket");
				continue;
			}

			if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
				close(sockfd);
				perror("listener: bind");
				continue;
			}

			break;
		}

		if (p == NULL) {
			fprintf(stderr, "listener: failed to bind socket\n");
			exit (2);

		}

		freeaddrinfo(servinfo);

		//	printf("listener: waiting to receive...\n");
		while (1) {
			addr_len = sizeof their_addr;
			if ((numbytes = recvfrom(sockfd, buf, MAXBUFLEN-1 , 0,
				(struct sockaddr *)&their_addr, &addr_len)) == -1) {
				perror("recvfrom");
				exit(1);
			}

		//	printf("listener: got packet from %s\n",
				inet_ntop(their_addr.ss_family, get_in_addr((struct sockaddr *)&their_addr), s, sizeof s);
		//	printf("listener: packet is %d bytes long\n", numbytes);
			buf[numbytes] = '\0';
//			fprintf(stderr, "%s\n", buf);fflush(stderr);

		//convert string with wordexp
			newargv.we_offs = 1;
			wordexp(buf, &newargv, WRDE_DOOFFS);

			for(i=1; i < newargv.we_wordc; i++) {
				 if ( !strcmp(newargv.we_wordv[i], "-a") ) { //adjust the local oscillator
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						if ((tempfloat > -400) && (tempfloat < 400)) {
							parameters.increment = parameters.increment - tempfloat/48000.0;
							if (tempfloat != 0) fprintf(stderr, "Tuning adj: %d Hz; LO => %d Hz\n", (int) tempfloat, (int) (parameters.increment*48000)); 
							if (tempfloat == 0) {
								fprintf(stderr, "\nLO: %d Hz\n", (int) (parameters.increment*48000));
								fprintf(stderr, "FSK pair: %.1f Hz : %.1f Hz : delta %.1f\n",  1000.0*integ_7_out,  1000.0*integ_16_out,  1000.0*(integ_7_out-integ_16_out)); 

								tempfloat=  (integ_7_out - MARK_freq); // 1 thousanths of Hz
								parameters.increment = parameters.increment + tempfloat/48.0; 
								//reset resonant filters
								integ_7_out = MARK_freq;
								integ_16_out = SPACE_freq;
								fprintf(stderr, "Updating: LO => %d Hz\n",  (int) (parameters.increment*48000)); 
								fprintf(stderr, "Updating FSK pair: %.1f Hz : %.1f Hz : delta %.1f\n",  1000.0*integ_7_out,  1000.0*integ_16_out,  1000.0*(integ_7_out-integ_16_out)); 
							
							}
							fflush(stderr);
						}
						else { //reset to 800 Hz
							parameters.localosc=  800;
							parameters.increment = parameters.localosc/48000.0;
							//reset resonant filters
							integ_7_out = MARK_freq;
							integ_16_out = SPACE_freq;
							fprintf(stderr, "\nLO reset: %d Hz\n", (int) (parameters.increment*48000));
					
						}
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-s") ) {  //use local oscillator as test signal
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						if (tempfloat > 0.0) {
							fprintf(stderr, "FSK upper test signal enabled\n"); fflush(stderr);
							if (parameters.localoscsv == 0.0) parameters.localoscsv = parameters.localosc; // save current value
							parameters.localosc = MARK_freq*1000; // set to MARK
							parameters.increment = parameters.localosc/48000.0; // recalculate corresponding increment
							parameters.testsignal = TRUE; // enable test
						} 
						else if (tempfloat < 0.0) {
							fprintf(stderr, "FSK lower test signal enabled\n"); fflush(stderr);
							if (parameters.localoscsv == 0.0) parameters.localoscsv = parameters.localosc; // save current value if not already saved
							parameters.localosc=  SPACE_freq*1000;
							parameters.increment = parameters.localosc/48000.0;
							parameters.testsignal = TRUE;
						}
						else {
							if (parameters.localoscsv != 0.0) {
								parameters.localosc=parameters.localoscsv; // restore previous value
								parameters.increment = parameters.localosc/48000.0;
							}
							parameters.localoscsv = 0.0; 
							parameters.testsignal = FALSE;
						}
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-n") ) { //adjust the local oscillator
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						if (tempfloat !=0) {
							parameters.signalVnoise = tempfloat;
							fprintf(stderr, "signal to noise displayed \n"); fflush(stderr);
						}
						else {
							parameters.signalVnoise = 0.0;
							fprintf(stderr, "envelope displayed \n"); fflush(stderr);
						}

						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-w") ) {  //wpm
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // wpm
						parameters.resampleFlag = TRUE;
						if (tempfloat <= 0) parameters.WordsPerMinute =  25;
						else parameters.WordsPerMinute = tempfloat;
						fprintf(stderr, "WPM set to %d\n", (int) parameters.WordsPerMinute); fflush(stderr);
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-r") ) {  //resonant filter
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // gain
//						if (tempfloat==0) {
//							parameters.resonantFilterFlag = FALSE;
//							fprintf(stderr, "Resonant filter disabled\n"); fflush(stderr);
//						}
//						else {
							parameters.resonantFilterFlag = TRUE; 	
							parameters.attenuate = exp(tempfloat*0.230258509)*.001; // .001 is the initial default coupling gain
							fprintf(stderr, "Resonant gain %.2f dB\n", tempfloat); fflush(stderr);
//						}
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-b") ) {  //resonant filter
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // gain
						if ((tempfloat <.5) || (tempfloat > 1.5)) {
							fprintf(stderr, "outside of range; balance set to 1.0\n"); fflush(stderr);
						}
						else {
							parameters.balance = tempfloat;
							fprintf(stderr, "balance %.2f:%.2f \n", tempfloat,1.0-tempfloat); fflush(stderr);
						}
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-d") ) {  //damper
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // gain
						if ((tempfloat >= 0.5) && (tempfloat<10.0)) {
							parameters.damper = tempfloat;
							fprintf(stderr, "damper set %0.2f\n",parameters.damper); fflush(stderr);
						}
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-t") ) {  //top threshold
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // threshold						
						parameters.threshold = exp((float) (tempfloat)*0.230258509);
						fprintf(stderr, "hysteresis-threshold set to %.2f dB, %f\n", tempfloat, parameters.threshold ); fflush(stderr);
						i++;
					} //else toofewargs ();
				}
				else if ( !strcmp(newargv.we_wordv[i], "-S") ) { 
					if (i+1 <= newargv.we_wordc) {
						sscanf(newargv.we_wordv[i + 1], "%f", &tempfloat); // frequency
						parameters.ScaleSpectrum = exp(tempfloat*0.230258509); // 1 is the initial default coupling gain
						fprintf(stderr, "scale spectrum set to %.2f dB, %f\n", tempfloat, parameters.ScaleSpectrum ); fflush(stderr);
						i++;
					} //else toofewargs ();
				}
			}

		}
		close(sockfd);
	
	}
   	pthread_exit((void *) 0);	

}




/****************************************************************************/
/*																			*/
/*  usage																	*/
/*																			*/
/****************************************************************************/

void usage(void)
{
	fprintf(stderr,
		"rtl_sdr, an I/Q recorder for RTL2832 based DVB-T receivers\n\n"
		"Usage:\t -f frequency_to_tune_to [Hz]\n"
		"\t[-s [1,-1] FSK test signal]\n"
		"\t[-c [o,1] 0 for CW decoding 1 for FSK decoding]\n"
		"\t[-r gain (db) resonant filter gain]\n"
		"\t[-t noise threshold (dB) ]\n"
		"\t[-b balance FSK pair\n"
		"\t[-d damping resonant filter\n"
		"\t[-w WPM (resampling rate for binary output)]\n"
		"\t[-h hostname host name for displays]\n"
		"\t[-i portno input controls]\n"
		"\t[-S Spectral Display (db) gain]\n");
	exit(1);
}

#define BUFFER	(600)
/****************************************************************************/
/*																			*/
/*  main																	*/
/*																			*/
/****************************************************************************/

int main(int argc, char **argv)
{
	int 		r, opt;


	int 		done = 0;
	short int 	rawinput[BUFFER];
	int			i = 0, j = 0, k = 0, l = 0, m = 0, n = 0, p = 0;
	int 		count = 0;
	short int 	Ibuf[9],Qbuf[9];
	int			inputbufPtr = 0;
	float		Iupsampled, Qupsampled;

    float 		float_IQ[4];
	char 		message[50];


	parameters.In_flg = TRUE;
	parameters.attenuate = 0.001;
	parameters.balance = 1.0;
	parameters.damper = 1.0;
	parameters.threshold = 50.0;
	parameters.resampleFlag=TRUE;
	parameters.WordsPerMinute =  25;
	parameters.cwFlg = FALSE;
	
 	parameters.localosc=LO; // initialised for 600Hz local oscillator
 	parameters.localoscsv=0.0;
	//parameters.frequency = defaultfreq - freqoffset - (int) parameters.localosc; // tuned frequency is above ... lower sid band.
	parameters.increment=parameters.localosc/48000.0; // initialising increment
	parameters.testsignal = FALSE; // can use local oscilator as a test signal
	parameters.signalVnoise = 0.0; // can use local oscilator as a test signal

	parameters.ScaleSpectrum = 1.0;

	int	sincptr = 0; // 279 entries 140th is the centre point 
	 
	for (j=0; j< 9; j++) Ibuf[j] = Qbuf[j]=0;
	
	
	for (ii=0; ii < FRST_FLTR; ii++) {
		lpx[ii] = lpy[ii] = lpu[ii] = lpv[ii] = 0.0;
		hpx[ii] = hpy[ii] = hpu[ii] = hpv[ii] = 0.0;
	}


	for (j=0; j < GAUSS_TAPS; j++) dataB[j]=-1.0;
	
	
	for (j=0; j < DIFFERENTIAL_TAPS; j++) sgfilterdata[j]=0.0;
	
	
	
		while ((opt = getopt(argc, argv, "d:r:b:w:t:h:s:c:")) != -1) {
		switch (opt) {
		case 'r': //  input gain for resonant filter stage
			parameters.resonantFilterFlag = TRUE;
			parameters.attenuate = exp(((float) (atof(optarg)))*0.230258509)*.001; // .001 is the initial default coupling gain
			break;
		case 'b': //  input gain for resonant filter stage
			parameters.balance = atof(optarg); // default is 1.0 
			break;
		case 'c': 			//  CW rather than FSK
			if (atoi(optarg) == 1) parameters.cwFlg = TRUE;
			else  parameters.cwFlg = FALSE;
			break;
		case 'd': // Damping for resonant filter
			parameters.damper = (float) (atof(optarg)); // should be in the range of .5-10 say default is 1.0
			if (parameters.damper < 0.45) parameters.damper = .45; 
			break;
		case 'w': // takes a wpm argument
			parameters.resampleFlag=TRUE;
			parameters.WordsPerMinute = (float) (atof(optarg));
			break;
		case 't': //threshold top level
			parameters.threshold = exp(((float) (atof(optarg)))*0.230258509);
			break;
		case 'h': // host ip
			hostipFlag = TRUE;
			hostipCount = 0;
			strcpy(&hostip[0], optarg);
			break;
		case 's': 
			if (atof(optarg) == 1) {
				if (parameters.localoscsv == 0.0) parameters.localoscsv = parameters.localosc;
				parameters.localosc = MARK_freq*1000;
				parameters.increment = parameters.localosc/48000.0;
				parameters.testsignal = TRUE;
			} else if (atof(optarg) == -1) {
				if (parameters.localoscsv == 0.0) parameters.localoscsv = parameters.localosc;
				parameters.localosc=  SPACE_freq*1000;
				parameters.increment = parameters.localosc/48000.0;
				parameters.testsignal = TRUE;
			} else {
				if (parameters.localoscsv != 0.0) parameters.localosc=parameters.localoscsv; // restore previous value
				parameters.localoscsv = 0.0; 
				parameters.increment = parameters.localosc/48000.0;
				parameters.testsignal = FALSE;
			}
			break;
		default:
			usage();
			break;
		}
	}

	strcpy(&hostip[0], "127.0.0.1"); /// local host is the default for the decoding. 

/*************** start input controls-monitor thread  **********************/
	
	pthread_create(&controls_thread, NULL, controls_loop, (void *) (&parameters));

//*************** start UDP data display and binary stream outputs *********/
	memset(&hintsa, 0, sizeof hintsa);
	hintsa.ai_family = AF_UNSPEC;
	hintsa.ai_socktype = SOCK_DGRAM;

	memset(&hintsb, 0, sizeof hintsb);
	hintsb.ai_family = AF_UNSPEC;
	hintsb.ai_socktype = SOCK_DGRAM;
	
	//second port 9224 is always set up for scope
	if ((rva = getaddrinfo(&hostip[0], "9224", &hintsa, &servinfoa)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rva));
		return 1;
	}
	// loop through all the results and make a socket
	for(porta = servinfoa; porta != NULL; porta = porta->ai_next) {
		if ((sockfda = socket(porta->ai_family, porta->ai_socktype,
				porta->ai_protocol)) == -1) {
			perror("udpsend: socket 9224");
			continue;
		}

		break;
	}

	if (porta == NULL) {
		fprintf(stderr, "udpsend: failed to bind socket 9224\n");
		return 2;
	}

	//second port 9222 is always set up 
	if ((rvb = getaddrinfo(&hostip[0], "9222", &hintsb, &servinfob)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rvb));
		return 1;
	}
	for(portb = servinfob; portb != NULL; portb = portb->ai_next) {
		if ((sockfdb = socket(portb->ai_family, portb->ai_socktype,
				portb->ai_protocol)) == -1) {
			perror("udpsend: socket 9222");
			continue;
		}
		break;
	}


/*************** Main processing loop  **********************/

		j = 0; //init J used for downsampling. As it hapens the input is at 12000Hz so upsampling by 4 and downsampling by 1 to 48000Hz


//		int fd = open("stdin", O_NONBLOCK);


	while (done != EOF) { // loop
		done = fread(&rawinput, sizeof (short int), BUFFER,stdin );
		// done = read(fd, &rawinput, 2 * sizeof (short int) );

		for (count=0; count < BUFFER; count+=2) {

// here we are upsampling
		//incoming data at 12kHz  sampling rate ... upsample to 48000Hz
		
		 for (k=0; k < 8; k++) Ibuf[k] = Ibuf[k+1];
			 Ibuf[8] = rawinput[count];
		 for (k=0; k < 8; k++) Qbuf[k] = Qbuf[k+1];
	 		 Qbuf[8] = rawinput[count+1];

			j = j % UPSAMP;		 // upsampling by 4  ...
			while (j < UPSAMP) {   // here we upsampl by UPSAMP and downsample then by DWNSAMP 
				Iupsampled = Qupsampled = 0.0;
				for (k=0; k < 4; k++) {
					Iupsampled += Ibuf[k] * sinc[HALFPOINT+(4-k)*UPSAMP + j];
					Qupsampled += Qbuf[k] * sinc[HALFPOINT+(4-k)*UPSAMP + j];
				}
					Iupsampled += Ibuf[k] * sinc[j];
					Qupsampled += Qbuf[k] * sinc[j];
				for (k=4; k <= 6; k++) {
					Iupsampled += Ibuf[k] * sinc[HALFPOINT-(k-4)*UPSAMP + j];
					Qupsampled += Qbuf[k] * sinc[HALFPOINT-(k-4)*UPSAMP + j];
				}


// now running at 48Khz  
				
				// run local oscillator nominally at 800Hz but adjustable by the -a switch
				 t+=parameters.increment; 
				 if (t >= 1.0) t = 0.0;
				 mixer=cos(6.283185307179586477*t);	

				
		//		fprintf(stdout, "%d %g %g\n",i++, Iupsampled, Qupsampled);

				pp = ii;					// ii -1
				ii = (ii + 1) % FRST_FLTR;	// ii
				jj = (ii + 1) % FRST_FLTR;	// ii -6
				kk = (jj + 1) % FRST_FLTR;	// ii -5
				ll = (kk + 1) % FRST_FLTR;	// ii -4
				mm = (ll + 1) % FRST_FLTR;	// ii -3
				nn = (mm + 1) % FRST_FLTR;	// ii -2
				oo = (nn + 1) % FRST_FLTR;	// ii -2

			// mixing the LO with the incoming I/Q ( presumably already band limited to 6KHz)
			// signal processing is within the 24KHz so the 6KHz +LO is in band.

				if (parameters.testsignal == FALSE) {
					lpx[ii] = (double) Iupsampled*mixer/16384.0; //input to I
					lpu[ii] = (double) Qupsampled*mixer/16384.0; //input to Q
				} else { // test mode
					lpx[ii] = mixer/128.0; //test tone
					lpu[ii] = sin(6.283185307179586477*t)/128.0;	
				}

			// perform the LP and HP filters

				hpx[ii] = lpy[ii] = (lpx[ii] + 7*lpx[pp] + 21*lpx[oo] + 35*lpx[nn] + 35*lpx[mm] + 21*lpx[ll] + 7*lpx[kk] + lpx[jj])/denomOne
					+ consta*lpy[pp] - constb*lpy[oo] + constc*lpy[nn] - constd*lpy[mm] + conste*lpy[ll] - constf*lpy[kk] + constg*lpy[jj]; 


				hpu[ii] = lpv[ii] = (lpu[ii] + 7*lpu[pp] + 21*lpu[oo] + 35*lpu[nn] + 35*lpu[mm] + 21*lpu[ll] + 7*lpu[kk] + lpu[jj])/denomOne
					+ consta*lpv[pp] - constb*lpv[oo] + constc*lpv[nn] - constd*lpv[mm] + conste*lpv[ll] - constf*lpv[kk] + constg*lpv[jj]; 



				// 1.057⋅yi = (1⋅xi + -2⋅xi-1 + 1⋅xi-2) + 1.997⋅yi-1 + -0.946⋅yi-2
				hpy[ii] = (hpx[ii] - 2*hpx[pp] + hpx[oo])/1.057 + 1.88930936613055818353*hpy[pp] - .89498580889309366130*hpy[oo];	
				hpv[ii] = (hpu[ii] - 2*hpu[pp] + hpu[oo])/1.057 + 1.88930936613055818353*hpv[pp] - .89498580889309366130*hpv[oo];	

		// transposed from pertecs ...  auto-centered tuning with feed back from side-tuned resonant filters
		// here we run two banks of three resonant filters. The first is tuned to 635Hz±10Hz and the second to 520±10Hz

				lug_0 = integ_0_out*integ_0_out + integ_1_out*integ_1_out;
				lug_1 = sqrt(integ_2_out*integ_2_out + integ_3_out*integ_3_out);
				lug_2 = integ_4_out*integ_4_out + integ_5_out*integ_5_out;
				lug_3 = integ_7_out -.01;
				lug_4 = integ_7_out;
				lug_5 = integ_7_out +.01;
				lug_6 = lug_0 - lug_2;
				lug_7 = lug_2 + lug_0;

				lug_8 = integ_9_out*integ_9_out + integ_10_out*integ_10_out;
				lug_9 = sqrt(integ_11_out*integ_11_out + integ_12_out*integ_12_out);
				lug_10 = integ_13_out*integ_13_out + integ_14_out*integ_14_out;
				lug_11 = integ_16_out -.01;
				lug_12 = integ_16_out;
				lug_13 = integ_16_out +.01;
				lug_14 = lug_8 - lug_10;
				lug_15 = lug_10 + lug_8;

		// the binary output stream determined by the realtive amplitude of the FSK pair above a threshold setting (-t)
		// don't have hysteresis 
//				 if (lug_1  > integ_8_out*1e-3 + parameters.threshold*100) COMPARATOR_0_out = 1;
//				 else if (lug_9  > integ_8_out*1e-3 + parameters.threshold*100) COMPARATOR_0_out = -1;
//				 else if (lug_1 < 2*integ_8_out*1e-3) COMPARATOR_0_out = -1;
				if (lug_1  > lug_9 + .01* parameters.threshold) COMPARATOR_0_out = 1;
				else if (lug_9  > lug_1 + .01* parameters.threshold) COMPARATOR_0_out = -1;
				else if (lug_1 < + .02* parameters.threshold) COMPARATOR_0_out = -1;


		// here we set up a rlaxation timer to track reception cycle ... triggered by a dit or dah signal being received 
		// only want the auto tuning to occur durring reception cycle
				 if (COMPARATOR_0_out >= 1) COMPARATOR_1_out = 240000;
				 else if (COMPARATOR_1_out > 0) COMPARATOR_1_out--;
				   
// update integrator input values
		// MARK filter bank
				integ_0_in = integ_2_in = integ_4_in =  parameters.attenuate*hpy[ii];
				integ_1_in = integ_3_in = integ_5_in =  -parameters.attenuate*hpv[ii]; 

				integ_0_in = integ_0_in + lug_3 * ((.1310) * integ_1_out - HDAMP * integ_0_out);
				integ_1_in = integ_1_in - lug_3 * ((.1310) * integ_0_out + HDAMP * integ_1_out);

				integ_2_in = integ_2_in + lug_4 * ((.1310) * integ_3_out - parameters.balance*parameters.damper*HDAMP * integ_2_out*(1 + 1.0e-10*integ_2_out*integ_2_out ));		//main pair parameters.damper* 
				integ_3_in = integ_3_in - lug_4 * ((.1310) * integ_2_out + parameters.balance*parameters.damper*HDAMP * integ_3_out*(1 + 1.0e-10*integ_3_out*integ_3_out ));		//parameters.damper*


				integ_4_in = integ_4_in + lug_5 * ((.1310) * integ_5_out - HDAMP * integ_4_out);
				integ_5_in = integ_5_in - lug_5 * ((.1310) * integ_4_out + HDAMP * integ_5_out);
				// feedback on tuning
				integ_6_in = 1e-7*((lug_0 > lug_2)-.5)*(lug_1 > lug_9)*(COMPARATOR_1_out > 0)  - 1e-3 * integ_6_out; 
				integ_7_in = (-1.0e-3*integ_6_out   + 1e-9*(integ_7_out - MARK_freq))*(COMPARATOR_1_out > 0);
		
		// SPACE  filter bank
				integ_9_in = integ_11_in = integ_13_in =  parameters.balance*parameters.attenuate*hpy[ii];
				integ_10_in = integ_12_in = integ_14_in =  -parameters.balance*parameters.attenuate*hpv[ii]; 

				integ_9_in = integ_9_in + lug_11 * ((.1310) * integ_10_out - LDAMP * integ_9_out);
				integ_10_in = integ_10_in - lug_11 * ((.1310) * integ_9_out + LDAMP * integ_10_out);

				integ_11_in = integ_11_in + lug_12 * ((.1310) * integ_12_out - parameters.damper*LDAMP * integ_11_out*(1 + 1.0e-10*integ_11_out*integ_11_out ));		//main pair parameters.damper* 
				integ_12_in = integ_12_in - lug_12 * ((.1310) * integ_11_out + parameters.damper*LDAMP * integ_12_out*(1 + 1.0e-10*integ_12_out*integ_12_out ));		//parameters.damper*


				integ_13_in = integ_13_in + lug_13 * ((.1310) * integ_14_out - LDAMP * integ_13_out);
				integ_14_in = integ_14_in - lug_13 * ((.1310) * integ_13_out + LDAMP * integ_14_out);
				// feedback on tuning
				integ_15_in = 1e-7*((lug_8 > lug_10)-.5)*(lug_9 > lug_1)*(COMPARATOR_1_out > 0)*(parameters.cwFlg == FALSE) - 1e-3 * integ_15_out; 
				integ_16_in = (-1.0e-3*integ_15_out  + 1e-9*(integ_16_out - SPACE_freq))*(COMPARATOR_1_out > 0);
		

		// update integrator output values
				integ_0_out += integ_0_in;
				integ_1_out += integ_1_in;

				integ_2_out += integ_2_in;
				integ_3_out += integ_3_in;

				integ_4_out += integ_4_in;
				integ_5_out += integ_5_in;

				integ_6_out += integ_6_in;
				integ_7_out += integ_7_in;

//				integ_8_out += integ_8_in;

				integ_9_out += integ_9_in;
				integ_10_out += integ_10_in;

				integ_11_out += integ_11_in;
				integ_12_out += integ_12_in;

				integ_13_out += integ_13_in;
				integ_14_out += integ_14_in;

				integ_15_out += integ_15_in;
				integ_16_out += integ_16_in;


			//	fprintf(stdout, "%d %g %g\n",i++, hpy[ii], integ_2_out);
			//	fprintf(stdout, "%d %d %g\n", i++, count, integ_7_out);
		// end of pertecs transpose
		
		

				// lets do some averaging 
				//dataAavg += (hpy[ii]*hpy[ii] + hpv[ii]*hpv[ii]);
				// this computing the sum of squares of both resonant filters combined
				dataAavg += lug_1 + lug_9;
				dataCavg += (lug_9 > lug_1)*lug_1 + (lug_1 > lug_9)*lug_9;

				// here the gaussian is being applied to the comparator output
				dataBavg += COMPARATOR_0_out;
				avgsum++; /// count how may have been summed

				// downsampling counter based on WPM rate 
				kdiv += parameters.WordsPerMinute/18.75; // lets make this the variable rate... at 25WPM  = 4800Hz
				kfft += 1; // 4800Hz for fft
			
//********************** Down to 4800Hz at 25WPM  *********************************
// In fact the downsampling is variable and tied to the WPM setting. This is a achieved by varying the time increment on kdiv 
// The rational for this is to perform gaussian filtering of the morse based on the dit/dah timing
// and then reclocking at roughly 3 (1-5) samples per dit period, i.e. 9 (6-15) samples per dah


				if (kdiv >= 10.0) { //drop another factor  at 25WPM its a factor of 10...  4800Hz resample for gaussian filters 

					// LOAD average of samples to INPUT GAUSSIAN FILTER
					dataA[dataIndx] = dataAavg/avgsum;
					dataB[dataIndx] = dataBavg/avgsum;
					dataC[dataIndx] = dataCavg/avgsum; // noise ratio dataA[dataIndx]/
					avgsum=0;
					dataAavg = 0.0; /// reset avg 
					dataBavg = 0.0; /// reset avg 
					dataCavg = 0.0; /// reset avg 
					
							// 200 taps in the gaussian ... working on data at 4800Hz, 
					gaussA = gaussB = gaussC = 0.0;
					m = dataIndx = (dataIndx + 1) % GAUSS_TAPS; // increment the data index, m is always being initialised to where the latest data point has been entered
					for (l=0; l < GAUSS_TAPS; l++ ) {
						gaussA = gaussA + dataA[m]*gaussian[l]; // smoothing of envelope
						gaussB = gaussB + dataB[m]*gaussian[l]; // smoothing of differential comparator output
						gaussC = gaussC + dataC[m]*gaussian[l]; // smoothing of noise assessment
						m = (m + 1) % GAUSS_TAPS;
					}							


/*					differentialdata[differentialIndx] = gaussB; 
	
					differential = 0.0;
					p = differentialIndx;
					for (n=0; n < DIFFERENTIAL_TAPS; n++ ) {
						differential = differential + differentialdata[p]*fcoef15[n];
						p = (p + 1) % DIFFERENTIAL_TAPS;
					}
					integ_8_in = differential - parameters.response * integ_8_out ;
					integ_8_out = integ_8_out + integ_8_in;
					differentialOut = (differentialIndx + 15 ) % DIFFERENTIAL_TAPS;		// if want the delayed input 0 - 30 with center tap at 15	
					differentialIndx = (differentialIndx + 1) % DIFFERENTIAL_TAPS;  // increment the data register by 1 ready for next time
*/

					sgfilterdata[sgfilterIndx] = gaussC*10; 
	
					sgfilter = 0.0;
					p = sgfilterIndx;
					for (n=0; n < DIFFERENTIAL_TAPS; n++ ) {
						sgfilter = sgfilter + sgfilterdata[p]*zcoef15[n];
						p = (p + 1) % DIFFERENTIAL_TAPS;
					}
					integ_8_in = sgfilter - .001 * integ_8_out ;
					integ_8_out += integ_8_in;
					sgfilterOut = (sgfilterIndx + 15 ) % DIFFERENTIAL_TAPS;		// if want the delayed input 0 - 30 with center tap at 15	
					sgfilterIndx = (sgfilterIndx + 1) % DIFFERENTIAL_TAPS;  // increment the data register by 1 ready for next time




//********************** Outputting binary data stream *********************************
					if (parameters.resampleFlag==TRUE) {
						resample += 1; //(wpm/25.0);
			
						if (resample >=120){ // 25wpm if resamplestep = wpm/25.0
							snprintf(message,80, "%d",(gaussB >= 0.0)); //outputs both binary and envelope
							if ((numbytes = sendto(sockfdb, &message[0], strlen(message), 0,portb->ai_addr, portb->ai_addrlen)) == -1) {
								perror("Binary stream error");
								exit (1);
							}
							resample = 0;
						}
					}

					
				//	fprintf(stdout, "%d %g %g %g %g\n",i++, integ_2_out, integ_11_out, integ_7_out,  gaussB); fflush(stdout);

//********************** Outputting to signal scope via UDP on selected port *********************************
					if (parameters.resonantFilterFlag==TRUE) {  // outputs with resonant filter
						//differentialdata[differentialIndx] = (float)  3*logf(1+  100.0 * gaussB); 
						// outputting the I for u/l resonant filters, smoothed comparator output (CW), 
//						if (parameters.signalVnoise == 0.0) 
							snprintf(message,80, "%ld %.8f %.8f %.5f %.8f", counter , (float) integ_2_out*100 , (float) integ_11_out*100 , (float) gaussB, (float) gaussA*100 );
//							sprintf(message, "%ld %.8f %.8f %.5f %.8f",counter , (float) integ_2_out/10000.0 , (float) integ_11_out/10000.0 , (float) gaussB, (float) gaussA*1e-4 );
//						else sprintf(message, "%ld %.8f %.8f %.5f %.8f",counter , (float) integ_2_out/10000.0 , (float) integ_11_out/10000.0 , (float) gaussB, (float) integ_8_out*1e-7 ); //
//					}				
						// send udp scope data to host

						if (hostipFlag == TRUE) {
							hostipCount = (hostipCount + 1) % 10;
							if (hostipCount == 0) {
								counter = (counter+1) % 10000000; 
								if ((numbytes = sendto(sockfda, &message[0], strlen(message), 0,porta->ai_addr, porta->ai_addrlen)) == -1) {
									perror("Display UDP error");
									exit (1);
								}
							}
						}
					}
					kdiv = 0.0; 
				}
				//********************** outputing I/Q data to fft calculation *******************************
				if (kfft >= 10.0) { //drop another factor  at 25WPM its a factor of 10...  4800Hz resample for gaussian filters 
					float_IQ[0] = (float) hpy[ii]*128*parameters.ScaleSpectrum;  // I before resonant filters
					float_IQ[1] = (float) hpv[ii]*128*parameters.ScaleSpectrum;	// Q before resonant filters
					if (parameters.cwFlg == FALSE){
						float_IQ[2] = (float) (integ_2_out + integ_11_out)*parameters.ScaleSpectrum/parameters.attenuate;   //I after resonant filters
						float_IQ[3] = (float) (integ_3_out + integ_12_out)*parameters.ScaleSpectrum/parameters.attenuate;	//Q after resonant filters
					} else {
						float_IQ[2] = (float) (integ_2_out)*parameters.ScaleSpectrum/parameters.attenuate;   //I after resonant filters
						float_IQ[3] = (float) (integ_3_out)*parameters.ScaleSpectrum/parameters.attenuate;	//Q after resonant filters					
					}	
					fwrite(&float_IQ[0], sizeof(float), 4, stdout); fflush (stdout);
				//*****************************************************
					kfft = 0.0; 
				}

				j+=DWNSAMP;
			}
		}
	}
}
