/* to compile:
 g++ spectrum3.c -o spectrum3 -lfftw3 -I /usr/local/fftw/include -L /usr/local/fftw/lib; sudo cp spectrum3 /usr/local/bin
 
outputs frequency on 4800Hz sample rate i/q signal
*/
#include <fftw3.h>



#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "getopt/getopt.h"

#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>


#define TRUE 		(1) 
#define FALSE 		(0) 

#define NUM_POINTS 960


#define REAL 0
#define IMAG 1
	
#define FIR_LEN  31

double fcoef10[] = {2.313507748290360e-01, 2.760524499654982e-02, -1.191054051465953e-01, -2.151183844547694e-01, -2.667709017823440e-01, -2.804001659836907e-01, -2.623433859131805e-01, -2.189377704251851e-01, -1.565205283740756e-01, -8.142886861422345e-02, -2.486130953451685e-18, 8.142886861422345e-02, 1.565205283740756e-01, 2.189377704251850e-01, 2.623433859131805e-01, 2.804001659836906e-01, 2.667709017823439e-01, 2.151183844547693e-01, 1.191054051465954e-01, -2.760524499654931e-02, -2.313507748290361e-01 };
double zcoef10[] = {-5.590062111801346e-02, -2.484472049689524e-02, 2.942137953579061e-03, 2.745995423340923e-02, 4.870872834259536e-02, 6.668846028113747e-02, 8.139915004903552e-02, 9.284079764628955e-02, 1.010134030728996e-01, 1.059169663288656e-01, 1.075514874141876e-01, 1.059169663288656e-01, 1.010134030728996e-01, 9.284079764628966e-02, 8.139915004903570e-02, 6.668846028113776e-02, 4.870872834259588e-02, 2.745995423341001e-02, 2.942137953580212e-03, -2.484472049689355e-02, -5.590062111801124e-02 };
double fcoef15[] = {1.805890978092128e-01, 7.811152320165679e-02, -5.654678698256484e-03, -7.204603451250098e-02, -1.223990708630513e-01, -1.580503143718819e-01, -1.803362916609674e-01, -1.905935293522823e-01, -1.901585540678009e-01, -1.803678924294981e-01, -1.625580710593481e-01, -1.380656165793256e-01, -1.082270556114050e-01, -7.437891477756088e-02, -3.785772069976773e-02, -2.168078613171118e-17, 3.785772069976766e-02, 7.437891477756081e-02, 1.082270556114050e-01, 1.380656165793255e-01, 1.625580710593481e-01, 1.803678924294981e-01, 1.901585540678010e-01, 1.905935293522822e-01, 1.803362916609674e-01, 1.580503143718819e-01, 1.223990708630513e-01, 7.204603451250102e-02, 5.654678698256546e-03, -7.811152320165669e-02, -1.805890978092130e-01 };
double zcoef15[] = {-4.105571847507337e-02, -2.639296187683288e-02, -1.274142987157447e-02, -1.011224592982157e-04, 1.152796035999595e-02, 2.214581858630802e-02, 3.175245221963798e-02, 4.034786125998584e-02, 4.793204570735161e-02, 5.450500556173526e-02, 6.006674082313684e-02, 6.461725149155630e-02, 6.815653756699364e-02, 7.068459904944890e-02, 7.220143593892205e-02, 7.270704823541310e-02, 7.220143593892205e-02, 7.068459904944890e-02, 6.815653756699364e-02, 6.461725149155630e-02, 6.006674082313684e-02, 5.450500556173528e-02, 4.793204570735163e-02, 4.034786125998586e-02, 3.175245221963800e-02, 2.214581858630803e-02, 1.152796035999597e-02, -1.011224592982007e-04, -1.274142987157447e-02, -2.639296187683284e-02, -4.105571847507331e-02 };

//ring buffers
double smoothing[FIR_LEN];
double differential[FIR_LEN];	
int		si = 0, di = 0;


// UDP updates of tuning
char 	hostipa[20], hostipb[20];
struct 	addrinfo hintsa, hintsb, *servinfoa,*servinfob, *pa, *pb;
int 	rv;
int 	sockfda, sockfdb;

int numbytes;
char s[INET6_ADDRSTRLEN];
char message[80];




/****************************************************************************/
/*																			*/
/*  usage																	*/
/*																			*/
/****************************************************************************/
// while true; do cat ~/eight.raw; done | pertecs -c clock -rate 4800 -r -ineof | spectrum3
// display of spectrum and smoothed spectrum
// ./rtl_cw -f 27e6 -g 50 -w 25 -q -   | spectrum3 -r | framesnd 192.168.1.7 9222 > /dev/null
// then on the mac
// framerec 9222 | pertecs -c displayspectrum -rate 100000

/*
notes on frequency setting
./rtl_cw -f 27.00009e6 -g 50 -w 25 -q -   | spectrum3 -f 
FSK 535Hz 650Hz
si5351 -f 27000020  -l  -w 20 The quick brown fox jumps.
*/


void usage(void)
{
	fprintf(stderr,
		"spectrum3, an I/Q spectrum analyser accepts raw I/Q floats at 4800Hz \n\n"
		"Usage:\t -f output frequency peaks [Hz] stdout\n"
		"\t -s output spectrum stdout \n");
	exit(1);
}






int main(int argc, char **argv) {
	int r, opt;

  	int i=0, k, n, ii, jj, kk, mm, done = 1, si=0, di=0;
	float 	rawinput[4], scaledisp = .001, gain = 1.0;
	int	cwflg = FALSE;



	
    fftw_complex signal1a[NUM_POINTS]; // four buffers overlapping 25% increments
    fftw_complex signal2a[NUM_POINTS];
    fftw_complex signal3a[NUM_POINTS]; 
    fftw_complex signal4a[NUM_POINTS];

    fftw_complex signal1b[NUM_POINTS]; // two buffers overlapping 25% increments
    fftw_complex signal2b[NUM_POINTS];
    fftw_complex signal3b[NUM_POINTS]; 
    fftw_complex signal4b[NUM_POINTS];
    
    
    fftw_complex resulta[NUM_POINTS];
    fftw_complex resultb[NUM_POINTS];
    
    double maga[NUM_POINTS], magb[NUM_POINTS];
    double smooth[NUM_POINTS];
    double diff[NUM_POINTS];
   	double	shapeii, shapejj, shapekk, shapemm;

	fftw_plan plan1a,plan2a,plan3a,plan4a;
	fftw_plan plan1b,plan2b,plan3b,plan4b;

	int	output_spectrm_Flag = FALSE;
	int	output_smth_Flag = FALSE;
	int	output_raw_Flag = FALSE;

	int j = 0, count = 0, markcount = 0, adjust = 0; 
	double Space = 0.0, Mark = 0.0;


	strcpy(&hostipa[0], "127.0.0.1"); // local host is the default for the rtl_cw process. 
	strcpy(&hostipb[0], "127.0.0.1"); // local host default for the rtl_cw process. 

	while ((opt = getopt(argc, argv, "sv:r:h:g:c:")) != -1) {
		switch (opt) {
		case 's': // line display
			 output_spectrm_Flag = TRUE;
			break;
		case 'v': //smooth display needs IP of display host 
			output_smth_Flag = TRUE;
			scaledisp = exp(((float) (atof(optarg)))*0.230258509)*.001; // .001 is the initial default coupling gain
			//fprintf(stderr, "%s\n", hostipb);
			break;
		case 'r':  
			output_raw_Flag = TRUE;  
			scaledisp = exp(((float) (atof(optarg)))*0.230258509)*.001; // .001 is the initial default coupling gain
			break;
		case 'h': //host ip
			strcpy(&hostipb[0], optarg);
			break;
		case 'g':  
			gain = exp(((float) (atof(optarg)))*0.230258509); // gain sets spectrum against the ref threshold
			break;
		case 'c':  
			cwflg = (int) atoi(optarg); // cw mode otherwise FSK
			break;

		default:
			usage();
			break;
		}
	}

// first port for 9223
		memset(&hintsa, 0, sizeof hintsa);
		hintsa.ai_family = AF_UNSPEC;
		hintsa.ai_socktype = SOCK_DGRAM;

		if ((rv = getaddrinfo(&hostipa[0], "9223", &hintsa, &servinfoa)) != 0) {
			fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
			return 1;
		}

	// loop through all the results and make a socket
		for(pa = servinfoa; pa != NULL; pa = pa->ai_next) {
			if ((sockfda = socket(pa->ai_family, pa->ai_socktype,
					pa->ai_protocol)) == -1) {
				perror("udpsend: socket 9223");
				continue;
			}

			break;
		}
		//	fprintf(stderr, "socketfd %d\n", sockfda); fflush(stderr);

		if (pa == NULL) {
			fprintf(stderr, "udpsend: failed to bind socket 9223\n");
			return 2;
		}

// second port for 9226
		if (( output_smth_Flag == TRUE )|| ( output_raw_Flag == TRUE )){
			memset(&hintsb, 0, sizeof hintsb);
			hintsb.ai_family = AF_UNSPEC;
			hintsb.ai_socktype = SOCK_DGRAM;

			if ((rv = getaddrinfo(&hostipb[0], "9226", &hintsb, &servinfob)) != 0) {
				fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
				return 1;
			}

		// loop through all the results and make a socket
			for(pb = servinfob; pb != NULL; pb = pb->ai_next) {
				if ((sockfdb = socket(pb->ai_family, pb->ai_socktype,
						pb->ai_protocol)) == -1) {
					perror("udpsend: socket 9226");
					continue;
				}

				break;
			}
			//	fprintf(stderr, "socketfd %d\n", sockfdb); fflush(stderr);

			if (pb == NULL) {
				fprintf(stderr, "udpsend: failed to bind socket 9226\n"); 
				return 2;
			}
		}


 	for (k = 0; k < NUM_POINTS; k++) maga[k] = magb[k] = smooth[k] = diff[k] = 0.0;

		plan1a = fftw_plan_dft_1d(NUM_POINTS,
										  signal1a,
										  resulta,
										  FFTW_FORWARD,
										  FFTW_ESTIMATE);
		plan2a = fftw_plan_dft_1d(NUM_POINTS,
										  signal2a,
										  resulta,
										  FFTW_FORWARD,
										  FFTW_ESTIMATE);
		plan3a = fftw_plan_dft_1d(NUM_POINTS,
										  signal3a,
										  resulta,
										  FFTW_FORWARD,
										  FFTW_ESTIMATE);
		plan4a = fftw_plan_dft_1d(NUM_POINTS,
										  signal4a,
										  resulta,
										  FFTW_FORWARD,
										  FFTW_ESTIMATE);

		plan1b = fftw_plan_dft_1d(NUM_POINTS,
										  signal1b,
										  resultb,
										  FFTW_FORWARD,
										  FFTW_ESTIMATE);
		plan2b = fftw_plan_dft_1d(NUM_POINTS,
										  signal2b,
										  resultb,
										  FFTW_FORWARD,
										  FFTW_ESTIMATE);
		plan3b = fftw_plan_dft_1d(NUM_POINTS,
										  signal3b,
										  resultb,
										  FFTW_FORWARD,
										  FFTW_ESTIMATE);
		plan4b = fftw_plan_dft_1d(NUM_POINTS,
										  signal4b,
										  resultb,
										  FFTW_FORWARD,
										  FFTW_ESTIMATE);


	ii = jj = kk = mm = 0;
    double max = 0.0;
	while (done != EOF) { // big loop

// doubling up on the fft to do both the raw IQ and resonant IQ
// reading Numpoints/4
		while ((ii < NUM_POINTS) && ( jj < NUM_POINTS) && (kk < NUM_POINTS) && ( mm < NUM_POINTS)) {
			done = fread(&rawinput, sizeof (float), 4,stdin );											//lets tie the spectral output to the 4800Hz

			
			shapeii =	(1 - cos(ii*3.1415926/NUM_POINTS) );
			signal1a[ii][REAL] = (double) rawinput[0] * shapeii;
			signal1a[ii][IMAG] = (double) rawinput[1] * shapeii;
			signal1b[ii][REAL] = (double) rawinput[2] * shapeii;
			signal1b[ii][IMAG] = (double) rawinput[3] * shapeii;

			jj = (ii + NUM_POINTS/4) % NUM_POINTS;
			shapejj =	(1 - cos(jj*3.1415926/NUM_POINTS) );
			signal2a[jj][REAL] = (double) rawinput[0] * shapejj;
			signal2a[jj][IMAG] = (double) rawinput[1] * shapejj;
			signal2b[jj][REAL] = (double) rawinput[2] * shapejj;
			signal2b[jj][IMAG] = (double) rawinput[3] * shapejj;

			kk = (jj + NUM_POINTS/4) % NUM_POINTS;
			shapekk =	(1 - cos(kk*3.1415926/NUM_POINTS) );
			signal3a[kk][REAL] = (double) rawinput[0] * shapekk;
			signal3a[kk][IMAG] = (double) rawinput[1] * shapekk;
			signal3b[kk][REAL] = (double) rawinput[2] * shapekk;
			signal3b[kk][IMAG] = (double) rawinput[3] * shapekk;

			mm = (kk + NUM_POINTS/4) % NUM_POINTS;
			shapemm =	(1 - cos(mm*3.1415926/NUM_POINTS) );
			signal4a[mm][REAL] = (double) rawinput[0] * shapemm;
			signal4a[mm][IMAG] = (double) rawinput[1] * shapemm;
			signal4b[mm][REAL] = (double) rawinput[2] * shapemm;
			signal4b[mm][IMAG] = (double) rawinput[3] * shapemm;

//			printf("%d %d %d %d\n",ii,jj,kk,mm); fflush(stdout);
			ii = ii + 1;
		}

//execute appropriate plan
		if (mm == NUM_POINTS-1) {
			fftw_execute(plan4a);
			fftw_execute(plan4b);
//			printf("plan4\n"); fflush(stdout);
		} else if (kk == NUM_POINTS-1) {
			fftw_execute(plan3a);
			fftw_execute(plan3b);
//			printf("plan3\n"); fflush(stdout);
		} else if (jj == NUM_POINTS-1) {
			fftw_execute(plan2a);
			fftw_execute(plan2b);
//			printf("plan2\n"); fflush(stdout);
		} else if (ii == NUM_POINTS) {
			fftw_execute(plan1a);
			fftw_execute(plan1b);
 	    	ii = 0;
//			printf("plan1\n"); fflush(stdout);
		}    		
// compute the magnatude a
		max = 0;
		count = 0;
		Mark = Space = 0.0;
		for (k = 0; k < NUM_POINTS/2; k++) {
			j = NUM_POINTS - 1 - k;
			maga[k] += .9* (resulta[j][REAL] * resulta[j][REAL] +
							  resulta[j][IMAG] * resulta[j][IMAG] +
							  resulta[k][REAL] * resulta[k][REAL] +
							  resulta[k][IMAG] * resulta[k][IMAG]) - .2 *maga[k] ;

			magb[k] += .9* (resultb[j][REAL] * resultb[j][REAL] +
							  resultb[j][IMAG] * resultb[j][IMAG] +
							  resultb[k][REAL] * resultb[k][REAL] +
							  resultb[k][IMAG] * resultb[k][IMAG]) - .2 *magb[k] ;


			j = (k + NUM_POINTS/2 - FIR_LEN/2)  % (NUM_POINTS/2);                 
			smooth[j] = diff[j] = 0; 
			differential[di] = smoothing[si] = maga[k]; //pick up the raw I/Q frequency curve
			for (n=0; n < FIR_LEN; n++){
				smooth[j]=smooth[j]+smoothing[si]*zcoef15[n];
				si = (si + 1) % FIR_LEN;
				diff[j]=diff[j]+differential[si]*fcoef15[n];
				di = (di + 1) % FIR_LEN;
			}
			si = (si + 1) % FIR_LEN;
			di = (di + 1) % FIR_LEN;
			 max = max *.99;
			
			if ((k >=40) && (k <=200)) {    //} && (smooth[k] > 100) ) { // only look at data in the range 40-200 = 200Hz-1000Hz

				if ((diff[k-1] >= 0.0) &&  (diff[k] < 0.0)){ 
					if (cwflg == TRUE) {
						if (smooth[k] > max) {
							max=smooth[k]; 
							Mark = 4800.0*k/NUM_POINTS;
						}

					}
/*					if ((gain*smooth[k] > 50.0) && (smooth[k] < 3* maga[k])) {  /// negative going zero crossing
						if ((count == 0) && (cwflg == FALSE)) {	
							Space = 4800.0*k/NUM_POINTS;
	//						printf("%d %gHz %g\n",count, Space, smooth[k] ); fflush(stdout);
						}
						else if ((count == 1) || (cwflg == TRUE)) {
							Mark = 4800.0*k/NUM_POINTS;
							if (((Mark > 590) || (Mark < 610)) && (600-Mark < 100) && (600-Mark > -100)) { markcount++; adjust+=600-Mark;}
							//fprintf(stderr,"-a %gHz\n",640-Mark ); fflush(stderr);
							// fprintf(stderr,"%gHz\t%gHz\t\t%g\t%g\t%g\t\t%g\n",Space,  Mark, max, smooth[k],maga[k],   Mark - Space ); fflush(stderr);
							//
							//if (output_spectrm_Flag == TRUE) printf("%d %d %g %g\n", k, k, smooth[k], diff[k] ); fflush(stdout);
						}
						count++;
						if (markcount >= 20) { 
//							sprintf(message, "-a %d",(int) (adjust/20.0) );
//							fprintf(stderr,"%s %d %d\n",message,strlen(message) ); fflush(stderr);
//							sprintf(message, "-a %d",(int) (adjust/20.0) );
//							if (0 != (int) (adjust/20)) {
//								if ((numbytes = sendto(sockfda, &message[0], strlen(message), 0,pa->ai_addr, pa->ai_addrlen)) == -1) {
//									perror("Display UDP error");
//									exit (1);
//								}
//							}
							markcount = 0;
							adjust = 0;
						}
					}
					*/
				}
			}
//			if ((k ==200) && (max > gain))
//				fprintf(stderr,"freq %d %f \n", (int) Mark, max ); fflush(stderr);

			
			
			if ((k >=0) && (k<240)){
				if (output_smth_Flag == TRUE) { 
		//			printf("%d %d %g %g\n", k, k, smooth[k], maga[k]); fflush(stdout); 
					sprintf(message, "%d %d %g %g", k, k, diff[k]*scaledisp, maga[k]*scaledisp );
					if ((numbytes = sendto(sockfdb, &message[0], strlen(message), 0,pb->ai_addr, pb->ai_addrlen)) == -1) {
						perror("Display UDP error");
						exit (1);
					}
		
				}
				else if (output_raw_Flag == TRUE) {
					//printf("%d %d %g %g\n", k, k, maga[k], magb[k]); fflush(stdout);
					sprintf(message, "%d %d %g %g", k, k, maga[k]*scaledisp, magb[k]*scaledisp);
					if ((numbytes = sendto(sockfdb, &message[0], strlen(message), 0,pb->ai_addr, pb->ai_addrlen)) == -1) {
						perror("Display UDP error");
						exit (1);
					}
				}
			}	
		}    
	}
	fftw_destroy_plan(plan1a);
	fftw_destroy_plan(plan2a);
	fftw_destroy_plan(plan3a);
	fftw_destroy_plan(plan4a);
	fftw_destroy_plan(plan1b);
	fftw_destroy_plan(plan2b);
	fftw_destroy_plan(plan3b);
	fftw_destroy_plan(plan4b);

	freeaddrinfo(servinfoa);
	freeaddrinfo(servinfob);
	close(sockfda);
	close(sockfdb);


    return 0;
}

