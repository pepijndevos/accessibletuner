#include <stdint.h>
#include <math.h>

//! Define the filter order for the IIR_filter struct.
#define FILTER_ORDER 2
#define SCALE 2048

/**********************************************************/
// Struct declarations
/**********************************************************/
//! Filter struct that holds coefficients and previous input samples.
struct IIR_filter{
    //! Memory nodes for previous input and output.
     int16_t filterNodes[FILTER_ORDER*2];
     int16_t filterCoefficients[(FILTER_ORDER*2)+1];  //!< Filter coefficients.
     };

int32_t inline mul_mov_24(int16_t coef, int16_t data) {
  return (int32_t)coef*(int32_t)data;
}

void inline smac_24(int32_t *ac, int16_t coef, int16_t data) {
  *ac += (int32_t)coef*(int32_t)data;
}

int16_t IIR2( struct IIR_filter *thisFilter, int16_t newSample ) {
  int32_t acc = 0;
  
// Calculate B2*x[n-2]
//  LOAD_COEF B2
//  LOAD_NODE X2
//  MUL_MOV_24
acc = mul_mov_24(thisFilter->filterCoefficients[2], thisFilter->filterNodes[1]);

// Calculate B1*x[n-1]
//  LOAD_COEF   B1
//  LOAD_NODE   X1
//  UPDATE_NODE X2
//  SMAC_24
thisFilter->filterNodes[1] = thisFilter->filterNodes[0];
smac_24(&acc, thisFilter->filterCoefficients[1], thisFilter->filterNodes[0]);
  
// Calculate B0*x[n]
//  LOAD_COEF    B0
//  movw  DATAL, NDATAL  // For this coefficient, the new sample is used.
//  UPDATE_NODE  X1
//  SMAC_24
thisFilter->filterNodes[0] = newSample;
smac_24(&acc, thisFilter->filterCoefficients[0], newSample);

// Calculate A2*y[n-2]
//  LOAD_COEF A2
//  LOAD_NODE Y2
//  SMAC_24
smac_24(&acc, thisFilter->filterCoefficients[4], thisFilter->filterNodes[3]);
  
// Calculate A1*y[n-1]
//  LOAD_COEF   A1
//  LOAD_NODE   Y1
//  UPDATE_NODE Y2
//  SMAC_24
thisFilter->filterNodes[3] = thisFilter->filterNodes[2];
smac_24(&acc, thisFilter->filterCoefficients[3], thisFilter->filterNodes[2]);

// Due to the coefficient scaling by a factor 2^11, the output requires
// downscaling. Instead of right-shifting the 24-bit result 11 times, simply
// shift the 16 most significant bits (high and middle byte) 3 times.

thisFilter->filterNodes[2] = acc>>11;

 return thisFilter->filterNodes[2];
}

struct IIR_filter iirpeak(float w0, float bw) {
  // bandwidth
  //float bw = w0/Q;
  // normalise
  bw *= M_PI;
  w0 *= M_PI;
  // -3 dB atenuation
  float gb = 1/sqrt(2);
  float beta = (gb/sqrt(1.0-pow(gb, 2.0)))*tan(bw/2.0);
  float gain = 1.0/(1.0+beta);

  float b1 = (1.0-gain);
  float b2 = 0; // optimisation??
  float b3 = -(1.0-gain);
  float a2 = -2*gain*cos(w0);
  float a3 = 2*gain-1;

  return {0, 0, 0, 0,
          round(b1*SCALE), round(b2*SCALE), round(b3*SCALE),
           round(-a2*SCALE), round(-a3*SCALE)};
}

