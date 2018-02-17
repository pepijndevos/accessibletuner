#include <stdint.h>

//! Define the filter order for the IIR_filter struct.
#define FILTER_ORDER 2

/**********************************************************/
// Struct declarations
/**********************************************************/
//! Filter struct that holds coefficients and previous input samples.
struct IIR_filter{
    //! Memory nodes for previous input and output.
     int16_t filterNodes[FILTER_ORDER*2];
     int16_t filterCoefficients[(FILTER_ORDER*2)+1];  //!< Filter coefficients.
     };

// Memory offsets for the filter nodes in the filter struct.
#define X1  0  //!< Offset for first filter input node.
#define X2  1  //!< Offset for second filter input node.
#define Y1  2  //!< Offset for first filter output node.
#define Y2  3  //!< Offset for second filter output node.

// Memory offsets for the coefficients in the filter struct.
#define B0  0 //!< Offset for first feed-forward coefficient.
#define B1  1 //!< Offset for second feed-forward coefficient.
#define B2  2 //!< Offset for third feed-forward coefficient.
#define A1  3 //!< Offset for first feedback coefficient.
#define A2  4 //!< Offset for second feedback coefficient.

int32_t mul_mov_24(int16_t coef, int16_t data) {
  int32_t ac = 0;
  asm (
  "muls  %B[COEF], %B[DATA] \n\t"
  "mov   %C[AC],   r0 \n\t"

  "mul  %A[COEF], %A[DATA] \n\t"
  "mov  %A[AC],   r0 \n\t"
  "mov  %B[AC],   r1 \n\t"

  "mulsu  %B[COEF], %A[DATA] \n\t"
  "add    %B[AC],   r0 \n\t"
  "adc    %C[AC],   r1 \n\t"

  "mulsu  %B[DATA], %A[COEF] \n\t"
  "add    %B[AC],   r0 \n\t"
  "adc    %C[AC],   r1 \n\t"
  : [AC] "=r" (ac)
  : [COEF] "a" (coef),
    [DATA] "a" (data)
  : "r0", "r1");

  return ac;
}

void smac_24(int32_t *ac, int16_t coef, int16_t data) {
//  asm (
//  "clr r2 \n\t"
//  "muls  %B[COEF], %B[DATA] \n\t"
//  "add   %C[AC],   r0 \n\t"
//
//  "mul  %A[COEF], %A[DATA] \n\t"
//  "add  %A[AC],   r0 \n\t"
//  "add  %B[AC],   r1 \n\t"
//  "adc  %C[AC],   r2 \n\t"
//
//  "mulsu  %B[COEF], %A[DATA] \n\t"
//  "add    %B[AC],   r0 \n\t"
//  "adc    %C[AC],   r1 \n\t"
//
//  "mulsu  %B[DATA], %A[COEF] \n\t"
//  "add    %B[AC],   r0 \n\t"
//  "adc    %C[AC],   r1 \n\t"
//  : "=r" (*ac)
//  : [COEF] "a" (coef),
//    [DATA] "a" (data),
//    [AC] "0" (*ac)
//  : "r0", "r1", "r2");
  *ac += (int32_t)coef*(int32_t)data;
}

int16_t IIR2( struct IIR_filter *thisFilter, int16_t newSample ) {
  int32_t acc = 0;
  
// Calculate B2*x[n-2]
//  LOAD_COEF B2
//  LOAD_NODE X2
//  MUL_MOV_24
acc = mul_mov_24(thisFilter->filterCoefficients[B2], thisFilter->filterNodes[X2]);

// Calculate B1*x[n-1]
//  LOAD_COEF   B1
//  LOAD_NODE   X1
//  UPDATE_NODE X2
//  SMAC_24
thisFilter->filterNodes[X2] = thisFilter->filterNodes[X1];
smac_24(&acc, thisFilter->filterCoefficients[B1], thisFilter->filterNodes[X1]);
  
// Calculate B0*x[n]
//  LOAD_COEF    B0
//  movw  DATAL, NDATAL  // For this coefficient, the new sample is used.
//  UPDATE_NODE  X1
//  SMAC_24
thisFilter->filterNodes[X1] = newSample;
smac_24(&acc, thisFilter->filterCoefficients[B0], newSample);

// Calculate A2*y[n-2]
//  LOAD_COEF A2
//  LOAD_NODE Y2
//  SMAC_24
smac_24(&acc, thisFilter->filterCoefficients[A2], thisFilter->filterNodes[Y2]);
  
// Calculate A1*y[n-1]
//  LOAD_COEF   A1
//  LOAD_NODE   Y1
//  UPDATE_NODE Y2
//  SMAC_24
thisFilter->filterNodes[Y2] = thisFilter->filterNodes[Y1];
smac_24(&acc, thisFilter->filterCoefficients[A1], thisFilter->filterNodes[Y1]);

// Due to the coefficient scaling by a factor 2^11, the output requires
// downscaling. Instead of right-shifting the 24-bit result 11 times, simply
// shift the 16 most significant bits (high and middle byte) 3 times.

asm volatile (
"asr  %C[AC] \n\t"
"ror  %B[AC] \n\t"
"asr  %C[AC] \n\t"
"ror  %B[AC] \n\t"
"asr  %C[AC] \n\t"
"ror  %B[AC] \n\t"
: "=r" (acc)
: [AC] "0" (acc));

thisFilter->filterNodes[Y1] = (uint32_t)acc>>8;

// Return the value stored in the return registers R16 and R17, which happens
// to be AC1 and AC2; the middle and high byte of accumulator.
 return thisFilter->filterNodes[Y1];
}

