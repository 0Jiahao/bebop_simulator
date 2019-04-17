/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;

/* Compute outputs: */
out[0] = xd[3];
out[1] = xd[4];
out[2] = (((real_t)(1.7649999999999999e+01)*xd[5])+((real_t)(-3.2250000000000001e-01)*xd[6]));
out[3] = (((real_t)(-1.8400000000000000e-01)*xd[3])+((real_t)(4.9050000000000002e+00)*xd[8]));
out[4] = (((real_t)(-2.0069999999999999e-01)*xd[4])+((real_t)(4.9050000000000002e+00)*xd[7]));
out[5] = ((((real_t)(-2.2919999999999998e+00)*xd[5])+((real_t)(-2.5499999999999998e+00)*xd[6]))+((real_t)(6.4200000000000004e-03)*u[2]));
out[6] = ((((real_t)(9.7420000000000009e+00)*xd[5])+((real_t)(-5.8449999999999998e+00)*xd[6]))+((real_t)(-6.5700000000000003e-01)*u[2]));
out[7] = (((real_t)(-6.7529000000000003e+00)*xd[7])+((real_t)(6.8658500000000000e+00)*u[0]));
out[8] = (((real_t)(-6.7317000000000000e+00)*xd[8])+((real_t)(6.8541699999999999e+00)*u[1]));
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(1.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(1.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(1.7649999999999999e+01);
out[30] = (real_t)(-3.2250000000000001e-01);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(-1.8400000000000000e-01);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(4.9050000000000002e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(-2.0069999999999999e-01);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(4.9050000000000002e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(-2.2919999999999998e+00);
out[66] = (real_t)(-2.5499999999999998e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(6.4200000000000004e-03);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(9.7420000000000009e+00);
out[78] = (real_t)(-5.8449999999999998e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(-6.5700000000000003e-01);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(-6.7529000000000003e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(6.8658500000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(-6.7317000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(6.8541699999999999e+00);
out[107] = (real_t)(0.0000000000000000e+00);
}



void acado_solve_dim9_triangular( real_t* const A, real_t* const b )
{

b[8] = b[8]/A[80];
b[7] -= + A[71]*b[8];
b[7] = b[7]/A[70];
b[6] -= + A[62]*b[8];
b[6] -= + A[61]*b[7];
b[6] = b[6]/A[60];
b[5] -= + A[53]*b[8];
b[5] -= + A[52]*b[7];
b[5] -= + A[51]*b[6];
b[5] = b[5]/A[50];
b[4] -= + A[44]*b[8];
b[4] -= + A[43]*b[7];
b[4] -= + A[42]*b[6];
b[4] -= + A[41]*b[5];
b[4] = b[4]/A[40];
b[3] -= + A[35]*b[8];
b[3] -= + A[34]*b[7];
b[3] -= + A[33]*b[6];
b[3] -= + A[32]*b[5];
b[3] -= + A[31]*b[4];
b[3] = b[3]/A[30];
b[2] -= + A[26]*b[8];
b[2] -= + A[25]*b[7];
b[2] -= + A[24]*b[6];
b[2] -= + A[23]*b[5];
b[2] -= + A[22]*b[4];
b[2] -= + A[21]*b[3];
b[2] = b[2]/A[20];
b[1] -= + A[17]*b[8];
b[1] -= + A[16]*b[7];
b[1] -= + A[15]*b[6];
b[1] -= + A[14]*b[5];
b[1] -= + A[13]*b[4];
b[1] -= + A[12]*b[3];
b[1] -= + A[11]*b[2];
b[1] = b[1]/A[10];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim9_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 9; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (8); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*9+i]);
	for( j=(i+1); j < 9; j++ ) {
		temp = fabs(A[j*9+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 9; ++k)
{
	acadoWorkspace.rk_dim9_swap = A[i*9+k];
	A[i*9+k] = A[indexMax*9+k];
	A[indexMax*9+k] = acadoWorkspace.rk_dim9_swap;
}
	acadoWorkspace.rk_dim9_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = acadoWorkspace.rk_dim9_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*9+i];
	for( j=i+1; j < 9; j++ ) {
		A[j*9+i] = -A[j*9+i]/A[i*9+i];
		for( k=i+1; k < 9; k++ ) {
			A[j*9+k] += A[j*9+i] * A[i*9+k];
		}
		b[j] += A[j*9+i] * b[i];
	}
}
det *= A[80];
det = fabs(det);
acado_solve_dim9_triangular( A, b );
return det;
}

void acado_solve_dim9_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim9_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim9_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim9_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim9_bPerm[3] = b[rk_perm[3]];
acadoWorkspace.rk_dim9_bPerm[4] = b[rk_perm[4]];
acadoWorkspace.rk_dim9_bPerm[5] = b[rk_perm[5]];
acadoWorkspace.rk_dim9_bPerm[6] = b[rk_perm[6]];
acadoWorkspace.rk_dim9_bPerm[7] = b[rk_perm[7]];
acadoWorkspace.rk_dim9_bPerm[8] = b[rk_perm[8]];
acadoWorkspace.rk_dim9_bPerm[1] += A[9]*acadoWorkspace.rk_dim9_bPerm[0];

acadoWorkspace.rk_dim9_bPerm[2] += A[18]*acadoWorkspace.rk_dim9_bPerm[0];
acadoWorkspace.rk_dim9_bPerm[2] += A[19]*acadoWorkspace.rk_dim9_bPerm[1];

acadoWorkspace.rk_dim9_bPerm[3] += A[27]*acadoWorkspace.rk_dim9_bPerm[0];
acadoWorkspace.rk_dim9_bPerm[3] += A[28]*acadoWorkspace.rk_dim9_bPerm[1];
acadoWorkspace.rk_dim9_bPerm[3] += A[29]*acadoWorkspace.rk_dim9_bPerm[2];

acadoWorkspace.rk_dim9_bPerm[4] += A[36]*acadoWorkspace.rk_dim9_bPerm[0];
acadoWorkspace.rk_dim9_bPerm[4] += A[37]*acadoWorkspace.rk_dim9_bPerm[1];
acadoWorkspace.rk_dim9_bPerm[4] += A[38]*acadoWorkspace.rk_dim9_bPerm[2];
acadoWorkspace.rk_dim9_bPerm[4] += A[39]*acadoWorkspace.rk_dim9_bPerm[3];

acadoWorkspace.rk_dim9_bPerm[5] += A[45]*acadoWorkspace.rk_dim9_bPerm[0];
acadoWorkspace.rk_dim9_bPerm[5] += A[46]*acadoWorkspace.rk_dim9_bPerm[1];
acadoWorkspace.rk_dim9_bPerm[5] += A[47]*acadoWorkspace.rk_dim9_bPerm[2];
acadoWorkspace.rk_dim9_bPerm[5] += A[48]*acadoWorkspace.rk_dim9_bPerm[3];
acadoWorkspace.rk_dim9_bPerm[5] += A[49]*acadoWorkspace.rk_dim9_bPerm[4];

acadoWorkspace.rk_dim9_bPerm[6] += A[54]*acadoWorkspace.rk_dim9_bPerm[0];
acadoWorkspace.rk_dim9_bPerm[6] += A[55]*acadoWorkspace.rk_dim9_bPerm[1];
acadoWorkspace.rk_dim9_bPerm[6] += A[56]*acadoWorkspace.rk_dim9_bPerm[2];
acadoWorkspace.rk_dim9_bPerm[6] += A[57]*acadoWorkspace.rk_dim9_bPerm[3];
acadoWorkspace.rk_dim9_bPerm[6] += A[58]*acadoWorkspace.rk_dim9_bPerm[4];
acadoWorkspace.rk_dim9_bPerm[6] += A[59]*acadoWorkspace.rk_dim9_bPerm[5];

acadoWorkspace.rk_dim9_bPerm[7] += A[63]*acadoWorkspace.rk_dim9_bPerm[0];
acadoWorkspace.rk_dim9_bPerm[7] += A[64]*acadoWorkspace.rk_dim9_bPerm[1];
acadoWorkspace.rk_dim9_bPerm[7] += A[65]*acadoWorkspace.rk_dim9_bPerm[2];
acadoWorkspace.rk_dim9_bPerm[7] += A[66]*acadoWorkspace.rk_dim9_bPerm[3];
acadoWorkspace.rk_dim9_bPerm[7] += A[67]*acadoWorkspace.rk_dim9_bPerm[4];
acadoWorkspace.rk_dim9_bPerm[7] += A[68]*acadoWorkspace.rk_dim9_bPerm[5];
acadoWorkspace.rk_dim9_bPerm[7] += A[69]*acadoWorkspace.rk_dim9_bPerm[6];

acadoWorkspace.rk_dim9_bPerm[8] += A[72]*acadoWorkspace.rk_dim9_bPerm[0];
acadoWorkspace.rk_dim9_bPerm[8] += A[73]*acadoWorkspace.rk_dim9_bPerm[1];
acadoWorkspace.rk_dim9_bPerm[8] += A[74]*acadoWorkspace.rk_dim9_bPerm[2];
acadoWorkspace.rk_dim9_bPerm[8] += A[75]*acadoWorkspace.rk_dim9_bPerm[3];
acadoWorkspace.rk_dim9_bPerm[8] += A[76]*acadoWorkspace.rk_dim9_bPerm[4];
acadoWorkspace.rk_dim9_bPerm[8] += A[77]*acadoWorkspace.rk_dim9_bPerm[5];
acadoWorkspace.rk_dim9_bPerm[8] += A[78]*acadoWorkspace.rk_dim9_bPerm[6];
acadoWorkspace.rk_dim9_bPerm[8] += A[79]*acadoWorkspace.rk_dim9_bPerm[7];


acado_solve_dim9_triangular( A, acadoWorkspace.rk_dim9_bPerm );
b[0] = acadoWorkspace.rk_dim9_bPerm[0];
b[1] = acadoWorkspace.rk_dim9_bPerm[1];
b[2] = acadoWorkspace.rk_dim9_bPerm[2];
b[3] = acadoWorkspace.rk_dim9_bPerm[3];
b[4] = acadoWorkspace.rk_dim9_bPerm[4];
b[5] = acadoWorkspace.rk_dim9_bPerm[5];
b[6] = acadoWorkspace.rk_dim9_bPerm[6];
b[7] = acadoWorkspace.rk_dim9_bPerm[7];
b[8] = acadoWorkspace.rk_dim9_bPerm[8];
}



/** Column vector of size: 1 */
static const real_t acado_Ah_mat[ 1 ] = 
{ 3.3333333333333333e-02 };


/* Fixed step size:0.0666667 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[9] = rk_eta[117];
acadoWorkspace.rk_xxx[10] = rk_eta[118];
acadoWorkspace.rk_xxx[11] = rk_eta[119];

for (run = 0; run < 1; ++run)
{
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 9; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 108 ]) );
for (j = 0; j < 9; ++j)
{
tmp_index1 = (run1 * 9) + (j);
acadoWorkspace.rk_A[tmp_index1 * 9] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 4] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 5] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 6] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 7] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 8] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 9) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 9] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 9 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 9 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 9 + 3] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 9 + 4] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 9 + 5] = acadoWorkspace.rk_kkk[run1 + 5] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 9 + 6] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 9 + 7] = acadoWorkspace.rk_kkk[run1 + 7] - acadoWorkspace.rk_rhsTemp[7];
acadoWorkspace.rk_b[run1 * 9 + 8] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[8];
}
det = acado_solve_dim9_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim9_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 9];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 9 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 9 + 2];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 9 + 3];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 9 + 4];
acadoWorkspace.rk_kkk[j + 5] += acadoWorkspace.rk_b[j * 9 + 5];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 9 + 6];
acadoWorkspace.rk_kkk[j + 7] += acadoWorkspace.rk_b[j * 9 + 7];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 9 + 8];
}
}
}
for (i = 0; i < 2; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 9; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 9] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 9 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 9 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
acadoWorkspace.rk_b[run1 * 9 + 3] = acadoWorkspace.rk_kkk[run1 + 3] - acadoWorkspace.rk_rhsTemp[3];
acadoWorkspace.rk_b[run1 * 9 + 4] = acadoWorkspace.rk_kkk[run1 + 4] - acadoWorkspace.rk_rhsTemp[4];
acadoWorkspace.rk_b[run1 * 9 + 5] = acadoWorkspace.rk_kkk[run1 + 5] - acadoWorkspace.rk_rhsTemp[5];
acadoWorkspace.rk_b[run1 * 9 + 6] = acadoWorkspace.rk_kkk[run1 + 6] - acadoWorkspace.rk_rhsTemp[6];
acadoWorkspace.rk_b[run1 * 9 + 7] = acadoWorkspace.rk_kkk[run1 + 7] - acadoWorkspace.rk_rhsTemp[7];
acadoWorkspace.rk_b[run1 * 9 + 8] = acadoWorkspace.rk_kkk[run1 + 8] - acadoWorkspace.rk_rhsTemp[8];
}
acado_solve_dim9_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim9_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 9];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 9 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 9 + 2];
acadoWorkspace.rk_kkk[j + 3] += acadoWorkspace.rk_b[j * 9 + 3];
acadoWorkspace.rk_kkk[j + 4] += acadoWorkspace.rk_b[j * 9 + 4];
acadoWorkspace.rk_kkk[j + 5] += acadoWorkspace.rk_b[j * 9 + 5];
acadoWorkspace.rk_kkk[j + 6] += acadoWorkspace.rk_b[j * 9 + 6];
acadoWorkspace.rk_kkk[j + 7] += acadoWorkspace.rk_b[j * 9 + 7];
acadoWorkspace.rk_kkk[j + 8] += acadoWorkspace.rk_b[j * 9 + 8];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 9; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 108 ]) );
for (j = 0; j < 9; ++j)
{
tmp_index1 = (run1 * 9) + (j);
acadoWorkspace.rk_A[tmp_index1 * 9] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 2)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 3)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 4] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 4)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 5] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 5)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 6] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 6)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 7] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 7)];
acadoWorkspace.rk_A[tmp_index1 * 9 + 8] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 108) + (j * 12 + 8)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 9) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 9; ++run1)
{
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_b[i * 9] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1)];
acadoWorkspace.rk_b[i * 9 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 12)];
acadoWorkspace.rk_b[i * 9 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 24)];
acadoWorkspace.rk_b[i * 9 + 3] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 36)];
acadoWorkspace.rk_b[i * 9 + 4] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 48)];
acadoWorkspace.rk_b[i * 9 + 5] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 60)];
acadoWorkspace.rk_b[i * 9 + 6] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 72)];
acadoWorkspace.rk_b[i * 9 + 7] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 84)];
acadoWorkspace.rk_b[i * 9 + 8] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (run1 + 96)];
}
if( 0 == run1 ) {
det = acado_solve_dim9_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim9_perm );
}
 else {
acado_solve_dim9_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim9_perm );
}
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 9];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 9 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 9 + 2];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 9 + 3];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 9 + 4];
acadoWorkspace.rk_diffK[i + 5] = acadoWorkspace.rk_b[i * 9 + 5];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 9 + 6];
acadoWorkspace.rk_diffK[i + 7] = acadoWorkspace.rk_b[i * 9 + 7];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 9 + 8];
}
for (i = 0; i < 9; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 12) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 12) + (run1)] += + acadoWorkspace.rk_diffK[i]*(real_t)6.6666666666666666e-02;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 9; ++j)
{
tmp_index1 = (i * 9) + (j);
tmp_index2 = (run1) + (j * 12);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 108) + (tmp_index2 + 9)];
}
}
acado_solve_dim9_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim9_perm );
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 9];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 9 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 9 + 2];
acadoWorkspace.rk_diffK[i + 3] = acadoWorkspace.rk_b[i * 9 + 3];
acadoWorkspace.rk_diffK[i + 4] = acadoWorkspace.rk_b[i * 9 + 4];
acadoWorkspace.rk_diffK[i + 5] = acadoWorkspace.rk_b[i * 9 + 5];
acadoWorkspace.rk_diffK[i + 6] = acadoWorkspace.rk_b[i * 9 + 6];
acadoWorkspace.rk_diffK[i + 7] = acadoWorkspace.rk_b[i * 9 + 7];
acadoWorkspace.rk_diffK[i + 8] = acadoWorkspace.rk_b[i * 9 + 8];
}
for (i = 0; i < 9; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 12) + (run1 + 9)] = + acadoWorkspace.rk_diffK[i]*(real_t)6.6666666666666666e-02;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)6.6666666666666666e-02;
rk_eta[1] += + acadoWorkspace.rk_kkk[1]*(real_t)6.6666666666666666e-02;
rk_eta[2] += + acadoWorkspace.rk_kkk[2]*(real_t)6.6666666666666666e-02;
rk_eta[3] += + acadoWorkspace.rk_kkk[3]*(real_t)6.6666666666666666e-02;
rk_eta[4] += + acadoWorkspace.rk_kkk[4]*(real_t)6.6666666666666666e-02;
rk_eta[5] += + acadoWorkspace.rk_kkk[5]*(real_t)6.6666666666666666e-02;
rk_eta[6] += + acadoWorkspace.rk_kkk[6]*(real_t)6.6666666666666666e-02;
rk_eta[7] += + acadoWorkspace.rk_kkk[7]*(real_t)6.6666666666666666e-02;
rk_eta[8] += + acadoWorkspace.rk_kkk[8]*(real_t)6.6666666666666666e-02;
for (i = 0; i < 9; ++i)
{
for (j = 0; j < 9; ++j)
{
tmp_index2 = (j) + (i * 9);
rk_eta[tmp_index2 + 9] = acadoWorkspace.rk_diffsNew2[(i * 12) + (j)];
}
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 90] = acadoWorkspace.rk_diffsNew2[(i * 12) + (j + 9)];
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 1.0000000000000000e+00;
}
for (i = 0; i < 9; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



