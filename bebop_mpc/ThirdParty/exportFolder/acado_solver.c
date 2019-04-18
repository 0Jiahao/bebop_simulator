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




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 9];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 9 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 9 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 9 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 9 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 9 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 9 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 9 + 7];
acadoWorkspace.state[8] = acadoVariables.x[lRun1 * 9 + 8];

acadoWorkspace.state[117] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.state[118] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.state[119] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.state[120] = acadoVariables.od[lRun1 * 2];
acadoWorkspace.state[121] = acadoVariables.od[lRun1 * 2 + 1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 9] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 9 + 9];
acadoWorkspace.d[lRun1 * 9 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 9 + 10];
acadoWorkspace.d[lRun1 * 9 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 9 + 11];
acadoWorkspace.d[lRun1 * 9 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 9 + 12];
acadoWorkspace.d[lRun1 * 9 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 9 + 13];
acadoWorkspace.d[lRun1 * 9 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 9 + 14];
acadoWorkspace.d[lRun1 * 9 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 9 + 15];
acadoWorkspace.d[lRun1 * 9 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 9 + 16];
acadoWorkspace.d[lRun1 * 9 + 8] = acadoWorkspace.state[8] - acadoVariables.x[lRun1 * 9 + 17];

acadoWorkspace.evGx[lRun1 * 81] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 81 + 1] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 81 + 2] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 81 + 3] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 81 + 4] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 81 + 5] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 81 + 6] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 81 + 7] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 81 + 8] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 81 + 9] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 81 + 10] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 81 + 11] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 81 + 12] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 81 + 13] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 81 + 14] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 81 + 15] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 81 + 16] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 81 + 17] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 81 + 18] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 81 + 19] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 81 + 20] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 81 + 21] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 81 + 22] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 81 + 23] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 81 + 24] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 81 + 25] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 81 + 26] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 81 + 27] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 81 + 28] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 81 + 29] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 81 + 30] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 81 + 31] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 81 + 32] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 81 + 33] = acadoWorkspace.state[42];
acadoWorkspace.evGx[lRun1 * 81 + 34] = acadoWorkspace.state[43];
acadoWorkspace.evGx[lRun1 * 81 + 35] = acadoWorkspace.state[44];
acadoWorkspace.evGx[lRun1 * 81 + 36] = acadoWorkspace.state[45];
acadoWorkspace.evGx[lRun1 * 81 + 37] = acadoWorkspace.state[46];
acadoWorkspace.evGx[lRun1 * 81 + 38] = acadoWorkspace.state[47];
acadoWorkspace.evGx[lRun1 * 81 + 39] = acadoWorkspace.state[48];
acadoWorkspace.evGx[lRun1 * 81 + 40] = acadoWorkspace.state[49];
acadoWorkspace.evGx[lRun1 * 81 + 41] = acadoWorkspace.state[50];
acadoWorkspace.evGx[lRun1 * 81 + 42] = acadoWorkspace.state[51];
acadoWorkspace.evGx[lRun1 * 81 + 43] = acadoWorkspace.state[52];
acadoWorkspace.evGx[lRun1 * 81 + 44] = acadoWorkspace.state[53];
acadoWorkspace.evGx[lRun1 * 81 + 45] = acadoWorkspace.state[54];
acadoWorkspace.evGx[lRun1 * 81 + 46] = acadoWorkspace.state[55];
acadoWorkspace.evGx[lRun1 * 81 + 47] = acadoWorkspace.state[56];
acadoWorkspace.evGx[lRun1 * 81 + 48] = acadoWorkspace.state[57];
acadoWorkspace.evGx[lRun1 * 81 + 49] = acadoWorkspace.state[58];
acadoWorkspace.evGx[lRun1 * 81 + 50] = acadoWorkspace.state[59];
acadoWorkspace.evGx[lRun1 * 81 + 51] = acadoWorkspace.state[60];
acadoWorkspace.evGx[lRun1 * 81 + 52] = acadoWorkspace.state[61];
acadoWorkspace.evGx[lRun1 * 81 + 53] = acadoWorkspace.state[62];
acadoWorkspace.evGx[lRun1 * 81 + 54] = acadoWorkspace.state[63];
acadoWorkspace.evGx[lRun1 * 81 + 55] = acadoWorkspace.state[64];
acadoWorkspace.evGx[lRun1 * 81 + 56] = acadoWorkspace.state[65];
acadoWorkspace.evGx[lRun1 * 81 + 57] = acadoWorkspace.state[66];
acadoWorkspace.evGx[lRun1 * 81 + 58] = acadoWorkspace.state[67];
acadoWorkspace.evGx[lRun1 * 81 + 59] = acadoWorkspace.state[68];
acadoWorkspace.evGx[lRun1 * 81 + 60] = acadoWorkspace.state[69];
acadoWorkspace.evGx[lRun1 * 81 + 61] = acadoWorkspace.state[70];
acadoWorkspace.evGx[lRun1 * 81 + 62] = acadoWorkspace.state[71];
acadoWorkspace.evGx[lRun1 * 81 + 63] = acadoWorkspace.state[72];
acadoWorkspace.evGx[lRun1 * 81 + 64] = acadoWorkspace.state[73];
acadoWorkspace.evGx[lRun1 * 81 + 65] = acadoWorkspace.state[74];
acadoWorkspace.evGx[lRun1 * 81 + 66] = acadoWorkspace.state[75];
acadoWorkspace.evGx[lRun1 * 81 + 67] = acadoWorkspace.state[76];
acadoWorkspace.evGx[lRun1 * 81 + 68] = acadoWorkspace.state[77];
acadoWorkspace.evGx[lRun1 * 81 + 69] = acadoWorkspace.state[78];
acadoWorkspace.evGx[lRun1 * 81 + 70] = acadoWorkspace.state[79];
acadoWorkspace.evGx[lRun1 * 81 + 71] = acadoWorkspace.state[80];
acadoWorkspace.evGx[lRun1 * 81 + 72] = acadoWorkspace.state[81];
acadoWorkspace.evGx[lRun1 * 81 + 73] = acadoWorkspace.state[82];
acadoWorkspace.evGx[lRun1 * 81 + 74] = acadoWorkspace.state[83];
acadoWorkspace.evGx[lRun1 * 81 + 75] = acadoWorkspace.state[84];
acadoWorkspace.evGx[lRun1 * 81 + 76] = acadoWorkspace.state[85];
acadoWorkspace.evGx[lRun1 * 81 + 77] = acadoWorkspace.state[86];
acadoWorkspace.evGx[lRun1 * 81 + 78] = acadoWorkspace.state[87];
acadoWorkspace.evGx[lRun1 * 81 + 79] = acadoWorkspace.state[88];
acadoWorkspace.evGx[lRun1 * 81 + 80] = acadoWorkspace.state[89];

acadoWorkspace.evGu[lRun1 * 27] = acadoWorkspace.state[90];
acadoWorkspace.evGu[lRun1 * 27 + 1] = acadoWorkspace.state[91];
acadoWorkspace.evGu[lRun1 * 27 + 2] = acadoWorkspace.state[92];
acadoWorkspace.evGu[lRun1 * 27 + 3] = acadoWorkspace.state[93];
acadoWorkspace.evGu[lRun1 * 27 + 4] = acadoWorkspace.state[94];
acadoWorkspace.evGu[lRun1 * 27 + 5] = acadoWorkspace.state[95];
acadoWorkspace.evGu[lRun1 * 27 + 6] = acadoWorkspace.state[96];
acadoWorkspace.evGu[lRun1 * 27 + 7] = acadoWorkspace.state[97];
acadoWorkspace.evGu[lRun1 * 27 + 8] = acadoWorkspace.state[98];
acadoWorkspace.evGu[lRun1 * 27 + 9] = acadoWorkspace.state[99];
acadoWorkspace.evGu[lRun1 * 27 + 10] = acadoWorkspace.state[100];
acadoWorkspace.evGu[lRun1 * 27 + 11] = acadoWorkspace.state[101];
acadoWorkspace.evGu[lRun1 * 27 + 12] = acadoWorkspace.state[102];
acadoWorkspace.evGu[lRun1 * 27 + 13] = acadoWorkspace.state[103];
acadoWorkspace.evGu[lRun1 * 27 + 14] = acadoWorkspace.state[104];
acadoWorkspace.evGu[lRun1 * 27 + 15] = acadoWorkspace.state[105];
acadoWorkspace.evGu[lRun1 * 27 + 16] = acadoWorkspace.state[106];
acadoWorkspace.evGu[lRun1 * 27 + 17] = acadoWorkspace.state[107];
acadoWorkspace.evGu[lRun1 * 27 + 18] = acadoWorkspace.state[108];
acadoWorkspace.evGu[lRun1 * 27 + 19] = acadoWorkspace.state[109];
acadoWorkspace.evGu[lRun1 * 27 + 20] = acadoWorkspace.state[110];
acadoWorkspace.evGu[lRun1 * 27 + 21] = acadoWorkspace.state[111];
acadoWorkspace.evGu[lRun1 * 27 + 22] = acadoWorkspace.state[112];
acadoWorkspace.evGu[lRun1 * 27 + 23] = acadoWorkspace.state[113];
acadoWorkspace.evGu[lRun1 * 27 + 24] = acadoWorkspace.state[114];
acadoWorkspace.evGu[lRun1 * 27 + 25] = acadoWorkspace.state[115];
acadoWorkspace.evGu[lRun1 * 27 + 26] = acadoWorkspace.state[116];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;

/* Compute outputs: */
out[0] = u[0];
out[1] = u[1];
out[2] = u[2];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[runObj * 3 + 2];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 2];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 3] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 3 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 3 + 2] = acadoWorkspace.objValueOut[2];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[270];
acadoWorkspace.objValueIn[1] = acadoVariables.x[271];
acadoWorkspace.objValueIn[2] = acadoVariables.x[272];
acadoWorkspace.objValueIn[3] = acadoVariables.x[273];
acadoWorkspace.objValueIn[4] = acadoVariables.x[274];
acadoWorkspace.objValueIn[5] = acadoVariables.x[275];
acadoWorkspace.objValueIn[6] = acadoVariables.x[276];
acadoWorkspace.objValueIn[7] = acadoVariables.x[277];
acadoWorkspace.objValueIn[8] = acadoVariables.x[278];
acadoWorkspace.objValueIn[9] = acadoVariables.od[60];
acadoWorkspace.objValueIn[10] = acadoVariables.od[61];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9] + Gx1[4]*Gu1[12] + Gx1[5]*Gu1[15] + Gx1[6]*Gu1[18] + Gx1[7]*Gu1[21] + Gx1[8]*Gu1[24];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10] + Gx1[4]*Gu1[13] + Gx1[5]*Gu1[16] + Gx1[6]*Gu1[19] + Gx1[7]*Gu1[22] + Gx1[8]*Gu1[25];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11] + Gx1[4]*Gu1[14] + Gx1[5]*Gu1[17] + Gx1[6]*Gu1[20] + Gx1[7]*Gu1[23] + Gx1[8]*Gu1[26];
Gu2[3] = + Gx1[9]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[12] + Gx1[14]*Gu1[15] + Gx1[15]*Gu1[18] + Gx1[16]*Gu1[21] + Gx1[17]*Gu1[24];
Gu2[4] = + Gx1[9]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[10] + Gx1[13]*Gu1[13] + Gx1[14]*Gu1[16] + Gx1[15]*Gu1[19] + Gx1[16]*Gu1[22] + Gx1[17]*Gu1[25];
Gu2[5] = + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[8] + Gx1[12]*Gu1[11] + Gx1[13]*Gu1[14] + Gx1[14]*Gu1[17] + Gx1[15]*Gu1[20] + Gx1[16]*Gu1[23] + Gx1[17]*Gu1[26];
Gu2[6] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[21]*Gu1[9] + Gx1[22]*Gu1[12] + Gx1[23]*Gu1[15] + Gx1[24]*Gu1[18] + Gx1[25]*Gu1[21] + Gx1[26]*Gu1[24];
Gu2[7] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[21]*Gu1[10] + Gx1[22]*Gu1[13] + Gx1[23]*Gu1[16] + Gx1[24]*Gu1[19] + Gx1[25]*Gu1[22] + Gx1[26]*Gu1[25];
Gu2[8] = + Gx1[18]*Gu1[2] + Gx1[19]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[11] + Gx1[22]*Gu1[14] + Gx1[23]*Gu1[17] + Gx1[24]*Gu1[20] + Gx1[25]*Gu1[23] + Gx1[26]*Gu1[26];
Gu2[9] = + Gx1[27]*Gu1[0] + Gx1[28]*Gu1[3] + Gx1[29]*Gu1[6] + Gx1[30]*Gu1[9] + Gx1[31]*Gu1[12] + Gx1[32]*Gu1[15] + Gx1[33]*Gu1[18] + Gx1[34]*Gu1[21] + Gx1[35]*Gu1[24];
Gu2[10] = + Gx1[27]*Gu1[1] + Gx1[28]*Gu1[4] + Gx1[29]*Gu1[7] + Gx1[30]*Gu1[10] + Gx1[31]*Gu1[13] + Gx1[32]*Gu1[16] + Gx1[33]*Gu1[19] + Gx1[34]*Gu1[22] + Gx1[35]*Gu1[25];
Gu2[11] = + Gx1[27]*Gu1[2] + Gx1[28]*Gu1[5] + Gx1[29]*Gu1[8] + Gx1[30]*Gu1[11] + Gx1[31]*Gu1[14] + Gx1[32]*Gu1[17] + Gx1[33]*Gu1[20] + Gx1[34]*Gu1[23] + Gx1[35]*Gu1[26];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[3] + Gx1[38]*Gu1[6] + Gx1[39]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[41]*Gu1[15] + Gx1[42]*Gu1[18] + Gx1[43]*Gu1[21] + Gx1[44]*Gu1[24];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[7] + Gx1[39]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[41]*Gu1[16] + Gx1[42]*Gu1[19] + Gx1[43]*Gu1[22] + Gx1[44]*Gu1[25];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[41]*Gu1[17] + Gx1[42]*Gu1[20] + Gx1[43]*Gu1[23] + Gx1[44]*Gu1[26];
Gu2[15] = + Gx1[45]*Gu1[0] + Gx1[46]*Gu1[3] + Gx1[47]*Gu1[6] + Gx1[48]*Gu1[9] + Gx1[49]*Gu1[12] + Gx1[50]*Gu1[15] + Gx1[51]*Gu1[18] + Gx1[52]*Gu1[21] + Gx1[53]*Gu1[24];
Gu2[16] = + Gx1[45]*Gu1[1] + Gx1[46]*Gu1[4] + Gx1[47]*Gu1[7] + Gx1[48]*Gu1[10] + Gx1[49]*Gu1[13] + Gx1[50]*Gu1[16] + Gx1[51]*Gu1[19] + Gx1[52]*Gu1[22] + Gx1[53]*Gu1[25];
Gu2[17] = + Gx1[45]*Gu1[2] + Gx1[46]*Gu1[5] + Gx1[47]*Gu1[8] + Gx1[48]*Gu1[11] + Gx1[49]*Gu1[14] + Gx1[50]*Gu1[17] + Gx1[51]*Gu1[20] + Gx1[52]*Gu1[23] + Gx1[53]*Gu1[26];
Gu2[18] = + Gx1[54]*Gu1[0] + Gx1[55]*Gu1[3] + Gx1[56]*Gu1[6] + Gx1[57]*Gu1[9] + Gx1[58]*Gu1[12] + Gx1[59]*Gu1[15] + Gx1[60]*Gu1[18] + Gx1[61]*Gu1[21] + Gx1[62]*Gu1[24];
Gu2[19] = + Gx1[54]*Gu1[1] + Gx1[55]*Gu1[4] + Gx1[56]*Gu1[7] + Gx1[57]*Gu1[10] + Gx1[58]*Gu1[13] + Gx1[59]*Gu1[16] + Gx1[60]*Gu1[19] + Gx1[61]*Gu1[22] + Gx1[62]*Gu1[25];
Gu2[20] = + Gx1[54]*Gu1[2] + Gx1[55]*Gu1[5] + Gx1[56]*Gu1[8] + Gx1[57]*Gu1[11] + Gx1[58]*Gu1[14] + Gx1[59]*Gu1[17] + Gx1[60]*Gu1[20] + Gx1[61]*Gu1[23] + Gx1[62]*Gu1[26];
Gu2[21] = + Gx1[63]*Gu1[0] + Gx1[64]*Gu1[3] + Gx1[65]*Gu1[6] + Gx1[66]*Gu1[9] + Gx1[67]*Gu1[12] + Gx1[68]*Gu1[15] + Gx1[69]*Gu1[18] + Gx1[70]*Gu1[21] + Gx1[71]*Gu1[24];
Gu2[22] = + Gx1[63]*Gu1[1] + Gx1[64]*Gu1[4] + Gx1[65]*Gu1[7] + Gx1[66]*Gu1[10] + Gx1[67]*Gu1[13] + Gx1[68]*Gu1[16] + Gx1[69]*Gu1[19] + Gx1[70]*Gu1[22] + Gx1[71]*Gu1[25];
Gu2[23] = + Gx1[63]*Gu1[2] + Gx1[64]*Gu1[5] + Gx1[65]*Gu1[8] + Gx1[66]*Gu1[11] + Gx1[67]*Gu1[14] + Gx1[68]*Gu1[17] + Gx1[69]*Gu1[20] + Gx1[70]*Gu1[23] + Gx1[71]*Gu1[26];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[3] + Gx1[74]*Gu1[6] + Gx1[75]*Gu1[9] + Gx1[76]*Gu1[12] + Gx1[77]*Gu1[15] + Gx1[78]*Gu1[18] + Gx1[79]*Gu1[21] + Gx1[80]*Gu1[24];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[7] + Gx1[75]*Gu1[10] + Gx1[76]*Gu1[13] + Gx1[77]*Gu1[16] + Gx1[78]*Gu1[19] + Gx1[79]*Gu1[22] + Gx1[80]*Gu1[25];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[11] + Gx1[76]*Gu1[14] + Gx1[77]*Gu1[17] + Gx1[78]*Gu1[20] + Gx1[79]*Gu1[23] + Gx1[80]*Gu1[26];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 270) + (iCol * 3)] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24];
acadoWorkspace.H[(iRow * 270) + (iCol * 3 + 1)] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25];
acadoWorkspace.H[(iRow * 270) + (iCol * 3 + 2)] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26];
acadoWorkspace.H[(iRow * 270 + 90) + (iCol * 3)] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24];
acadoWorkspace.H[(iRow * 270 + 90) + (iCol * 3 + 1)] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25];
acadoWorkspace.H[(iRow * 270 + 90) + (iCol * 3 + 2)] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26];
acadoWorkspace.H[(iRow * 270 + 180) + (iCol * 3)] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24];
acadoWorkspace.H[(iRow * 270 + 180) + (iCol * 3 + 1)] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25];
acadoWorkspace.H[(iRow * 270 + 180) + (iCol * 3 + 2)] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26];
}

void acado_multBTW1_R1( real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 273] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + (real_t)1.0000000000000000e+00;
acadoWorkspace.H[iRow * 273 + 1] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25];
acadoWorkspace.H[iRow * 273 + 2] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26];
acadoWorkspace.H[iRow * 273 + 90] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24];
acadoWorkspace.H[iRow * 273 + 91] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + (real_t)1.0000000000000000e+00;
acadoWorkspace.H[iRow * 273 + 92] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26];
acadoWorkspace.H[iRow * 273 + 180] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24];
acadoWorkspace.H[iRow * 273 + 181] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25];
acadoWorkspace.H[iRow * 273 + 182] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + (real_t)1.0000000000000000e+00;
acadoWorkspace.H[iRow * 273] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 273 + 91] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 273 + 182] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[9]*Gu1[3] + Gx1[18]*Gu1[6] + Gx1[27]*Gu1[9] + Gx1[36]*Gu1[12] + Gx1[45]*Gu1[15] + Gx1[54]*Gu1[18] + Gx1[63]*Gu1[21] + Gx1[72]*Gu1[24];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[9]*Gu1[4] + Gx1[18]*Gu1[7] + Gx1[27]*Gu1[10] + Gx1[36]*Gu1[13] + Gx1[45]*Gu1[16] + Gx1[54]*Gu1[19] + Gx1[63]*Gu1[22] + Gx1[72]*Gu1[25];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[9]*Gu1[5] + Gx1[18]*Gu1[8] + Gx1[27]*Gu1[11] + Gx1[36]*Gu1[14] + Gx1[45]*Gu1[17] + Gx1[54]*Gu1[20] + Gx1[63]*Gu1[23] + Gx1[72]*Gu1[26];
Gu2[3] = + Gx1[1]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[19]*Gu1[6] + Gx1[28]*Gu1[9] + Gx1[37]*Gu1[12] + Gx1[46]*Gu1[15] + Gx1[55]*Gu1[18] + Gx1[64]*Gu1[21] + Gx1[73]*Gu1[24];
Gu2[4] = + Gx1[1]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[19]*Gu1[7] + Gx1[28]*Gu1[10] + Gx1[37]*Gu1[13] + Gx1[46]*Gu1[16] + Gx1[55]*Gu1[19] + Gx1[64]*Gu1[22] + Gx1[73]*Gu1[25];
Gu2[5] = + Gx1[1]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[19]*Gu1[8] + Gx1[28]*Gu1[11] + Gx1[37]*Gu1[14] + Gx1[46]*Gu1[17] + Gx1[55]*Gu1[20] + Gx1[64]*Gu1[23] + Gx1[73]*Gu1[26];
Gu2[6] = + Gx1[2]*Gu1[0] + Gx1[11]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[29]*Gu1[9] + Gx1[38]*Gu1[12] + Gx1[47]*Gu1[15] + Gx1[56]*Gu1[18] + Gx1[65]*Gu1[21] + Gx1[74]*Gu1[24];
Gu2[7] = + Gx1[2]*Gu1[1] + Gx1[11]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[29]*Gu1[10] + Gx1[38]*Gu1[13] + Gx1[47]*Gu1[16] + Gx1[56]*Gu1[19] + Gx1[65]*Gu1[22] + Gx1[74]*Gu1[25];
Gu2[8] = + Gx1[2]*Gu1[2] + Gx1[11]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[29]*Gu1[11] + Gx1[38]*Gu1[14] + Gx1[47]*Gu1[17] + Gx1[56]*Gu1[20] + Gx1[65]*Gu1[23] + Gx1[74]*Gu1[26];
Gu2[9] = + Gx1[3]*Gu1[0] + Gx1[12]*Gu1[3] + Gx1[21]*Gu1[6] + Gx1[30]*Gu1[9] + Gx1[39]*Gu1[12] + Gx1[48]*Gu1[15] + Gx1[57]*Gu1[18] + Gx1[66]*Gu1[21] + Gx1[75]*Gu1[24];
Gu2[10] = + Gx1[3]*Gu1[1] + Gx1[12]*Gu1[4] + Gx1[21]*Gu1[7] + Gx1[30]*Gu1[10] + Gx1[39]*Gu1[13] + Gx1[48]*Gu1[16] + Gx1[57]*Gu1[19] + Gx1[66]*Gu1[22] + Gx1[75]*Gu1[25];
Gu2[11] = + Gx1[3]*Gu1[2] + Gx1[12]*Gu1[5] + Gx1[21]*Gu1[8] + Gx1[30]*Gu1[11] + Gx1[39]*Gu1[14] + Gx1[48]*Gu1[17] + Gx1[57]*Gu1[20] + Gx1[66]*Gu1[23] + Gx1[75]*Gu1[26];
Gu2[12] = + Gx1[4]*Gu1[0] + Gx1[13]*Gu1[3] + Gx1[22]*Gu1[6] + Gx1[31]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[49]*Gu1[15] + Gx1[58]*Gu1[18] + Gx1[67]*Gu1[21] + Gx1[76]*Gu1[24];
Gu2[13] = + Gx1[4]*Gu1[1] + Gx1[13]*Gu1[4] + Gx1[22]*Gu1[7] + Gx1[31]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[49]*Gu1[16] + Gx1[58]*Gu1[19] + Gx1[67]*Gu1[22] + Gx1[76]*Gu1[25];
Gu2[14] = + Gx1[4]*Gu1[2] + Gx1[13]*Gu1[5] + Gx1[22]*Gu1[8] + Gx1[31]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[49]*Gu1[17] + Gx1[58]*Gu1[20] + Gx1[67]*Gu1[23] + Gx1[76]*Gu1[26];
Gu2[15] = + Gx1[5]*Gu1[0] + Gx1[14]*Gu1[3] + Gx1[23]*Gu1[6] + Gx1[32]*Gu1[9] + Gx1[41]*Gu1[12] + Gx1[50]*Gu1[15] + Gx1[59]*Gu1[18] + Gx1[68]*Gu1[21] + Gx1[77]*Gu1[24];
Gu2[16] = + Gx1[5]*Gu1[1] + Gx1[14]*Gu1[4] + Gx1[23]*Gu1[7] + Gx1[32]*Gu1[10] + Gx1[41]*Gu1[13] + Gx1[50]*Gu1[16] + Gx1[59]*Gu1[19] + Gx1[68]*Gu1[22] + Gx1[77]*Gu1[25];
Gu2[17] = + Gx1[5]*Gu1[2] + Gx1[14]*Gu1[5] + Gx1[23]*Gu1[8] + Gx1[32]*Gu1[11] + Gx1[41]*Gu1[14] + Gx1[50]*Gu1[17] + Gx1[59]*Gu1[20] + Gx1[68]*Gu1[23] + Gx1[77]*Gu1[26];
Gu2[18] = + Gx1[6]*Gu1[0] + Gx1[15]*Gu1[3] + Gx1[24]*Gu1[6] + Gx1[33]*Gu1[9] + Gx1[42]*Gu1[12] + Gx1[51]*Gu1[15] + Gx1[60]*Gu1[18] + Gx1[69]*Gu1[21] + Gx1[78]*Gu1[24];
Gu2[19] = + Gx1[6]*Gu1[1] + Gx1[15]*Gu1[4] + Gx1[24]*Gu1[7] + Gx1[33]*Gu1[10] + Gx1[42]*Gu1[13] + Gx1[51]*Gu1[16] + Gx1[60]*Gu1[19] + Gx1[69]*Gu1[22] + Gx1[78]*Gu1[25];
Gu2[20] = + Gx1[6]*Gu1[2] + Gx1[15]*Gu1[5] + Gx1[24]*Gu1[8] + Gx1[33]*Gu1[11] + Gx1[42]*Gu1[14] + Gx1[51]*Gu1[17] + Gx1[60]*Gu1[20] + Gx1[69]*Gu1[23] + Gx1[78]*Gu1[26];
Gu2[21] = + Gx1[7]*Gu1[0] + Gx1[16]*Gu1[3] + Gx1[25]*Gu1[6] + Gx1[34]*Gu1[9] + Gx1[43]*Gu1[12] + Gx1[52]*Gu1[15] + Gx1[61]*Gu1[18] + Gx1[70]*Gu1[21] + Gx1[79]*Gu1[24];
Gu2[22] = + Gx1[7]*Gu1[1] + Gx1[16]*Gu1[4] + Gx1[25]*Gu1[7] + Gx1[34]*Gu1[10] + Gx1[43]*Gu1[13] + Gx1[52]*Gu1[16] + Gx1[61]*Gu1[19] + Gx1[70]*Gu1[22] + Gx1[79]*Gu1[25];
Gu2[23] = + Gx1[7]*Gu1[2] + Gx1[16]*Gu1[5] + Gx1[25]*Gu1[8] + Gx1[34]*Gu1[11] + Gx1[43]*Gu1[14] + Gx1[52]*Gu1[17] + Gx1[61]*Gu1[20] + Gx1[70]*Gu1[23] + Gx1[79]*Gu1[26];
Gu2[24] = + Gx1[8]*Gu1[0] + Gx1[17]*Gu1[3] + Gx1[26]*Gu1[6] + Gx1[35]*Gu1[9] + Gx1[44]*Gu1[12] + Gx1[53]*Gu1[15] + Gx1[62]*Gu1[18] + Gx1[71]*Gu1[21] + Gx1[80]*Gu1[24];
Gu2[25] = + Gx1[8]*Gu1[1] + Gx1[17]*Gu1[4] + Gx1[26]*Gu1[7] + Gx1[35]*Gu1[10] + Gx1[44]*Gu1[13] + Gx1[53]*Gu1[16] + Gx1[62]*Gu1[19] + Gx1[71]*Gu1[22] + Gx1[80]*Gu1[25];
Gu2[26] = + Gx1[8]*Gu1[2] + Gx1[17]*Gu1[5] + Gx1[26]*Gu1[8] + Gx1[35]*Gu1[11] + Gx1[44]*Gu1[14] + Gx1[53]*Gu1[17] + Gx1[62]*Gu1[20] + Gx1[71]*Gu1[23] + Gx1[80]*Gu1[26];
}

void acado_multQEW2( real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Gu2[0];
Gu3[1] = + Gu2[1];
Gu3[2] = + Gu2[2];
Gu3[3] = + Gu2[3];
Gu3[4] = + Gu2[4];
Gu3[5] = + Gu2[5];
Gu3[6] = + Gu2[6];
Gu3[7] = + Gu2[7];
Gu3[8] = + Gu2[8];
Gu3[9] = + Gu2[9];
Gu3[10] = + Gu2[10];
Gu3[11] = + Gu2[11];
Gu3[12] = + Gu2[12];
Gu3[13] = + Gu2[13];
Gu3[14] = + Gu2[14];
Gu3[15] = + Gu2[15];
Gu3[16] = + Gu2[16];
Gu3[17] = + Gu2[17];
Gu3[18] = + Gu2[18];
Gu3[19] = + Gu2[19];
Gu3[20] = + Gu2[20];
Gu3[21] = + Gu2[21];
Gu3[22] = + Gu2[22];
Gu3[23] = + Gu2[23];
Gu3[24] = + Gu2[24];
Gu3[25] = + Gu2[25];
Gu3[26] = + Gu2[26];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[9]*w11[1] + Gx1[18]*w11[2] + Gx1[27]*w11[3] + Gx1[36]*w11[4] + Gx1[45]*w11[5] + Gx1[54]*w11[6] + Gx1[63]*w11[7] + Gx1[72]*w11[8] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[10]*w11[1] + Gx1[19]*w11[2] + Gx1[28]*w11[3] + Gx1[37]*w11[4] + Gx1[46]*w11[5] + Gx1[55]*w11[6] + Gx1[64]*w11[7] + Gx1[73]*w11[8] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[11]*w11[1] + Gx1[20]*w11[2] + Gx1[29]*w11[3] + Gx1[38]*w11[4] + Gx1[47]*w11[5] + Gx1[56]*w11[6] + Gx1[65]*w11[7] + Gx1[74]*w11[8] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[12]*w11[1] + Gx1[21]*w11[2] + Gx1[30]*w11[3] + Gx1[39]*w11[4] + Gx1[48]*w11[5] + Gx1[57]*w11[6] + Gx1[66]*w11[7] + Gx1[75]*w11[8] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[13]*w11[1] + Gx1[22]*w11[2] + Gx1[31]*w11[3] + Gx1[40]*w11[4] + Gx1[49]*w11[5] + Gx1[58]*w11[6] + Gx1[67]*w11[7] + Gx1[76]*w11[8] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[14]*w11[1] + Gx1[23]*w11[2] + Gx1[32]*w11[3] + Gx1[41]*w11[4] + Gx1[50]*w11[5] + Gx1[59]*w11[6] + Gx1[68]*w11[7] + Gx1[77]*w11[8] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[15]*w11[1] + Gx1[24]*w11[2] + Gx1[33]*w11[3] + Gx1[42]*w11[4] + Gx1[51]*w11[5] + Gx1[60]*w11[6] + Gx1[69]*w11[7] + Gx1[78]*w11[8] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[16]*w11[1] + Gx1[25]*w11[2] + Gx1[34]*w11[3] + Gx1[43]*w11[4] + Gx1[52]*w11[5] + Gx1[61]*w11[6] + Gx1[70]*w11[7] + Gx1[79]*w11[8] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[17]*w11[1] + Gx1[26]*w11[2] + Gx1[35]*w11[3] + Gx1[44]*w11[4] + Gx1[53]*w11[5] + Gx1[62]*w11[6] + Gx1[71]*w11[7] + Gx1[80]*w11[8] + w12[8];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5] + Gu1[18]*w11[6] + Gu1[21]*w11[7] + Gu1[24]*w11[8];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5] + Gu1[19]*w11[6] + Gu1[22]*w11[7] + Gu1[25]*w11[8];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5] + Gu1[20]*w11[6] + Gu1[23]*w11[7] + Gu1[26]*w11[8];
}

void acado_macQSbarW2( real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + w12[0];
w13[1] = + w12[1];
w13[2] = + w12[2];
w13[3] = + w12[3];
w13[4] = + w12[4];
w13[5] = + w12[5];
w13[6] = + w12[6];
w13[7] = + w12[7];
w13[8] = + w12[8];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8];
w12[1] += + Gx1[9]*w11[0] + Gx1[10]*w11[1] + Gx1[11]*w11[2] + Gx1[12]*w11[3] + Gx1[13]*w11[4] + Gx1[14]*w11[5] + Gx1[15]*w11[6] + Gx1[16]*w11[7] + Gx1[17]*w11[8];
w12[2] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8];
w12[3] += + Gx1[27]*w11[0] + Gx1[28]*w11[1] + Gx1[29]*w11[2] + Gx1[30]*w11[3] + Gx1[31]*w11[4] + Gx1[32]*w11[5] + Gx1[33]*w11[6] + Gx1[34]*w11[7] + Gx1[35]*w11[8];
w12[4] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8];
w12[5] += + Gx1[45]*w11[0] + Gx1[46]*w11[1] + Gx1[47]*w11[2] + Gx1[48]*w11[3] + Gx1[49]*w11[4] + Gx1[50]*w11[5] + Gx1[51]*w11[6] + Gx1[52]*w11[7] + Gx1[53]*w11[8];
w12[6] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8];
w12[7] += + Gx1[63]*w11[0] + Gx1[64]*w11[1] + Gx1[65]*w11[2] + Gx1[66]*w11[3] + Gx1[67]*w11[4] + Gx1[68]*w11[5] + Gx1[69]*w11[6] + Gx1[70]*w11[7] + Gx1[71]*w11[8];
w12[8] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8];
w12[1] += + Gx1[9]*w11[0] + Gx1[10]*w11[1] + Gx1[11]*w11[2] + Gx1[12]*w11[3] + Gx1[13]*w11[4] + Gx1[14]*w11[5] + Gx1[15]*w11[6] + Gx1[16]*w11[7] + Gx1[17]*w11[8];
w12[2] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8];
w12[3] += + Gx1[27]*w11[0] + Gx1[28]*w11[1] + Gx1[29]*w11[2] + Gx1[30]*w11[3] + Gx1[31]*w11[4] + Gx1[32]*w11[5] + Gx1[33]*w11[6] + Gx1[34]*w11[7] + Gx1[35]*w11[8];
w12[4] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8];
w12[5] += + Gx1[45]*w11[0] + Gx1[46]*w11[1] + Gx1[47]*w11[2] + Gx1[48]*w11[3] + Gx1[49]*w11[4] + Gx1[50]*w11[5] + Gx1[51]*w11[6] + Gx1[52]*w11[7] + Gx1[53]*w11[8];
w12[6] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8];
w12[7] += + Gx1[63]*w11[0] + Gx1[64]*w11[1] + Gx1[65]*w11[2] + Gx1[66]*w11[3] + Gx1[67]*w11[4] + Gx1[68]*w11[5] + Gx1[69]*w11[6] + Gx1[70]*w11[7] + Gx1[71]*w11[8];
w12[8] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2];
w12[1] += + Gu1[3]*U1[0] + Gu1[4]*U1[1] + Gu1[5]*U1[2];
w12[2] += + Gu1[6]*U1[0] + Gu1[7]*U1[1] + Gu1[8]*U1[2];
w12[3] += + Gu1[9]*U1[0] + Gu1[10]*U1[1] + Gu1[11]*U1[2];
w12[4] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2];
w12[5] += + Gu1[15]*U1[0] + Gu1[16]*U1[1] + Gu1[17]*U1[2];
w12[6] += + Gu1[18]*U1[0] + Gu1[19]*U1[1] + Gu1[20]*U1[2];
w12[7] += + Gu1[21]*U1[0] + Gu1[22]*U1[1] + Gu1[23]*U1[2];
w12[8] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 270) + (iCol * 3)] = acadoWorkspace.H[(iCol * 270) + (iRow * 3)];
acadoWorkspace.H[(iRow * 270) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 270 + 90) + (iRow * 3)];
acadoWorkspace.H[(iRow * 270) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 270 + 180) + (iRow * 3)];
acadoWorkspace.H[(iRow * 270 + 90) + (iCol * 3)] = acadoWorkspace.H[(iCol * 270) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 270 + 90) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 270 + 90) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 270 + 90) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 270 + 180) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 270 + 180) + (iCol * 3)] = acadoWorkspace.H[(iCol * 270) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 270 + 180) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 270 + 90) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 270 + 180) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 270 + 180) + (iRow * 3 + 2)];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = +Dy1[0];
RDy1[1] = +Dy1[1];
RDy1[2] = +Dy1[2];
}

void acado_multQDy( real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = 0.0;
;
QDy1[1] = 0.0;
;
QDy1[2] = 0.0;
;
QDy1[3] = 0.0;
;
QDy1[4] = 0.0;
;
QDy1[5] = 0.0;
;
QDy1[6] = 0.0;
;
QDy1[7] = 0.0;
;
QDy1[8] = 0.0;
;
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = + (real_t)1.0000000000000000e+01*Gx1[0];
Gx2[1] = + (real_t)1.0000000000000000e+01*Gx1[1];
Gx2[2] = + (real_t)1.0000000000000000e+01*Gx1[2];
Gx2[3] = + (real_t)1.0000000000000000e+01*Gx1[3];
Gx2[4] = + (real_t)1.0000000000000000e+01*Gx1[4];
Gx2[5] = + (real_t)1.0000000000000000e+01*Gx1[5];
Gx2[6] = + (real_t)1.0000000000000000e+01*Gx1[6];
Gx2[7] = + (real_t)1.0000000000000000e+01*Gx1[7];
Gx2[8] = + (real_t)1.0000000000000000e+01*Gx1[8];
Gx2[9] = + (real_t)1.0000000000000000e+01*Gx1[9];
Gx2[10] = + (real_t)1.0000000000000000e+01*Gx1[10];
Gx2[11] = + (real_t)1.0000000000000000e+01*Gx1[11];
Gx2[12] = + (real_t)1.0000000000000000e+01*Gx1[12];
Gx2[13] = + (real_t)1.0000000000000000e+01*Gx1[13];
Gx2[14] = + (real_t)1.0000000000000000e+01*Gx1[14];
Gx2[15] = + (real_t)1.0000000000000000e+01*Gx1[15];
Gx2[16] = + (real_t)1.0000000000000000e+01*Gx1[16];
Gx2[17] = + (real_t)1.0000000000000000e+01*Gx1[17];
Gx2[18] = + (real_t)1.0000000000000000e+01*Gx1[18];
Gx2[19] = + (real_t)1.0000000000000000e+01*Gx1[19];
Gx2[20] = + (real_t)1.0000000000000000e+01*Gx1[20];
Gx2[21] = + (real_t)1.0000000000000000e+01*Gx1[21];
Gx2[22] = + (real_t)1.0000000000000000e+01*Gx1[22];
Gx2[23] = + (real_t)1.0000000000000000e+01*Gx1[23];
Gx2[24] = + (real_t)1.0000000000000000e+01*Gx1[24];
Gx2[25] = + (real_t)1.0000000000000000e+01*Gx1[25];
Gx2[26] = + (real_t)1.0000000000000000e+01*Gx1[26];
Gx2[27] = 0.0;
;
Gx2[28] = 0.0;
;
Gx2[29] = 0.0;
;
Gx2[30] = 0.0;
;
Gx2[31] = 0.0;
;
Gx2[32] = 0.0;
;
Gx2[33] = 0.0;
;
Gx2[34] = 0.0;
;
Gx2[35] = 0.0;
;
Gx2[36] = 0.0;
;
Gx2[37] = 0.0;
;
Gx2[38] = 0.0;
;
Gx2[39] = 0.0;
;
Gx2[40] = 0.0;
;
Gx2[41] = 0.0;
;
Gx2[42] = 0.0;
;
Gx2[43] = 0.0;
;
Gx2[44] = 0.0;
;
Gx2[45] = 0.0;
;
Gx2[46] = 0.0;
;
Gx2[47] = 0.0;
;
Gx2[48] = 0.0;
;
Gx2[49] = 0.0;
;
Gx2[50] = 0.0;
;
Gx2[51] = 0.0;
;
Gx2[52] = 0.0;
;
Gx2[53] = 0.0;
;
Gx2[54] = 0.0;
;
Gx2[55] = 0.0;
;
Gx2[56] = 0.0;
;
Gx2[57] = 0.0;
;
Gx2[58] = 0.0;
;
Gx2[59] = 0.0;
;
Gx2[60] = 0.0;
;
Gx2[61] = 0.0;
;
Gx2[62] = 0.0;
;
Gx2[63] = 0.0;
;
Gx2[64] = 0.0;
;
Gx2[65] = 0.0;
;
Gx2[66] = 0.0;
;
Gx2[67] = 0.0;
;
Gx2[68] = 0.0;
;
Gx2[69] = 0.0;
;
Gx2[70] = 0.0;
;
Gx2[71] = 0.0;
;
Gx2[72] = 0.0;
;
Gx2[73] = 0.0;
;
Gx2[74] = 0.0;
;
Gx2[75] = 0.0;
;
Gx2[76] = 0.0;
;
Gx2[77] = 0.0;
;
Gx2[78] = 0.0;
;
Gx2[79] = 0.0;
;
Gx2[80] = 0.0;
;
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + (real_t)1.0000000000000000e+01*Gu1[0];
Gu2[1] = + (real_t)1.0000000000000000e+01*Gu1[1];
Gu2[2] = + (real_t)1.0000000000000000e+01*Gu1[2];
Gu2[3] = + (real_t)1.0000000000000000e+01*Gu1[3];
Gu2[4] = + (real_t)1.0000000000000000e+01*Gu1[4];
Gu2[5] = + (real_t)1.0000000000000000e+01*Gu1[5];
Gu2[6] = + (real_t)1.0000000000000000e+01*Gu1[6];
Gu2[7] = + (real_t)1.0000000000000000e+01*Gu1[7];
Gu2[8] = + (real_t)1.0000000000000000e+01*Gu1[8];
Gu2[9] = 0.0;
;
Gu2[10] = 0.0;
;
Gu2[11] = 0.0;
;
Gu2[12] = 0.0;
;
Gu2[13] = 0.0;
;
Gu2[14] = 0.0;
;
Gu2[15] = 0.0;
;
Gu2[16] = 0.0;
;
Gu2[17] = 0.0;
;
Gu2[18] = 0.0;
;
Gu2[19] = 0.0;
;
Gu2[20] = 0.0;
;
Gu2[21] = 0.0;
;
Gu2[22] = 0.0;
;
Gu2[23] = 0.0;
;
Gu2[24] = 0.0;
;
Gu2[25] = 0.0;
;
Gu2[26] = 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 60 */
static const int xBoundIndices[ 60 ] = 
{ 12, 13, 21, 22, 30, 31, 39, 40, 48, 49, 57, 58, 66, 67, 75, 76, 84, 85, 93, 94, 102, 103, 111, 112, 120, 121, 129, 130, 138, 139, 147, 148, 156, 157, 165, 166, 174, 175, 183, 184, 192, 193, 201, 202, 210, 211, 219, 220, 228, 229, 237, 238, 246, 247, 255, 256, 264, 265, 273, 274 };
for (lRun2 = 0; lRun2 < 30; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 61)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 27 ]), &(acadoWorkspace.E[ lRun3 * 27 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 30; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (9)) * (9)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (9)) * (3)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (9)) * (3)) + (0) ]) );
}

acado_multQN1Gu( &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (30)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 29; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 27 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 81 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.evGu[ lRun2 * 27 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun1 = 0; lRun1 < 270; ++lRun1)
acadoWorkspace.sbar[lRun1 + 9] = acadoWorkspace.d[lRun1];

acadoWorkspace.lb[0] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[59];
acadoWorkspace.lb[60] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[60];
acadoWorkspace.lb[61] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[61];
acadoWorkspace.lb[62] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[62];
acadoWorkspace.lb[63] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[63];
acadoWorkspace.lb[64] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[64];
acadoWorkspace.lb[65] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[65];
acadoWorkspace.lb[66] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[66];
acadoWorkspace.lb[67] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[67];
acadoWorkspace.lb[68] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[68];
acadoWorkspace.lb[69] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[69];
acadoWorkspace.lb[70] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[70];
acadoWorkspace.lb[71] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[71];
acadoWorkspace.lb[72] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[72];
acadoWorkspace.lb[73] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[73];
acadoWorkspace.lb[74] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[74];
acadoWorkspace.lb[75] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[75];
acadoWorkspace.lb[76] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[76];
acadoWorkspace.lb[77] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[77];
acadoWorkspace.lb[78] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[78];
acadoWorkspace.lb[79] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[79];
acadoWorkspace.lb[80] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[80];
acadoWorkspace.lb[81] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[81];
acadoWorkspace.lb[82] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[82];
acadoWorkspace.lb[83] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[83];
acadoWorkspace.lb[84] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[84];
acadoWorkspace.lb[85] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[85];
acadoWorkspace.lb[86] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[86];
acadoWorkspace.lb[87] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[87];
acadoWorkspace.lb[88] = (real_t)-8.7222222222222229e-02 - acadoVariables.u[88];
acadoWorkspace.lb[89] = (real_t)-5.0000000000000000e-01 - acadoVariables.u[89];
acadoWorkspace.ub[0] = (real_t)8.7222222222222229e-02 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)8.7222222222222229e-02 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)5.0000000000000000e-01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)8.7222222222222229e-02 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)8.7222222222222229e-02 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)5.0000000000000000e-01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)8.7222222222222229e-02 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)8.7222222222222229e-02 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)5.0000000000000000e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)8.7222222222222229e-02 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)8.7222222222222229e-02 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)5.0000000000000000e-01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)8.7222222222222229e-02 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)8.7222222222222229e-02 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)5.0000000000000000e-01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)8.7222222222222229e-02 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)8.7222222222222229e-02 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)5.0000000000000000e-01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)8.7222222222222229e-02 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)8.7222222222222229e-02 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)5.0000000000000000e-01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)8.7222222222222229e-02 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)8.7222222222222229e-02 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)5.0000000000000000e-01 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)8.7222222222222229e-02 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)8.7222222222222229e-02 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)5.0000000000000000e-01 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)8.7222222222222229e-02 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)8.7222222222222229e-02 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)5.0000000000000000e-01 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)8.7222222222222229e-02 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)8.7222222222222229e-02 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)5.0000000000000000e-01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)8.7222222222222229e-02 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)8.7222222222222229e-02 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)5.0000000000000000e-01 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)8.7222222222222229e-02 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)8.7222222222222229e-02 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)5.0000000000000000e-01 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)8.7222222222222229e-02 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)8.7222222222222229e-02 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)5.0000000000000000e-01 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)8.7222222222222229e-02 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)8.7222222222222229e-02 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)5.0000000000000000e-01 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)8.7222222222222229e-02 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)8.7222222222222229e-02 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)5.0000000000000000e-01 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)8.7222222222222229e-02 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)8.7222222222222229e-02 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)5.0000000000000000e-01 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)8.7222222222222229e-02 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)8.7222222222222229e-02 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)5.0000000000000000e-01 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)8.7222222222222229e-02 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)8.7222222222222229e-02 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)5.0000000000000000e-01 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)8.7222222222222229e-02 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)8.7222222222222229e-02 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)5.0000000000000000e-01 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)8.7222222222222229e-02 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)8.7222222222222229e-02 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)5.0000000000000000e-01 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)8.7222222222222229e-02 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)8.7222222222222229e-02 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)5.0000000000000000e-01 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)8.7222222222222229e-02 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)8.7222222222222229e-02 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)5.0000000000000000e-01 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)8.7222222222222229e-02 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)8.7222222222222229e-02 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)5.0000000000000000e-01 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)8.7222222222222229e-02 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)8.7222222222222229e-02 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)5.0000000000000000e-01 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)8.7222222222222229e-02 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)8.7222222222222229e-02 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)5.0000000000000000e-01 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)8.7222222222222229e-02 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)8.7222222222222229e-02 - acadoVariables.u[79];
acadoWorkspace.ub[80] = (real_t)5.0000000000000000e-01 - acadoVariables.u[80];
acadoWorkspace.ub[81] = (real_t)8.7222222222222229e-02 - acadoVariables.u[81];
acadoWorkspace.ub[82] = (real_t)8.7222222222222229e-02 - acadoVariables.u[82];
acadoWorkspace.ub[83] = (real_t)5.0000000000000000e-01 - acadoVariables.u[83];
acadoWorkspace.ub[84] = (real_t)8.7222222222222229e-02 - acadoVariables.u[84];
acadoWorkspace.ub[85] = (real_t)8.7222222222222229e-02 - acadoVariables.u[85];
acadoWorkspace.ub[86] = (real_t)5.0000000000000000e-01 - acadoVariables.u[86];
acadoWorkspace.ub[87] = (real_t)8.7222222222222229e-02 - acadoVariables.u[87];
acadoWorkspace.ub[88] = (real_t)8.7222222222222229e-02 - acadoVariables.u[88];
acadoWorkspace.ub[89] = (real_t)5.0000000000000000e-01 - acadoVariables.u[89];

for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 9;
lRun4 = ((lRun3) / (9)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 59)) / (2)) + (lRun4)) - (1)) * (9)) + ((lRun3) % (9));
acadoWorkspace.A[(lRun1 * 90) + (lRun2 * 3)] = acadoWorkspace.E[lRun5 * 3];
acadoWorkspace.A[(lRun1 * 90) + (lRun2 * 3 + 1)] = acadoWorkspace.E[lRun5 * 3 + 1];
acadoWorkspace.A[(lRun1 * 90) + (lRun2 * 3 + 2)] = acadoWorkspace.E[lRun5 * 3 + 2];
}
}

}

void acado_condenseFdb(  )
{
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.Dy[70] -= acadoVariables.y[70];
acadoWorkspace.Dy[71] -= acadoVariables.y[71];
acadoWorkspace.Dy[72] -= acadoVariables.y[72];
acadoWorkspace.Dy[73] -= acadoVariables.y[73];
acadoWorkspace.Dy[74] -= acadoVariables.y[74];
acadoWorkspace.Dy[75] -= acadoVariables.y[75];
acadoWorkspace.Dy[76] -= acadoVariables.y[76];
acadoWorkspace.Dy[77] -= acadoVariables.y[77];
acadoWorkspace.Dy[78] -= acadoVariables.y[78];
acadoWorkspace.Dy[79] -= acadoVariables.y[79];
acadoWorkspace.Dy[80] -= acadoVariables.y[80];
acadoWorkspace.Dy[81] -= acadoVariables.y[81];
acadoWorkspace.Dy[82] -= acadoVariables.y[82];
acadoWorkspace.Dy[83] -= acadoVariables.y[83];
acadoWorkspace.Dy[84] -= acadoVariables.y[84];
acadoWorkspace.Dy[85] -= acadoVariables.y[85];
acadoWorkspace.Dy[86] -= acadoVariables.y[86];
acadoWorkspace.Dy[87] -= acadoVariables.y[87];
acadoWorkspace.Dy[88] -= acadoVariables.y[88];
acadoWorkspace.Dy[89] -= acadoVariables.y[89];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 39 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 51 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 57 ]), &(acadoWorkspace.g[ 57 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.g[ 63 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.g[ 66 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 69 ]), &(acadoWorkspace.g[ 69 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.g[ 75 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.g[ 78 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 81 ]), &(acadoWorkspace.g[ 81 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 84 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 87 ]), &(acadoWorkspace.g[ 87 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 3 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 6 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 9 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 18 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 21 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 27 ]), &(acadoWorkspace.QDy[ 81 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.QDy[ 99 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 39 ]), &(acadoWorkspace.QDy[ 117 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 42 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 51 ]), &(acadoWorkspace.QDy[ 153 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 54 ]), &(acadoWorkspace.QDy[ 162 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 57 ]), &(acadoWorkspace.QDy[ 171 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 180 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 63 ]), &(acadoWorkspace.QDy[ 189 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.QDy[ 198 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 69 ]), &(acadoWorkspace.QDy[ 207 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 216 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.QDy[ 225 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 78 ]), &(acadoWorkspace.QDy[ 234 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 81 ]), &(acadoWorkspace.QDy[ 243 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 252 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 87 ]), &(acadoWorkspace.QDy[ 261 ]) );

acadoWorkspace.QDy[270] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[0];
acadoWorkspace.QDy[271] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[1];
acadoWorkspace.QDy[272] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[273] = 0.0;
;
acadoWorkspace.QDy[274] = 0.0;
;
acadoWorkspace.QDy[275] = 0.0;
;
acadoWorkspace.QDy[276] = 0.0;
;
acadoWorkspace.QDy[277] = 0.0;
;
acadoWorkspace.QDy[278] = 0.0;
;

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 9 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1053 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1134 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1215 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1377 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1458 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1539 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1620 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 189 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1701 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.sbar[ 198 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1782 ]), &(acadoWorkspace.sbar[ 198 ]), &(acadoWorkspace.sbar[ 207 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1863 ]), &(acadoWorkspace.sbar[ 207 ]), &(acadoWorkspace.sbar[ 216 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.sbar[ 225 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2025 ]), &(acadoWorkspace.sbar[ 225 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2106 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 243 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2187 ]), &(acadoWorkspace.sbar[ 243 ]), &(acadoWorkspace.sbar[ 252 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.sbar[ 261 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2349 ]), &(acadoWorkspace.sbar[ 261 ]), &(acadoWorkspace.sbar[ 270 ]) );

acadoWorkspace.w1[0] = + (real_t)1.0000000000000000e+01*acadoWorkspace.sbar[270] + acadoWorkspace.QDy[270];
acadoWorkspace.w1[1] = + (real_t)1.0000000000000000e+01*acadoWorkspace.sbar[271] + acadoWorkspace.QDy[271];
acadoWorkspace.w1[2] = + (real_t)1.0000000000000000e+01*acadoWorkspace.sbar[272] + acadoWorkspace.QDy[272];
acadoWorkspace.w1[3] = + acadoWorkspace.QDy[273];
acadoWorkspace.w1[4] = + acadoWorkspace.QDy[274];
acadoWorkspace.w1[5] = + acadoWorkspace.QDy[275];
acadoWorkspace.w1[6] = + acadoWorkspace.QDy[276];
acadoWorkspace.w1[7] = + acadoWorkspace.QDy[277];
acadoWorkspace.w1[8] = + acadoWorkspace.QDy[278];
acado_macBTw1( &(acadoWorkspace.evGu[ 783 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 87 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2349 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 261 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 261 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 84 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2268 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 252 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 252 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 729 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 81 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2187 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 243 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 243 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 702 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 78 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2106 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 234 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 234 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 675 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 75 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2025 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 225 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 225 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1944 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 216 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 216 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 621 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 69 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1863 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 207 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 207 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 594 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 66 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1782 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 198 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 198 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 567 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 63 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1701 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 189 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 189 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 540 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1620 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 180 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 180 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 513 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 57 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1539 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 171 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 171 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1458 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 162 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 162 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 459 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 51 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1377 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 153 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 153 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 45 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1215 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 135 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 135 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 378 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1134 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 351 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1053 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 972 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 297 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 891 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 99 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 99 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 810 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 729 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 81 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 81 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 567 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 63 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


tmp = acadoWorkspace.sbar[12] + acadoVariables.x[12];
acadoWorkspace.lbA[0] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[0] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[13] + acadoVariables.x[13];
acadoWorkspace.lbA[1] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[1] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[21] + acadoVariables.x[21];
acadoWorkspace.lbA[2] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[2] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[22] + acadoVariables.x[22];
acadoWorkspace.lbA[3] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[3] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[30] + acadoVariables.x[30];
acadoWorkspace.lbA[4] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[4] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[31] + acadoVariables.x[31];
acadoWorkspace.lbA[5] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[5] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[39] + acadoVariables.x[39];
acadoWorkspace.lbA[6] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[6] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[40] + acadoVariables.x[40];
acadoWorkspace.lbA[7] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[7] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[48] + acadoVariables.x[48];
acadoWorkspace.lbA[8] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[8] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[49] + acadoVariables.x[49];
acadoWorkspace.lbA[9] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[9] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[57] + acadoVariables.x[57];
acadoWorkspace.lbA[10] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[10] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[58] + acadoVariables.x[58];
acadoWorkspace.lbA[11] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[11] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[66] + acadoVariables.x[66];
acadoWorkspace.lbA[12] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[12] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[67] + acadoVariables.x[67];
acadoWorkspace.lbA[13] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[13] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[75] + acadoVariables.x[75];
acadoWorkspace.lbA[14] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[14] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[76] + acadoVariables.x[76];
acadoWorkspace.lbA[15] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[15] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[84] + acadoVariables.x[84];
acadoWorkspace.lbA[16] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[16] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[85] + acadoVariables.x[85];
acadoWorkspace.lbA[17] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[17] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[93] + acadoVariables.x[93];
acadoWorkspace.lbA[18] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[18] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[94] + acadoVariables.x[94];
acadoWorkspace.lbA[19] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[19] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[102] + acadoVariables.x[102];
acadoWorkspace.lbA[20] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[20] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[103] + acadoVariables.x[103];
acadoWorkspace.lbA[21] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[21] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[111] + acadoVariables.x[111];
acadoWorkspace.lbA[22] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[22] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[112] + acadoVariables.x[112];
acadoWorkspace.lbA[23] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[23] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[120] + acadoVariables.x[120];
acadoWorkspace.lbA[24] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[24] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[121] + acadoVariables.x[121];
acadoWorkspace.lbA[25] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[25] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[129] + acadoVariables.x[129];
acadoWorkspace.lbA[26] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[26] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[130] + acadoVariables.x[130];
acadoWorkspace.lbA[27] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[27] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[138] + acadoVariables.x[138];
acadoWorkspace.lbA[28] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[28] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[139] + acadoVariables.x[139];
acadoWorkspace.lbA[29] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[29] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[147] + acadoVariables.x[147];
acadoWorkspace.lbA[30] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[30] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[148] + acadoVariables.x[148];
acadoWorkspace.lbA[31] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[31] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[156] + acadoVariables.x[156];
acadoWorkspace.lbA[32] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[32] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[157] + acadoVariables.x[157];
acadoWorkspace.lbA[33] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[33] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[165] + acadoVariables.x[165];
acadoWorkspace.lbA[34] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[34] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[166] + acadoVariables.x[166];
acadoWorkspace.lbA[35] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[35] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[174] + acadoVariables.x[174];
acadoWorkspace.lbA[36] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[36] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[175] + acadoVariables.x[175];
acadoWorkspace.lbA[37] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[37] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[183] + acadoVariables.x[183];
acadoWorkspace.lbA[38] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[38] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[184] + acadoVariables.x[184];
acadoWorkspace.lbA[39] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[39] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[192] + acadoVariables.x[192];
acadoWorkspace.lbA[40] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[40] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[193] + acadoVariables.x[193];
acadoWorkspace.lbA[41] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[41] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[201] + acadoVariables.x[201];
acadoWorkspace.lbA[42] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[42] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[202] + acadoVariables.x[202];
acadoWorkspace.lbA[43] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[43] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[210] + acadoVariables.x[210];
acadoWorkspace.lbA[44] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[44] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[211] + acadoVariables.x[211];
acadoWorkspace.lbA[45] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[45] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[219] + acadoVariables.x[219];
acadoWorkspace.lbA[46] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[46] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[220] + acadoVariables.x[220];
acadoWorkspace.lbA[47] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[47] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[228] + acadoVariables.x[228];
acadoWorkspace.lbA[48] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[48] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[229] + acadoVariables.x[229];
acadoWorkspace.lbA[49] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[49] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[237] + acadoVariables.x[237];
acadoWorkspace.lbA[50] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[50] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[238] + acadoVariables.x[238];
acadoWorkspace.lbA[51] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[51] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[246] + acadoVariables.x[246];
acadoWorkspace.lbA[52] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[52] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[247] + acadoVariables.x[247];
acadoWorkspace.lbA[53] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[53] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[255] + acadoVariables.x[255];
acadoWorkspace.lbA[54] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[54] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[256] + acadoVariables.x[256];
acadoWorkspace.lbA[55] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[55] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[264] + acadoVariables.x[264];
acadoWorkspace.lbA[56] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[56] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[265] + acadoVariables.x[265];
acadoWorkspace.lbA[57] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[57] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[273] + acadoVariables.x[273];
acadoWorkspace.lbA[58] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[58] = (real_t)5.0000000000000000e-01 - tmp;
tmp = acadoWorkspace.sbar[274] + acadoVariables.x[274];
acadoWorkspace.lbA[59] = (real_t)-5.0000000000000000e-01 - tmp;
acadoWorkspace.ubA[59] = (real_t)5.0000000000000000e-01 - tmp;

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoVariables.u[80] += acadoWorkspace.x[80];
acadoVariables.u[81] += acadoWorkspace.x[81];
acadoVariables.u[82] += acadoWorkspace.x[82];
acadoVariables.u[83] += acadoWorkspace.x[83];
acadoVariables.u[84] += acadoWorkspace.x[84];
acadoVariables.u[85] += acadoWorkspace.x[85];
acadoVariables.u[86] += acadoWorkspace.x[86];
acadoVariables.u[87] += acadoWorkspace.x[87];
acadoVariables.u[88] += acadoWorkspace.x[88];
acadoVariables.u[89] += acadoWorkspace.x[89];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
for (lRun1 = 0; lRun1 < 270; ++lRun1)
acadoWorkspace.sbar[lRun1 + 9] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 9 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.evGu[ 27 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.evGu[ 81 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.evGu[ 135 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.evGu[ 162 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.evGu[ 189 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.evGu[ 243 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.evGu[ 270 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.evGu[ 297 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.evGu[ 324 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1053 ]), &(acadoWorkspace.evGu[ 351 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1134 ]), &(acadoWorkspace.evGu[ 378 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1215 ]), &(acadoWorkspace.evGu[ 405 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1377 ]), &(acadoWorkspace.evGu[ 459 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1458 ]), &(acadoWorkspace.evGu[ 486 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1539 ]), &(acadoWorkspace.evGu[ 513 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1620 ]), &(acadoWorkspace.evGu[ 540 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 189 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1701 ]), &(acadoWorkspace.evGu[ 567 ]), &(acadoWorkspace.x[ 63 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.sbar[ 198 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1782 ]), &(acadoWorkspace.evGu[ 594 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.sbar[ 198 ]), &(acadoWorkspace.sbar[ 207 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1863 ]), &(acadoWorkspace.evGu[ 621 ]), &(acadoWorkspace.x[ 69 ]), &(acadoWorkspace.sbar[ 207 ]), &(acadoWorkspace.sbar[ 216 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.evGu[ 648 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.sbar[ 225 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2025 ]), &(acadoWorkspace.evGu[ 675 ]), &(acadoWorkspace.x[ 75 ]), &(acadoWorkspace.sbar[ 225 ]), &(acadoWorkspace.sbar[ 234 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2106 ]), &(acadoWorkspace.evGu[ 702 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.sbar[ 243 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2187 ]), &(acadoWorkspace.evGu[ 729 ]), &(acadoWorkspace.x[ 81 ]), &(acadoWorkspace.sbar[ 243 ]), &(acadoWorkspace.sbar[ 252 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.evGu[ 756 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.sbar[ 261 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2349 ]), &(acadoWorkspace.evGu[ 783 ]), &(acadoWorkspace.x[ 87 ]), &(acadoWorkspace.sbar[ 261 ]), &(acadoWorkspace.sbar[ 270 ]) );
for (lRun1 = 0; lRun1 < 279; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 9];
acadoWorkspace.state[1] = acadoVariables.x[index * 9 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 9 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 9 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 9 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 9 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 9 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 9 + 7];
acadoWorkspace.state[8] = acadoVariables.x[index * 9 + 8];
acadoWorkspace.state[117] = acadoVariables.u[index * 3];
acadoWorkspace.state[118] = acadoVariables.u[index * 3 + 1];
acadoWorkspace.state[119] = acadoVariables.u[index * 3 + 2];
acadoWorkspace.state[120] = acadoVariables.od[index * 2];
acadoWorkspace.state[121] = acadoVariables.od[index * 2 + 1];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 9 + 9] = acadoWorkspace.state[0];
acadoVariables.x[index * 9 + 10] = acadoWorkspace.state[1];
acadoVariables.x[index * 9 + 11] = acadoWorkspace.state[2];
acadoVariables.x[index * 9 + 12] = acadoWorkspace.state[3];
acadoVariables.x[index * 9 + 13] = acadoWorkspace.state[4];
acadoVariables.x[index * 9 + 14] = acadoWorkspace.state[5];
acadoVariables.x[index * 9 + 15] = acadoWorkspace.state[6];
acadoVariables.x[index * 9 + 16] = acadoWorkspace.state[7];
acadoVariables.x[index * 9 + 17] = acadoWorkspace.state[8];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
acadoVariables.x[index * 9] = acadoVariables.x[index * 9 + 9];
acadoVariables.x[index * 9 + 1] = acadoVariables.x[index * 9 + 10];
acadoVariables.x[index * 9 + 2] = acadoVariables.x[index * 9 + 11];
acadoVariables.x[index * 9 + 3] = acadoVariables.x[index * 9 + 12];
acadoVariables.x[index * 9 + 4] = acadoVariables.x[index * 9 + 13];
acadoVariables.x[index * 9 + 5] = acadoVariables.x[index * 9 + 14];
acadoVariables.x[index * 9 + 6] = acadoVariables.x[index * 9 + 15];
acadoVariables.x[index * 9 + 7] = acadoVariables.x[index * 9 + 16];
acadoVariables.x[index * 9 + 8] = acadoVariables.x[index * 9 + 17];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[270] = xEnd[0];
acadoVariables.x[271] = xEnd[1];
acadoVariables.x[272] = xEnd[2];
acadoVariables.x[273] = xEnd[3];
acadoVariables.x[274] = xEnd[4];
acadoVariables.x[275] = xEnd[5];
acadoVariables.x[276] = xEnd[6];
acadoVariables.x[277] = xEnd[7];
acadoVariables.x[278] = xEnd[8];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[270];
acadoWorkspace.state[1] = acadoVariables.x[271];
acadoWorkspace.state[2] = acadoVariables.x[272];
acadoWorkspace.state[3] = acadoVariables.x[273];
acadoWorkspace.state[4] = acadoVariables.x[274];
acadoWorkspace.state[5] = acadoVariables.x[275];
acadoWorkspace.state[6] = acadoVariables.x[276];
acadoWorkspace.state[7] = acadoVariables.x[277];
acadoWorkspace.state[8] = acadoVariables.x[278];
if (uEnd != 0)
{
acadoWorkspace.state[117] = uEnd[0];
acadoWorkspace.state[118] = uEnd[1];
acadoWorkspace.state[119] = uEnd[2];
}
else
{
acadoWorkspace.state[117] = acadoVariables.u[87];
acadoWorkspace.state[118] = acadoVariables.u[88];
acadoWorkspace.state[119] = acadoVariables.u[89];
}
acadoWorkspace.state[120] = acadoVariables.od[60];
acadoWorkspace.state[121] = acadoVariables.od[61];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[270] = acadoWorkspace.state[0];
acadoVariables.x[271] = acadoWorkspace.state[1];
acadoVariables.x[272] = acadoWorkspace.state[2];
acadoVariables.x[273] = acadoWorkspace.state[3];
acadoVariables.x[274] = acadoWorkspace.state[4];
acadoVariables.x[275] = acadoWorkspace.state[5];
acadoVariables.x[276] = acadoWorkspace.state[6];
acadoVariables.x[277] = acadoWorkspace.state[7];
acadoVariables.x[278] = acadoWorkspace.state[8];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[87] = uEnd[0];
acadoVariables.u[88] = uEnd[1];
acadoVariables.u[89] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89];
kkt = fabs( kkt );
for (index = 0; index < 90; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index + 90];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 3 */
real_t tmpDy[ 3 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 2];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 3] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 3];
acadoWorkspace.Dy[lRun1 * 3 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 3 + 1];
acadoWorkspace.Dy[lRun1 * 3 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 3 + 2];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[270];
acadoWorkspace.objValueIn[1] = acadoVariables.x[271];
acadoWorkspace.objValueIn[2] = acadoVariables.x[272];
acadoWorkspace.objValueIn[3] = acadoVariables.x[273];
acadoWorkspace.objValueIn[4] = acadoVariables.x[274];
acadoWorkspace.objValueIn[5] = acadoVariables.x[275];
acadoWorkspace.objValueIn[6] = acadoVariables.x[276];
acadoWorkspace.objValueIn[7] = acadoVariables.x[277];
acadoWorkspace.objValueIn[8] = acadoVariables.x[278];
acadoWorkspace.objValueIn[9] = acadoVariables.od[60];
acadoWorkspace.objValueIn[10] = acadoVariables.od[61];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 3];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 3 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 3 + 2];
objVal += + acadoWorkspace.Dy[lRun1 * 3]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 3 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 3 + 2]*tmpDy[2];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)1.0000000000000000e+01;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)1.0000000000000000e+01;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)1.0000000000000000e+01;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

