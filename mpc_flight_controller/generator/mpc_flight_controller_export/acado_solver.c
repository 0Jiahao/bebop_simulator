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
int lRun2;
ret = 0;
for (lRun1 = 0; lRun1 < 37; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 12];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 12 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 12 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 12 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 12 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 12 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 12 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 12 + 7];
acadoWorkspace.state[8] = acadoVariables.x[lRun1 * 12 + 8];
acadoWorkspace.state[9] = acadoVariables.x[lRun1 * 12 + 9];
acadoWorkspace.state[10] = acadoVariables.x[lRun1 * 12 + 10];
acadoWorkspace.state[11] = acadoVariables.x[lRun1 * 12 + 11];

acadoWorkspace.state[204] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.state[205] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.state[206] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.state[207] = acadoVariables.u[lRun1 * 4 + 3];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 12] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 12 + 12];
acadoWorkspace.d[lRun1 * 12 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 12 + 13];
acadoWorkspace.d[lRun1 * 12 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 12 + 14];
acadoWorkspace.d[lRun1 * 12 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 12 + 15];
acadoWorkspace.d[lRun1 * 12 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 12 + 16];
acadoWorkspace.d[lRun1 * 12 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 12 + 17];
acadoWorkspace.d[lRun1 * 12 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 12 + 18];
acadoWorkspace.d[lRun1 * 12 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 12 + 19];
acadoWorkspace.d[lRun1 * 12 + 8] = acadoWorkspace.state[8] - acadoVariables.x[lRun1 * 12 + 20];
acadoWorkspace.d[lRun1 * 12 + 9] = acadoWorkspace.state[9] - acadoVariables.x[lRun1 * 12 + 21];
acadoWorkspace.d[lRun1 * 12 + 10] = acadoWorkspace.state[10] - acadoVariables.x[lRun1 * 12 + 22];
acadoWorkspace.d[lRun1 * 12 + 11] = acadoWorkspace.state[11] - acadoVariables.x[lRun1 * 12 + 23];

for (lRun2 = 0; lRun2 < 144; ++lRun2)
acadoWorkspace.evGx[(0) + ((lRun2) + (lRun1 * 144))] = acadoWorkspace.state[lRun2 + 12];


acadoWorkspace.evGu[lRun1 * 48] = acadoWorkspace.state[156];
acadoWorkspace.evGu[lRun1 * 48 + 1] = acadoWorkspace.state[157];
acadoWorkspace.evGu[lRun1 * 48 + 2] = acadoWorkspace.state[158];
acadoWorkspace.evGu[lRun1 * 48 + 3] = acadoWorkspace.state[159];
acadoWorkspace.evGu[lRun1 * 48 + 4] = acadoWorkspace.state[160];
acadoWorkspace.evGu[lRun1 * 48 + 5] = acadoWorkspace.state[161];
acadoWorkspace.evGu[lRun1 * 48 + 6] = acadoWorkspace.state[162];
acadoWorkspace.evGu[lRun1 * 48 + 7] = acadoWorkspace.state[163];
acadoWorkspace.evGu[lRun1 * 48 + 8] = acadoWorkspace.state[164];
acadoWorkspace.evGu[lRun1 * 48 + 9] = acadoWorkspace.state[165];
acadoWorkspace.evGu[lRun1 * 48 + 10] = acadoWorkspace.state[166];
acadoWorkspace.evGu[lRun1 * 48 + 11] = acadoWorkspace.state[167];
acadoWorkspace.evGu[lRun1 * 48 + 12] = acadoWorkspace.state[168];
acadoWorkspace.evGu[lRun1 * 48 + 13] = acadoWorkspace.state[169];
acadoWorkspace.evGu[lRun1 * 48 + 14] = acadoWorkspace.state[170];
acadoWorkspace.evGu[lRun1 * 48 + 15] = acadoWorkspace.state[171];
acadoWorkspace.evGu[lRun1 * 48 + 16] = acadoWorkspace.state[172];
acadoWorkspace.evGu[lRun1 * 48 + 17] = acadoWorkspace.state[173];
acadoWorkspace.evGu[lRun1 * 48 + 18] = acadoWorkspace.state[174];
acadoWorkspace.evGu[lRun1 * 48 + 19] = acadoWorkspace.state[175];
acadoWorkspace.evGu[lRun1 * 48 + 20] = acadoWorkspace.state[176];
acadoWorkspace.evGu[lRun1 * 48 + 21] = acadoWorkspace.state[177];
acadoWorkspace.evGu[lRun1 * 48 + 22] = acadoWorkspace.state[178];
acadoWorkspace.evGu[lRun1 * 48 + 23] = acadoWorkspace.state[179];
acadoWorkspace.evGu[lRun1 * 48 + 24] = acadoWorkspace.state[180];
acadoWorkspace.evGu[lRun1 * 48 + 25] = acadoWorkspace.state[181];
acadoWorkspace.evGu[lRun1 * 48 + 26] = acadoWorkspace.state[182];
acadoWorkspace.evGu[lRun1 * 48 + 27] = acadoWorkspace.state[183];
acadoWorkspace.evGu[lRun1 * 48 + 28] = acadoWorkspace.state[184];
acadoWorkspace.evGu[lRun1 * 48 + 29] = acadoWorkspace.state[185];
acadoWorkspace.evGu[lRun1 * 48 + 30] = acadoWorkspace.state[186];
acadoWorkspace.evGu[lRun1 * 48 + 31] = acadoWorkspace.state[187];
acadoWorkspace.evGu[lRun1 * 48 + 32] = acadoWorkspace.state[188];
acadoWorkspace.evGu[lRun1 * 48 + 33] = acadoWorkspace.state[189];
acadoWorkspace.evGu[lRun1 * 48 + 34] = acadoWorkspace.state[190];
acadoWorkspace.evGu[lRun1 * 48 + 35] = acadoWorkspace.state[191];
acadoWorkspace.evGu[lRun1 * 48 + 36] = acadoWorkspace.state[192];
acadoWorkspace.evGu[lRun1 * 48 + 37] = acadoWorkspace.state[193];
acadoWorkspace.evGu[lRun1 * 48 + 38] = acadoWorkspace.state[194];
acadoWorkspace.evGu[lRun1 * 48 + 39] = acadoWorkspace.state[195];
acadoWorkspace.evGu[lRun1 * 48 + 40] = acadoWorkspace.state[196];
acadoWorkspace.evGu[lRun1 * 48 + 41] = acadoWorkspace.state[197];
acadoWorkspace.evGu[lRun1 * 48 + 42] = acadoWorkspace.state[198];
acadoWorkspace.evGu[lRun1 * 48 + 43] = acadoWorkspace.state[199];
acadoWorkspace.evGu[lRun1 * 48 + 44] = acadoWorkspace.state[200];
acadoWorkspace.evGu[lRun1 * 48 + 45] = acadoWorkspace.state[201];
acadoWorkspace.evGu[lRun1 * 48 + 46] = acadoWorkspace.state[202];
acadoWorkspace.evGu[lRun1 * 48 + 47] = acadoWorkspace.state[203];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 12;

/* Compute outputs: */
out[0] = u[0];
out[1] = u[1];
out[2] = u[2];
out[3] = u[3];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 23. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[9]));
a[1] = (sin(xd[9]));
a[2] = ((xd[3]*a[0])+(xd[4]*a[1]));
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (cos(xd[9]));
a[7] = (sin(xd[9]));
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[9])));
a[13] = (a[12]*xd[3]);
a[14] = (cos(xd[9]));
a[15] = (a[14]*xd[4]);
a[16] = (a[13]+a[15]);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = a[2];
out[1] = a[3];
out[2] = a[4];
out[3] = a[5];
out[4] = a[6];
out[5] = a[7];
out[6] = a[8];
out[7] = a[9];
out[8] = a[10];
out[9] = a[11];
out[10] = a[16];
out[11] = a[17];
out[12] = a[18];
out[13] = a[19];
out[14] = a[20];
out[15] = a[21];
out[16] = a[22];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 37; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 12];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 12 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 12 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 12 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 12 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 12 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 12 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 12 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 12 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 12 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[runObj * 12 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[runObj * 12 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[13] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.u[runObj * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 4] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 4 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 4 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 4 + 3] = acadoWorkspace.objValueOut[3];

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[444];
acadoWorkspace.objValueIn[1] = acadoVariables.x[445];
acadoWorkspace.objValueIn[2] = acadoVariables.x[446];
acadoWorkspace.objValueIn[3] = acadoVariables.x[447];
acadoWorkspace.objValueIn[4] = acadoVariables.x[448];
acadoWorkspace.objValueIn[5] = acadoVariables.x[449];
acadoWorkspace.objValueIn[6] = acadoVariables.x[450];
acadoWorkspace.objValueIn[7] = acadoVariables.x[451];
acadoWorkspace.objValueIn[8] = acadoVariables.x[452];
acadoWorkspace.objValueIn[9] = acadoVariables.x[453];
acadoWorkspace.objValueIn[10] = acadoVariables.x[454];
acadoWorkspace.objValueIn[11] = acadoVariables.x[455];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
int lRun1;
int lRun2;
for (lRun1 = 0;lRun1 < 12; ++lRun1)
for (lRun2 = 0;lRun2 < 12; ++lRun2)
Gx2[(lRun1 * 12) + (lRun2)] = Gx1[(lRun1 * 12) + (lRun2)];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[24] + Gx1[3]*Gx2[36] + Gx1[4]*Gx2[48] + Gx1[5]*Gx2[60] + Gx1[6]*Gx2[72] + Gx1[7]*Gx2[84] + Gx1[8]*Gx2[96] + Gx1[9]*Gx2[108] + Gx1[10]*Gx2[120] + Gx1[11]*Gx2[132];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[25] + Gx1[3]*Gx2[37] + Gx1[4]*Gx2[49] + Gx1[5]*Gx2[61] + Gx1[6]*Gx2[73] + Gx1[7]*Gx2[85] + Gx1[8]*Gx2[97] + Gx1[9]*Gx2[109] + Gx1[10]*Gx2[121] + Gx1[11]*Gx2[133];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[26] + Gx1[3]*Gx2[38] + Gx1[4]*Gx2[50] + Gx1[5]*Gx2[62] + Gx1[6]*Gx2[74] + Gx1[7]*Gx2[86] + Gx1[8]*Gx2[98] + Gx1[9]*Gx2[110] + Gx1[10]*Gx2[122] + Gx1[11]*Gx2[134];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[27] + Gx1[3]*Gx2[39] + Gx1[4]*Gx2[51] + Gx1[5]*Gx2[63] + Gx1[6]*Gx2[75] + Gx1[7]*Gx2[87] + Gx1[8]*Gx2[99] + Gx1[9]*Gx2[111] + Gx1[10]*Gx2[123] + Gx1[11]*Gx2[135];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[16] + Gx1[2]*Gx2[28] + Gx1[3]*Gx2[40] + Gx1[4]*Gx2[52] + Gx1[5]*Gx2[64] + Gx1[6]*Gx2[76] + Gx1[7]*Gx2[88] + Gx1[8]*Gx2[100] + Gx1[9]*Gx2[112] + Gx1[10]*Gx2[124] + Gx1[11]*Gx2[136];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[17] + Gx1[2]*Gx2[29] + Gx1[3]*Gx2[41] + Gx1[4]*Gx2[53] + Gx1[5]*Gx2[65] + Gx1[6]*Gx2[77] + Gx1[7]*Gx2[89] + Gx1[8]*Gx2[101] + Gx1[9]*Gx2[113] + Gx1[10]*Gx2[125] + Gx1[11]*Gx2[137];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[18] + Gx1[2]*Gx2[30] + Gx1[3]*Gx2[42] + Gx1[4]*Gx2[54] + Gx1[5]*Gx2[66] + Gx1[6]*Gx2[78] + Gx1[7]*Gx2[90] + Gx1[8]*Gx2[102] + Gx1[9]*Gx2[114] + Gx1[10]*Gx2[126] + Gx1[11]*Gx2[138];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[19] + Gx1[2]*Gx2[31] + Gx1[3]*Gx2[43] + Gx1[4]*Gx2[55] + Gx1[5]*Gx2[67] + Gx1[6]*Gx2[79] + Gx1[7]*Gx2[91] + Gx1[8]*Gx2[103] + Gx1[9]*Gx2[115] + Gx1[10]*Gx2[127] + Gx1[11]*Gx2[139];
Gx3[8] = + Gx1[0]*Gx2[8] + Gx1[1]*Gx2[20] + Gx1[2]*Gx2[32] + Gx1[3]*Gx2[44] + Gx1[4]*Gx2[56] + Gx1[5]*Gx2[68] + Gx1[6]*Gx2[80] + Gx1[7]*Gx2[92] + Gx1[8]*Gx2[104] + Gx1[9]*Gx2[116] + Gx1[10]*Gx2[128] + Gx1[11]*Gx2[140];
Gx3[9] = + Gx1[0]*Gx2[9] + Gx1[1]*Gx2[21] + Gx1[2]*Gx2[33] + Gx1[3]*Gx2[45] + Gx1[4]*Gx2[57] + Gx1[5]*Gx2[69] + Gx1[6]*Gx2[81] + Gx1[7]*Gx2[93] + Gx1[8]*Gx2[105] + Gx1[9]*Gx2[117] + Gx1[10]*Gx2[129] + Gx1[11]*Gx2[141];
Gx3[10] = + Gx1[0]*Gx2[10] + Gx1[1]*Gx2[22] + Gx1[2]*Gx2[34] + Gx1[3]*Gx2[46] + Gx1[4]*Gx2[58] + Gx1[5]*Gx2[70] + Gx1[6]*Gx2[82] + Gx1[7]*Gx2[94] + Gx1[8]*Gx2[106] + Gx1[9]*Gx2[118] + Gx1[10]*Gx2[130] + Gx1[11]*Gx2[142];
Gx3[11] = + Gx1[0]*Gx2[11] + Gx1[1]*Gx2[23] + Gx1[2]*Gx2[35] + Gx1[3]*Gx2[47] + Gx1[4]*Gx2[59] + Gx1[5]*Gx2[71] + Gx1[6]*Gx2[83] + Gx1[7]*Gx2[95] + Gx1[8]*Gx2[107] + Gx1[9]*Gx2[119] + Gx1[10]*Gx2[131] + Gx1[11]*Gx2[143];
Gx3[12] = + Gx1[12]*Gx2[0] + Gx1[13]*Gx2[12] + Gx1[14]*Gx2[24] + Gx1[15]*Gx2[36] + Gx1[16]*Gx2[48] + Gx1[17]*Gx2[60] + Gx1[18]*Gx2[72] + Gx1[19]*Gx2[84] + Gx1[20]*Gx2[96] + Gx1[21]*Gx2[108] + Gx1[22]*Gx2[120] + Gx1[23]*Gx2[132];
Gx3[13] = + Gx1[12]*Gx2[1] + Gx1[13]*Gx2[13] + Gx1[14]*Gx2[25] + Gx1[15]*Gx2[37] + Gx1[16]*Gx2[49] + Gx1[17]*Gx2[61] + Gx1[18]*Gx2[73] + Gx1[19]*Gx2[85] + Gx1[20]*Gx2[97] + Gx1[21]*Gx2[109] + Gx1[22]*Gx2[121] + Gx1[23]*Gx2[133];
Gx3[14] = + Gx1[12]*Gx2[2] + Gx1[13]*Gx2[14] + Gx1[14]*Gx2[26] + Gx1[15]*Gx2[38] + Gx1[16]*Gx2[50] + Gx1[17]*Gx2[62] + Gx1[18]*Gx2[74] + Gx1[19]*Gx2[86] + Gx1[20]*Gx2[98] + Gx1[21]*Gx2[110] + Gx1[22]*Gx2[122] + Gx1[23]*Gx2[134];
Gx3[15] = + Gx1[12]*Gx2[3] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[27] + Gx1[15]*Gx2[39] + Gx1[16]*Gx2[51] + Gx1[17]*Gx2[63] + Gx1[18]*Gx2[75] + Gx1[19]*Gx2[87] + Gx1[20]*Gx2[99] + Gx1[21]*Gx2[111] + Gx1[22]*Gx2[123] + Gx1[23]*Gx2[135];
Gx3[16] = + Gx1[12]*Gx2[4] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[28] + Gx1[15]*Gx2[40] + Gx1[16]*Gx2[52] + Gx1[17]*Gx2[64] + Gx1[18]*Gx2[76] + Gx1[19]*Gx2[88] + Gx1[20]*Gx2[100] + Gx1[21]*Gx2[112] + Gx1[22]*Gx2[124] + Gx1[23]*Gx2[136];
Gx3[17] = + Gx1[12]*Gx2[5] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[29] + Gx1[15]*Gx2[41] + Gx1[16]*Gx2[53] + Gx1[17]*Gx2[65] + Gx1[18]*Gx2[77] + Gx1[19]*Gx2[89] + Gx1[20]*Gx2[101] + Gx1[21]*Gx2[113] + Gx1[22]*Gx2[125] + Gx1[23]*Gx2[137];
Gx3[18] = + Gx1[12]*Gx2[6] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[30] + Gx1[15]*Gx2[42] + Gx1[16]*Gx2[54] + Gx1[17]*Gx2[66] + Gx1[18]*Gx2[78] + Gx1[19]*Gx2[90] + Gx1[20]*Gx2[102] + Gx1[21]*Gx2[114] + Gx1[22]*Gx2[126] + Gx1[23]*Gx2[138];
Gx3[19] = + Gx1[12]*Gx2[7] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[31] + Gx1[15]*Gx2[43] + Gx1[16]*Gx2[55] + Gx1[17]*Gx2[67] + Gx1[18]*Gx2[79] + Gx1[19]*Gx2[91] + Gx1[20]*Gx2[103] + Gx1[21]*Gx2[115] + Gx1[22]*Gx2[127] + Gx1[23]*Gx2[139];
Gx3[20] = + Gx1[12]*Gx2[8] + Gx1[13]*Gx2[20] + Gx1[14]*Gx2[32] + Gx1[15]*Gx2[44] + Gx1[16]*Gx2[56] + Gx1[17]*Gx2[68] + Gx1[18]*Gx2[80] + Gx1[19]*Gx2[92] + Gx1[20]*Gx2[104] + Gx1[21]*Gx2[116] + Gx1[22]*Gx2[128] + Gx1[23]*Gx2[140];
Gx3[21] = + Gx1[12]*Gx2[9] + Gx1[13]*Gx2[21] + Gx1[14]*Gx2[33] + Gx1[15]*Gx2[45] + Gx1[16]*Gx2[57] + Gx1[17]*Gx2[69] + Gx1[18]*Gx2[81] + Gx1[19]*Gx2[93] + Gx1[20]*Gx2[105] + Gx1[21]*Gx2[117] + Gx1[22]*Gx2[129] + Gx1[23]*Gx2[141];
Gx3[22] = + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[22] + Gx1[14]*Gx2[34] + Gx1[15]*Gx2[46] + Gx1[16]*Gx2[58] + Gx1[17]*Gx2[70] + Gx1[18]*Gx2[82] + Gx1[19]*Gx2[94] + Gx1[20]*Gx2[106] + Gx1[21]*Gx2[118] + Gx1[22]*Gx2[130] + Gx1[23]*Gx2[142];
Gx3[23] = + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[23] + Gx1[14]*Gx2[35] + Gx1[15]*Gx2[47] + Gx1[16]*Gx2[59] + Gx1[17]*Gx2[71] + Gx1[18]*Gx2[83] + Gx1[19]*Gx2[95] + Gx1[20]*Gx2[107] + Gx1[21]*Gx2[119] + Gx1[22]*Gx2[131] + Gx1[23]*Gx2[143];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[12] + Gx1[26]*Gx2[24] + Gx1[27]*Gx2[36] + Gx1[28]*Gx2[48] + Gx1[29]*Gx2[60] + Gx1[30]*Gx2[72] + Gx1[31]*Gx2[84] + Gx1[32]*Gx2[96] + Gx1[33]*Gx2[108] + Gx1[34]*Gx2[120] + Gx1[35]*Gx2[132];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[13] + Gx1[26]*Gx2[25] + Gx1[27]*Gx2[37] + Gx1[28]*Gx2[49] + Gx1[29]*Gx2[61] + Gx1[30]*Gx2[73] + Gx1[31]*Gx2[85] + Gx1[32]*Gx2[97] + Gx1[33]*Gx2[109] + Gx1[34]*Gx2[121] + Gx1[35]*Gx2[133];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[14] + Gx1[26]*Gx2[26] + Gx1[27]*Gx2[38] + Gx1[28]*Gx2[50] + Gx1[29]*Gx2[62] + Gx1[30]*Gx2[74] + Gx1[31]*Gx2[86] + Gx1[32]*Gx2[98] + Gx1[33]*Gx2[110] + Gx1[34]*Gx2[122] + Gx1[35]*Gx2[134];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[15] + Gx1[26]*Gx2[27] + Gx1[27]*Gx2[39] + Gx1[28]*Gx2[51] + Gx1[29]*Gx2[63] + Gx1[30]*Gx2[75] + Gx1[31]*Gx2[87] + Gx1[32]*Gx2[99] + Gx1[33]*Gx2[111] + Gx1[34]*Gx2[123] + Gx1[35]*Gx2[135];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[16] + Gx1[26]*Gx2[28] + Gx1[27]*Gx2[40] + Gx1[28]*Gx2[52] + Gx1[29]*Gx2[64] + Gx1[30]*Gx2[76] + Gx1[31]*Gx2[88] + Gx1[32]*Gx2[100] + Gx1[33]*Gx2[112] + Gx1[34]*Gx2[124] + Gx1[35]*Gx2[136];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[17] + Gx1[26]*Gx2[29] + Gx1[27]*Gx2[41] + Gx1[28]*Gx2[53] + Gx1[29]*Gx2[65] + Gx1[30]*Gx2[77] + Gx1[31]*Gx2[89] + Gx1[32]*Gx2[101] + Gx1[33]*Gx2[113] + Gx1[34]*Gx2[125] + Gx1[35]*Gx2[137];
Gx3[30] = + Gx1[24]*Gx2[6] + Gx1[25]*Gx2[18] + Gx1[26]*Gx2[30] + Gx1[27]*Gx2[42] + Gx1[28]*Gx2[54] + Gx1[29]*Gx2[66] + Gx1[30]*Gx2[78] + Gx1[31]*Gx2[90] + Gx1[32]*Gx2[102] + Gx1[33]*Gx2[114] + Gx1[34]*Gx2[126] + Gx1[35]*Gx2[138];
Gx3[31] = + Gx1[24]*Gx2[7] + Gx1[25]*Gx2[19] + Gx1[26]*Gx2[31] + Gx1[27]*Gx2[43] + Gx1[28]*Gx2[55] + Gx1[29]*Gx2[67] + Gx1[30]*Gx2[79] + Gx1[31]*Gx2[91] + Gx1[32]*Gx2[103] + Gx1[33]*Gx2[115] + Gx1[34]*Gx2[127] + Gx1[35]*Gx2[139];
Gx3[32] = + Gx1[24]*Gx2[8] + Gx1[25]*Gx2[20] + Gx1[26]*Gx2[32] + Gx1[27]*Gx2[44] + Gx1[28]*Gx2[56] + Gx1[29]*Gx2[68] + Gx1[30]*Gx2[80] + Gx1[31]*Gx2[92] + Gx1[32]*Gx2[104] + Gx1[33]*Gx2[116] + Gx1[34]*Gx2[128] + Gx1[35]*Gx2[140];
Gx3[33] = + Gx1[24]*Gx2[9] + Gx1[25]*Gx2[21] + Gx1[26]*Gx2[33] + Gx1[27]*Gx2[45] + Gx1[28]*Gx2[57] + Gx1[29]*Gx2[69] + Gx1[30]*Gx2[81] + Gx1[31]*Gx2[93] + Gx1[32]*Gx2[105] + Gx1[33]*Gx2[117] + Gx1[34]*Gx2[129] + Gx1[35]*Gx2[141];
Gx3[34] = + Gx1[24]*Gx2[10] + Gx1[25]*Gx2[22] + Gx1[26]*Gx2[34] + Gx1[27]*Gx2[46] + Gx1[28]*Gx2[58] + Gx1[29]*Gx2[70] + Gx1[30]*Gx2[82] + Gx1[31]*Gx2[94] + Gx1[32]*Gx2[106] + Gx1[33]*Gx2[118] + Gx1[34]*Gx2[130] + Gx1[35]*Gx2[142];
Gx3[35] = + Gx1[24]*Gx2[11] + Gx1[25]*Gx2[23] + Gx1[26]*Gx2[35] + Gx1[27]*Gx2[47] + Gx1[28]*Gx2[59] + Gx1[29]*Gx2[71] + Gx1[30]*Gx2[83] + Gx1[31]*Gx2[95] + Gx1[32]*Gx2[107] + Gx1[33]*Gx2[119] + Gx1[34]*Gx2[131] + Gx1[35]*Gx2[143];
Gx3[36] = + Gx1[36]*Gx2[0] + Gx1[37]*Gx2[12] + Gx1[38]*Gx2[24] + Gx1[39]*Gx2[36] + Gx1[40]*Gx2[48] + Gx1[41]*Gx2[60] + Gx1[42]*Gx2[72] + Gx1[43]*Gx2[84] + Gx1[44]*Gx2[96] + Gx1[45]*Gx2[108] + Gx1[46]*Gx2[120] + Gx1[47]*Gx2[132];
Gx3[37] = + Gx1[36]*Gx2[1] + Gx1[37]*Gx2[13] + Gx1[38]*Gx2[25] + Gx1[39]*Gx2[37] + Gx1[40]*Gx2[49] + Gx1[41]*Gx2[61] + Gx1[42]*Gx2[73] + Gx1[43]*Gx2[85] + Gx1[44]*Gx2[97] + Gx1[45]*Gx2[109] + Gx1[46]*Gx2[121] + Gx1[47]*Gx2[133];
Gx3[38] = + Gx1[36]*Gx2[2] + Gx1[37]*Gx2[14] + Gx1[38]*Gx2[26] + Gx1[39]*Gx2[38] + Gx1[40]*Gx2[50] + Gx1[41]*Gx2[62] + Gx1[42]*Gx2[74] + Gx1[43]*Gx2[86] + Gx1[44]*Gx2[98] + Gx1[45]*Gx2[110] + Gx1[46]*Gx2[122] + Gx1[47]*Gx2[134];
Gx3[39] = + Gx1[36]*Gx2[3] + Gx1[37]*Gx2[15] + Gx1[38]*Gx2[27] + Gx1[39]*Gx2[39] + Gx1[40]*Gx2[51] + Gx1[41]*Gx2[63] + Gx1[42]*Gx2[75] + Gx1[43]*Gx2[87] + Gx1[44]*Gx2[99] + Gx1[45]*Gx2[111] + Gx1[46]*Gx2[123] + Gx1[47]*Gx2[135];
Gx3[40] = + Gx1[36]*Gx2[4] + Gx1[37]*Gx2[16] + Gx1[38]*Gx2[28] + Gx1[39]*Gx2[40] + Gx1[40]*Gx2[52] + Gx1[41]*Gx2[64] + Gx1[42]*Gx2[76] + Gx1[43]*Gx2[88] + Gx1[44]*Gx2[100] + Gx1[45]*Gx2[112] + Gx1[46]*Gx2[124] + Gx1[47]*Gx2[136];
Gx3[41] = + Gx1[36]*Gx2[5] + Gx1[37]*Gx2[17] + Gx1[38]*Gx2[29] + Gx1[39]*Gx2[41] + Gx1[40]*Gx2[53] + Gx1[41]*Gx2[65] + Gx1[42]*Gx2[77] + Gx1[43]*Gx2[89] + Gx1[44]*Gx2[101] + Gx1[45]*Gx2[113] + Gx1[46]*Gx2[125] + Gx1[47]*Gx2[137];
Gx3[42] = + Gx1[36]*Gx2[6] + Gx1[37]*Gx2[18] + Gx1[38]*Gx2[30] + Gx1[39]*Gx2[42] + Gx1[40]*Gx2[54] + Gx1[41]*Gx2[66] + Gx1[42]*Gx2[78] + Gx1[43]*Gx2[90] + Gx1[44]*Gx2[102] + Gx1[45]*Gx2[114] + Gx1[46]*Gx2[126] + Gx1[47]*Gx2[138];
Gx3[43] = + Gx1[36]*Gx2[7] + Gx1[37]*Gx2[19] + Gx1[38]*Gx2[31] + Gx1[39]*Gx2[43] + Gx1[40]*Gx2[55] + Gx1[41]*Gx2[67] + Gx1[42]*Gx2[79] + Gx1[43]*Gx2[91] + Gx1[44]*Gx2[103] + Gx1[45]*Gx2[115] + Gx1[46]*Gx2[127] + Gx1[47]*Gx2[139];
Gx3[44] = + Gx1[36]*Gx2[8] + Gx1[37]*Gx2[20] + Gx1[38]*Gx2[32] + Gx1[39]*Gx2[44] + Gx1[40]*Gx2[56] + Gx1[41]*Gx2[68] + Gx1[42]*Gx2[80] + Gx1[43]*Gx2[92] + Gx1[44]*Gx2[104] + Gx1[45]*Gx2[116] + Gx1[46]*Gx2[128] + Gx1[47]*Gx2[140];
Gx3[45] = + Gx1[36]*Gx2[9] + Gx1[37]*Gx2[21] + Gx1[38]*Gx2[33] + Gx1[39]*Gx2[45] + Gx1[40]*Gx2[57] + Gx1[41]*Gx2[69] + Gx1[42]*Gx2[81] + Gx1[43]*Gx2[93] + Gx1[44]*Gx2[105] + Gx1[45]*Gx2[117] + Gx1[46]*Gx2[129] + Gx1[47]*Gx2[141];
Gx3[46] = + Gx1[36]*Gx2[10] + Gx1[37]*Gx2[22] + Gx1[38]*Gx2[34] + Gx1[39]*Gx2[46] + Gx1[40]*Gx2[58] + Gx1[41]*Gx2[70] + Gx1[42]*Gx2[82] + Gx1[43]*Gx2[94] + Gx1[44]*Gx2[106] + Gx1[45]*Gx2[118] + Gx1[46]*Gx2[130] + Gx1[47]*Gx2[142];
Gx3[47] = + Gx1[36]*Gx2[11] + Gx1[37]*Gx2[23] + Gx1[38]*Gx2[35] + Gx1[39]*Gx2[47] + Gx1[40]*Gx2[59] + Gx1[41]*Gx2[71] + Gx1[42]*Gx2[83] + Gx1[43]*Gx2[95] + Gx1[44]*Gx2[107] + Gx1[45]*Gx2[119] + Gx1[46]*Gx2[131] + Gx1[47]*Gx2[143];
Gx3[48] = + Gx1[48]*Gx2[0] + Gx1[49]*Gx2[12] + Gx1[50]*Gx2[24] + Gx1[51]*Gx2[36] + Gx1[52]*Gx2[48] + Gx1[53]*Gx2[60] + Gx1[54]*Gx2[72] + Gx1[55]*Gx2[84] + Gx1[56]*Gx2[96] + Gx1[57]*Gx2[108] + Gx1[58]*Gx2[120] + Gx1[59]*Gx2[132];
Gx3[49] = + Gx1[48]*Gx2[1] + Gx1[49]*Gx2[13] + Gx1[50]*Gx2[25] + Gx1[51]*Gx2[37] + Gx1[52]*Gx2[49] + Gx1[53]*Gx2[61] + Gx1[54]*Gx2[73] + Gx1[55]*Gx2[85] + Gx1[56]*Gx2[97] + Gx1[57]*Gx2[109] + Gx1[58]*Gx2[121] + Gx1[59]*Gx2[133];
Gx3[50] = + Gx1[48]*Gx2[2] + Gx1[49]*Gx2[14] + Gx1[50]*Gx2[26] + Gx1[51]*Gx2[38] + Gx1[52]*Gx2[50] + Gx1[53]*Gx2[62] + Gx1[54]*Gx2[74] + Gx1[55]*Gx2[86] + Gx1[56]*Gx2[98] + Gx1[57]*Gx2[110] + Gx1[58]*Gx2[122] + Gx1[59]*Gx2[134];
Gx3[51] = + Gx1[48]*Gx2[3] + Gx1[49]*Gx2[15] + Gx1[50]*Gx2[27] + Gx1[51]*Gx2[39] + Gx1[52]*Gx2[51] + Gx1[53]*Gx2[63] + Gx1[54]*Gx2[75] + Gx1[55]*Gx2[87] + Gx1[56]*Gx2[99] + Gx1[57]*Gx2[111] + Gx1[58]*Gx2[123] + Gx1[59]*Gx2[135];
Gx3[52] = + Gx1[48]*Gx2[4] + Gx1[49]*Gx2[16] + Gx1[50]*Gx2[28] + Gx1[51]*Gx2[40] + Gx1[52]*Gx2[52] + Gx1[53]*Gx2[64] + Gx1[54]*Gx2[76] + Gx1[55]*Gx2[88] + Gx1[56]*Gx2[100] + Gx1[57]*Gx2[112] + Gx1[58]*Gx2[124] + Gx1[59]*Gx2[136];
Gx3[53] = + Gx1[48]*Gx2[5] + Gx1[49]*Gx2[17] + Gx1[50]*Gx2[29] + Gx1[51]*Gx2[41] + Gx1[52]*Gx2[53] + Gx1[53]*Gx2[65] + Gx1[54]*Gx2[77] + Gx1[55]*Gx2[89] + Gx1[56]*Gx2[101] + Gx1[57]*Gx2[113] + Gx1[58]*Gx2[125] + Gx1[59]*Gx2[137];
Gx3[54] = + Gx1[48]*Gx2[6] + Gx1[49]*Gx2[18] + Gx1[50]*Gx2[30] + Gx1[51]*Gx2[42] + Gx1[52]*Gx2[54] + Gx1[53]*Gx2[66] + Gx1[54]*Gx2[78] + Gx1[55]*Gx2[90] + Gx1[56]*Gx2[102] + Gx1[57]*Gx2[114] + Gx1[58]*Gx2[126] + Gx1[59]*Gx2[138];
Gx3[55] = + Gx1[48]*Gx2[7] + Gx1[49]*Gx2[19] + Gx1[50]*Gx2[31] + Gx1[51]*Gx2[43] + Gx1[52]*Gx2[55] + Gx1[53]*Gx2[67] + Gx1[54]*Gx2[79] + Gx1[55]*Gx2[91] + Gx1[56]*Gx2[103] + Gx1[57]*Gx2[115] + Gx1[58]*Gx2[127] + Gx1[59]*Gx2[139];
Gx3[56] = + Gx1[48]*Gx2[8] + Gx1[49]*Gx2[20] + Gx1[50]*Gx2[32] + Gx1[51]*Gx2[44] + Gx1[52]*Gx2[56] + Gx1[53]*Gx2[68] + Gx1[54]*Gx2[80] + Gx1[55]*Gx2[92] + Gx1[56]*Gx2[104] + Gx1[57]*Gx2[116] + Gx1[58]*Gx2[128] + Gx1[59]*Gx2[140];
Gx3[57] = + Gx1[48]*Gx2[9] + Gx1[49]*Gx2[21] + Gx1[50]*Gx2[33] + Gx1[51]*Gx2[45] + Gx1[52]*Gx2[57] + Gx1[53]*Gx2[69] + Gx1[54]*Gx2[81] + Gx1[55]*Gx2[93] + Gx1[56]*Gx2[105] + Gx1[57]*Gx2[117] + Gx1[58]*Gx2[129] + Gx1[59]*Gx2[141];
Gx3[58] = + Gx1[48]*Gx2[10] + Gx1[49]*Gx2[22] + Gx1[50]*Gx2[34] + Gx1[51]*Gx2[46] + Gx1[52]*Gx2[58] + Gx1[53]*Gx2[70] + Gx1[54]*Gx2[82] + Gx1[55]*Gx2[94] + Gx1[56]*Gx2[106] + Gx1[57]*Gx2[118] + Gx1[58]*Gx2[130] + Gx1[59]*Gx2[142];
Gx3[59] = + Gx1[48]*Gx2[11] + Gx1[49]*Gx2[23] + Gx1[50]*Gx2[35] + Gx1[51]*Gx2[47] + Gx1[52]*Gx2[59] + Gx1[53]*Gx2[71] + Gx1[54]*Gx2[83] + Gx1[55]*Gx2[95] + Gx1[56]*Gx2[107] + Gx1[57]*Gx2[119] + Gx1[58]*Gx2[131] + Gx1[59]*Gx2[143];
Gx3[60] = + Gx1[60]*Gx2[0] + Gx1[61]*Gx2[12] + Gx1[62]*Gx2[24] + Gx1[63]*Gx2[36] + Gx1[64]*Gx2[48] + Gx1[65]*Gx2[60] + Gx1[66]*Gx2[72] + Gx1[67]*Gx2[84] + Gx1[68]*Gx2[96] + Gx1[69]*Gx2[108] + Gx1[70]*Gx2[120] + Gx1[71]*Gx2[132];
Gx3[61] = + Gx1[60]*Gx2[1] + Gx1[61]*Gx2[13] + Gx1[62]*Gx2[25] + Gx1[63]*Gx2[37] + Gx1[64]*Gx2[49] + Gx1[65]*Gx2[61] + Gx1[66]*Gx2[73] + Gx1[67]*Gx2[85] + Gx1[68]*Gx2[97] + Gx1[69]*Gx2[109] + Gx1[70]*Gx2[121] + Gx1[71]*Gx2[133];
Gx3[62] = + Gx1[60]*Gx2[2] + Gx1[61]*Gx2[14] + Gx1[62]*Gx2[26] + Gx1[63]*Gx2[38] + Gx1[64]*Gx2[50] + Gx1[65]*Gx2[62] + Gx1[66]*Gx2[74] + Gx1[67]*Gx2[86] + Gx1[68]*Gx2[98] + Gx1[69]*Gx2[110] + Gx1[70]*Gx2[122] + Gx1[71]*Gx2[134];
Gx3[63] = + Gx1[60]*Gx2[3] + Gx1[61]*Gx2[15] + Gx1[62]*Gx2[27] + Gx1[63]*Gx2[39] + Gx1[64]*Gx2[51] + Gx1[65]*Gx2[63] + Gx1[66]*Gx2[75] + Gx1[67]*Gx2[87] + Gx1[68]*Gx2[99] + Gx1[69]*Gx2[111] + Gx1[70]*Gx2[123] + Gx1[71]*Gx2[135];
Gx3[64] = + Gx1[60]*Gx2[4] + Gx1[61]*Gx2[16] + Gx1[62]*Gx2[28] + Gx1[63]*Gx2[40] + Gx1[64]*Gx2[52] + Gx1[65]*Gx2[64] + Gx1[66]*Gx2[76] + Gx1[67]*Gx2[88] + Gx1[68]*Gx2[100] + Gx1[69]*Gx2[112] + Gx1[70]*Gx2[124] + Gx1[71]*Gx2[136];
Gx3[65] = + Gx1[60]*Gx2[5] + Gx1[61]*Gx2[17] + Gx1[62]*Gx2[29] + Gx1[63]*Gx2[41] + Gx1[64]*Gx2[53] + Gx1[65]*Gx2[65] + Gx1[66]*Gx2[77] + Gx1[67]*Gx2[89] + Gx1[68]*Gx2[101] + Gx1[69]*Gx2[113] + Gx1[70]*Gx2[125] + Gx1[71]*Gx2[137];
Gx3[66] = + Gx1[60]*Gx2[6] + Gx1[61]*Gx2[18] + Gx1[62]*Gx2[30] + Gx1[63]*Gx2[42] + Gx1[64]*Gx2[54] + Gx1[65]*Gx2[66] + Gx1[66]*Gx2[78] + Gx1[67]*Gx2[90] + Gx1[68]*Gx2[102] + Gx1[69]*Gx2[114] + Gx1[70]*Gx2[126] + Gx1[71]*Gx2[138];
Gx3[67] = + Gx1[60]*Gx2[7] + Gx1[61]*Gx2[19] + Gx1[62]*Gx2[31] + Gx1[63]*Gx2[43] + Gx1[64]*Gx2[55] + Gx1[65]*Gx2[67] + Gx1[66]*Gx2[79] + Gx1[67]*Gx2[91] + Gx1[68]*Gx2[103] + Gx1[69]*Gx2[115] + Gx1[70]*Gx2[127] + Gx1[71]*Gx2[139];
Gx3[68] = + Gx1[60]*Gx2[8] + Gx1[61]*Gx2[20] + Gx1[62]*Gx2[32] + Gx1[63]*Gx2[44] + Gx1[64]*Gx2[56] + Gx1[65]*Gx2[68] + Gx1[66]*Gx2[80] + Gx1[67]*Gx2[92] + Gx1[68]*Gx2[104] + Gx1[69]*Gx2[116] + Gx1[70]*Gx2[128] + Gx1[71]*Gx2[140];
Gx3[69] = + Gx1[60]*Gx2[9] + Gx1[61]*Gx2[21] + Gx1[62]*Gx2[33] + Gx1[63]*Gx2[45] + Gx1[64]*Gx2[57] + Gx1[65]*Gx2[69] + Gx1[66]*Gx2[81] + Gx1[67]*Gx2[93] + Gx1[68]*Gx2[105] + Gx1[69]*Gx2[117] + Gx1[70]*Gx2[129] + Gx1[71]*Gx2[141];
Gx3[70] = + Gx1[60]*Gx2[10] + Gx1[61]*Gx2[22] + Gx1[62]*Gx2[34] + Gx1[63]*Gx2[46] + Gx1[64]*Gx2[58] + Gx1[65]*Gx2[70] + Gx1[66]*Gx2[82] + Gx1[67]*Gx2[94] + Gx1[68]*Gx2[106] + Gx1[69]*Gx2[118] + Gx1[70]*Gx2[130] + Gx1[71]*Gx2[142];
Gx3[71] = + Gx1[60]*Gx2[11] + Gx1[61]*Gx2[23] + Gx1[62]*Gx2[35] + Gx1[63]*Gx2[47] + Gx1[64]*Gx2[59] + Gx1[65]*Gx2[71] + Gx1[66]*Gx2[83] + Gx1[67]*Gx2[95] + Gx1[68]*Gx2[107] + Gx1[69]*Gx2[119] + Gx1[70]*Gx2[131] + Gx1[71]*Gx2[143];
Gx3[72] = + Gx1[72]*Gx2[0] + Gx1[73]*Gx2[12] + Gx1[74]*Gx2[24] + Gx1[75]*Gx2[36] + Gx1[76]*Gx2[48] + Gx1[77]*Gx2[60] + Gx1[78]*Gx2[72] + Gx1[79]*Gx2[84] + Gx1[80]*Gx2[96] + Gx1[81]*Gx2[108] + Gx1[82]*Gx2[120] + Gx1[83]*Gx2[132];
Gx3[73] = + Gx1[72]*Gx2[1] + Gx1[73]*Gx2[13] + Gx1[74]*Gx2[25] + Gx1[75]*Gx2[37] + Gx1[76]*Gx2[49] + Gx1[77]*Gx2[61] + Gx1[78]*Gx2[73] + Gx1[79]*Gx2[85] + Gx1[80]*Gx2[97] + Gx1[81]*Gx2[109] + Gx1[82]*Gx2[121] + Gx1[83]*Gx2[133];
Gx3[74] = + Gx1[72]*Gx2[2] + Gx1[73]*Gx2[14] + Gx1[74]*Gx2[26] + Gx1[75]*Gx2[38] + Gx1[76]*Gx2[50] + Gx1[77]*Gx2[62] + Gx1[78]*Gx2[74] + Gx1[79]*Gx2[86] + Gx1[80]*Gx2[98] + Gx1[81]*Gx2[110] + Gx1[82]*Gx2[122] + Gx1[83]*Gx2[134];
Gx3[75] = + Gx1[72]*Gx2[3] + Gx1[73]*Gx2[15] + Gx1[74]*Gx2[27] + Gx1[75]*Gx2[39] + Gx1[76]*Gx2[51] + Gx1[77]*Gx2[63] + Gx1[78]*Gx2[75] + Gx1[79]*Gx2[87] + Gx1[80]*Gx2[99] + Gx1[81]*Gx2[111] + Gx1[82]*Gx2[123] + Gx1[83]*Gx2[135];
Gx3[76] = + Gx1[72]*Gx2[4] + Gx1[73]*Gx2[16] + Gx1[74]*Gx2[28] + Gx1[75]*Gx2[40] + Gx1[76]*Gx2[52] + Gx1[77]*Gx2[64] + Gx1[78]*Gx2[76] + Gx1[79]*Gx2[88] + Gx1[80]*Gx2[100] + Gx1[81]*Gx2[112] + Gx1[82]*Gx2[124] + Gx1[83]*Gx2[136];
Gx3[77] = + Gx1[72]*Gx2[5] + Gx1[73]*Gx2[17] + Gx1[74]*Gx2[29] + Gx1[75]*Gx2[41] + Gx1[76]*Gx2[53] + Gx1[77]*Gx2[65] + Gx1[78]*Gx2[77] + Gx1[79]*Gx2[89] + Gx1[80]*Gx2[101] + Gx1[81]*Gx2[113] + Gx1[82]*Gx2[125] + Gx1[83]*Gx2[137];
Gx3[78] = + Gx1[72]*Gx2[6] + Gx1[73]*Gx2[18] + Gx1[74]*Gx2[30] + Gx1[75]*Gx2[42] + Gx1[76]*Gx2[54] + Gx1[77]*Gx2[66] + Gx1[78]*Gx2[78] + Gx1[79]*Gx2[90] + Gx1[80]*Gx2[102] + Gx1[81]*Gx2[114] + Gx1[82]*Gx2[126] + Gx1[83]*Gx2[138];
Gx3[79] = + Gx1[72]*Gx2[7] + Gx1[73]*Gx2[19] + Gx1[74]*Gx2[31] + Gx1[75]*Gx2[43] + Gx1[76]*Gx2[55] + Gx1[77]*Gx2[67] + Gx1[78]*Gx2[79] + Gx1[79]*Gx2[91] + Gx1[80]*Gx2[103] + Gx1[81]*Gx2[115] + Gx1[82]*Gx2[127] + Gx1[83]*Gx2[139];
Gx3[80] = + Gx1[72]*Gx2[8] + Gx1[73]*Gx2[20] + Gx1[74]*Gx2[32] + Gx1[75]*Gx2[44] + Gx1[76]*Gx2[56] + Gx1[77]*Gx2[68] + Gx1[78]*Gx2[80] + Gx1[79]*Gx2[92] + Gx1[80]*Gx2[104] + Gx1[81]*Gx2[116] + Gx1[82]*Gx2[128] + Gx1[83]*Gx2[140];
Gx3[81] = + Gx1[72]*Gx2[9] + Gx1[73]*Gx2[21] + Gx1[74]*Gx2[33] + Gx1[75]*Gx2[45] + Gx1[76]*Gx2[57] + Gx1[77]*Gx2[69] + Gx1[78]*Gx2[81] + Gx1[79]*Gx2[93] + Gx1[80]*Gx2[105] + Gx1[81]*Gx2[117] + Gx1[82]*Gx2[129] + Gx1[83]*Gx2[141];
Gx3[82] = + Gx1[72]*Gx2[10] + Gx1[73]*Gx2[22] + Gx1[74]*Gx2[34] + Gx1[75]*Gx2[46] + Gx1[76]*Gx2[58] + Gx1[77]*Gx2[70] + Gx1[78]*Gx2[82] + Gx1[79]*Gx2[94] + Gx1[80]*Gx2[106] + Gx1[81]*Gx2[118] + Gx1[82]*Gx2[130] + Gx1[83]*Gx2[142];
Gx3[83] = + Gx1[72]*Gx2[11] + Gx1[73]*Gx2[23] + Gx1[74]*Gx2[35] + Gx1[75]*Gx2[47] + Gx1[76]*Gx2[59] + Gx1[77]*Gx2[71] + Gx1[78]*Gx2[83] + Gx1[79]*Gx2[95] + Gx1[80]*Gx2[107] + Gx1[81]*Gx2[119] + Gx1[82]*Gx2[131] + Gx1[83]*Gx2[143];
Gx3[84] = + Gx1[84]*Gx2[0] + Gx1[85]*Gx2[12] + Gx1[86]*Gx2[24] + Gx1[87]*Gx2[36] + Gx1[88]*Gx2[48] + Gx1[89]*Gx2[60] + Gx1[90]*Gx2[72] + Gx1[91]*Gx2[84] + Gx1[92]*Gx2[96] + Gx1[93]*Gx2[108] + Gx1[94]*Gx2[120] + Gx1[95]*Gx2[132];
Gx3[85] = + Gx1[84]*Gx2[1] + Gx1[85]*Gx2[13] + Gx1[86]*Gx2[25] + Gx1[87]*Gx2[37] + Gx1[88]*Gx2[49] + Gx1[89]*Gx2[61] + Gx1[90]*Gx2[73] + Gx1[91]*Gx2[85] + Gx1[92]*Gx2[97] + Gx1[93]*Gx2[109] + Gx1[94]*Gx2[121] + Gx1[95]*Gx2[133];
Gx3[86] = + Gx1[84]*Gx2[2] + Gx1[85]*Gx2[14] + Gx1[86]*Gx2[26] + Gx1[87]*Gx2[38] + Gx1[88]*Gx2[50] + Gx1[89]*Gx2[62] + Gx1[90]*Gx2[74] + Gx1[91]*Gx2[86] + Gx1[92]*Gx2[98] + Gx1[93]*Gx2[110] + Gx1[94]*Gx2[122] + Gx1[95]*Gx2[134];
Gx3[87] = + Gx1[84]*Gx2[3] + Gx1[85]*Gx2[15] + Gx1[86]*Gx2[27] + Gx1[87]*Gx2[39] + Gx1[88]*Gx2[51] + Gx1[89]*Gx2[63] + Gx1[90]*Gx2[75] + Gx1[91]*Gx2[87] + Gx1[92]*Gx2[99] + Gx1[93]*Gx2[111] + Gx1[94]*Gx2[123] + Gx1[95]*Gx2[135];
Gx3[88] = + Gx1[84]*Gx2[4] + Gx1[85]*Gx2[16] + Gx1[86]*Gx2[28] + Gx1[87]*Gx2[40] + Gx1[88]*Gx2[52] + Gx1[89]*Gx2[64] + Gx1[90]*Gx2[76] + Gx1[91]*Gx2[88] + Gx1[92]*Gx2[100] + Gx1[93]*Gx2[112] + Gx1[94]*Gx2[124] + Gx1[95]*Gx2[136];
Gx3[89] = + Gx1[84]*Gx2[5] + Gx1[85]*Gx2[17] + Gx1[86]*Gx2[29] + Gx1[87]*Gx2[41] + Gx1[88]*Gx2[53] + Gx1[89]*Gx2[65] + Gx1[90]*Gx2[77] + Gx1[91]*Gx2[89] + Gx1[92]*Gx2[101] + Gx1[93]*Gx2[113] + Gx1[94]*Gx2[125] + Gx1[95]*Gx2[137];
Gx3[90] = + Gx1[84]*Gx2[6] + Gx1[85]*Gx2[18] + Gx1[86]*Gx2[30] + Gx1[87]*Gx2[42] + Gx1[88]*Gx2[54] + Gx1[89]*Gx2[66] + Gx1[90]*Gx2[78] + Gx1[91]*Gx2[90] + Gx1[92]*Gx2[102] + Gx1[93]*Gx2[114] + Gx1[94]*Gx2[126] + Gx1[95]*Gx2[138];
Gx3[91] = + Gx1[84]*Gx2[7] + Gx1[85]*Gx2[19] + Gx1[86]*Gx2[31] + Gx1[87]*Gx2[43] + Gx1[88]*Gx2[55] + Gx1[89]*Gx2[67] + Gx1[90]*Gx2[79] + Gx1[91]*Gx2[91] + Gx1[92]*Gx2[103] + Gx1[93]*Gx2[115] + Gx1[94]*Gx2[127] + Gx1[95]*Gx2[139];
Gx3[92] = + Gx1[84]*Gx2[8] + Gx1[85]*Gx2[20] + Gx1[86]*Gx2[32] + Gx1[87]*Gx2[44] + Gx1[88]*Gx2[56] + Gx1[89]*Gx2[68] + Gx1[90]*Gx2[80] + Gx1[91]*Gx2[92] + Gx1[92]*Gx2[104] + Gx1[93]*Gx2[116] + Gx1[94]*Gx2[128] + Gx1[95]*Gx2[140];
Gx3[93] = + Gx1[84]*Gx2[9] + Gx1[85]*Gx2[21] + Gx1[86]*Gx2[33] + Gx1[87]*Gx2[45] + Gx1[88]*Gx2[57] + Gx1[89]*Gx2[69] + Gx1[90]*Gx2[81] + Gx1[91]*Gx2[93] + Gx1[92]*Gx2[105] + Gx1[93]*Gx2[117] + Gx1[94]*Gx2[129] + Gx1[95]*Gx2[141];
Gx3[94] = + Gx1[84]*Gx2[10] + Gx1[85]*Gx2[22] + Gx1[86]*Gx2[34] + Gx1[87]*Gx2[46] + Gx1[88]*Gx2[58] + Gx1[89]*Gx2[70] + Gx1[90]*Gx2[82] + Gx1[91]*Gx2[94] + Gx1[92]*Gx2[106] + Gx1[93]*Gx2[118] + Gx1[94]*Gx2[130] + Gx1[95]*Gx2[142];
Gx3[95] = + Gx1[84]*Gx2[11] + Gx1[85]*Gx2[23] + Gx1[86]*Gx2[35] + Gx1[87]*Gx2[47] + Gx1[88]*Gx2[59] + Gx1[89]*Gx2[71] + Gx1[90]*Gx2[83] + Gx1[91]*Gx2[95] + Gx1[92]*Gx2[107] + Gx1[93]*Gx2[119] + Gx1[94]*Gx2[131] + Gx1[95]*Gx2[143];
Gx3[96] = + Gx1[96]*Gx2[0] + Gx1[97]*Gx2[12] + Gx1[98]*Gx2[24] + Gx1[99]*Gx2[36] + Gx1[100]*Gx2[48] + Gx1[101]*Gx2[60] + Gx1[102]*Gx2[72] + Gx1[103]*Gx2[84] + Gx1[104]*Gx2[96] + Gx1[105]*Gx2[108] + Gx1[106]*Gx2[120] + Gx1[107]*Gx2[132];
Gx3[97] = + Gx1[96]*Gx2[1] + Gx1[97]*Gx2[13] + Gx1[98]*Gx2[25] + Gx1[99]*Gx2[37] + Gx1[100]*Gx2[49] + Gx1[101]*Gx2[61] + Gx1[102]*Gx2[73] + Gx1[103]*Gx2[85] + Gx1[104]*Gx2[97] + Gx1[105]*Gx2[109] + Gx1[106]*Gx2[121] + Gx1[107]*Gx2[133];
Gx3[98] = + Gx1[96]*Gx2[2] + Gx1[97]*Gx2[14] + Gx1[98]*Gx2[26] + Gx1[99]*Gx2[38] + Gx1[100]*Gx2[50] + Gx1[101]*Gx2[62] + Gx1[102]*Gx2[74] + Gx1[103]*Gx2[86] + Gx1[104]*Gx2[98] + Gx1[105]*Gx2[110] + Gx1[106]*Gx2[122] + Gx1[107]*Gx2[134];
Gx3[99] = + Gx1[96]*Gx2[3] + Gx1[97]*Gx2[15] + Gx1[98]*Gx2[27] + Gx1[99]*Gx2[39] + Gx1[100]*Gx2[51] + Gx1[101]*Gx2[63] + Gx1[102]*Gx2[75] + Gx1[103]*Gx2[87] + Gx1[104]*Gx2[99] + Gx1[105]*Gx2[111] + Gx1[106]*Gx2[123] + Gx1[107]*Gx2[135];
Gx3[100] = + Gx1[96]*Gx2[4] + Gx1[97]*Gx2[16] + Gx1[98]*Gx2[28] + Gx1[99]*Gx2[40] + Gx1[100]*Gx2[52] + Gx1[101]*Gx2[64] + Gx1[102]*Gx2[76] + Gx1[103]*Gx2[88] + Gx1[104]*Gx2[100] + Gx1[105]*Gx2[112] + Gx1[106]*Gx2[124] + Gx1[107]*Gx2[136];
Gx3[101] = + Gx1[96]*Gx2[5] + Gx1[97]*Gx2[17] + Gx1[98]*Gx2[29] + Gx1[99]*Gx2[41] + Gx1[100]*Gx2[53] + Gx1[101]*Gx2[65] + Gx1[102]*Gx2[77] + Gx1[103]*Gx2[89] + Gx1[104]*Gx2[101] + Gx1[105]*Gx2[113] + Gx1[106]*Gx2[125] + Gx1[107]*Gx2[137];
Gx3[102] = + Gx1[96]*Gx2[6] + Gx1[97]*Gx2[18] + Gx1[98]*Gx2[30] + Gx1[99]*Gx2[42] + Gx1[100]*Gx2[54] + Gx1[101]*Gx2[66] + Gx1[102]*Gx2[78] + Gx1[103]*Gx2[90] + Gx1[104]*Gx2[102] + Gx1[105]*Gx2[114] + Gx1[106]*Gx2[126] + Gx1[107]*Gx2[138];
Gx3[103] = + Gx1[96]*Gx2[7] + Gx1[97]*Gx2[19] + Gx1[98]*Gx2[31] + Gx1[99]*Gx2[43] + Gx1[100]*Gx2[55] + Gx1[101]*Gx2[67] + Gx1[102]*Gx2[79] + Gx1[103]*Gx2[91] + Gx1[104]*Gx2[103] + Gx1[105]*Gx2[115] + Gx1[106]*Gx2[127] + Gx1[107]*Gx2[139];
Gx3[104] = + Gx1[96]*Gx2[8] + Gx1[97]*Gx2[20] + Gx1[98]*Gx2[32] + Gx1[99]*Gx2[44] + Gx1[100]*Gx2[56] + Gx1[101]*Gx2[68] + Gx1[102]*Gx2[80] + Gx1[103]*Gx2[92] + Gx1[104]*Gx2[104] + Gx1[105]*Gx2[116] + Gx1[106]*Gx2[128] + Gx1[107]*Gx2[140];
Gx3[105] = + Gx1[96]*Gx2[9] + Gx1[97]*Gx2[21] + Gx1[98]*Gx2[33] + Gx1[99]*Gx2[45] + Gx1[100]*Gx2[57] + Gx1[101]*Gx2[69] + Gx1[102]*Gx2[81] + Gx1[103]*Gx2[93] + Gx1[104]*Gx2[105] + Gx1[105]*Gx2[117] + Gx1[106]*Gx2[129] + Gx1[107]*Gx2[141];
Gx3[106] = + Gx1[96]*Gx2[10] + Gx1[97]*Gx2[22] + Gx1[98]*Gx2[34] + Gx1[99]*Gx2[46] + Gx1[100]*Gx2[58] + Gx1[101]*Gx2[70] + Gx1[102]*Gx2[82] + Gx1[103]*Gx2[94] + Gx1[104]*Gx2[106] + Gx1[105]*Gx2[118] + Gx1[106]*Gx2[130] + Gx1[107]*Gx2[142];
Gx3[107] = + Gx1[96]*Gx2[11] + Gx1[97]*Gx2[23] + Gx1[98]*Gx2[35] + Gx1[99]*Gx2[47] + Gx1[100]*Gx2[59] + Gx1[101]*Gx2[71] + Gx1[102]*Gx2[83] + Gx1[103]*Gx2[95] + Gx1[104]*Gx2[107] + Gx1[105]*Gx2[119] + Gx1[106]*Gx2[131] + Gx1[107]*Gx2[143];
Gx3[108] = + Gx1[108]*Gx2[0] + Gx1[109]*Gx2[12] + Gx1[110]*Gx2[24] + Gx1[111]*Gx2[36] + Gx1[112]*Gx2[48] + Gx1[113]*Gx2[60] + Gx1[114]*Gx2[72] + Gx1[115]*Gx2[84] + Gx1[116]*Gx2[96] + Gx1[117]*Gx2[108] + Gx1[118]*Gx2[120] + Gx1[119]*Gx2[132];
Gx3[109] = + Gx1[108]*Gx2[1] + Gx1[109]*Gx2[13] + Gx1[110]*Gx2[25] + Gx1[111]*Gx2[37] + Gx1[112]*Gx2[49] + Gx1[113]*Gx2[61] + Gx1[114]*Gx2[73] + Gx1[115]*Gx2[85] + Gx1[116]*Gx2[97] + Gx1[117]*Gx2[109] + Gx1[118]*Gx2[121] + Gx1[119]*Gx2[133];
Gx3[110] = + Gx1[108]*Gx2[2] + Gx1[109]*Gx2[14] + Gx1[110]*Gx2[26] + Gx1[111]*Gx2[38] + Gx1[112]*Gx2[50] + Gx1[113]*Gx2[62] + Gx1[114]*Gx2[74] + Gx1[115]*Gx2[86] + Gx1[116]*Gx2[98] + Gx1[117]*Gx2[110] + Gx1[118]*Gx2[122] + Gx1[119]*Gx2[134];
Gx3[111] = + Gx1[108]*Gx2[3] + Gx1[109]*Gx2[15] + Gx1[110]*Gx2[27] + Gx1[111]*Gx2[39] + Gx1[112]*Gx2[51] + Gx1[113]*Gx2[63] + Gx1[114]*Gx2[75] + Gx1[115]*Gx2[87] + Gx1[116]*Gx2[99] + Gx1[117]*Gx2[111] + Gx1[118]*Gx2[123] + Gx1[119]*Gx2[135];
Gx3[112] = + Gx1[108]*Gx2[4] + Gx1[109]*Gx2[16] + Gx1[110]*Gx2[28] + Gx1[111]*Gx2[40] + Gx1[112]*Gx2[52] + Gx1[113]*Gx2[64] + Gx1[114]*Gx2[76] + Gx1[115]*Gx2[88] + Gx1[116]*Gx2[100] + Gx1[117]*Gx2[112] + Gx1[118]*Gx2[124] + Gx1[119]*Gx2[136];
Gx3[113] = + Gx1[108]*Gx2[5] + Gx1[109]*Gx2[17] + Gx1[110]*Gx2[29] + Gx1[111]*Gx2[41] + Gx1[112]*Gx2[53] + Gx1[113]*Gx2[65] + Gx1[114]*Gx2[77] + Gx1[115]*Gx2[89] + Gx1[116]*Gx2[101] + Gx1[117]*Gx2[113] + Gx1[118]*Gx2[125] + Gx1[119]*Gx2[137];
Gx3[114] = + Gx1[108]*Gx2[6] + Gx1[109]*Gx2[18] + Gx1[110]*Gx2[30] + Gx1[111]*Gx2[42] + Gx1[112]*Gx2[54] + Gx1[113]*Gx2[66] + Gx1[114]*Gx2[78] + Gx1[115]*Gx2[90] + Gx1[116]*Gx2[102] + Gx1[117]*Gx2[114] + Gx1[118]*Gx2[126] + Gx1[119]*Gx2[138];
Gx3[115] = + Gx1[108]*Gx2[7] + Gx1[109]*Gx2[19] + Gx1[110]*Gx2[31] + Gx1[111]*Gx2[43] + Gx1[112]*Gx2[55] + Gx1[113]*Gx2[67] + Gx1[114]*Gx2[79] + Gx1[115]*Gx2[91] + Gx1[116]*Gx2[103] + Gx1[117]*Gx2[115] + Gx1[118]*Gx2[127] + Gx1[119]*Gx2[139];
Gx3[116] = + Gx1[108]*Gx2[8] + Gx1[109]*Gx2[20] + Gx1[110]*Gx2[32] + Gx1[111]*Gx2[44] + Gx1[112]*Gx2[56] + Gx1[113]*Gx2[68] + Gx1[114]*Gx2[80] + Gx1[115]*Gx2[92] + Gx1[116]*Gx2[104] + Gx1[117]*Gx2[116] + Gx1[118]*Gx2[128] + Gx1[119]*Gx2[140];
Gx3[117] = + Gx1[108]*Gx2[9] + Gx1[109]*Gx2[21] + Gx1[110]*Gx2[33] + Gx1[111]*Gx2[45] + Gx1[112]*Gx2[57] + Gx1[113]*Gx2[69] + Gx1[114]*Gx2[81] + Gx1[115]*Gx2[93] + Gx1[116]*Gx2[105] + Gx1[117]*Gx2[117] + Gx1[118]*Gx2[129] + Gx1[119]*Gx2[141];
Gx3[118] = + Gx1[108]*Gx2[10] + Gx1[109]*Gx2[22] + Gx1[110]*Gx2[34] + Gx1[111]*Gx2[46] + Gx1[112]*Gx2[58] + Gx1[113]*Gx2[70] + Gx1[114]*Gx2[82] + Gx1[115]*Gx2[94] + Gx1[116]*Gx2[106] + Gx1[117]*Gx2[118] + Gx1[118]*Gx2[130] + Gx1[119]*Gx2[142];
Gx3[119] = + Gx1[108]*Gx2[11] + Gx1[109]*Gx2[23] + Gx1[110]*Gx2[35] + Gx1[111]*Gx2[47] + Gx1[112]*Gx2[59] + Gx1[113]*Gx2[71] + Gx1[114]*Gx2[83] + Gx1[115]*Gx2[95] + Gx1[116]*Gx2[107] + Gx1[117]*Gx2[119] + Gx1[118]*Gx2[131] + Gx1[119]*Gx2[143];
Gx3[120] = + Gx1[120]*Gx2[0] + Gx1[121]*Gx2[12] + Gx1[122]*Gx2[24] + Gx1[123]*Gx2[36] + Gx1[124]*Gx2[48] + Gx1[125]*Gx2[60] + Gx1[126]*Gx2[72] + Gx1[127]*Gx2[84] + Gx1[128]*Gx2[96] + Gx1[129]*Gx2[108] + Gx1[130]*Gx2[120] + Gx1[131]*Gx2[132];
Gx3[121] = + Gx1[120]*Gx2[1] + Gx1[121]*Gx2[13] + Gx1[122]*Gx2[25] + Gx1[123]*Gx2[37] + Gx1[124]*Gx2[49] + Gx1[125]*Gx2[61] + Gx1[126]*Gx2[73] + Gx1[127]*Gx2[85] + Gx1[128]*Gx2[97] + Gx1[129]*Gx2[109] + Gx1[130]*Gx2[121] + Gx1[131]*Gx2[133];
Gx3[122] = + Gx1[120]*Gx2[2] + Gx1[121]*Gx2[14] + Gx1[122]*Gx2[26] + Gx1[123]*Gx2[38] + Gx1[124]*Gx2[50] + Gx1[125]*Gx2[62] + Gx1[126]*Gx2[74] + Gx1[127]*Gx2[86] + Gx1[128]*Gx2[98] + Gx1[129]*Gx2[110] + Gx1[130]*Gx2[122] + Gx1[131]*Gx2[134];
Gx3[123] = + Gx1[120]*Gx2[3] + Gx1[121]*Gx2[15] + Gx1[122]*Gx2[27] + Gx1[123]*Gx2[39] + Gx1[124]*Gx2[51] + Gx1[125]*Gx2[63] + Gx1[126]*Gx2[75] + Gx1[127]*Gx2[87] + Gx1[128]*Gx2[99] + Gx1[129]*Gx2[111] + Gx1[130]*Gx2[123] + Gx1[131]*Gx2[135];
Gx3[124] = + Gx1[120]*Gx2[4] + Gx1[121]*Gx2[16] + Gx1[122]*Gx2[28] + Gx1[123]*Gx2[40] + Gx1[124]*Gx2[52] + Gx1[125]*Gx2[64] + Gx1[126]*Gx2[76] + Gx1[127]*Gx2[88] + Gx1[128]*Gx2[100] + Gx1[129]*Gx2[112] + Gx1[130]*Gx2[124] + Gx1[131]*Gx2[136];
Gx3[125] = + Gx1[120]*Gx2[5] + Gx1[121]*Gx2[17] + Gx1[122]*Gx2[29] + Gx1[123]*Gx2[41] + Gx1[124]*Gx2[53] + Gx1[125]*Gx2[65] + Gx1[126]*Gx2[77] + Gx1[127]*Gx2[89] + Gx1[128]*Gx2[101] + Gx1[129]*Gx2[113] + Gx1[130]*Gx2[125] + Gx1[131]*Gx2[137];
Gx3[126] = + Gx1[120]*Gx2[6] + Gx1[121]*Gx2[18] + Gx1[122]*Gx2[30] + Gx1[123]*Gx2[42] + Gx1[124]*Gx2[54] + Gx1[125]*Gx2[66] + Gx1[126]*Gx2[78] + Gx1[127]*Gx2[90] + Gx1[128]*Gx2[102] + Gx1[129]*Gx2[114] + Gx1[130]*Gx2[126] + Gx1[131]*Gx2[138];
Gx3[127] = + Gx1[120]*Gx2[7] + Gx1[121]*Gx2[19] + Gx1[122]*Gx2[31] + Gx1[123]*Gx2[43] + Gx1[124]*Gx2[55] + Gx1[125]*Gx2[67] + Gx1[126]*Gx2[79] + Gx1[127]*Gx2[91] + Gx1[128]*Gx2[103] + Gx1[129]*Gx2[115] + Gx1[130]*Gx2[127] + Gx1[131]*Gx2[139];
Gx3[128] = + Gx1[120]*Gx2[8] + Gx1[121]*Gx2[20] + Gx1[122]*Gx2[32] + Gx1[123]*Gx2[44] + Gx1[124]*Gx2[56] + Gx1[125]*Gx2[68] + Gx1[126]*Gx2[80] + Gx1[127]*Gx2[92] + Gx1[128]*Gx2[104] + Gx1[129]*Gx2[116] + Gx1[130]*Gx2[128] + Gx1[131]*Gx2[140];
Gx3[129] = + Gx1[120]*Gx2[9] + Gx1[121]*Gx2[21] + Gx1[122]*Gx2[33] + Gx1[123]*Gx2[45] + Gx1[124]*Gx2[57] + Gx1[125]*Gx2[69] + Gx1[126]*Gx2[81] + Gx1[127]*Gx2[93] + Gx1[128]*Gx2[105] + Gx1[129]*Gx2[117] + Gx1[130]*Gx2[129] + Gx1[131]*Gx2[141];
Gx3[130] = + Gx1[120]*Gx2[10] + Gx1[121]*Gx2[22] + Gx1[122]*Gx2[34] + Gx1[123]*Gx2[46] + Gx1[124]*Gx2[58] + Gx1[125]*Gx2[70] + Gx1[126]*Gx2[82] + Gx1[127]*Gx2[94] + Gx1[128]*Gx2[106] + Gx1[129]*Gx2[118] + Gx1[130]*Gx2[130] + Gx1[131]*Gx2[142];
Gx3[131] = + Gx1[120]*Gx2[11] + Gx1[121]*Gx2[23] + Gx1[122]*Gx2[35] + Gx1[123]*Gx2[47] + Gx1[124]*Gx2[59] + Gx1[125]*Gx2[71] + Gx1[126]*Gx2[83] + Gx1[127]*Gx2[95] + Gx1[128]*Gx2[107] + Gx1[129]*Gx2[119] + Gx1[130]*Gx2[131] + Gx1[131]*Gx2[143];
Gx3[132] = + Gx1[132]*Gx2[0] + Gx1[133]*Gx2[12] + Gx1[134]*Gx2[24] + Gx1[135]*Gx2[36] + Gx1[136]*Gx2[48] + Gx1[137]*Gx2[60] + Gx1[138]*Gx2[72] + Gx1[139]*Gx2[84] + Gx1[140]*Gx2[96] + Gx1[141]*Gx2[108] + Gx1[142]*Gx2[120] + Gx1[143]*Gx2[132];
Gx3[133] = + Gx1[132]*Gx2[1] + Gx1[133]*Gx2[13] + Gx1[134]*Gx2[25] + Gx1[135]*Gx2[37] + Gx1[136]*Gx2[49] + Gx1[137]*Gx2[61] + Gx1[138]*Gx2[73] + Gx1[139]*Gx2[85] + Gx1[140]*Gx2[97] + Gx1[141]*Gx2[109] + Gx1[142]*Gx2[121] + Gx1[143]*Gx2[133];
Gx3[134] = + Gx1[132]*Gx2[2] + Gx1[133]*Gx2[14] + Gx1[134]*Gx2[26] + Gx1[135]*Gx2[38] + Gx1[136]*Gx2[50] + Gx1[137]*Gx2[62] + Gx1[138]*Gx2[74] + Gx1[139]*Gx2[86] + Gx1[140]*Gx2[98] + Gx1[141]*Gx2[110] + Gx1[142]*Gx2[122] + Gx1[143]*Gx2[134];
Gx3[135] = + Gx1[132]*Gx2[3] + Gx1[133]*Gx2[15] + Gx1[134]*Gx2[27] + Gx1[135]*Gx2[39] + Gx1[136]*Gx2[51] + Gx1[137]*Gx2[63] + Gx1[138]*Gx2[75] + Gx1[139]*Gx2[87] + Gx1[140]*Gx2[99] + Gx1[141]*Gx2[111] + Gx1[142]*Gx2[123] + Gx1[143]*Gx2[135];
Gx3[136] = + Gx1[132]*Gx2[4] + Gx1[133]*Gx2[16] + Gx1[134]*Gx2[28] + Gx1[135]*Gx2[40] + Gx1[136]*Gx2[52] + Gx1[137]*Gx2[64] + Gx1[138]*Gx2[76] + Gx1[139]*Gx2[88] + Gx1[140]*Gx2[100] + Gx1[141]*Gx2[112] + Gx1[142]*Gx2[124] + Gx1[143]*Gx2[136];
Gx3[137] = + Gx1[132]*Gx2[5] + Gx1[133]*Gx2[17] + Gx1[134]*Gx2[29] + Gx1[135]*Gx2[41] + Gx1[136]*Gx2[53] + Gx1[137]*Gx2[65] + Gx1[138]*Gx2[77] + Gx1[139]*Gx2[89] + Gx1[140]*Gx2[101] + Gx1[141]*Gx2[113] + Gx1[142]*Gx2[125] + Gx1[143]*Gx2[137];
Gx3[138] = + Gx1[132]*Gx2[6] + Gx1[133]*Gx2[18] + Gx1[134]*Gx2[30] + Gx1[135]*Gx2[42] + Gx1[136]*Gx2[54] + Gx1[137]*Gx2[66] + Gx1[138]*Gx2[78] + Gx1[139]*Gx2[90] + Gx1[140]*Gx2[102] + Gx1[141]*Gx2[114] + Gx1[142]*Gx2[126] + Gx1[143]*Gx2[138];
Gx3[139] = + Gx1[132]*Gx2[7] + Gx1[133]*Gx2[19] + Gx1[134]*Gx2[31] + Gx1[135]*Gx2[43] + Gx1[136]*Gx2[55] + Gx1[137]*Gx2[67] + Gx1[138]*Gx2[79] + Gx1[139]*Gx2[91] + Gx1[140]*Gx2[103] + Gx1[141]*Gx2[115] + Gx1[142]*Gx2[127] + Gx1[143]*Gx2[139];
Gx3[140] = + Gx1[132]*Gx2[8] + Gx1[133]*Gx2[20] + Gx1[134]*Gx2[32] + Gx1[135]*Gx2[44] + Gx1[136]*Gx2[56] + Gx1[137]*Gx2[68] + Gx1[138]*Gx2[80] + Gx1[139]*Gx2[92] + Gx1[140]*Gx2[104] + Gx1[141]*Gx2[116] + Gx1[142]*Gx2[128] + Gx1[143]*Gx2[140];
Gx3[141] = + Gx1[132]*Gx2[9] + Gx1[133]*Gx2[21] + Gx1[134]*Gx2[33] + Gx1[135]*Gx2[45] + Gx1[136]*Gx2[57] + Gx1[137]*Gx2[69] + Gx1[138]*Gx2[81] + Gx1[139]*Gx2[93] + Gx1[140]*Gx2[105] + Gx1[141]*Gx2[117] + Gx1[142]*Gx2[129] + Gx1[143]*Gx2[141];
Gx3[142] = + Gx1[132]*Gx2[10] + Gx1[133]*Gx2[22] + Gx1[134]*Gx2[34] + Gx1[135]*Gx2[46] + Gx1[136]*Gx2[58] + Gx1[137]*Gx2[70] + Gx1[138]*Gx2[82] + Gx1[139]*Gx2[94] + Gx1[140]*Gx2[106] + Gx1[141]*Gx2[118] + Gx1[142]*Gx2[130] + Gx1[143]*Gx2[142];
Gx3[143] = + Gx1[132]*Gx2[11] + Gx1[133]*Gx2[23] + Gx1[134]*Gx2[35] + Gx1[135]*Gx2[47] + Gx1[136]*Gx2[59] + Gx1[137]*Gx2[71] + Gx1[138]*Gx2[83] + Gx1[139]*Gx2[95] + Gx1[140]*Gx2[107] + Gx1[141]*Gx2[119] + Gx1[142]*Gx2[131] + Gx1[143]*Gx2[143];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28] + Gx1[8]*Gu1[32] + Gx1[9]*Gu1[36] + Gx1[10]*Gu1[40] + Gx1[11]*Gu1[44];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29] + Gx1[8]*Gu1[33] + Gx1[9]*Gu1[37] + Gx1[10]*Gu1[41] + Gx1[11]*Gu1[45];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30] + Gx1[8]*Gu1[34] + Gx1[9]*Gu1[38] + Gx1[10]*Gu1[42] + Gx1[11]*Gu1[46];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31] + Gx1[8]*Gu1[35] + Gx1[9]*Gu1[39] + Gx1[10]*Gu1[43] + Gx1[11]*Gu1[47];
Gu2[4] = + Gx1[12]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[14]*Gu1[8] + Gx1[15]*Gu1[12] + Gx1[16]*Gu1[16] + Gx1[17]*Gu1[20] + Gx1[18]*Gu1[24] + Gx1[19]*Gu1[28] + Gx1[20]*Gu1[32] + Gx1[21]*Gu1[36] + Gx1[22]*Gu1[40] + Gx1[23]*Gu1[44];
Gu2[5] = + Gx1[12]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[14]*Gu1[9] + Gx1[15]*Gu1[13] + Gx1[16]*Gu1[17] + Gx1[17]*Gu1[21] + Gx1[18]*Gu1[25] + Gx1[19]*Gu1[29] + Gx1[20]*Gu1[33] + Gx1[21]*Gu1[37] + Gx1[22]*Gu1[41] + Gx1[23]*Gu1[45];
Gu2[6] = + Gx1[12]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[10] + Gx1[15]*Gu1[14] + Gx1[16]*Gu1[18] + Gx1[17]*Gu1[22] + Gx1[18]*Gu1[26] + Gx1[19]*Gu1[30] + Gx1[20]*Gu1[34] + Gx1[21]*Gu1[38] + Gx1[22]*Gu1[42] + Gx1[23]*Gu1[46];
Gu2[7] = + Gx1[12]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[11] + Gx1[15]*Gu1[15] + Gx1[16]*Gu1[19] + Gx1[17]*Gu1[23] + Gx1[18]*Gu1[27] + Gx1[19]*Gu1[31] + Gx1[20]*Gu1[35] + Gx1[21]*Gu1[39] + Gx1[22]*Gu1[43] + Gx1[23]*Gu1[47];
Gu2[8] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[27]*Gu1[12] + Gx1[28]*Gu1[16] + Gx1[29]*Gu1[20] + Gx1[30]*Gu1[24] + Gx1[31]*Gu1[28] + Gx1[32]*Gu1[32] + Gx1[33]*Gu1[36] + Gx1[34]*Gu1[40] + Gx1[35]*Gu1[44];
Gu2[9] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[27]*Gu1[13] + Gx1[28]*Gu1[17] + Gx1[29]*Gu1[21] + Gx1[30]*Gu1[25] + Gx1[31]*Gu1[29] + Gx1[32]*Gu1[33] + Gx1[33]*Gu1[37] + Gx1[34]*Gu1[41] + Gx1[35]*Gu1[45];
Gu2[10] = + Gx1[24]*Gu1[2] + Gx1[25]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[27]*Gu1[14] + Gx1[28]*Gu1[18] + Gx1[29]*Gu1[22] + Gx1[30]*Gu1[26] + Gx1[31]*Gu1[30] + Gx1[32]*Gu1[34] + Gx1[33]*Gu1[38] + Gx1[34]*Gu1[42] + Gx1[35]*Gu1[46];
Gu2[11] = + Gx1[24]*Gu1[3] + Gx1[25]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[27]*Gu1[15] + Gx1[28]*Gu1[19] + Gx1[29]*Gu1[23] + Gx1[30]*Gu1[27] + Gx1[31]*Gu1[31] + Gx1[32]*Gu1[35] + Gx1[33]*Gu1[39] + Gx1[34]*Gu1[43] + Gx1[35]*Gu1[47];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[40]*Gu1[16] + Gx1[41]*Gu1[20] + Gx1[42]*Gu1[24] + Gx1[43]*Gu1[28] + Gx1[44]*Gu1[32] + Gx1[45]*Gu1[36] + Gx1[46]*Gu1[40] + Gx1[47]*Gu1[44];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[40]*Gu1[17] + Gx1[41]*Gu1[21] + Gx1[42]*Gu1[25] + Gx1[43]*Gu1[29] + Gx1[44]*Gu1[33] + Gx1[45]*Gu1[37] + Gx1[46]*Gu1[41] + Gx1[47]*Gu1[45];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[6] + Gx1[38]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[40]*Gu1[18] + Gx1[41]*Gu1[22] + Gx1[42]*Gu1[26] + Gx1[43]*Gu1[30] + Gx1[44]*Gu1[34] + Gx1[45]*Gu1[38] + Gx1[46]*Gu1[42] + Gx1[47]*Gu1[46];
Gu2[15] = + Gx1[36]*Gu1[3] + Gx1[37]*Gu1[7] + Gx1[38]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[40]*Gu1[19] + Gx1[41]*Gu1[23] + Gx1[42]*Gu1[27] + Gx1[43]*Gu1[31] + Gx1[44]*Gu1[35] + Gx1[45]*Gu1[39] + Gx1[46]*Gu1[43] + Gx1[47]*Gu1[47];
Gu2[16] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[4] + Gx1[50]*Gu1[8] + Gx1[51]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[54]*Gu1[24] + Gx1[55]*Gu1[28] + Gx1[56]*Gu1[32] + Gx1[57]*Gu1[36] + Gx1[58]*Gu1[40] + Gx1[59]*Gu1[44];
Gu2[17] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[5] + Gx1[50]*Gu1[9] + Gx1[51]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[54]*Gu1[25] + Gx1[55]*Gu1[29] + Gx1[56]*Gu1[33] + Gx1[57]*Gu1[37] + Gx1[58]*Gu1[41] + Gx1[59]*Gu1[45];
Gu2[18] = + Gx1[48]*Gu1[2] + Gx1[49]*Gu1[6] + Gx1[50]*Gu1[10] + Gx1[51]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[54]*Gu1[26] + Gx1[55]*Gu1[30] + Gx1[56]*Gu1[34] + Gx1[57]*Gu1[38] + Gx1[58]*Gu1[42] + Gx1[59]*Gu1[46];
Gu2[19] = + Gx1[48]*Gu1[3] + Gx1[49]*Gu1[7] + Gx1[50]*Gu1[11] + Gx1[51]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[54]*Gu1[27] + Gx1[55]*Gu1[31] + Gx1[56]*Gu1[35] + Gx1[57]*Gu1[39] + Gx1[58]*Gu1[43] + Gx1[59]*Gu1[47];
Gu2[20] = + Gx1[60]*Gu1[0] + Gx1[61]*Gu1[4] + Gx1[62]*Gu1[8] + Gx1[63]*Gu1[12] + Gx1[64]*Gu1[16] + Gx1[65]*Gu1[20] + Gx1[66]*Gu1[24] + Gx1[67]*Gu1[28] + Gx1[68]*Gu1[32] + Gx1[69]*Gu1[36] + Gx1[70]*Gu1[40] + Gx1[71]*Gu1[44];
Gu2[21] = + Gx1[60]*Gu1[1] + Gx1[61]*Gu1[5] + Gx1[62]*Gu1[9] + Gx1[63]*Gu1[13] + Gx1[64]*Gu1[17] + Gx1[65]*Gu1[21] + Gx1[66]*Gu1[25] + Gx1[67]*Gu1[29] + Gx1[68]*Gu1[33] + Gx1[69]*Gu1[37] + Gx1[70]*Gu1[41] + Gx1[71]*Gu1[45];
Gu2[22] = + Gx1[60]*Gu1[2] + Gx1[61]*Gu1[6] + Gx1[62]*Gu1[10] + Gx1[63]*Gu1[14] + Gx1[64]*Gu1[18] + Gx1[65]*Gu1[22] + Gx1[66]*Gu1[26] + Gx1[67]*Gu1[30] + Gx1[68]*Gu1[34] + Gx1[69]*Gu1[38] + Gx1[70]*Gu1[42] + Gx1[71]*Gu1[46];
Gu2[23] = + Gx1[60]*Gu1[3] + Gx1[61]*Gu1[7] + Gx1[62]*Gu1[11] + Gx1[63]*Gu1[15] + Gx1[64]*Gu1[19] + Gx1[65]*Gu1[23] + Gx1[66]*Gu1[27] + Gx1[67]*Gu1[31] + Gx1[68]*Gu1[35] + Gx1[69]*Gu1[39] + Gx1[70]*Gu1[43] + Gx1[71]*Gu1[47];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[12] + Gx1[76]*Gu1[16] + Gx1[77]*Gu1[20] + Gx1[78]*Gu1[24] + Gx1[79]*Gu1[28] + Gx1[80]*Gu1[32] + Gx1[81]*Gu1[36] + Gx1[82]*Gu1[40] + Gx1[83]*Gu1[44];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[9] + Gx1[75]*Gu1[13] + Gx1[76]*Gu1[17] + Gx1[77]*Gu1[21] + Gx1[78]*Gu1[25] + Gx1[79]*Gu1[29] + Gx1[80]*Gu1[33] + Gx1[81]*Gu1[37] + Gx1[82]*Gu1[41] + Gx1[83]*Gu1[45];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[6] + Gx1[74]*Gu1[10] + Gx1[75]*Gu1[14] + Gx1[76]*Gu1[18] + Gx1[77]*Gu1[22] + Gx1[78]*Gu1[26] + Gx1[79]*Gu1[30] + Gx1[80]*Gu1[34] + Gx1[81]*Gu1[38] + Gx1[82]*Gu1[42] + Gx1[83]*Gu1[46];
Gu2[27] = + Gx1[72]*Gu1[3] + Gx1[73]*Gu1[7] + Gx1[74]*Gu1[11] + Gx1[75]*Gu1[15] + Gx1[76]*Gu1[19] + Gx1[77]*Gu1[23] + Gx1[78]*Gu1[27] + Gx1[79]*Gu1[31] + Gx1[80]*Gu1[35] + Gx1[81]*Gu1[39] + Gx1[82]*Gu1[43] + Gx1[83]*Gu1[47];
Gu2[28] = + Gx1[84]*Gu1[0] + Gx1[85]*Gu1[4] + Gx1[86]*Gu1[8] + Gx1[87]*Gu1[12] + Gx1[88]*Gu1[16] + Gx1[89]*Gu1[20] + Gx1[90]*Gu1[24] + Gx1[91]*Gu1[28] + Gx1[92]*Gu1[32] + Gx1[93]*Gu1[36] + Gx1[94]*Gu1[40] + Gx1[95]*Gu1[44];
Gu2[29] = + Gx1[84]*Gu1[1] + Gx1[85]*Gu1[5] + Gx1[86]*Gu1[9] + Gx1[87]*Gu1[13] + Gx1[88]*Gu1[17] + Gx1[89]*Gu1[21] + Gx1[90]*Gu1[25] + Gx1[91]*Gu1[29] + Gx1[92]*Gu1[33] + Gx1[93]*Gu1[37] + Gx1[94]*Gu1[41] + Gx1[95]*Gu1[45];
Gu2[30] = + Gx1[84]*Gu1[2] + Gx1[85]*Gu1[6] + Gx1[86]*Gu1[10] + Gx1[87]*Gu1[14] + Gx1[88]*Gu1[18] + Gx1[89]*Gu1[22] + Gx1[90]*Gu1[26] + Gx1[91]*Gu1[30] + Gx1[92]*Gu1[34] + Gx1[93]*Gu1[38] + Gx1[94]*Gu1[42] + Gx1[95]*Gu1[46];
Gu2[31] = + Gx1[84]*Gu1[3] + Gx1[85]*Gu1[7] + Gx1[86]*Gu1[11] + Gx1[87]*Gu1[15] + Gx1[88]*Gu1[19] + Gx1[89]*Gu1[23] + Gx1[90]*Gu1[27] + Gx1[91]*Gu1[31] + Gx1[92]*Gu1[35] + Gx1[93]*Gu1[39] + Gx1[94]*Gu1[43] + Gx1[95]*Gu1[47];
Gu2[32] = + Gx1[96]*Gu1[0] + Gx1[97]*Gu1[4] + Gx1[98]*Gu1[8] + Gx1[99]*Gu1[12] + Gx1[100]*Gu1[16] + Gx1[101]*Gu1[20] + Gx1[102]*Gu1[24] + Gx1[103]*Gu1[28] + Gx1[104]*Gu1[32] + Gx1[105]*Gu1[36] + Gx1[106]*Gu1[40] + Gx1[107]*Gu1[44];
Gu2[33] = + Gx1[96]*Gu1[1] + Gx1[97]*Gu1[5] + Gx1[98]*Gu1[9] + Gx1[99]*Gu1[13] + Gx1[100]*Gu1[17] + Gx1[101]*Gu1[21] + Gx1[102]*Gu1[25] + Gx1[103]*Gu1[29] + Gx1[104]*Gu1[33] + Gx1[105]*Gu1[37] + Gx1[106]*Gu1[41] + Gx1[107]*Gu1[45];
Gu2[34] = + Gx1[96]*Gu1[2] + Gx1[97]*Gu1[6] + Gx1[98]*Gu1[10] + Gx1[99]*Gu1[14] + Gx1[100]*Gu1[18] + Gx1[101]*Gu1[22] + Gx1[102]*Gu1[26] + Gx1[103]*Gu1[30] + Gx1[104]*Gu1[34] + Gx1[105]*Gu1[38] + Gx1[106]*Gu1[42] + Gx1[107]*Gu1[46];
Gu2[35] = + Gx1[96]*Gu1[3] + Gx1[97]*Gu1[7] + Gx1[98]*Gu1[11] + Gx1[99]*Gu1[15] + Gx1[100]*Gu1[19] + Gx1[101]*Gu1[23] + Gx1[102]*Gu1[27] + Gx1[103]*Gu1[31] + Gx1[104]*Gu1[35] + Gx1[105]*Gu1[39] + Gx1[106]*Gu1[43] + Gx1[107]*Gu1[47];
Gu2[36] = + Gx1[108]*Gu1[0] + Gx1[109]*Gu1[4] + Gx1[110]*Gu1[8] + Gx1[111]*Gu1[12] + Gx1[112]*Gu1[16] + Gx1[113]*Gu1[20] + Gx1[114]*Gu1[24] + Gx1[115]*Gu1[28] + Gx1[116]*Gu1[32] + Gx1[117]*Gu1[36] + Gx1[118]*Gu1[40] + Gx1[119]*Gu1[44];
Gu2[37] = + Gx1[108]*Gu1[1] + Gx1[109]*Gu1[5] + Gx1[110]*Gu1[9] + Gx1[111]*Gu1[13] + Gx1[112]*Gu1[17] + Gx1[113]*Gu1[21] + Gx1[114]*Gu1[25] + Gx1[115]*Gu1[29] + Gx1[116]*Gu1[33] + Gx1[117]*Gu1[37] + Gx1[118]*Gu1[41] + Gx1[119]*Gu1[45];
Gu2[38] = + Gx1[108]*Gu1[2] + Gx1[109]*Gu1[6] + Gx1[110]*Gu1[10] + Gx1[111]*Gu1[14] + Gx1[112]*Gu1[18] + Gx1[113]*Gu1[22] + Gx1[114]*Gu1[26] + Gx1[115]*Gu1[30] + Gx1[116]*Gu1[34] + Gx1[117]*Gu1[38] + Gx1[118]*Gu1[42] + Gx1[119]*Gu1[46];
Gu2[39] = + Gx1[108]*Gu1[3] + Gx1[109]*Gu1[7] + Gx1[110]*Gu1[11] + Gx1[111]*Gu1[15] + Gx1[112]*Gu1[19] + Gx1[113]*Gu1[23] + Gx1[114]*Gu1[27] + Gx1[115]*Gu1[31] + Gx1[116]*Gu1[35] + Gx1[117]*Gu1[39] + Gx1[118]*Gu1[43] + Gx1[119]*Gu1[47];
Gu2[40] = + Gx1[120]*Gu1[0] + Gx1[121]*Gu1[4] + Gx1[122]*Gu1[8] + Gx1[123]*Gu1[12] + Gx1[124]*Gu1[16] + Gx1[125]*Gu1[20] + Gx1[126]*Gu1[24] + Gx1[127]*Gu1[28] + Gx1[128]*Gu1[32] + Gx1[129]*Gu1[36] + Gx1[130]*Gu1[40] + Gx1[131]*Gu1[44];
Gu2[41] = + Gx1[120]*Gu1[1] + Gx1[121]*Gu1[5] + Gx1[122]*Gu1[9] + Gx1[123]*Gu1[13] + Gx1[124]*Gu1[17] + Gx1[125]*Gu1[21] + Gx1[126]*Gu1[25] + Gx1[127]*Gu1[29] + Gx1[128]*Gu1[33] + Gx1[129]*Gu1[37] + Gx1[130]*Gu1[41] + Gx1[131]*Gu1[45];
Gu2[42] = + Gx1[120]*Gu1[2] + Gx1[121]*Gu1[6] + Gx1[122]*Gu1[10] + Gx1[123]*Gu1[14] + Gx1[124]*Gu1[18] + Gx1[125]*Gu1[22] + Gx1[126]*Gu1[26] + Gx1[127]*Gu1[30] + Gx1[128]*Gu1[34] + Gx1[129]*Gu1[38] + Gx1[130]*Gu1[42] + Gx1[131]*Gu1[46];
Gu2[43] = + Gx1[120]*Gu1[3] + Gx1[121]*Gu1[7] + Gx1[122]*Gu1[11] + Gx1[123]*Gu1[15] + Gx1[124]*Gu1[19] + Gx1[125]*Gu1[23] + Gx1[126]*Gu1[27] + Gx1[127]*Gu1[31] + Gx1[128]*Gu1[35] + Gx1[129]*Gu1[39] + Gx1[130]*Gu1[43] + Gx1[131]*Gu1[47];
Gu2[44] = + Gx1[132]*Gu1[0] + Gx1[133]*Gu1[4] + Gx1[134]*Gu1[8] + Gx1[135]*Gu1[12] + Gx1[136]*Gu1[16] + Gx1[137]*Gu1[20] + Gx1[138]*Gu1[24] + Gx1[139]*Gu1[28] + Gx1[140]*Gu1[32] + Gx1[141]*Gu1[36] + Gx1[142]*Gu1[40] + Gx1[143]*Gu1[44];
Gu2[45] = + Gx1[132]*Gu1[1] + Gx1[133]*Gu1[5] + Gx1[134]*Gu1[9] + Gx1[135]*Gu1[13] + Gx1[136]*Gu1[17] + Gx1[137]*Gu1[21] + Gx1[138]*Gu1[25] + Gx1[139]*Gu1[29] + Gx1[140]*Gu1[33] + Gx1[141]*Gu1[37] + Gx1[142]*Gu1[41] + Gx1[143]*Gu1[45];
Gu2[46] = + Gx1[132]*Gu1[2] + Gx1[133]*Gu1[6] + Gx1[134]*Gu1[10] + Gx1[135]*Gu1[14] + Gx1[136]*Gu1[18] + Gx1[137]*Gu1[22] + Gx1[138]*Gu1[26] + Gx1[139]*Gu1[30] + Gx1[140]*Gu1[34] + Gx1[141]*Gu1[38] + Gx1[142]*Gu1[42] + Gx1[143]*Gu1[46];
Gu2[47] = + Gx1[132]*Gu1[3] + Gx1[133]*Gu1[7] + Gx1[134]*Gu1[11] + Gx1[135]*Gu1[15] + Gx1[136]*Gu1[19] + Gx1[137]*Gu1[23] + Gx1[138]*Gu1[27] + Gx1[139]*Gu1[31] + Gx1[140]*Gu1[35] + Gx1[141]*Gu1[39] + Gx1[142]*Gu1[43] + Gx1[143]*Gu1[47];
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
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
Gu2[40] = Gu1[40];
Gu2[41] = Gu1[41];
Gu2[42] = Gu1[42];
Gu2[43] = Gu1[43];
Gu2[44] = Gu1[44];
Gu2[45] = Gu1[45];
Gu2[46] = Gu1[46];
Gu2[47] = Gu1[47];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 592) + (iCol * 4)] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + Gu1[40]*Gu2[40] + Gu1[44]*Gu2[44];
acadoWorkspace.H[(iRow * 592) + (iCol * 4 + 1)] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + Gu1[40]*Gu2[41] + Gu1[44]*Gu2[45];
acadoWorkspace.H[(iRow * 592) + (iCol * 4 + 2)] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + Gu1[40]*Gu2[42] + Gu1[44]*Gu2[46];
acadoWorkspace.H[(iRow * 592) + (iCol * 4 + 3)] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + Gu1[40]*Gu2[43] + Gu1[44]*Gu2[47];
acadoWorkspace.H[(iRow * 592 + 148) + (iCol * 4)] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + Gu1[41]*Gu2[40] + Gu1[45]*Gu2[44];
acadoWorkspace.H[(iRow * 592 + 148) + (iCol * 4 + 1)] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + Gu1[41]*Gu2[41] + Gu1[45]*Gu2[45];
acadoWorkspace.H[(iRow * 592 + 148) + (iCol * 4 + 2)] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + Gu1[41]*Gu2[42] + Gu1[45]*Gu2[46];
acadoWorkspace.H[(iRow * 592 + 148) + (iCol * 4 + 3)] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + Gu1[41]*Gu2[43] + Gu1[45]*Gu2[47];
acadoWorkspace.H[(iRow * 592 + 296) + (iCol * 4)] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + Gu1[42]*Gu2[40] + Gu1[46]*Gu2[44];
acadoWorkspace.H[(iRow * 592 + 296) + (iCol * 4 + 1)] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + Gu1[42]*Gu2[41] + Gu1[46]*Gu2[45];
acadoWorkspace.H[(iRow * 592 + 296) + (iCol * 4 + 2)] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + Gu1[42]*Gu2[42] + Gu1[46]*Gu2[46];
acadoWorkspace.H[(iRow * 592 + 296) + (iCol * 4 + 3)] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + Gu1[42]*Gu2[43] + Gu1[46]*Gu2[47];
acadoWorkspace.H[(iRow * 592 + 444) + (iCol * 4)] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + Gu1[43]*Gu2[40] + Gu1[47]*Gu2[44];
acadoWorkspace.H[(iRow * 592 + 444) + (iCol * 4 + 1)] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + Gu1[43]*Gu2[41] + Gu1[47]*Gu2[45];
acadoWorkspace.H[(iRow * 592 + 444) + (iCol * 4 + 2)] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + Gu1[43]*Gu2[42] + Gu1[47]*Gu2[46];
acadoWorkspace.H[(iRow * 592 + 444) + (iCol * 4 + 3)] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + Gu1[43]*Gu2[43] + Gu1[47]*Gu2[47];
}

void acado_multBTW1_R1( real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 596] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + Gu1[32]*Gu2[32] + Gu1[36]*Gu2[36] + Gu1[40]*Gu2[40] + Gu1[44]*Gu2[44] + (real_t)1.0000000000000000e+00;
acadoWorkspace.H[iRow * 596 + 1] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + Gu1[32]*Gu2[33] + Gu1[36]*Gu2[37] + Gu1[40]*Gu2[41] + Gu1[44]*Gu2[45];
acadoWorkspace.H[iRow * 596 + 2] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + Gu1[32]*Gu2[34] + Gu1[36]*Gu2[38] + Gu1[40]*Gu2[42] + Gu1[44]*Gu2[46];
acadoWorkspace.H[iRow * 596 + 3] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + Gu1[32]*Gu2[35] + Gu1[36]*Gu2[39] + Gu1[40]*Gu2[43] + Gu1[44]*Gu2[47];
acadoWorkspace.H[iRow * 596 + 148] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + Gu1[33]*Gu2[32] + Gu1[37]*Gu2[36] + Gu1[41]*Gu2[40] + Gu1[45]*Gu2[44];
acadoWorkspace.H[iRow * 596 + 149] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + Gu1[33]*Gu2[33] + Gu1[37]*Gu2[37] + Gu1[41]*Gu2[41] + Gu1[45]*Gu2[45] + (real_t)1.0000000000000000e+00;
acadoWorkspace.H[iRow * 596 + 150] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + Gu1[33]*Gu2[34] + Gu1[37]*Gu2[38] + Gu1[41]*Gu2[42] + Gu1[45]*Gu2[46];
acadoWorkspace.H[iRow * 596 + 151] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + Gu1[33]*Gu2[35] + Gu1[37]*Gu2[39] + Gu1[41]*Gu2[43] + Gu1[45]*Gu2[47];
acadoWorkspace.H[iRow * 596 + 296] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + Gu1[34]*Gu2[32] + Gu1[38]*Gu2[36] + Gu1[42]*Gu2[40] + Gu1[46]*Gu2[44];
acadoWorkspace.H[iRow * 596 + 297] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + Gu1[34]*Gu2[33] + Gu1[38]*Gu2[37] + Gu1[42]*Gu2[41] + Gu1[46]*Gu2[45];
acadoWorkspace.H[iRow * 596 + 298] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + Gu1[34]*Gu2[34] + Gu1[38]*Gu2[38] + Gu1[42]*Gu2[42] + Gu1[46]*Gu2[46] + (real_t)1.0000000000000000e+00;
acadoWorkspace.H[iRow * 596 + 299] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + Gu1[34]*Gu2[35] + Gu1[38]*Gu2[39] + Gu1[42]*Gu2[43] + Gu1[46]*Gu2[47];
acadoWorkspace.H[iRow * 596 + 444] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + Gu1[35]*Gu2[32] + Gu1[39]*Gu2[36] + Gu1[43]*Gu2[40] + Gu1[47]*Gu2[44];
acadoWorkspace.H[iRow * 596 + 445] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + Gu1[35]*Gu2[33] + Gu1[39]*Gu2[37] + Gu1[43]*Gu2[41] + Gu1[47]*Gu2[45];
acadoWorkspace.H[iRow * 596 + 446] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + Gu1[35]*Gu2[34] + Gu1[39]*Gu2[38] + Gu1[43]*Gu2[42] + Gu1[47]*Gu2[46];
acadoWorkspace.H[iRow * 596 + 447] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + Gu1[35]*Gu2[35] + Gu1[39]*Gu2[39] + Gu1[43]*Gu2[43] + Gu1[47]*Gu2[47] + (real_t)1.0000000000000000e+00;
acadoWorkspace.H[iRow * 596] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 596 + 149] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 596 + 298] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 596 + 447] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[12]*Gu1[4] + Gx1[24]*Gu1[8] + Gx1[36]*Gu1[12] + Gx1[48]*Gu1[16] + Gx1[60]*Gu1[20] + Gx1[72]*Gu1[24] + Gx1[84]*Gu1[28] + Gx1[96]*Gu1[32] + Gx1[108]*Gu1[36] + Gx1[120]*Gu1[40] + Gx1[132]*Gu1[44];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[12]*Gu1[5] + Gx1[24]*Gu1[9] + Gx1[36]*Gu1[13] + Gx1[48]*Gu1[17] + Gx1[60]*Gu1[21] + Gx1[72]*Gu1[25] + Gx1[84]*Gu1[29] + Gx1[96]*Gu1[33] + Gx1[108]*Gu1[37] + Gx1[120]*Gu1[41] + Gx1[132]*Gu1[45];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[12]*Gu1[6] + Gx1[24]*Gu1[10] + Gx1[36]*Gu1[14] + Gx1[48]*Gu1[18] + Gx1[60]*Gu1[22] + Gx1[72]*Gu1[26] + Gx1[84]*Gu1[30] + Gx1[96]*Gu1[34] + Gx1[108]*Gu1[38] + Gx1[120]*Gu1[42] + Gx1[132]*Gu1[46];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[12]*Gu1[7] + Gx1[24]*Gu1[11] + Gx1[36]*Gu1[15] + Gx1[48]*Gu1[19] + Gx1[60]*Gu1[23] + Gx1[72]*Gu1[27] + Gx1[84]*Gu1[31] + Gx1[96]*Gu1[35] + Gx1[108]*Gu1[39] + Gx1[120]*Gu1[43] + Gx1[132]*Gu1[47];
Gu2[4] = + Gx1[1]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[25]*Gu1[8] + Gx1[37]*Gu1[12] + Gx1[49]*Gu1[16] + Gx1[61]*Gu1[20] + Gx1[73]*Gu1[24] + Gx1[85]*Gu1[28] + Gx1[97]*Gu1[32] + Gx1[109]*Gu1[36] + Gx1[121]*Gu1[40] + Gx1[133]*Gu1[44];
Gu2[5] = + Gx1[1]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[25]*Gu1[9] + Gx1[37]*Gu1[13] + Gx1[49]*Gu1[17] + Gx1[61]*Gu1[21] + Gx1[73]*Gu1[25] + Gx1[85]*Gu1[29] + Gx1[97]*Gu1[33] + Gx1[109]*Gu1[37] + Gx1[121]*Gu1[41] + Gx1[133]*Gu1[45];
Gu2[6] = + Gx1[1]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[25]*Gu1[10] + Gx1[37]*Gu1[14] + Gx1[49]*Gu1[18] + Gx1[61]*Gu1[22] + Gx1[73]*Gu1[26] + Gx1[85]*Gu1[30] + Gx1[97]*Gu1[34] + Gx1[109]*Gu1[38] + Gx1[121]*Gu1[42] + Gx1[133]*Gu1[46];
Gu2[7] = + Gx1[1]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[25]*Gu1[11] + Gx1[37]*Gu1[15] + Gx1[49]*Gu1[19] + Gx1[61]*Gu1[23] + Gx1[73]*Gu1[27] + Gx1[85]*Gu1[31] + Gx1[97]*Gu1[35] + Gx1[109]*Gu1[39] + Gx1[121]*Gu1[43] + Gx1[133]*Gu1[47];
Gu2[8] = + Gx1[2]*Gu1[0] + Gx1[14]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[38]*Gu1[12] + Gx1[50]*Gu1[16] + Gx1[62]*Gu1[20] + Gx1[74]*Gu1[24] + Gx1[86]*Gu1[28] + Gx1[98]*Gu1[32] + Gx1[110]*Gu1[36] + Gx1[122]*Gu1[40] + Gx1[134]*Gu1[44];
Gu2[9] = + Gx1[2]*Gu1[1] + Gx1[14]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[38]*Gu1[13] + Gx1[50]*Gu1[17] + Gx1[62]*Gu1[21] + Gx1[74]*Gu1[25] + Gx1[86]*Gu1[29] + Gx1[98]*Gu1[33] + Gx1[110]*Gu1[37] + Gx1[122]*Gu1[41] + Gx1[134]*Gu1[45];
Gu2[10] = + Gx1[2]*Gu1[2] + Gx1[14]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[38]*Gu1[14] + Gx1[50]*Gu1[18] + Gx1[62]*Gu1[22] + Gx1[74]*Gu1[26] + Gx1[86]*Gu1[30] + Gx1[98]*Gu1[34] + Gx1[110]*Gu1[38] + Gx1[122]*Gu1[42] + Gx1[134]*Gu1[46];
Gu2[11] = + Gx1[2]*Gu1[3] + Gx1[14]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[38]*Gu1[15] + Gx1[50]*Gu1[19] + Gx1[62]*Gu1[23] + Gx1[74]*Gu1[27] + Gx1[86]*Gu1[31] + Gx1[98]*Gu1[35] + Gx1[110]*Gu1[39] + Gx1[122]*Gu1[43] + Gx1[134]*Gu1[47];
Gu2[12] = + Gx1[3]*Gu1[0] + Gx1[15]*Gu1[4] + Gx1[27]*Gu1[8] + Gx1[39]*Gu1[12] + Gx1[51]*Gu1[16] + Gx1[63]*Gu1[20] + Gx1[75]*Gu1[24] + Gx1[87]*Gu1[28] + Gx1[99]*Gu1[32] + Gx1[111]*Gu1[36] + Gx1[123]*Gu1[40] + Gx1[135]*Gu1[44];
Gu2[13] = + Gx1[3]*Gu1[1] + Gx1[15]*Gu1[5] + Gx1[27]*Gu1[9] + Gx1[39]*Gu1[13] + Gx1[51]*Gu1[17] + Gx1[63]*Gu1[21] + Gx1[75]*Gu1[25] + Gx1[87]*Gu1[29] + Gx1[99]*Gu1[33] + Gx1[111]*Gu1[37] + Gx1[123]*Gu1[41] + Gx1[135]*Gu1[45];
Gu2[14] = + Gx1[3]*Gu1[2] + Gx1[15]*Gu1[6] + Gx1[27]*Gu1[10] + Gx1[39]*Gu1[14] + Gx1[51]*Gu1[18] + Gx1[63]*Gu1[22] + Gx1[75]*Gu1[26] + Gx1[87]*Gu1[30] + Gx1[99]*Gu1[34] + Gx1[111]*Gu1[38] + Gx1[123]*Gu1[42] + Gx1[135]*Gu1[46];
Gu2[15] = + Gx1[3]*Gu1[3] + Gx1[15]*Gu1[7] + Gx1[27]*Gu1[11] + Gx1[39]*Gu1[15] + Gx1[51]*Gu1[19] + Gx1[63]*Gu1[23] + Gx1[75]*Gu1[27] + Gx1[87]*Gu1[31] + Gx1[99]*Gu1[35] + Gx1[111]*Gu1[39] + Gx1[123]*Gu1[43] + Gx1[135]*Gu1[47];
Gu2[16] = + Gx1[4]*Gu1[0] + Gx1[16]*Gu1[4] + Gx1[28]*Gu1[8] + Gx1[40]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[64]*Gu1[20] + Gx1[76]*Gu1[24] + Gx1[88]*Gu1[28] + Gx1[100]*Gu1[32] + Gx1[112]*Gu1[36] + Gx1[124]*Gu1[40] + Gx1[136]*Gu1[44];
Gu2[17] = + Gx1[4]*Gu1[1] + Gx1[16]*Gu1[5] + Gx1[28]*Gu1[9] + Gx1[40]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[64]*Gu1[21] + Gx1[76]*Gu1[25] + Gx1[88]*Gu1[29] + Gx1[100]*Gu1[33] + Gx1[112]*Gu1[37] + Gx1[124]*Gu1[41] + Gx1[136]*Gu1[45];
Gu2[18] = + Gx1[4]*Gu1[2] + Gx1[16]*Gu1[6] + Gx1[28]*Gu1[10] + Gx1[40]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[64]*Gu1[22] + Gx1[76]*Gu1[26] + Gx1[88]*Gu1[30] + Gx1[100]*Gu1[34] + Gx1[112]*Gu1[38] + Gx1[124]*Gu1[42] + Gx1[136]*Gu1[46];
Gu2[19] = + Gx1[4]*Gu1[3] + Gx1[16]*Gu1[7] + Gx1[28]*Gu1[11] + Gx1[40]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[64]*Gu1[23] + Gx1[76]*Gu1[27] + Gx1[88]*Gu1[31] + Gx1[100]*Gu1[35] + Gx1[112]*Gu1[39] + Gx1[124]*Gu1[43] + Gx1[136]*Gu1[47];
Gu2[20] = + Gx1[5]*Gu1[0] + Gx1[17]*Gu1[4] + Gx1[29]*Gu1[8] + Gx1[41]*Gu1[12] + Gx1[53]*Gu1[16] + Gx1[65]*Gu1[20] + Gx1[77]*Gu1[24] + Gx1[89]*Gu1[28] + Gx1[101]*Gu1[32] + Gx1[113]*Gu1[36] + Gx1[125]*Gu1[40] + Gx1[137]*Gu1[44];
Gu2[21] = + Gx1[5]*Gu1[1] + Gx1[17]*Gu1[5] + Gx1[29]*Gu1[9] + Gx1[41]*Gu1[13] + Gx1[53]*Gu1[17] + Gx1[65]*Gu1[21] + Gx1[77]*Gu1[25] + Gx1[89]*Gu1[29] + Gx1[101]*Gu1[33] + Gx1[113]*Gu1[37] + Gx1[125]*Gu1[41] + Gx1[137]*Gu1[45];
Gu2[22] = + Gx1[5]*Gu1[2] + Gx1[17]*Gu1[6] + Gx1[29]*Gu1[10] + Gx1[41]*Gu1[14] + Gx1[53]*Gu1[18] + Gx1[65]*Gu1[22] + Gx1[77]*Gu1[26] + Gx1[89]*Gu1[30] + Gx1[101]*Gu1[34] + Gx1[113]*Gu1[38] + Gx1[125]*Gu1[42] + Gx1[137]*Gu1[46];
Gu2[23] = + Gx1[5]*Gu1[3] + Gx1[17]*Gu1[7] + Gx1[29]*Gu1[11] + Gx1[41]*Gu1[15] + Gx1[53]*Gu1[19] + Gx1[65]*Gu1[23] + Gx1[77]*Gu1[27] + Gx1[89]*Gu1[31] + Gx1[101]*Gu1[35] + Gx1[113]*Gu1[39] + Gx1[125]*Gu1[43] + Gx1[137]*Gu1[47];
Gu2[24] = + Gx1[6]*Gu1[0] + Gx1[18]*Gu1[4] + Gx1[30]*Gu1[8] + Gx1[42]*Gu1[12] + Gx1[54]*Gu1[16] + Gx1[66]*Gu1[20] + Gx1[78]*Gu1[24] + Gx1[90]*Gu1[28] + Gx1[102]*Gu1[32] + Gx1[114]*Gu1[36] + Gx1[126]*Gu1[40] + Gx1[138]*Gu1[44];
Gu2[25] = + Gx1[6]*Gu1[1] + Gx1[18]*Gu1[5] + Gx1[30]*Gu1[9] + Gx1[42]*Gu1[13] + Gx1[54]*Gu1[17] + Gx1[66]*Gu1[21] + Gx1[78]*Gu1[25] + Gx1[90]*Gu1[29] + Gx1[102]*Gu1[33] + Gx1[114]*Gu1[37] + Gx1[126]*Gu1[41] + Gx1[138]*Gu1[45];
Gu2[26] = + Gx1[6]*Gu1[2] + Gx1[18]*Gu1[6] + Gx1[30]*Gu1[10] + Gx1[42]*Gu1[14] + Gx1[54]*Gu1[18] + Gx1[66]*Gu1[22] + Gx1[78]*Gu1[26] + Gx1[90]*Gu1[30] + Gx1[102]*Gu1[34] + Gx1[114]*Gu1[38] + Gx1[126]*Gu1[42] + Gx1[138]*Gu1[46];
Gu2[27] = + Gx1[6]*Gu1[3] + Gx1[18]*Gu1[7] + Gx1[30]*Gu1[11] + Gx1[42]*Gu1[15] + Gx1[54]*Gu1[19] + Gx1[66]*Gu1[23] + Gx1[78]*Gu1[27] + Gx1[90]*Gu1[31] + Gx1[102]*Gu1[35] + Gx1[114]*Gu1[39] + Gx1[126]*Gu1[43] + Gx1[138]*Gu1[47];
Gu2[28] = + Gx1[7]*Gu1[0] + Gx1[19]*Gu1[4] + Gx1[31]*Gu1[8] + Gx1[43]*Gu1[12] + Gx1[55]*Gu1[16] + Gx1[67]*Gu1[20] + Gx1[79]*Gu1[24] + Gx1[91]*Gu1[28] + Gx1[103]*Gu1[32] + Gx1[115]*Gu1[36] + Gx1[127]*Gu1[40] + Gx1[139]*Gu1[44];
Gu2[29] = + Gx1[7]*Gu1[1] + Gx1[19]*Gu1[5] + Gx1[31]*Gu1[9] + Gx1[43]*Gu1[13] + Gx1[55]*Gu1[17] + Gx1[67]*Gu1[21] + Gx1[79]*Gu1[25] + Gx1[91]*Gu1[29] + Gx1[103]*Gu1[33] + Gx1[115]*Gu1[37] + Gx1[127]*Gu1[41] + Gx1[139]*Gu1[45];
Gu2[30] = + Gx1[7]*Gu1[2] + Gx1[19]*Gu1[6] + Gx1[31]*Gu1[10] + Gx1[43]*Gu1[14] + Gx1[55]*Gu1[18] + Gx1[67]*Gu1[22] + Gx1[79]*Gu1[26] + Gx1[91]*Gu1[30] + Gx1[103]*Gu1[34] + Gx1[115]*Gu1[38] + Gx1[127]*Gu1[42] + Gx1[139]*Gu1[46];
Gu2[31] = + Gx1[7]*Gu1[3] + Gx1[19]*Gu1[7] + Gx1[31]*Gu1[11] + Gx1[43]*Gu1[15] + Gx1[55]*Gu1[19] + Gx1[67]*Gu1[23] + Gx1[79]*Gu1[27] + Gx1[91]*Gu1[31] + Gx1[103]*Gu1[35] + Gx1[115]*Gu1[39] + Gx1[127]*Gu1[43] + Gx1[139]*Gu1[47];
Gu2[32] = + Gx1[8]*Gu1[0] + Gx1[20]*Gu1[4] + Gx1[32]*Gu1[8] + Gx1[44]*Gu1[12] + Gx1[56]*Gu1[16] + Gx1[68]*Gu1[20] + Gx1[80]*Gu1[24] + Gx1[92]*Gu1[28] + Gx1[104]*Gu1[32] + Gx1[116]*Gu1[36] + Gx1[128]*Gu1[40] + Gx1[140]*Gu1[44];
Gu2[33] = + Gx1[8]*Gu1[1] + Gx1[20]*Gu1[5] + Gx1[32]*Gu1[9] + Gx1[44]*Gu1[13] + Gx1[56]*Gu1[17] + Gx1[68]*Gu1[21] + Gx1[80]*Gu1[25] + Gx1[92]*Gu1[29] + Gx1[104]*Gu1[33] + Gx1[116]*Gu1[37] + Gx1[128]*Gu1[41] + Gx1[140]*Gu1[45];
Gu2[34] = + Gx1[8]*Gu1[2] + Gx1[20]*Gu1[6] + Gx1[32]*Gu1[10] + Gx1[44]*Gu1[14] + Gx1[56]*Gu1[18] + Gx1[68]*Gu1[22] + Gx1[80]*Gu1[26] + Gx1[92]*Gu1[30] + Gx1[104]*Gu1[34] + Gx1[116]*Gu1[38] + Gx1[128]*Gu1[42] + Gx1[140]*Gu1[46];
Gu2[35] = + Gx1[8]*Gu1[3] + Gx1[20]*Gu1[7] + Gx1[32]*Gu1[11] + Gx1[44]*Gu1[15] + Gx1[56]*Gu1[19] + Gx1[68]*Gu1[23] + Gx1[80]*Gu1[27] + Gx1[92]*Gu1[31] + Gx1[104]*Gu1[35] + Gx1[116]*Gu1[39] + Gx1[128]*Gu1[43] + Gx1[140]*Gu1[47];
Gu2[36] = + Gx1[9]*Gu1[0] + Gx1[21]*Gu1[4] + Gx1[33]*Gu1[8] + Gx1[45]*Gu1[12] + Gx1[57]*Gu1[16] + Gx1[69]*Gu1[20] + Gx1[81]*Gu1[24] + Gx1[93]*Gu1[28] + Gx1[105]*Gu1[32] + Gx1[117]*Gu1[36] + Gx1[129]*Gu1[40] + Gx1[141]*Gu1[44];
Gu2[37] = + Gx1[9]*Gu1[1] + Gx1[21]*Gu1[5] + Gx1[33]*Gu1[9] + Gx1[45]*Gu1[13] + Gx1[57]*Gu1[17] + Gx1[69]*Gu1[21] + Gx1[81]*Gu1[25] + Gx1[93]*Gu1[29] + Gx1[105]*Gu1[33] + Gx1[117]*Gu1[37] + Gx1[129]*Gu1[41] + Gx1[141]*Gu1[45];
Gu2[38] = + Gx1[9]*Gu1[2] + Gx1[21]*Gu1[6] + Gx1[33]*Gu1[10] + Gx1[45]*Gu1[14] + Gx1[57]*Gu1[18] + Gx1[69]*Gu1[22] + Gx1[81]*Gu1[26] + Gx1[93]*Gu1[30] + Gx1[105]*Gu1[34] + Gx1[117]*Gu1[38] + Gx1[129]*Gu1[42] + Gx1[141]*Gu1[46];
Gu2[39] = + Gx1[9]*Gu1[3] + Gx1[21]*Gu1[7] + Gx1[33]*Gu1[11] + Gx1[45]*Gu1[15] + Gx1[57]*Gu1[19] + Gx1[69]*Gu1[23] + Gx1[81]*Gu1[27] + Gx1[93]*Gu1[31] + Gx1[105]*Gu1[35] + Gx1[117]*Gu1[39] + Gx1[129]*Gu1[43] + Gx1[141]*Gu1[47];
Gu2[40] = + Gx1[10]*Gu1[0] + Gx1[22]*Gu1[4] + Gx1[34]*Gu1[8] + Gx1[46]*Gu1[12] + Gx1[58]*Gu1[16] + Gx1[70]*Gu1[20] + Gx1[82]*Gu1[24] + Gx1[94]*Gu1[28] + Gx1[106]*Gu1[32] + Gx1[118]*Gu1[36] + Gx1[130]*Gu1[40] + Gx1[142]*Gu1[44];
Gu2[41] = + Gx1[10]*Gu1[1] + Gx1[22]*Gu1[5] + Gx1[34]*Gu1[9] + Gx1[46]*Gu1[13] + Gx1[58]*Gu1[17] + Gx1[70]*Gu1[21] + Gx1[82]*Gu1[25] + Gx1[94]*Gu1[29] + Gx1[106]*Gu1[33] + Gx1[118]*Gu1[37] + Gx1[130]*Gu1[41] + Gx1[142]*Gu1[45];
Gu2[42] = + Gx1[10]*Gu1[2] + Gx1[22]*Gu1[6] + Gx1[34]*Gu1[10] + Gx1[46]*Gu1[14] + Gx1[58]*Gu1[18] + Gx1[70]*Gu1[22] + Gx1[82]*Gu1[26] + Gx1[94]*Gu1[30] + Gx1[106]*Gu1[34] + Gx1[118]*Gu1[38] + Gx1[130]*Gu1[42] + Gx1[142]*Gu1[46];
Gu2[43] = + Gx1[10]*Gu1[3] + Gx1[22]*Gu1[7] + Gx1[34]*Gu1[11] + Gx1[46]*Gu1[15] + Gx1[58]*Gu1[19] + Gx1[70]*Gu1[23] + Gx1[82]*Gu1[27] + Gx1[94]*Gu1[31] + Gx1[106]*Gu1[35] + Gx1[118]*Gu1[39] + Gx1[130]*Gu1[43] + Gx1[142]*Gu1[47];
Gu2[44] = + Gx1[11]*Gu1[0] + Gx1[23]*Gu1[4] + Gx1[35]*Gu1[8] + Gx1[47]*Gu1[12] + Gx1[59]*Gu1[16] + Gx1[71]*Gu1[20] + Gx1[83]*Gu1[24] + Gx1[95]*Gu1[28] + Gx1[107]*Gu1[32] + Gx1[119]*Gu1[36] + Gx1[131]*Gu1[40] + Gx1[143]*Gu1[44];
Gu2[45] = + Gx1[11]*Gu1[1] + Gx1[23]*Gu1[5] + Gx1[35]*Gu1[9] + Gx1[47]*Gu1[13] + Gx1[59]*Gu1[17] + Gx1[71]*Gu1[21] + Gx1[83]*Gu1[25] + Gx1[95]*Gu1[29] + Gx1[107]*Gu1[33] + Gx1[119]*Gu1[37] + Gx1[131]*Gu1[41] + Gx1[143]*Gu1[45];
Gu2[46] = + Gx1[11]*Gu1[2] + Gx1[23]*Gu1[6] + Gx1[35]*Gu1[10] + Gx1[47]*Gu1[14] + Gx1[59]*Gu1[18] + Gx1[71]*Gu1[22] + Gx1[83]*Gu1[26] + Gx1[95]*Gu1[30] + Gx1[107]*Gu1[34] + Gx1[119]*Gu1[38] + Gx1[131]*Gu1[42] + Gx1[143]*Gu1[46];
Gu2[47] = + Gx1[11]*Gu1[3] + Gx1[23]*Gu1[7] + Gx1[35]*Gu1[11] + Gx1[47]*Gu1[15] + Gx1[59]*Gu1[19] + Gx1[71]*Gu1[23] + Gx1[83]*Gu1[27] + Gx1[95]*Gu1[31] + Gx1[107]*Gu1[35] + Gx1[119]*Gu1[39] + Gx1[131]*Gu1[43] + Gx1[143]*Gu1[47];
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
Gu3[27] = + Gu2[27];
Gu3[28] = + Gu2[28];
Gu3[29] = + Gu2[29];
Gu3[30] = + Gu2[30];
Gu3[31] = + Gu2[31];
Gu3[32] = + Gu2[32];
Gu3[33] = + Gu2[33];
Gu3[34] = + Gu2[34];
Gu3[35] = + Gu2[35];
Gu3[36] = + Gu2[36];
Gu3[37] = + Gu2[37];
Gu3[38] = + Gu2[38];
Gu3[39] = + Gu2[39];
Gu3[40] = + Gu2[40];
Gu3[41] = + Gu2[41];
Gu3[42] = + Gu2[42];
Gu3[43] = + Gu2[43];
Gu3[44] = + Gu2[44];
Gu3[45] = + Gu2[45];
Gu3[46] = + Gu2[46];
Gu3[47] = + Gu2[47];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[12]*w11[1] + Gx1[24]*w11[2] + Gx1[36]*w11[3] + Gx1[48]*w11[4] + Gx1[60]*w11[5] + Gx1[72]*w11[6] + Gx1[84]*w11[7] + Gx1[96]*w11[8] + Gx1[108]*w11[9] + Gx1[120]*w11[10] + Gx1[132]*w11[11] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[13]*w11[1] + Gx1[25]*w11[2] + Gx1[37]*w11[3] + Gx1[49]*w11[4] + Gx1[61]*w11[5] + Gx1[73]*w11[6] + Gx1[85]*w11[7] + Gx1[97]*w11[8] + Gx1[109]*w11[9] + Gx1[121]*w11[10] + Gx1[133]*w11[11] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[14]*w11[1] + Gx1[26]*w11[2] + Gx1[38]*w11[3] + Gx1[50]*w11[4] + Gx1[62]*w11[5] + Gx1[74]*w11[6] + Gx1[86]*w11[7] + Gx1[98]*w11[8] + Gx1[110]*w11[9] + Gx1[122]*w11[10] + Gx1[134]*w11[11] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[15]*w11[1] + Gx1[27]*w11[2] + Gx1[39]*w11[3] + Gx1[51]*w11[4] + Gx1[63]*w11[5] + Gx1[75]*w11[6] + Gx1[87]*w11[7] + Gx1[99]*w11[8] + Gx1[111]*w11[9] + Gx1[123]*w11[10] + Gx1[135]*w11[11] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[16]*w11[1] + Gx1[28]*w11[2] + Gx1[40]*w11[3] + Gx1[52]*w11[4] + Gx1[64]*w11[5] + Gx1[76]*w11[6] + Gx1[88]*w11[7] + Gx1[100]*w11[8] + Gx1[112]*w11[9] + Gx1[124]*w11[10] + Gx1[136]*w11[11] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[17]*w11[1] + Gx1[29]*w11[2] + Gx1[41]*w11[3] + Gx1[53]*w11[4] + Gx1[65]*w11[5] + Gx1[77]*w11[6] + Gx1[89]*w11[7] + Gx1[101]*w11[8] + Gx1[113]*w11[9] + Gx1[125]*w11[10] + Gx1[137]*w11[11] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[18]*w11[1] + Gx1[30]*w11[2] + Gx1[42]*w11[3] + Gx1[54]*w11[4] + Gx1[66]*w11[5] + Gx1[78]*w11[6] + Gx1[90]*w11[7] + Gx1[102]*w11[8] + Gx1[114]*w11[9] + Gx1[126]*w11[10] + Gx1[138]*w11[11] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[19]*w11[1] + Gx1[31]*w11[2] + Gx1[43]*w11[3] + Gx1[55]*w11[4] + Gx1[67]*w11[5] + Gx1[79]*w11[6] + Gx1[91]*w11[7] + Gx1[103]*w11[8] + Gx1[115]*w11[9] + Gx1[127]*w11[10] + Gx1[139]*w11[11] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[20]*w11[1] + Gx1[32]*w11[2] + Gx1[44]*w11[3] + Gx1[56]*w11[4] + Gx1[68]*w11[5] + Gx1[80]*w11[6] + Gx1[92]*w11[7] + Gx1[104]*w11[8] + Gx1[116]*w11[9] + Gx1[128]*w11[10] + Gx1[140]*w11[11] + w12[8];
w13[9] = + Gx1[9]*w11[0] + Gx1[21]*w11[1] + Gx1[33]*w11[2] + Gx1[45]*w11[3] + Gx1[57]*w11[4] + Gx1[69]*w11[5] + Gx1[81]*w11[6] + Gx1[93]*w11[7] + Gx1[105]*w11[8] + Gx1[117]*w11[9] + Gx1[129]*w11[10] + Gx1[141]*w11[11] + w12[9];
w13[10] = + Gx1[10]*w11[0] + Gx1[22]*w11[1] + Gx1[34]*w11[2] + Gx1[46]*w11[3] + Gx1[58]*w11[4] + Gx1[70]*w11[5] + Gx1[82]*w11[6] + Gx1[94]*w11[7] + Gx1[106]*w11[8] + Gx1[118]*w11[9] + Gx1[130]*w11[10] + Gx1[142]*w11[11] + w12[10];
w13[11] = + Gx1[11]*w11[0] + Gx1[23]*w11[1] + Gx1[35]*w11[2] + Gx1[47]*w11[3] + Gx1[59]*w11[4] + Gx1[71]*w11[5] + Gx1[83]*w11[6] + Gx1[95]*w11[7] + Gx1[107]*w11[8] + Gx1[119]*w11[9] + Gx1[131]*w11[10] + Gx1[143]*w11[11] + w12[11];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[4]*w11[1] + Gu1[8]*w11[2] + Gu1[12]*w11[3] + Gu1[16]*w11[4] + Gu1[20]*w11[5] + Gu1[24]*w11[6] + Gu1[28]*w11[7] + Gu1[32]*w11[8] + Gu1[36]*w11[9] + Gu1[40]*w11[10] + Gu1[44]*w11[11];
U1[1] += + Gu1[1]*w11[0] + Gu1[5]*w11[1] + Gu1[9]*w11[2] + Gu1[13]*w11[3] + Gu1[17]*w11[4] + Gu1[21]*w11[5] + Gu1[25]*w11[6] + Gu1[29]*w11[7] + Gu1[33]*w11[8] + Gu1[37]*w11[9] + Gu1[41]*w11[10] + Gu1[45]*w11[11];
U1[2] += + Gu1[2]*w11[0] + Gu1[6]*w11[1] + Gu1[10]*w11[2] + Gu1[14]*w11[3] + Gu1[18]*w11[4] + Gu1[22]*w11[5] + Gu1[26]*w11[6] + Gu1[30]*w11[7] + Gu1[34]*w11[8] + Gu1[38]*w11[9] + Gu1[42]*w11[10] + Gu1[46]*w11[11];
U1[3] += + Gu1[3]*w11[0] + Gu1[7]*w11[1] + Gu1[11]*w11[2] + Gu1[15]*w11[3] + Gu1[19]*w11[4] + Gu1[23]*w11[5] + Gu1[27]*w11[6] + Gu1[31]*w11[7] + Gu1[35]*w11[8] + Gu1[39]*w11[9] + Gu1[43]*w11[10] + Gu1[47]*w11[11];
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
w13[9] = + w12[9];
w13[10] = + w12[10];
w13[11] = + w12[11];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11];
w12[1] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5] + Gx1[18]*w11[6] + Gx1[19]*w11[7] + Gx1[20]*w11[8] + Gx1[21]*w11[9] + Gx1[22]*w11[10] + Gx1[23]*w11[11];
w12[2] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5] + Gx1[30]*w11[6] + Gx1[31]*w11[7] + Gx1[32]*w11[8] + Gx1[33]*w11[9] + Gx1[34]*w11[10] + Gx1[35]*w11[11];
w12[3] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8] + Gx1[45]*w11[9] + Gx1[46]*w11[10] + Gx1[47]*w11[11];
w12[4] += + Gx1[48]*w11[0] + Gx1[49]*w11[1] + Gx1[50]*w11[2] + Gx1[51]*w11[3] + Gx1[52]*w11[4] + Gx1[53]*w11[5] + Gx1[54]*w11[6] + Gx1[55]*w11[7] + Gx1[56]*w11[8] + Gx1[57]*w11[9] + Gx1[58]*w11[10] + Gx1[59]*w11[11];
w12[5] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9] + Gx1[70]*w11[10] + Gx1[71]*w11[11];
w12[6] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8] + Gx1[81]*w11[9] + Gx1[82]*w11[10] + Gx1[83]*w11[11];
w12[7] += + Gx1[84]*w11[0] + Gx1[85]*w11[1] + Gx1[86]*w11[2] + Gx1[87]*w11[3] + Gx1[88]*w11[4] + Gx1[89]*w11[5] + Gx1[90]*w11[6] + Gx1[91]*w11[7] + Gx1[92]*w11[8] + Gx1[93]*w11[9] + Gx1[94]*w11[10] + Gx1[95]*w11[11];
w12[8] += + Gx1[96]*w11[0] + Gx1[97]*w11[1] + Gx1[98]*w11[2] + Gx1[99]*w11[3] + Gx1[100]*w11[4] + Gx1[101]*w11[5] + Gx1[102]*w11[6] + Gx1[103]*w11[7] + Gx1[104]*w11[8] + Gx1[105]*w11[9] + Gx1[106]*w11[10] + Gx1[107]*w11[11];
w12[9] += + Gx1[108]*w11[0] + Gx1[109]*w11[1] + Gx1[110]*w11[2] + Gx1[111]*w11[3] + Gx1[112]*w11[4] + Gx1[113]*w11[5] + Gx1[114]*w11[6] + Gx1[115]*w11[7] + Gx1[116]*w11[8] + Gx1[117]*w11[9] + Gx1[118]*w11[10] + Gx1[119]*w11[11];
w12[10] += + Gx1[120]*w11[0] + Gx1[121]*w11[1] + Gx1[122]*w11[2] + Gx1[123]*w11[3] + Gx1[124]*w11[4] + Gx1[125]*w11[5] + Gx1[126]*w11[6] + Gx1[127]*w11[7] + Gx1[128]*w11[8] + Gx1[129]*w11[9] + Gx1[130]*w11[10] + Gx1[131]*w11[11];
w12[11] += + Gx1[132]*w11[0] + Gx1[133]*w11[1] + Gx1[134]*w11[2] + Gx1[135]*w11[3] + Gx1[136]*w11[4] + Gx1[137]*w11[5] + Gx1[138]*w11[6] + Gx1[139]*w11[7] + Gx1[140]*w11[8] + Gx1[141]*w11[9] + Gx1[142]*w11[10] + Gx1[143]*w11[11];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11];
w12[1] += + Gx1[12]*w11[0] + Gx1[13]*w11[1] + Gx1[14]*w11[2] + Gx1[15]*w11[3] + Gx1[16]*w11[4] + Gx1[17]*w11[5] + Gx1[18]*w11[6] + Gx1[19]*w11[7] + Gx1[20]*w11[8] + Gx1[21]*w11[9] + Gx1[22]*w11[10] + Gx1[23]*w11[11];
w12[2] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5] + Gx1[30]*w11[6] + Gx1[31]*w11[7] + Gx1[32]*w11[8] + Gx1[33]*w11[9] + Gx1[34]*w11[10] + Gx1[35]*w11[11];
w12[3] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8] + Gx1[45]*w11[9] + Gx1[46]*w11[10] + Gx1[47]*w11[11];
w12[4] += + Gx1[48]*w11[0] + Gx1[49]*w11[1] + Gx1[50]*w11[2] + Gx1[51]*w11[3] + Gx1[52]*w11[4] + Gx1[53]*w11[5] + Gx1[54]*w11[6] + Gx1[55]*w11[7] + Gx1[56]*w11[8] + Gx1[57]*w11[9] + Gx1[58]*w11[10] + Gx1[59]*w11[11];
w12[5] += + Gx1[60]*w11[0] + Gx1[61]*w11[1] + Gx1[62]*w11[2] + Gx1[63]*w11[3] + Gx1[64]*w11[4] + Gx1[65]*w11[5] + Gx1[66]*w11[6] + Gx1[67]*w11[7] + Gx1[68]*w11[8] + Gx1[69]*w11[9] + Gx1[70]*w11[10] + Gx1[71]*w11[11];
w12[6] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8] + Gx1[81]*w11[9] + Gx1[82]*w11[10] + Gx1[83]*w11[11];
w12[7] += + Gx1[84]*w11[0] + Gx1[85]*w11[1] + Gx1[86]*w11[2] + Gx1[87]*w11[3] + Gx1[88]*w11[4] + Gx1[89]*w11[5] + Gx1[90]*w11[6] + Gx1[91]*w11[7] + Gx1[92]*w11[8] + Gx1[93]*w11[9] + Gx1[94]*w11[10] + Gx1[95]*w11[11];
w12[8] += + Gx1[96]*w11[0] + Gx1[97]*w11[1] + Gx1[98]*w11[2] + Gx1[99]*w11[3] + Gx1[100]*w11[4] + Gx1[101]*w11[5] + Gx1[102]*w11[6] + Gx1[103]*w11[7] + Gx1[104]*w11[8] + Gx1[105]*w11[9] + Gx1[106]*w11[10] + Gx1[107]*w11[11];
w12[9] += + Gx1[108]*w11[0] + Gx1[109]*w11[1] + Gx1[110]*w11[2] + Gx1[111]*w11[3] + Gx1[112]*w11[4] + Gx1[113]*w11[5] + Gx1[114]*w11[6] + Gx1[115]*w11[7] + Gx1[116]*w11[8] + Gx1[117]*w11[9] + Gx1[118]*w11[10] + Gx1[119]*w11[11];
w12[10] += + Gx1[120]*w11[0] + Gx1[121]*w11[1] + Gx1[122]*w11[2] + Gx1[123]*w11[3] + Gx1[124]*w11[4] + Gx1[125]*w11[5] + Gx1[126]*w11[6] + Gx1[127]*w11[7] + Gx1[128]*w11[8] + Gx1[129]*w11[9] + Gx1[130]*w11[10] + Gx1[131]*w11[11];
w12[11] += + Gx1[132]*w11[0] + Gx1[133]*w11[1] + Gx1[134]*w11[2] + Gx1[135]*w11[3] + Gx1[136]*w11[4] + Gx1[137]*w11[5] + Gx1[138]*w11[6] + Gx1[139]*w11[7] + Gx1[140]*w11[8] + Gx1[141]*w11[9] + Gx1[142]*w11[10] + Gx1[143]*w11[11];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2] + Gu1[3]*U1[3];
w12[1] += + Gu1[4]*U1[0] + Gu1[5]*U1[1] + Gu1[6]*U1[2] + Gu1[7]*U1[3];
w12[2] += + Gu1[8]*U1[0] + Gu1[9]*U1[1] + Gu1[10]*U1[2] + Gu1[11]*U1[3];
w12[3] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2] + Gu1[15]*U1[3];
w12[4] += + Gu1[16]*U1[0] + Gu1[17]*U1[1] + Gu1[18]*U1[2] + Gu1[19]*U1[3];
w12[5] += + Gu1[20]*U1[0] + Gu1[21]*U1[1] + Gu1[22]*U1[2] + Gu1[23]*U1[3];
w12[6] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2] + Gu1[27]*U1[3];
w12[7] += + Gu1[28]*U1[0] + Gu1[29]*U1[1] + Gu1[30]*U1[2] + Gu1[31]*U1[3];
w12[8] += + Gu1[32]*U1[0] + Gu1[33]*U1[1] + Gu1[34]*U1[2] + Gu1[35]*U1[3];
w12[9] += + Gu1[36]*U1[0] + Gu1[37]*U1[1] + Gu1[38]*U1[2] + Gu1[39]*U1[3];
w12[10] += + Gu1[40]*U1[0] + Gu1[41]*U1[1] + Gu1[42]*U1[2] + Gu1[43]*U1[3];
w12[11] += + Gu1[44]*U1[0] + Gu1[45]*U1[1] + Gu1[46]*U1[2] + Gu1[47]*U1[3];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 592) + (iCol * 4)] = acadoWorkspace.H[(iCol * 592) + (iRow * 4)];
acadoWorkspace.H[(iRow * 592) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 592 + 148) + (iRow * 4)];
acadoWorkspace.H[(iRow * 592) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 592 + 296) + (iRow * 4)];
acadoWorkspace.H[(iRow * 592) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 592 + 444) + (iRow * 4)];
acadoWorkspace.H[(iRow * 592 + 148) + (iCol * 4)] = acadoWorkspace.H[(iCol * 592) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 592 + 148) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 592 + 148) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 592 + 148) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 592 + 296) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 592 + 148) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 592 + 444) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 592 + 296) + (iCol * 4)] = acadoWorkspace.H[(iCol * 592) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 592 + 296) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 592 + 148) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 592 + 296) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 592 + 296) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 592 + 296) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 592 + 444) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 592 + 444) + (iCol * 4)] = acadoWorkspace.H[(iCol * 592) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 592 + 444) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 592 + 148) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 592 + 444) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 592 + 296) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 592 + 444) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 592 + 444) + (iRow * 4 + 3)];
}

void acado_multRDy( real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = +Dy1[0];
RDy1[1] = +Dy1[1];
RDy1[2] = +Dy1[2];
RDy1[3] = +Dy1[3];
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
QDy1[9] = 0.0;
;
QDy1[10] = 0.0;
;
QDy1[11] = 0.0;
;
}

void acado_multQN1Gx( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = +Gx1[0];
Gx2[1] = +Gx1[1];
Gx2[2] = +Gx1[2];
Gx2[3] = +Gx1[3];
Gx2[4] = +Gx1[4];
Gx2[5] = +Gx1[5];
Gx2[6] = +Gx1[6];
Gx2[7] = +Gx1[7];
Gx2[8] = +Gx1[8];
Gx2[9] = +Gx1[9];
Gx2[10] = +Gx1[10];
Gx2[11] = +Gx1[11];
Gx2[12] = +Gx1[12];
Gx2[13] = +Gx1[13];
Gx2[14] = +Gx1[14];
Gx2[15] = +Gx1[15];
Gx2[16] = +Gx1[16];
Gx2[17] = +Gx1[17];
Gx2[18] = +Gx1[18];
Gx2[19] = +Gx1[19];
Gx2[20] = +Gx1[20];
Gx2[21] = +Gx1[21];
Gx2[22] = +Gx1[22];
Gx2[23] = +Gx1[23];
Gx2[24] = + (real_t)1.0000000000000000e+01*Gx1[24];
Gx2[25] = + (real_t)1.0000000000000000e+01*Gx1[25];
Gx2[26] = + (real_t)1.0000000000000000e+01*Gx1[26];
Gx2[27] = + (real_t)1.0000000000000000e+01*Gx1[27];
Gx2[28] = + (real_t)1.0000000000000000e+01*Gx1[28];
Gx2[29] = + (real_t)1.0000000000000000e+01*Gx1[29];
Gx2[30] = + (real_t)1.0000000000000000e+01*Gx1[30];
Gx2[31] = + (real_t)1.0000000000000000e+01*Gx1[31];
Gx2[32] = + (real_t)1.0000000000000000e+01*Gx1[32];
Gx2[33] = + (real_t)1.0000000000000000e+01*Gx1[33];
Gx2[34] = + (real_t)1.0000000000000000e+01*Gx1[34];
Gx2[35] = + (real_t)1.0000000000000000e+01*Gx1[35];
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
Gx2[81] = 0.0;
;
Gx2[82] = 0.0;
;
Gx2[83] = 0.0;
;
Gx2[84] = 0.0;
;
Gx2[85] = 0.0;
;
Gx2[86] = 0.0;
;
Gx2[87] = 0.0;
;
Gx2[88] = 0.0;
;
Gx2[89] = 0.0;
;
Gx2[90] = 0.0;
;
Gx2[91] = 0.0;
;
Gx2[92] = 0.0;
;
Gx2[93] = 0.0;
;
Gx2[94] = 0.0;
;
Gx2[95] = 0.0;
;
Gx2[96] = 0.0;
;
Gx2[97] = 0.0;
;
Gx2[98] = 0.0;
;
Gx2[99] = 0.0;
;
Gx2[100] = 0.0;
;
Gx2[101] = 0.0;
;
Gx2[102] = 0.0;
;
Gx2[103] = 0.0;
;
Gx2[104] = 0.0;
;
Gx2[105] = 0.0;
;
Gx2[106] = 0.0;
;
Gx2[107] = 0.0;
;
Gx2[108] = 0.0;
;
Gx2[109] = 0.0;
;
Gx2[110] = 0.0;
;
Gx2[111] = 0.0;
;
Gx2[112] = 0.0;
;
Gx2[113] = 0.0;
;
Gx2[114] = 0.0;
;
Gx2[115] = 0.0;
;
Gx2[116] = 0.0;
;
Gx2[117] = 0.0;
;
Gx2[118] = 0.0;
;
Gx2[119] = 0.0;
;
Gx2[120] = 0.0;
;
Gx2[121] = 0.0;
;
Gx2[122] = 0.0;
;
Gx2[123] = 0.0;
;
Gx2[124] = 0.0;
;
Gx2[125] = 0.0;
;
Gx2[126] = 0.0;
;
Gx2[127] = 0.0;
;
Gx2[128] = 0.0;
;
Gx2[129] = 0.0;
;
Gx2[130] = 0.0;
;
Gx2[131] = 0.0;
;
Gx2[132] = 0.0;
;
Gx2[133] = 0.0;
;
Gx2[134] = 0.0;
;
Gx2[135] = 0.0;
;
Gx2[136] = 0.0;
;
Gx2[137] = 0.0;
;
Gx2[138] = 0.0;
;
Gx2[139] = 0.0;
;
Gx2[140] = 0.0;
;
Gx2[141] = 0.0;
;
Gx2[142] = 0.0;
;
Gx2[143] = 0.0;
;
}

void acado_multQN1Gu( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = +Gu1[0];
Gu2[1] = +Gu1[1];
Gu2[2] = +Gu1[2];
Gu2[3] = +Gu1[3];
Gu2[4] = +Gu1[4];
Gu2[5] = +Gu1[5];
Gu2[6] = +Gu1[6];
Gu2[7] = +Gu1[7];
Gu2[8] = + (real_t)1.0000000000000000e+01*Gu1[8];
Gu2[9] = + (real_t)1.0000000000000000e+01*Gu1[9];
Gu2[10] = + (real_t)1.0000000000000000e+01*Gu1[10];
Gu2[11] = + (real_t)1.0000000000000000e+01*Gu1[11];
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
Gu2[27] = 0.0;
;
Gu2[28] = 0.0;
;
Gu2[29] = 0.0;
;
Gu2[30] = 0.0;
;
Gu2[31] = 0.0;
;
Gu2[32] = 0.0;
;
Gu2[33] = 0.0;
;
Gu2[34] = 0.0;
;
Gu2[35] = 0.0;
;
Gu2[36] = 0.0;
;
Gu2[37] = 0.0;
;
Gu2[38] = 0.0;
;
Gu2[39] = 0.0;
;
Gu2[40] = 0.0;
;
Gu2[41] = 0.0;
;
Gu2[42] = 0.0;
;
Gu2[43] = 0.0;
;
Gu2[44] = 0.0;
;
Gu2[45] = 0.0;
;
Gu2[46] = 0.0;
;
Gu2[47] = 0.0;
;
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 148) + (col * 4)] = + Hx[0]*E[0] + Hx[1]*E[4] + Hx[2]*E[8] + Hx[3]*E[12] + Hx[4]*E[16] + Hx[5]*E[20] + Hx[6]*E[24] + Hx[7]*E[28] + Hx[8]*E[32] + Hx[9]*E[36] + Hx[10]*E[40] + Hx[11]*E[44];
acadoWorkspace.A[(row * 148) + (col * 4 + 1)] = + Hx[0]*E[1] + Hx[1]*E[5] + Hx[2]*E[9] + Hx[3]*E[13] + Hx[4]*E[17] + Hx[5]*E[21] + Hx[6]*E[25] + Hx[7]*E[29] + Hx[8]*E[33] + Hx[9]*E[37] + Hx[10]*E[41] + Hx[11]*E[45];
acadoWorkspace.A[(row * 148) + (col * 4 + 2)] = + Hx[0]*E[2] + Hx[1]*E[6] + Hx[2]*E[10] + Hx[3]*E[14] + Hx[4]*E[18] + Hx[5]*E[22] + Hx[6]*E[26] + Hx[7]*E[30] + Hx[8]*E[34] + Hx[9]*E[38] + Hx[10]*E[42] + Hx[11]*E[46];
acadoWorkspace.A[(row * 148) + (col * 4 + 3)] = + Hx[0]*E[3] + Hx[1]*E[7] + Hx[2]*E[11] + Hx[3]*E[15] + Hx[4]*E[19] + Hx[5]*E[23] + Hx[6]*E[27] + Hx[7]*E[31] + Hx[8]*E[35] + Hx[9]*E[39] + Hx[10]*E[43] + Hx[11]*E[47];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4] + Hx[5]*tmpd[5] + Hx[6]*tmpd[6] + Hx[7]*tmpd[7] + Hx[8]*tmpd[8] + Hx[9]*tmpd[9] + Hx[10]*tmpd[10] + Hx[11]*tmpd[11];
lbA[0] -= acadoWorkspace.evHxd[0];
ubA[0] -= acadoWorkspace.evHxd[0];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
acado_moveGxT( acadoWorkspace.evGx, acadoWorkspace.C );
acado_multGxGx( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.C, &(acadoWorkspace.C[ 144 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.C[ 144 ]), &(acadoWorkspace.C[ 288 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.C[ 288 ]), &(acadoWorkspace.C[ 432 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.C[ 432 ]), &(acadoWorkspace.C[ 576 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.C[ 576 ]), &(acadoWorkspace.C[ 720 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.C[ 720 ]), &(acadoWorkspace.C[ 864 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.C[ 864 ]), &(acadoWorkspace.C[ 1008 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.C[ 1008 ]), &(acadoWorkspace.C[ 1152 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.C[ 1152 ]), &(acadoWorkspace.C[ 1296 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.C[ 1296 ]), &(acadoWorkspace.C[ 1440 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.C[ 1440 ]), &(acadoWorkspace.C[ 1584 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1728 ]), &(acadoWorkspace.C[ 1584 ]), &(acadoWorkspace.C[ 1728 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 1872 ]), &(acadoWorkspace.C[ 1728 ]), &(acadoWorkspace.C[ 1872 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 2016 ]), &(acadoWorkspace.C[ 1872 ]), &(acadoWorkspace.C[ 2016 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 2160 ]), &(acadoWorkspace.C[ 2016 ]), &(acadoWorkspace.C[ 2160 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 2304 ]), &(acadoWorkspace.C[ 2160 ]), &(acadoWorkspace.C[ 2304 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 2448 ]), &(acadoWorkspace.C[ 2304 ]), &(acadoWorkspace.C[ 2448 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.C[ 2448 ]), &(acadoWorkspace.C[ 2592 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 2736 ]), &(acadoWorkspace.C[ 2592 ]), &(acadoWorkspace.C[ 2736 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 2880 ]), &(acadoWorkspace.C[ 2736 ]), &(acadoWorkspace.C[ 2880 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 3024 ]), &(acadoWorkspace.C[ 2880 ]), &(acadoWorkspace.C[ 3024 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 3168 ]), &(acadoWorkspace.C[ 3024 ]), &(acadoWorkspace.C[ 3168 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 3312 ]), &(acadoWorkspace.C[ 3168 ]), &(acadoWorkspace.C[ 3312 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 3456 ]), &(acadoWorkspace.C[ 3312 ]), &(acadoWorkspace.C[ 3456 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 3600 ]), &(acadoWorkspace.C[ 3456 ]), &(acadoWorkspace.C[ 3600 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 3744 ]), &(acadoWorkspace.C[ 3600 ]), &(acadoWorkspace.C[ 3744 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 3888 ]), &(acadoWorkspace.C[ 3744 ]), &(acadoWorkspace.C[ 3888 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 4032 ]), &(acadoWorkspace.C[ 3888 ]), &(acadoWorkspace.C[ 4032 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 4176 ]), &(acadoWorkspace.C[ 4032 ]), &(acadoWorkspace.C[ 4176 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 4320 ]), &(acadoWorkspace.C[ 4176 ]), &(acadoWorkspace.C[ 4320 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 4464 ]), &(acadoWorkspace.C[ 4320 ]), &(acadoWorkspace.C[ 4464 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 4608 ]), &(acadoWorkspace.C[ 4464 ]), &(acadoWorkspace.C[ 4608 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 4752 ]), &(acadoWorkspace.C[ 4608 ]), &(acadoWorkspace.C[ 4752 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 4896 ]), &(acadoWorkspace.C[ 4752 ]), &(acadoWorkspace.C[ 4896 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 5040 ]), &(acadoWorkspace.C[ 4896 ]), &(acadoWorkspace.C[ 5040 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 5184 ]), &(acadoWorkspace.C[ 5040 ]), &(acadoWorkspace.C[ 5184 ]) );
for (lRun2 = 0; lRun2 < 37; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 75)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 48 ]), &(acadoWorkspace.E[ lRun3 * 48 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 37; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (12)) * (12)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (12)) * (4)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (12)) * (4)) + (0) ]) );
}

acado_multQN1Gu( &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (37)) - (1)) * (12)) * (4)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 36; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 48 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 144 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (12)) * (4)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.evGu[ lRun2 * 48 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 37; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun2 = 0; lRun2 < 444; ++lRun2)
acadoWorkspace.sbar[lRun2 + 12] = acadoWorkspace.d[lRun2];

acadoWorkspace.lb[0] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[59];
acadoWorkspace.lb[60] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[60];
acadoWorkspace.lb[61] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[61];
acadoWorkspace.lb[62] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[62];
acadoWorkspace.lb[63] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[63];
acadoWorkspace.lb[64] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[64];
acadoWorkspace.lb[65] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[65];
acadoWorkspace.lb[66] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[66];
acadoWorkspace.lb[67] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[67];
acadoWorkspace.lb[68] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[68];
acadoWorkspace.lb[69] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[69];
acadoWorkspace.lb[70] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[70];
acadoWorkspace.lb[71] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[71];
acadoWorkspace.lb[72] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[72];
acadoWorkspace.lb[73] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[73];
acadoWorkspace.lb[74] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[74];
acadoWorkspace.lb[75] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[75];
acadoWorkspace.lb[76] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[76];
acadoWorkspace.lb[77] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[77];
acadoWorkspace.lb[78] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[78];
acadoWorkspace.lb[79] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[79];
acadoWorkspace.lb[80] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[80];
acadoWorkspace.lb[81] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[81];
acadoWorkspace.lb[82] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[82];
acadoWorkspace.lb[83] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[83];
acadoWorkspace.lb[84] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[84];
acadoWorkspace.lb[85] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[85];
acadoWorkspace.lb[86] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[86];
acadoWorkspace.lb[87] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[87];
acadoWorkspace.lb[88] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[88];
acadoWorkspace.lb[89] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[89];
acadoWorkspace.lb[90] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[90];
acadoWorkspace.lb[91] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[91];
acadoWorkspace.lb[92] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[92];
acadoWorkspace.lb[93] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[93];
acadoWorkspace.lb[94] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[94];
acadoWorkspace.lb[95] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[95];
acadoWorkspace.lb[96] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[96];
acadoWorkspace.lb[97] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[97];
acadoWorkspace.lb[98] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[98];
acadoWorkspace.lb[99] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[99];
acadoWorkspace.lb[100] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[100];
acadoWorkspace.lb[101] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[101];
acadoWorkspace.lb[102] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[102];
acadoWorkspace.lb[103] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[103];
acadoWorkspace.lb[104] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[104];
acadoWorkspace.lb[105] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[105];
acadoWorkspace.lb[106] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[106];
acadoWorkspace.lb[107] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[107];
acadoWorkspace.lb[108] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[108];
acadoWorkspace.lb[109] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[109];
acadoWorkspace.lb[110] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[110];
acadoWorkspace.lb[111] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[111];
acadoWorkspace.lb[112] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[112];
acadoWorkspace.lb[113] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[113];
acadoWorkspace.lb[114] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[114];
acadoWorkspace.lb[115] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[115];
acadoWorkspace.lb[116] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[116];
acadoWorkspace.lb[117] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[117];
acadoWorkspace.lb[118] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[118];
acadoWorkspace.lb[119] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[119];
acadoWorkspace.lb[120] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[120];
acadoWorkspace.lb[121] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[121];
acadoWorkspace.lb[122] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[122];
acadoWorkspace.lb[123] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[123];
acadoWorkspace.lb[124] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[124];
acadoWorkspace.lb[125] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[125];
acadoWorkspace.lb[126] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[126];
acadoWorkspace.lb[127] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[127];
acadoWorkspace.lb[128] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[128];
acadoWorkspace.lb[129] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[129];
acadoWorkspace.lb[130] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[130];
acadoWorkspace.lb[131] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[131];
acadoWorkspace.lb[132] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[132];
acadoWorkspace.lb[133] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[133];
acadoWorkspace.lb[134] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[134];
acadoWorkspace.lb[135] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[135];
acadoWorkspace.lb[136] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[136];
acadoWorkspace.lb[137] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[137];
acadoWorkspace.lb[138] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[138];
acadoWorkspace.lb[139] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[139];
acadoWorkspace.lb[140] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[140];
acadoWorkspace.lb[141] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[141];
acadoWorkspace.lb[142] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[142];
acadoWorkspace.lb[143] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[143];
acadoWorkspace.lb[144] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[144];
acadoWorkspace.lb[145] = (real_t)-8.7266467511653900e-02 - acadoVariables.u[145];
acadoWorkspace.lb[146] = (real_t)-1.0000000000000000e+00 - acadoVariables.u[146];
acadoWorkspace.lb[147] = (real_t)-1.5707963705062866e+00 - acadoVariables.u[147];
acadoWorkspace.ub[0] = (real_t)8.7266467511653900e-02 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)8.7266467511653900e-02 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+00 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.5707963705062866e+00 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)8.7266467511653900e-02 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)8.7266467511653900e-02 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+00 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.5707963705062866e+00 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)8.7266467511653900e-02 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)8.7266467511653900e-02 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+00 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.5707963705062866e+00 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)8.7266467511653900e-02 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)8.7266467511653900e-02 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+00 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.5707963705062866e+00 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)8.7266467511653900e-02 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)8.7266467511653900e-02 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+00 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.5707963705062866e+00 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)8.7266467511653900e-02 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)8.7266467511653900e-02 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+00 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.5707963705062866e+00 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)8.7266467511653900e-02 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)8.7266467511653900e-02 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+00 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)1.5707963705062866e+00 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)8.7266467511653900e-02 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)8.7266467511653900e-02 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)1.0000000000000000e+00 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)1.5707963705062866e+00 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)8.7266467511653900e-02 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)8.7266467511653900e-02 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)1.0000000000000000e+00 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)1.5707963705062866e+00 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)8.7266467511653900e-02 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)8.7266467511653900e-02 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)1.0000000000000000e+00 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)1.5707963705062866e+00 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)8.7266467511653900e-02 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)8.7266467511653900e-02 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)1.0000000000000000e+00 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)1.5707963705062866e+00 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)8.7266467511653900e-02 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)8.7266467511653900e-02 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)1.0000000000000000e+00 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)1.5707963705062866e+00 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)8.7266467511653900e-02 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)8.7266467511653900e-02 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)1.0000000000000000e+00 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)1.5707963705062866e+00 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)8.7266467511653900e-02 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)8.7266467511653900e-02 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)1.0000000000000000e+00 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)1.5707963705062866e+00 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)8.7266467511653900e-02 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)8.7266467511653900e-02 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)1.0000000000000000e+00 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)1.5707963705062866e+00 - acadoVariables.u[59];
acadoWorkspace.ub[60] = (real_t)8.7266467511653900e-02 - acadoVariables.u[60];
acadoWorkspace.ub[61] = (real_t)8.7266467511653900e-02 - acadoVariables.u[61];
acadoWorkspace.ub[62] = (real_t)1.0000000000000000e+00 - acadoVariables.u[62];
acadoWorkspace.ub[63] = (real_t)1.5707963705062866e+00 - acadoVariables.u[63];
acadoWorkspace.ub[64] = (real_t)8.7266467511653900e-02 - acadoVariables.u[64];
acadoWorkspace.ub[65] = (real_t)8.7266467511653900e-02 - acadoVariables.u[65];
acadoWorkspace.ub[66] = (real_t)1.0000000000000000e+00 - acadoVariables.u[66];
acadoWorkspace.ub[67] = (real_t)1.5707963705062866e+00 - acadoVariables.u[67];
acadoWorkspace.ub[68] = (real_t)8.7266467511653900e-02 - acadoVariables.u[68];
acadoWorkspace.ub[69] = (real_t)8.7266467511653900e-02 - acadoVariables.u[69];
acadoWorkspace.ub[70] = (real_t)1.0000000000000000e+00 - acadoVariables.u[70];
acadoWorkspace.ub[71] = (real_t)1.5707963705062866e+00 - acadoVariables.u[71];
acadoWorkspace.ub[72] = (real_t)8.7266467511653900e-02 - acadoVariables.u[72];
acadoWorkspace.ub[73] = (real_t)8.7266467511653900e-02 - acadoVariables.u[73];
acadoWorkspace.ub[74] = (real_t)1.0000000000000000e+00 - acadoVariables.u[74];
acadoWorkspace.ub[75] = (real_t)1.5707963705062866e+00 - acadoVariables.u[75];
acadoWorkspace.ub[76] = (real_t)8.7266467511653900e-02 - acadoVariables.u[76];
acadoWorkspace.ub[77] = (real_t)8.7266467511653900e-02 - acadoVariables.u[77];
acadoWorkspace.ub[78] = (real_t)1.0000000000000000e+00 - acadoVariables.u[78];
acadoWorkspace.ub[79] = (real_t)1.5707963705062866e+00 - acadoVariables.u[79];
acadoWorkspace.ub[80] = (real_t)8.7266467511653900e-02 - acadoVariables.u[80];
acadoWorkspace.ub[81] = (real_t)8.7266467511653900e-02 - acadoVariables.u[81];
acadoWorkspace.ub[82] = (real_t)1.0000000000000000e+00 - acadoVariables.u[82];
acadoWorkspace.ub[83] = (real_t)1.5707963705062866e+00 - acadoVariables.u[83];
acadoWorkspace.ub[84] = (real_t)8.7266467511653900e-02 - acadoVariables.u[84];
acadoWorkspace.ub[85] = (real_t)8.7266467511653900e-02 - acadoVariables.u[85];
acadoWorkspace.ub[86] = (real_t)1.0000000000000000e+00 - acadoVariables.u[86];
acadoWorkspace.ub[87] = (real_t)1.5707963705062866e+00 - acadoVariables.u[87];
acadoWorkspace.ub[88] = (real_t)8.7266467511653900e-02 - acadoVariables.u[88];
acadoWorkspace.ub[89] = (real_t)8.7266467511653900e-02 - acadoVariables.u[89];
acadoWorkspace.ub[90] = (real_t)1.0000000000000000e+00 - acadoVariables.u[90];
acadoWorkspace.ub[91] = (real_t)1.5707963705062866e+00 - acadoVariables.u[91];
acadoWorkspace.ub[92] = (real_t)8.7266467511653900e-02 - acadoVariables.u[92];
acadoWorkspace.ub[93] = (real_t)8.7266467511653900e-02 - acadoVariables.u[93];
acadoWorkspace.ub[94] = (real_t)1.0000000000000000e+00 - acadoVariables.u[94];
acadoWorkspace.ub[95] = (real_t)1.5707963705062866e+00 - acadoVariables.u[95];
acadoWorkspace.ub[96] = (real_t)8.7266467511653900e-02 - acadoVariables.u[96];
acadoWorkspace.ub[97] = (real_t)8.7266467511653900e-02 - acadoVariables.u[97];
acadoWorkspace.ub[98] = (real_t)1.0000000000000000e+00 - acadoVariables.u[98];
acadoWorkspace.ub[99] = (real_t)1.5707963705062866e+00 - acadoVariables.u[99];
acadoWorkspace.ub[100] = (real_t)8.7266467511653900e-02 - acadoVariables.u[100];
acadoWorkspace.ub[101] = (real_t)8.7266467511653900e-02 - acadoVariables.u[101];
acadoWorkspace.ub[102] = (real_t)1.0000000000000000e+00 - acadoVariables.u[102];
acadoWorkspace.ub[103] = (real_t)1.5707963705062866e+00 - acadoVariables.u[103];
acadoWorkspace.ub[104] = (real_t)8.7266467511653900e-02 - acadoVariables.u[104];
acadoWorkspace.ub[105] = (real_t)8.7266467511653900e-02 - acadoVariables.u[105];
acadoWorkspace.ub[106] = (real_t)1.0000000000000000e+00 - acadoVariables.u[106];
acadoWorkspace.ub[107] = (real_t)1.5707963705062866e+00 - acadoVariables.u[107];
acadoWorkspace.ub[108] = (real_t)8.7266467511653900e-02 - acadoVariables.u[108];
acadoWorkspace.ub[109] = (real_t)8.7266467511653900e-02 - acadoVariables.u[109];
acadoWorkspace.ub[110] = (real_t)1.0000000000000000e+00 - acadoVariables.u[110];
acadoWorkspace.ub[111] = (real_t)1.5707963705062866e+00 - acadoVariables.u[111];
acadoWorkspace.ub[112] = (real_t)8.7266467511653900e-02 - acadoVariables.u[112];
acadoWorkspace.ub[113] = (real_t)8.7266467511653900e-02 - acadoVariables.u[113];
acadoWorkspace.ub[114] = (real_t)1.0000000000000000e+00 - acadoVariables.u[114];
acadoWorkspace.ub[115] = (real_t)1.5707963705062866e+00 - acadoVariables.u[115];
acadoWorkspace.ub[116] = (real_t)8.7266467511653900e-02 - acadoVariables.u[116];
acadoWorkspace.ub[117] = (real_t)8.7266467511653900e-02 - acadoVariables.u[117];
acadoWorkspace.ub[118] = (real_t)1.0000000000000000e+00 - acadoVariables.u[118];
acadoWorkspace.ub[119] = (real_t)1.5707963705062866e+00 - acadoVariables.u[119];
acadoWorkspace.ub[120] = (real_t)8.7266467511653900e-02 - acadoVariables.u[120];
acadoWorkspace.ub[121] = (real_t)8.7266467511653900e-02 - acadoVariables.u[121];
acadoWorkspace.ub[122] = (real_t)1.0000000000000000e+00 - acadoVariables.u[122];
acadoWorkspace.ub[123] = (real_t)1.5707963705062866e+00 - acadoVariables.u[123];
acadoWorkspace.ub[124] = (real_t)8.7266467511653900e-02 - acadoVariables.u[124];
acadoWorkspace.ub[125] = (real_t)8.7266467511653900e-02 - acadoVariables.u[125];
acadoWorkspace.ub[126] = (real_t)1.0000000000000000e+00 - acadoVariables.u[126];
acadoWorkspace.ub[127] = (real_t)1.5707963705062866e+00 - acadoVariables.u[127];
acadoWorkspace.ub[128] = (real_t)8.7266467511653900e-02 - acadoVariables.u[128];
acadoWorkspace.ub[129] = (real_t)8.7266467511653900e-02 - acadoVariables.u[129];
acadoWorkspace.ub[130] = (real_t)1.0000000000000000e+00 - acadoVariables.u[130];
acadoWorkspace.ub[131] = (real_t)1.5707963705062866e+00 - acadoVariables.u[131];
acadoWorkspace.ub[132] = (real_t)8.7266467511653900e-02 - acadoVariables.u[132];
acadoWorkspace.ub[133] = (real_t)8.7266467511653900e-02 - acadoVariables.u[133];
acadoWorkspace.ub[134] = (real_t)1.0000000000000000e+00 - acadoVariables.u[134];
acadoWorkspace.ub[135] = (real_t)1.5707963705062866e+00 - acadoVariables.u[135];
acadoWorkspace.ub[136] = (real_t)8.7266467511653900e-02 - acadoVariables.u[136];
acadoWorkspace.ub[137] = (real_t)8.7266467511653900e-02 - acadoVariables.u[137];
acadoWorkspace.ub[138] = (real_t)1.0000000000000000e+00 - acadoVariables.u[138];
acadoWorkspace.ub[139] = (real_t)1.5707963705062866e+00 - acadoVariables.u[139];
acadoWorkspace.ub[140] = (real_t)8.7266467511653900e-02 - acadoVariables.u[140];
acadoWorkspace.ub[141] = (real_t)8.7266467511653900e-02 - acadoVariables.u[141];
acadoWorkspace.ub[142] = (real_t)1.0000000000000000e+00 - acadoVariables.u[142];
acadoWorkspace.ub[143] = (real_t)1.5707963705062866e+00 - acadoVariables.u[143];
acadoWorkspace.ub[144] = (real_t)8.7266467511653900e-02 - acadoVariables.u[144];
acadoWorkspace.ub[145] = (real_t)8.7266467511653900e-02 - acadoVariables.u[145];
acadoWorkspace.ub[146] = (real_t)1.0000000000000000e+00 - acadoVariables.u[146];
acadoWorkspace.ub[147] = (real_t)1.5707963705062866e+00 - acadoVariables.u[147];

for (lRun1 = 0; lRun1 < 37; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 12];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 12 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 12 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 12 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 12 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.x[lRun1 * 12 + 5];
acadoWorkspace.conValueIn[6] = acadoVariables.x[lRun1 * 12 + 6];
acadoWorkspace.conValueIn[7] = acadoVariables.x[lRun1 * 12 + 7];
acadoWorkspace.conValueIn[8] = acadoVariables.x[lRun1 * 12 + 8];
acadoWorkspace.conValueIn[9] = acadoVariables.x[lRun1 * 12 + 9];
acadoWorkspace.conValueIn[10] = acadoVariables.x[lRun1 * 12 + 10];
acadoWorkspace.conValueIn[11] = acadoVariables.x[lRun1 * 12 + 11];
acadoWorkspace.conValueIn[12] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.conValueIn[13] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[14] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[15] = acadoVariables.u[lRun1 * 4 + 3];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1] = acadoWorkspace.conValueOut[0];

acadoWorkspace.evHx[lRun1 * 12] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evHx[lRun1 * 12 + 1] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evHx[lRun1 * 12 + 2] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evHx[lRun1 * 12 + 3] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 12 + 4] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 12 + 5] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 12 + 6] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 12 + 7] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 12 + 8] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 12 + 9] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 12 + 10] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 12 + 11] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHu[lRun1 * 4] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHu[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHu[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHu[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[16];
}



for (lRun2 = 0; lRun2 < 36; ++lRun2)
{
for (lRun3 = 0; lRun3 < lRun2 + 1; ++lRun3)
{
lRun4 = (((lRun3) * (lRun3 * -1 + 73)) / (2)) + (lRun2);
lRun5 = lRun2 + 1;
acado_multHxE( &(acadoWorkspace.evHx[ lRun2 * 12 + 12 ]), &(acadoWorkspace.E[ lRun4 * 48 ]), lRun5, lRun3 );
}
}

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[1] = acadoWorkspace.evHu[1];
acadoWorkspace.A[2] = acadoWorkspace.evHu[2];
acadoWorkspace.A[3] = acadoWorkspace.evHu[3];
acadoWorkspace.A[152] = acadoWorkspace.evHu[4];
acadoWorkspace.A[153] = acadoWorkspace.evHu[5];
acadoWorkspace.A[154] = acadoWorkspace.evHu[6];
acadoWorkspace.A[155] = acadoWorkspace.evHu[7];
acadoWorkspace.A[304] = acadoWorkspace.evHu[8];
acadoWorkspace.A[305] = acadoWorkspace.evHu[9];
acadoWorkspace.A[306] = acadoWorkspace.evHu[10];
acadoWorkspace.A[307] = acadoWorkspace.evHu[11];
acadoWorkspace.A[456] = acadoWorkspace.evHu[12];
acadoWorkspace.A[457] = acadoWorkspace.evHu[13];
acadoWorkspace.A[458] = acadoWorkspace.evHu[14];
acadoWorkspace.A[459] = acadoWorkspace.evHu[15];
acadoWorkspace.A[608] = acadoWorkspace.evHu[16];
acadoWorkspace.A[609] = acadoWorkspace.evHu[17];
acadoWorkspace.A[610] = acadoWorkspace.evHu[18];
acadoWorkspace.A[611] = acadoWorkspace.evHu[19];
acadoWorkspace.A[760] = acadoWorkspace.evHu[20];
acadoWorkspace.A[761] = acadoWorkspace.evHu[21];
acadoWorkspace.A[762] = acadoWorkspace.evHu[22];
acadoWorkspace.A[763] = acadoWorkspace.evHu[23];
acadoWorkspace.A[912] = acadoWorkspace.evHu[24];
acadoWorkspace.A[913] = acadoWorkspace.evHu[25];
acadoWorkspace.A[914] = acadoWorkspace.evHu[26];
acadoWorkspace.A[915] = acadoWorkspace.evHu[27];
acadoWorkspace.A[1064] = acadoWorkspace.evHu[28];
acadoWorkspace.A[1065] = acadoWorkspace.evHu[29];
acadoWorkspace.A[1066] = acadoWorkspace.evHu[30];
acadoWorkspace.A[1067] = acadoWorkspace.evHu[31];
acadoWorkspace.A[1216] = acadoWorkspace.evHu[32];
acadoWorkspace.A[1217] = acadoWorkspace.evHu[33];
acadoWorkspace.A[1218] = acadoWorkspace.evHu[34];
acadoWorkspace.A[1219] = acadoWorkspace.evHu[35];
acadoWorkspace.A[1368] = acadoWorkspace.evHu[36];
acadoWorkspace.A[1369] = acadoWorkspace.evHu[37];
acadoWorkspace.A[1370] = acadoWorkspace.evHu[38];
acadoWorkspace.A[1371] = acadoWorkspace.evHu[39];
acadoWorkspace.A[1520] = acadoWorkspace.evHu[40];
acadoWorkspace.A[1521] = acadoWorkspace.evHu[41];
acadoWorkspace.A[1522] = acadoWorkspace.evHu[42];
acadoWorkspace.A[1523] = acadoWorkspace.evHu[43];
acadoWorkspace.A[1672] = acadoWorkspace.evHu[44];
acadoWorkspace.A[1673] = acadoWorkspace.evHu[45];
acadoWorkspace.A[1674] = acadoWorkspace.evHu[46];
acadoWorkspace.A[1675] = acadoWorkspace.evHu[47];
acadoWorkspace.A[1824] = acadoWorkspace.evHu[48];
acadoWorkspace.A[1825] = acadoWorkspace.evHu[49];
acadoWorkspace.A[1826] = acadoWorkspace.evHu[50];
acadoWorkspace.A[1827] = acadoWorkspace.evHu[51];
acadoWorkspace.A[1976] = acadoWorkspace.evHu[52];
acadoWorkspace.A[1977] = acadoWorkspace.evHu[53];
acadoWorkspace.A[1978] = acadoWorkspace.evHu[54];
acadoWorkspace.A[1979] = acadoWorkspace.evHu[55];
acadoWorkspace.A[2128] = acadoWorkspace.evHu[56];
acadoWorkspace.A[2129] = acadoWorkspace.evHu[57];
acadoWorkspace.A[2130] = acadoWorkspace.evHu[58];
acadoWorkspace.A[2131] = acadoWorkspace.evHu[59];
acadoWorkspace.A[2280] = acadoWorkspace.evHu[60];
acadoWorkspace.A[2281] = acadoWorkspace.evHu[61];
acadoWorkspace.A[2282] = acadoWorkspace.evHu[62];
acadoWorkspace.A[2283] = acadoWorkspace.evHu[63];
acadoWorkspace.A[2432] = acadoWorkspace.evHu[64];
acadoWorkspace.A[2433] = acadoWorkspace.evHu[65];
acadoWorkspace.A[2434] = acadoWorkspace.evHu[66];
acadoWorkspace.A[2435] = acadoWorkspace.evHu[67];
acadoWorkspace.A[2584] = acadoWorkspace.evHu[68];
acadoWorkspace.A[2585] = acadoWorkspace.evHu[69];
acadoWorkspace.A[2586] = acadoWorkspace.evHu[70];
acadoWorkspace.A[2587] = acadoWorkspace.evHu[71];
acadoWorkspace.A[2736] = acadoWorkspace.evHu[72];
acadoWorkspace.A[2737] = acadoWorkspace.evHu[73];
acadoWorkspace.A[2738] = acadoWorkspace.evHu[74];
acadoWorkspace.A[2739] = acadoWorkspace.evHu[75];
acadoWorkspace.A[2888] = acadoWorkspace.evHu[76];
acadoWorkspace.A[2889] = acadoWorkspace.evHu[77];
acadoWorkspace.A[2890] = acadoWorkspace.evHu[78];
acadoWorkspace.A[2891] = acadoWorkspace.evHu[79];
acadoWorkspace.A[3040] = acadoWorkspace.evHu[80];
acadoWorkspace.A[3041] = acadoWorkspace.evHu[81];
acadoWorkspace.A[3042] = acadoWorkspace.evHu[82];
acadoWorkspace.A[3043] = acadoWorkspace.evHu[83];
acadoWorkspace.A[3192] = acadoWorkspace.evHu[84];
acadoWorkspace.A[3193] = acadoWorkspace.evHu[85];
acadoWorkspace.A[3194] = acadoWorkspace.evHu[86];
acadoWorkspace.A[3195] = acadoWorkspace.evHu[87];
acadoWorkspace.A[3344] = acadoWorkspace.evHu[88];
acadoWorkspace.A[3345] = acadoWorkspace.evHu[89];
acadoWorkspace.A[3346] = acadoWorkspace.evHu[90];
acadoWorkspace.A[3347] = acadoWorkspace.evHu[91];
acadoWorkspace.A[3496] = acadoWorkspace.evHu[92];
acadoWorkspace.A[3497] = acadoWorkspace.evHu[93];
acadoWorkspace.A[3498] = acadoWorkspace.evHu[94];
acadoWorkspace.A[3499] = acadoWorkspace.evHu[95];
acadoWorkspace.A[3648] = acadoWorkspace.evHu[96];
acadoWorkspace.A[3649] = acadoWorkspace.evHu[97];
acadoWorkspace.A[3650] = acadoWorkspace.evHu[98];
acadoWorkspace.A[3651] = acadoWorkspace.evHu[99];
acadoWorkspace.A[3800] = acadoWorkspace.evHu[100];
acadoWorkspace.A[3801] = acadoWorkspace.evHu[101];
acadoWorkspace.A[3802] = acadoWorkspace.evHu[102];
acadoWorkspace.A[3803] = acadoWorkspace.evHu[103];
acadoWorkspace.A[3952] = acadoWorkspace.evHu[104];
acadoWorkspace.A[3953] = acadoWorkspace.evHu[105];
acadoWorkspace.A[3954] = acadoWorkspace.evHu[106];
acadoWorkspace.A[3955] = acadoWorkspace.evHu[107];
acadoWorkspace.A[4104] = acadoWorkspace.evHu[108];
acadoWorkspace.A[4105] = acadoWorkspace.evHu[109];
acadoWorkspace.A[4106] = acadoWorkspace.evHu[110];
acadoWorkspace.A[4107] = acadoWorkspace.evHu[111];
acadoWorkspace.A[4256] = acadoWorkspace.evHu[112];
acadoWorkspace.A[4257] = acadoWorkspace.evHu[113];
acadoWorkspace.A[4258] = acadoWorkspace.evHu[114];
acadoWorkspace.A[4259] = acadoWorkspace.evHu[115];
acadoWorkspace.A[4408] = acadoWorkspace.evHu[116];
acadoWorkspace.A[4409] = acadoWorkspace.evHu[117];
acadoWorkspace.A[4410] = acadoWorkspace.evHu[118];
acadoWorkspace.A[4411] = acadoWorkspace.evHu[119];
acadoWorkspace.A[4560] = acadoWorkspace.evHu[120];
acadoWorkspace.A[4561] = acadoWorkspace.evHu[121];
acadoWorkspace.A[4562] = acadoWorkspace.evHu[122];
acadoWorkspace.A[4563] = acadoWorkspace.evHu[123];
acadoWorkspace.A[4712] = acadoWorkspace.evHu[124];
acadoWorkspace.A[4713] = acadoWorkspace.evHu[125];
acadoWorkspace.A[4714] = acadoWorkspace.evHu[126];
acadoWorkspace.A[4715] = acadoWorkspace.evHu[127];
acadoWorkspace.A[4864] = acadoWorkspace.evHu[128];
acadoWorkspace.A[4865] = acadoWorkspace.evHu[129];
acadoWorkspace.A[4866] = acadoWorkspace.evHu[130];
acadoWorkspace.A[4867] = acadoWorkspace.evHu[131];
acadoWorkspace.A[5016] = acadoWorkspace.evHu[132];
acadoWorkspace.A[5017] = acadoWorkspace.evHu[133];
acadoWorkspace.A[5018] = acadoWorkspace.evHu[134];
acadoWorkspace.A[5019] = acadoWorkspace.evHu[135];
acadoWorkspace.A[5168] = acadoWorkspace.evHu[136];
acadoWorkspace.A[5169] = acadoWorkspace.evHu[137];
acadoWorkspace.A[5170] = acadoWorkspace.evHu[138];
acadoWorkspace.A[5171] = acadoWorkspace.evHu[139];
acadoWorkspace.A[5320] = acadoWorkspace.evHu[140];
acadoWorkspace.A[5321] = acadoWorkspace.evHu[141];
acadoWorkspace.A[5322] = acadoWorkspace.evHu[142];
acadoWorkspace.A[5323] = acadoWorkspace.evHu[143];
acadoWorkspace.A[5472] = acadoWorkspace.evHu[144];
acadoWorkspace.A[5473] = acadoWorkspace.evHu[145];
acadoWorkspace.A[5474] = acadoWorkspace.evHu[146];
acadoWorkspace.A[5475] = acadoWorkspace.evHu[147];
acadoWorkspace.lbA[0] = - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = - acadoWorkspace.evH[36];

acadoWorkspace.ubA[0] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[36];

}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];
acadoWorkspace.Dx0[10] = acadoVariables.x0[10] - acadoVariables.x[10];
acadoWorkspace.Dx0[11] = acadoVariables.x0[11] - acadoVariables.x[11];
for (lRun1 = 0; lRun1 < 148; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.Dy[ 4 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 52 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 68 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 76 ]), &(acadoWorkspace.g[ 76 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 80 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 84 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.g[ 88 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 92 ]), &(acadoWorkspace.g[ 92 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 96 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.g[ 100 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 104 ]), &(acadoWorkspace.g[ 104 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 108 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.g[ 112 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 116 ]), &(acadoWorkspace.g[ 116 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 120 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 124 ]), &(acadoWorkspace.g[ 124 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.g[ 128 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 132 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.g[ 136 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.g[ 140 ]) );
acado_multRDy( &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 144 ]) );

acado_multQDy( acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Dy[ 4 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 8 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 16 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 60 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 28 ]), &(acadoWorkspace.QDy[ 84 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 32 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.QDy[ 132 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 52 ]), &(acadoWorkspace.QDy[ 156 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 56 ]), &(acadoWorkspace.QDy[ 168 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 180 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 64 ]), &(acadoWorkspace.QDy[ 192 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 68 ]), &(acadoWorkspace.QDy[ 204 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 216 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 76 ]), &(acadoWorkspace.QDy[ 228 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 240 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 252 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.QDy[ 264 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 92 ]), &(acadoWorkspace.QDy[ 276 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 288 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 100 ]), &(acadoWorkspace.QDy[ 300 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 104 ]), &(acadoWorkspace.QDy[ 312 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 324 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 112 ]), &(acadoWorkspace.QDy[ 336 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 116 ]), &(acadoWorkspace.QDy[ 348 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 360 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 124 ]), &(acadoWorkspace.QDy[ 372 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 128 ]), &(acadoWorkspace.QDy[ 384 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 396 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 136 ]), &(acadoWorkspace.QDy[ 408 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 140 ]), &(acadoWorkspace.QDy[ 420 ]) );
acado_multQDy( &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 432 ]) );

acadoWorkspace.QDy[444] = +acadoWorkspace.DyN[0];
acadoWorkspace.QDy[445] = +acadoWorkspace.DyN[1];
acadoWorkspace.QDy[446] = + (real_t)1.0000000000000000e+01*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[447] = 0.0;
;
acadoWorkspace.QDy[448] = 0.0;
;
acadoWorkspace.QDy[449] = 0.0;
;
acadoWorkspace.QDy[450] = 0.0;
;
acadoWorkspace.QDy[451] = 0.0;
;
acadoWorkspace.QDy[452] = 0.0;
;
acadoWorkspace.QDy[453] = 0.0;
;
acadoWorkspace.QDy[454] = 0.0;
;
acadoWorkspace.QDy[455] = 0.0;
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
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 12 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1728 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1872 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2016 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2160 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 192 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2304 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.sbar[ 204 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2448 ]), &(acadoWorkspace.sbar[ 204 ]), &(acadoWorkspace.sbar[ 216 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.sbar[ 228 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2736 ]), &(acadoWorkspace.sbar[ 228 ]), &(acadoWorkspace.sbar[ 240 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2880 ]), &(acadoWorkspace.sbar[ 240 ]), &(acadoWorkspace.sbar[ 252 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3024 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.sbar[ 264 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3168 ]), &(acadoWorkspace.sbar[ 264 ]), &(acadoWorkspace.sbar[ 276 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3312 ]), &(acadoWorkspace.sbar[ 276 ]), &(acadoWorkspace.sbar[ 288 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3456 ]), &(acadoWorkspace.sbar[ 288 ]), &(acadoWorkspace.sbar[ 300 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3600 ]), &(acadoWorkspace.sbar[ 300 ]), &(acadoWorkspace.sbar[ 312 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3744 ]), &(acadoWorkspace.sbar[ 312 ]), &(acadoWorkspace.sbar[ 324 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 3888 ]), &(acadoWorkspace.sbar[ 324 ]), &(acadoWorkspace.sbar[ 336 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4032 ]), &(acadoWorkspace.sbar[ 336 ]), &(acadoWorkspace.sbar[ 348 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4176 ]), &(acadoWorkspace.sbar[ 348 ]), &(acadoWorkspace.sbar[ 360 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4320 ]), &(acadoWorkspace.sbar[ 360 ]), &(acadoWorkspace.sbar[ 372 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4464 ]), &(acadoWorkspace.sbar[ 372 ]), &(acadoWorkspace.sbar[ 384 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4608 ]), &(acadoWorkspace.sbar[ 384 ]), &(acadoWorkspace.sbar[ 396 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4752 ]), &(acadoWorkspace.sbar[ 396 ]), &(acadoWorkspace.sbar[ 408 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 4896 ]), &(acadoWorkspace.sbar[ 408 ]), &(acadoWorkspace.sbar[ 420 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 5040 ]), &(acadoWorkspace.sbar[ 420 ]), &(acadoWorkspace.sbar[ 432 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 5184 ]), &(acadoWorkspace.sbar[ 432 ]), &(acadoWorkspace.sbar[ 444 ]) );

acadoWorkspace.w1[0] = +acadoWorkspace.sbar[444] + acadoWorkspace.QDy[444];
acadoWorkspace.w1[1] = +acadoWorkspace.sbar[445] + acadoWorkspace.QDy[445];
acadoWorkspace.w1[2] = + (real_t)1.0000000000000000e+01*acadoWorkspace.sbar[446] + acadoWorkspace.QDy[446];
acadoWorkspace.w1[3] = + acadoWorkspace.QDy[447];
acadoWorkspace.w1[4] = + acadoWorkspace.QDy[448];
acadoWorkspace.w1[5] = + acadoWorkspace.QDy[449];
acadoWorkspace.w1[6] = + acadoWorkspace.QDy[450];
acadoWorkspace.w1[7] = + acadoWorkspace.QDy[451];
acadoWorkspace.w1[8] = + acadoWorkspace.QDy[452];
acadoWorkspace.w1[9] = + acadoWorkspace.QDy[453];
acadoWorkspace.w1[10] = + acadoWorkspace.QDy[454];
acadoWorkspace.w1[11] = + acadoWorkspace.QDy[455];
acado_macBTw1( &(acadoWorkspace.evGu[ 1728 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 144 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 5184 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 432 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 432 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1680 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 140 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 5040 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 420 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 420 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1632 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 136 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4896 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 408 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 408 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1584 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 132 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4752 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 396 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 396 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1536 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 128 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4608 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 384 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 384 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1488 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 124 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4464 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 372 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 372 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1440 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 120 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4320 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 360 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 360 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1392 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 116 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4176 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 348 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 348 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1344 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 112 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 4032 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 336 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 336 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1296 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 108 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3888 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 324 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 324 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1248 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 104 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3744 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 312 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 312 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1200 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 100 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3600 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 300 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 300 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1152 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 96 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3456 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 288 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 288 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1104 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 92 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3312 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 276 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 276 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1056 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 88 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3168 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 264 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 264 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 1008 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 84 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 3024 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 252 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 252 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 960 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 80 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2880 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 240 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 240 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 912 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 76 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2736 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 228 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 228 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 216 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 216 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 816 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 68 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2448 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 204 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 204 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 768 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 64 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2304 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 192 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 192 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 720 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2160 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 180 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 180 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 672 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2016 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 168 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 168 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 624 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1872 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 156 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 156 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1728 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 528 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1584 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 132 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 132 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1440 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 336 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1008 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 84 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 84 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 864 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 240 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 720 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 60 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 48 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.sbar[ 12 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );



acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, acadoWorkspace.lbA, acadoWorkspace.ubA );
acado_macHxd( &(acadoWorkspace.evHx[ 12 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.lbA[ 1 ]), &(acadoWorkspace.ubA[ 1 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.lbA[ 2 ]), &(acadoWorkspace.ubA[ 2 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.lbA[ 3 ]), &(acadoWorkspace.ubA[ 3 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.lbA[ 4 ]), &(acadoWorkspace.ubA[ 4 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.lbA[ 5 ]), &(acadoWorkspace.ubA[ 5 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.lbA[ 6 ]), &(acadoWorkspace.ubA[ 6 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.lbA[ 7 ]), &(acadoWorkspace.ubA[ 7 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.lbA[ 8 ]), &(acadoWorkspace.ubA[ 8 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.lbA[ 9 ]), &(acadoWorkspace.ubA[ 9 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.lbA[ 11 ]), &(acadoWorkspace.ubA[ 11 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.lbA[ 12 ]), &(acadoWorkspace.ubA[ 12 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.lbA[ 13 ]), &(acadoWorkspace.ubA[ 13 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.lbA[ 14 ]), &(acadoWorkspace.ubA[ 14 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.lbA[ 15 ]), &(acadoWorkspace.ubA[ 15 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.sbar[ 204 ]), &(acadoWorkspace.lbA[ 17 ]), &(acadoWorkspace.ubA[ 17 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.lbA[ 18 ]), &(acadoWorkspace.ubA[ 18 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.sbar[ 228 ]), &(acadoWorkspace.lbA[ 19 ]), &(acadoWorkspace.ubA[ 19 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.sbar[ 240 ]), &(acadoWorkspace.lbA[ 20 ]), &(acadoWorkspace.ubA[ 20 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 252 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.lbA[ 21 ]), &(acadoWorkspace.ubA[ 21 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 264 ]), &(acadoWorkspace.sbar[ 264 ]), &(acadoWorkspace.lbA[ 22 ]), &(acadoWorkspace.ubA[ 22 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 276 ]), &(acadoWorkspace.sbar[ 276 ]), &(acadoWorkspace.lbA[ 23 ]), &(acadoWorkspace.ubA[ 23 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 288 ]), &(acadoWorkspace.sbar[ 288 ]), &(acadoWorkspace.lbA[ 24 ]), &(acadoWorkspace.ubA[ 24 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 300 ]), &(acadoWorkspace.sbar[ 300 ]), &(acadoWorkspace.lbA[ 25 ]), &(acadoWorkspace.ubA[ 25 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 312 ]), &(acadoWorkspace.sbar[ 312 ]), &(acadoWorkspace.lbA[ 26 ]), &(acadoWorkspace.ubA[ 26 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 324 ]), &(acadoWorkspace.sbar[ 324 ]), &(acadoWorkspace.lbA[ 27 ]), &(acadoWorkspace.ubA[ 27 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 336 ]), &(acadoWorkspace.sbar[ 336 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 348 ]), &(acadoWorkspace.sbar[ 348 ]), &(acadoWorkspace.lbA[ 29 ]), &(acadoWorkspace.ubA[ 29 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 360 ]), &(acadoWorkspace.sbar[ 360 ]), &(acadoWorkspace.lbA[ 30 ]), &(acadoWorkspace.ubA[ 30 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 372 ]), &(acadoWorkspace.sbar[ 372 ]), &(acadoWorkspace.lbA[ 31 ]), &(acadoWorkspace.ubA[ 31 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 384 ]), &(acadoWorkspace.sbar[ 384 ]), &(acadoWorkspace.lbA[ 32 ]), &(acadoWorkspace.ubA[ 32 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 396 ]), &(acadoWorkspace.sbar[ 396 ]), &(acadoWorkspace.lbA[ 33 ]), &(acadoWorkspace.ubA[ 33 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 408 ]), &(acadoWorkspace.sbar[ 408 ]), &(acadoWorkspace.lbA[ 34 ]), &(acadoWorkspace.ubA[ 34 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 420 ]), &(acadoWorkspace.sbar[ 420 ]), &(acadoWorkspace.lbA[ 35 ]), &(acadoWorkspace.ubA[ 35 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 432 ]), &(acadoWorkspace.sbar[ 432 ]), &(acadoWorkspace.lbA[ 36 ]), &(acadoWorkspace.ubA[ 36 ]) );

}

void acado_expand(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 148; ++lRun1)
acadoVariables.u[lRun1] += acadoWorkspace.x[lRun1];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
for (lRun1 = 0; lRun1 < 444; ++lRun1)
acadoWorkspace.sbar[lRun1 + 12] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 12 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 12 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 288 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 432 ]), &(acadoWorkspace.evGu[ 144 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 60 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 720 ]), &(acadoWorkspace.evGu[ 240 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 60 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 864 ]), &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 84 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1008 ]), &(acadoWorkspace.evGu[ 336 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 84 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.evGu[ 384 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1440 ]), &(acadoWorkspace.evGu[ 480 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 132 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1584 ]), &(acadoWorkspace.evGu[ 528 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 132 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1728 ]), &(acadoWorkspace.evGu[ 576 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 156 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1872 ]), &(acadoWorkspace.evGu[ 624 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 156 ]), &(acadoWorkspace.sbar[ 168 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2016 ]), &(acadoWorkspace.evGu[ 672 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 168 ]), &(acadoWorkspace.sbar[ 180 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2160 ]), &(acadoWorkspace.evGu[ 720 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.sbar[ 192 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2304 ]), &(acadoWorkspace.evGu[ 768 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.sbar[ 192 ]), &(acadoWorkspace.sbar[ 204 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2448 ]), &(acadoWorkspace.evGu[ 816 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.sbar[ 204 ]), &(acadoWorkspace.sbar[ 216 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.evGu[ 864 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.sbar[ 228 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2736 ]), &(acadoWorkspace.evGu[ 912 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.sbar[ 228 ]), &(acadoWorkspace.sbar[ 240 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2880 ]), &(acadoWorkspace.evGu[ 960 ]), &(acadoWorkspace.x[ 80 ]), &(acadoWorkspace.sbar[ 240 ]), &(acadoWorkspace.sbar[ 252 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3024 ]), &(acadoWorkspace.evGu[ 1008 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.sbar[ 264 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3168 ]), &(acadoWorkspace.evGu[ 1056 ]), &(acadoWorkspace.x[ 88 ]), &(acadoWorkspace.sbar[ 264 ]), &(acadoWorkspace.sbar[ 276 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3312 ]), &(acadoWorkspace.evGu[ 1104 ]), &(acadoWorkspace.x[ 92 ]), &(acadoWorkspace.sbar[ 276 ]), &(acadoWorkspace.sbar[ 288 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3456 ]), &(acadoWorkspace.evGu[ 1152 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.sbar[ 288 ]), &(acadoWorkspace.sbar[ 300 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3600 ]), &(acadoWorkspace.evGu[ 1200 ]), &(acadoWorkspace.x[ 100 ]), &(acadoWorkspace.sbar[ 300 ]), &(acadoWorkspace.sbar[ 312 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3744 ]), &(acadoWorkspace.evGu[ 1248 ]), &(acadoWorkspace.x[ 104 ]), &(acadoWorkspace.sbar[ 312 ]), &(acadoWorkspace.sbar[ 324 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 3888 ]), &(acadoWorkspace.evGu[ 1296 ]), &(acadoWorkspace.x[ 108 ]), &(acadoWorkspace.sbar[ 324 ]), &(acadoWorkspace.sbar[ 336 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4032 ]), &(acadoWorkspace.evGu[ 1344 ]), &(acadoWorkspace.x[ 112 ]), &(acadoWorkspace.sbar[ 336 ]), &(acadoWorkspace.sbar[ 348 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4176 ]), &(acadoWorkspace.evGu[ 1392 ]), &(acadoWorkspace.x[ 116 ]), &(acadoWorkspace.sbar[ 348 ]), &(acadoWorkspace.sbar[ 360 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4320 ]), &(acadoWorkspace.evGu[ 1440 ]), &(acadoWorkspace.x[ 120 ]), &(acadoWorkspace.sbar[ 360 ]), &(acadoWorkspace.sbar[ 372 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4464 ]), &(acadoWorkspace.evGu[ 1488 ]), &(acadoWorkspace.x[ 124 ]), &(acadoWorkspace.sbar[ 372 ]), &(acadoWorkspace.sbar[ 384 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4608 ]), &(acadoWorkspace.evGu[ 1536 ]), &(acadoWorkspace.x[ 128 ]), &(acadoWorkspace.sbar[ 384 ]), &(acadoWorkspace.sbar[ 396 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4752 ]), &(acadoWorkspace.evGu[ 1584 ]), &(acadoWorkspace.x[ 132 ]), &(acadoWorkspace.sbar[ 396 ]), &(acadoWorkspace.sbar[ 408 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 4896 ]), &(acadoWorkspace.evGu[ 1632 ]), &(acadoWorkspace.x[ 136 ]), &(acadoWorkspace.sbar[ 408 ]), &(acadoWorkspace.sbar[ 420 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 5040 ]), &(acadoWorkspace.evGu[ 1680 ]), &(acadoWorkspace.x[ 140 ]), &(acadoWorkspace.sbar[ 420 ]), &(acadoWorkspace.sbar[ 432 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 5184 ]), &(acadoWorkspace.evGu[ 1728 ]), &(acadoWorkspace.x[ 144 ]), &(acadoWorkspace.sbar[ 432 ]), &(acadoWorkspace.sbar[ 444 ]) );
for (lRun1 = 0; lRun1 < 456; ++lRun1)
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
for (index = 0; index < 37; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 12];
acadoWorkspace.state[1] = acadoVariables.x[index * 12 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 12 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 12 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 12 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 12 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 12 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 12 + 7];
acadoWorkspace.state[8] = acadoVariables.x[index * 12 + 8];
acadoWorkspace.state[9] = acadoVariables.x[index * 12 + 9];
acadoWorkspace.state[10] = acadoVariables.x[index * 12 + 10];
acadoWorkspace.state[11] = acadoVariables.x[index * 12 + 11];
acadoWorkspace.state[204] = acadoVariables.u[index * 4];
acadoWorkspace.state[205] = acadoVariables.u[index * 4 + 1];
acadoWorkspace.state[206] = acadoVariables.u[index * 4 + 2];
acadoWorkspace.state[207] = acadoVariables.u[index * 4 + 3];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 12 + 12] = acadoWorkspace.state[0];
acadoVariables.x[index * 12 + 13] = acadoWorkspace.state[1];
acadoVariables.x[index * 12 + 14] = acadoWorkspace.state[2];
acadoVariables.x[index * 12 + 15] = acadoWorkspace.state[3];
acadoVariables.x[index * 12 + 16] = acadoWorkspace.state[4];
acadoVariables.x[index * 12 + 17] = acadoWorkspace.state[5];
acadoVariables.x[index * 12 + 18] = acadoWorkspace.state[6];
acadoVariables.x[index * 12 + 19] = acadoWorkspace.state[7];
acadoVariables.x[index * 12 + 20] = acadoWorkspace.state[8];
acadoVariables.x[index * 12 + 21] = acadoWorkspace.state[9];
acadoVariables.x[index * 12 + 22] = acadoWorkspace.state[10];
acadoVariables.x[index * 12 + 23] = acadoWorkspace.state[11];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 37; ++index)
{
acadoVariables.x[index * 12] = acadoVariables.x[index * 12 + 12];
acadoVariables.x[index * 12 + 1] = acadoVariables.x[index * 12 + 13];
acadoVariables.x[index * 12 + 2] = acadoVariables.x[index * 12 + 14];
acadoVariables.x[index * 12 + 3] = acadoVariables.x[index * 12 + 15];
acadoVariables.x[index * 12 + 4] = acadoVariables.x[index * 12 + 16];
acadoVariables.x[index * 12 + 5] = acadoVariables.x[index * 12 + 17];
acadoVariables.x[index * 12 + 6] = acadoVariables.x[index * 12 + 18];
acadoVariables.x[index * 12 + 7] = acadoVariables.x[index * 12 + 19];
acadoVariables.x[index * 12 + 8] = acadoVariables.x[index * 12 + 20];
acadoVariables.x[index * 12 + 9] = acadoVariables.x[index * 12 + 21];
acadoVariables.x[index * 12 + 10] = acadoVariables.x[index * 12 + 22];
acadoVariables.x[index * 12 + 11] = acadoVariables.x[index * 12 + 23];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[444] = xEnd[0];
acadoVariables.x[445] = xEnd[1];
acadoVariables.x[446] = xEnd[2];
acadoVariables.x[447] = xEnd[3];
acadoVariables.x[448] = xEnd[4];
acadoVariables.x[449] = xEnd[5];
acadoVariables.x[450] = xEnd[6];
acadoVariables.x[451] = xEnd[7];
acadoVariables.x[452] = xEnd[8];
acadoVariables.x[453] = xEnd[9];
acadoVariables.x[454] = xEnd[10];
acadoVariables.x[455] = xEnd[11];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[444];
acadoWorkspace.state[1] = acadoVariables.x[445];
acadoWorkspace.state[2] = acadoVariables.x[446];
acadoWorkspace.state[3] = acadoVariables.x[447];
acadoWorkspace.state[4] = acadoVariables.x[448];
acadoWorkspace.state[5] = acadoVariables.x[449];
acadoWorkspace.state[6] = acadoVariables.x[450];
acadoWorkspace.state[7] = acadoVariables.x[451];
acadoWorkspace.state[8] = acadoVariables.x[452];
acadoWorkspace.state[9] = acadoVariables.x[453];
acadoWorkspace.state[10] = acadoVariables.x[454];
acadoWorkspace.state[11] = acadoVariables.x[455];
if (uEnd != 0)
{
acadoWorkspace.state[204] = uEnd[0];
acadoWorkspace.state[205] = uEnd[1];
acadoWorkspace.state[206] = uEnd[2];
acadoWorkspace.state[207] = uEnd[3];
}
else
{
acadoWorkspace.state[204] = acadoVariables.u[144];
acadoWorkspace.state[205] = acadoVariables.u[145];
acadoWorkspace.state[206] = acadoVariables.u[146];
acadoWorkspace.state[207] = acadoVariables.u[147];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[444] = acadoWorkspace.state[0];
acadoVariables.x[445] = acadoWorkspace.state[1];
acadoVariables.x[446] = acadoWorkspace.state[2];
acadoVariables.x[447] = acadoWorkspace.state[3];
acadoVariables.x[448] = acadoWorkspace.state[4];
acadoVariables.x[449] = acadoWorkspace.state[5];
acadoVariables.x[450] = acadoWorkspace.state[6];
acadoVariables.x[451] = acadoWorkspace.state[7];
acadoVariables.x[452] = acadoWorkspace.state[8];
acadoVariables.x[453] = acadoWorkspace.state[9];
acadoVariables.x[454] = acadoWorkspace.state[10];
acadoVariables.x[455] = acadoWorkspace.state[11];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 36; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[144] = uEnd[0];
acadoVariables.u[145] = uEnd[1];
acadoVariables.u[146] = uEnd[2];
acadoVariables.u[147] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110] + acadoWorkspace.g[111]*acadoWorkspace.x[111] + acadoWorkspace.g[112]*acadoWorkspace.x[112] + acadoWorkspace.g[113]*acadoWorkspace.x[113] + acadoWorkspace.g[114]*acadoWorkspace.x[114] + acadoWorkspace.g[115]*acadoWorkspace.x[115] + acadoWorkspace.g[116]*acadoWorkspace.x[116] + acadoWorkspace.g[117]*acadoWorkspace.x[117] + acadoWorkspace.g[118]*acadoWorkspace.x[118] + acadoWorkspace.g[119]*acadoWorkspace.x[119] + acadoWorkspace.g[120]*acadoWorkspace.x[120] + acadoWorkspace.g[121]*acadoWorkspace.x[121] + acadoWorkspace.g[122]*acadoWorkspace.x[122] + acadoWorkspace.g[123]*acadoWorkspace.x[123] + acadoWorkspace.g[124]*acadoWorkspace.x[124] + acadoWorkspace.g[125]*acadoWorkspace.x[125] + acadoWorkspace.g[126]*acadoWorkspace.x[126] + acadoWorkspace.g[127]*acadoWorkspace.x[127] + acadoWorkspace.g[128]*acadoWorkspace.x[128] + acadoWorkspace.g[129]*acadoWorkspace.x[129] + acadoWorkspace.g[130]*acadoWorkspace.x[130] + acadoWorkspace.g[131]*acadoWorkspace.x[131] + acadoWorkspace.g[132]*acadoWorkspace.x[132] + acadoWorkspace.g[133]*acadoWorkspace.x[133] + acadoWorkspace.g[134]*acadoWorkspace.x[134] + acadoWorkspace.g[135]*acadoWorkspace.x[135] + acadoWorkspace.g[136]*acadoWorkspace.x[136] + acadoWorkspace.g[137]*acadoWorkspace.x[137] + acadoWorkspace.g[138]*acadoWorkspace.x[138] + acadoWorkspace.g[139]*acadoWorkspace.x[139] + acadoWorkspace.g[140]*acadoWorkspace.x[140] + acadoWorkspace.g[141]*acadoWorkspace.x[141] + acadoWorkspace.g[142]*acadoWorkspace.x[142] + acadoWorkspace.g[143]*acadoWorkspace.x[143] + acadoWorkspace.g[144]*acadoWorkspace.x[144] + acadoWorkspace.g[145]*acadoWorkspace.x[145] + acadoWorkspace.g[146]*acadoWorkspace.x[146] + acadoWorkspace.g[147]*acadoWorkspace.x[147];
kkt = fabs( kkt );
for (index = 0; index < 148; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 37; ++index)
{
prd = acadoWorkspace.y[index + 148];
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
/** Row vector of size: 4 */
real_t tmpDy[ 4 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 37; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 12];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 12 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 12 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 12 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 12 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 12 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 12 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 12 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 12 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 12 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[lRun1 * 12 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[lRun1 * 12 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[13] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.u[lRun1 * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 4] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 4];
acadoWorkspace.Dy[lRun1 * 4 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 4 + 1];
acadoWorkspace.Dy[lRun1 * 4 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 4 + 2];
acadoWorkspace.Dy[lRun1 * 4 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 4 + 3];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[444];
acadoWorkspace.objValueIn[1] = acadoVariables.x[445];
acadoWorkspace.objValueIn[2] = acadoVariables.x[446];
acadoWorkspace.objValueIn[3] = acadoVariables.x[447];
acadoWorkspace.objValueIn[4] = acadoVariables.x[448];
acadoWorkspace.objValueIn[5] = acadoVariables.x[449];
acadoWorkspace.objValueIn[6] = acadoVariables.x[450];
acadoWorkspace.objValueIn[7] = acadoVariables.x[451];
acadoWorkspace.objValueIn[8] = acadoVariables.x[452];
acadoWorkspace.objValueIn[9] = acadoVariables.x[453];
acadoWorkspace.objValueIn[10] = acadoVariables.x[454];
acadoWorkspace.objValueIn[11] = acadoVariables.x[455];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 37; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 4];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 4 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 4 + 2];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 4 + 3];
objVal += + acadoWorkspace.Dy[lRun1 * 4]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 4 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 4 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 4 + 3]*tmpDy[3];
}

tmpDyN[0] = + acadoWorkspace.DyN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1];
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)1.0000000000000000e+01;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

