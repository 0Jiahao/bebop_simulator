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
const real_t* u = in + 12;
/* Vector of auxiliary variables; number of elements: 3. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (tan(xd[8]));
a[1] = (cos(xd[7]));
a[2] = (tan(xd[7]));

/* Compute outputs: */
out[0] = (xd[0]+((real_t)(6.6666670143604279e-02)*xd[3]));
out[1] = (xd[1]+((real_t)(6.6666670143604279e-02)*xd[4]));
out[2] = (((xd[2]+((real_t)(1.0672486585754219e+00)*xd[5]))-((real_t)(1.0049332724894500e-01)*xd[6]))+((real_t)(1.9197303582300001e-03)*u[2]));
out[3] = (((real_t)(9.8780826220014994e-01)*xd[3])+(((a[0]/a[1])*(real_t)(9.8066501617431641e+00))*(real_t)(6.6666670143604279e-02)));
out[4] = (((real_t)(9.8670911430742803e-01)*xd[4])-((a[2]*(real_t)(9.8066501617431641e+00))*(real_t)(6.6666670143604279e-02)));
out[5] = ((((real_t)(8.1484431444008099e-01)*xd[5])-((real_t)(1.2754246439467801e-01)*xd[6]))+((real_t)(3.4798765237579999e-03)*u[2]));
out[6] = ((((real_t)(4.8726223064037300e-01)*xd[5])+((real_t)(6.3713514738349597e-01)*xd[6]))-((real_t)(3.5522579058221003e-02)*u[2]));
out[7] = (((real_t)(6.0657452197261597e-01)*xd[7])+((real_t)(3.6197489307367903e-01)*u[0]));
out[8] = (((real_t)(5.9102501204030700e-01)*xd[8])+((real_t)(3.7750910669390098e-01)*u[1]));
out[9] = ((xd[9]+((real_t)(5.8094630551531801e-01)*xd[10]))-((real_t)(2.3800609512830001e-03)*xd[11]));
out[10] = ((((real_t)(9.3646534939894399e-01)*xd[10])-((real_t)(4.6804990378125001e-02)*xd[11]))+((real_t)(5.1259007219671144e-04)*u[3]));
out[11] = ((((real_t)(2.3691436602558699e-01)*xd[10])+((real_t)(8.2032901524311996e-01)*xd[11]))-((real_t)(1.3647220727940000e-02)*u[3]));
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 7. */
real_t* a = acadoWorkspace.rhs_aux;

/* Compute intermediate quantities: */
a[0] = (tan(xd[8]));
a[1] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[7])));
a[2] = (cos(xd[7]));
a[3] = ((real_t)(1.0000000000000000e+00)/a[2]);
a[4] = (a[3]*a[3]);
a[5] = ((real_t)(1.0000000000000000e+00)/(pow((cos(xd[8])),2)));
a[6] = ((real_t)(1.0000000000000000e+00)/(pow((cos(xd[7])),2)));

/* Compute outputs: */
out[0] = (real_t)(1.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(6.6666670143604279e-02);
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
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(1.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(6.6666670143604279e-02);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(1.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(1.0672486585754219e+00);
out[38] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0049332724894500e-01));
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(1.9197303582300001e-03);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(9.8780826220014994e-01);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = ((((real_t)(0.0000000000000000e+00)-((a[0]*a[1])*a[4]))*(real_t)(9.8066501617431641e+00))*(real_t)(6.6666670143604279e-02));
out[56] = (((a[5]*a[3])*(real_t)(9.8066501617431641e+00))*(real_t)(6.6666670143604279e-02));
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(9.8670911430742803e-01);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = ((real_t)(0.0000000000000000e+00)-((a[6]*(real_t)(9.8066501617431641e+00))*(real_t)(6.6666670143604279e-02)));
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(8.1484431444008099e-01);
out[86] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.2754246439467801e-01));
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(3.4798765237579999e-03);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(4.8726223064037300e-01);
out[102] = (real_t)(6.3713514738349597e-01);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = ((real_t)(0.0000000000000000e+00)-(real_t)(3.5522579058221003e-02));
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(6.0657452197261597e-01);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(3.6197489307367903e-01);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(5.9102501204030700e-01);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(3.7750910669390098e-01);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(1.0000000000000000e+00);
out[154] = (real_t)(5.8094630551531801e-01);
out[155] = ((real_t)(0.0000000000000000e+00)-(real_t)(2.3800609512830001e-03));
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = (real_t)(0.0000000000000000e+00);
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = (real_t)(0.0000000000000000e+00);
out[165] = (real_t)(0.0000000000000000e+00);
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(0.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = (real_t)(0.0000000000000000e+00);
out[170] = (real_t)(9.3646534939894399e-01);
out[171] = ((real_t)(0.0000000000000000e+00)-(real_t)(4.6804990378125001e-02));
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(5.1259007219671144e-04);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = (real_t)(0.0000000000000000e+00);
out[178] = (real_t)(0.0000000000000000e+00);
out[179] = (real_t)(0.0000000000000000e+00);
out[180] = (real_t)(0.0000000000000000e+00);
out[181] = (real_t)(0.0000000000000000e+00);
out[182] = (real_t)(0.0000000000000000e+00);
out[183] = (real_t)(0.0000000000000000e+00);
out[184] = (real_t)(0.0000000000000000e+00);
out[185] = (real_t)(0.0000000000000000e+00);
out[186] = (real_t)(2.3691436602558699e-01);
out[187] = (real_t)(8.2032901524311996e-01);
out[188] = (real_t)(0.0000000000000000e+00);
out[189] = (real_t)(0.0000000000000000e+00);
out[190] = (real_t)(0.0000000000000000e+00);
out[191] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.3647220727940000e-02));
}



/* Fixed step size:0.0333333 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int tmp_index;
acadoWorkspace.rk_xxx[12] = rk_eta[204];
acadoWorkspace.rk_xxx[13] = rk_eta[205];
acadoWorkspace.rk_xxx[14] = rk_eta[206];
acadoWorkspace.rk_xxx[15] = rk_eta[207];
acadoWorkspace.rk_xxx[16] = rk_eta[208];
acadoWorkspace.rk_xxx[17] = rk_eta[209];
acadoWorkspace.rk_xxx[18] = rk_eta[210];
acadoWorkspace.rk_xxx[19] = rk_eta[211];
acadoWorkspace.rk_xxx[20] = rk_eta[212];
acadoWorkspace.rk_xxx[21] = rk_eta[213];
acadoWorkspace.rk_xxx[22] = rk_eta[214];
acadoWorkspace.rk_xxx[23] = rk_eta[215];
acadoWorkspace.rk_xxx[24] = rk_eta[216];
acadoWorkspace.rk_xxx[25] = rk_eta[217];
acadoWorkspace.rk_xxx[26] = rk_eta[218];
acadoWorkspace.rk_xxx[27] = rk_eta[219];
acadoWorkspace.rk_xxx[28] = rk_eta[220];
acadoWorkspace.rk_xxx[29] = rk_eta[221];
acadoWorkspace.rk_xxx[30] = rk_eta[222];
acadoWorkspace.rk_xxx[31] = rk_eta[223];
acadoWorkspace.rk_xxx[32] = rk_eta[224];
acadoWorkspace.rk_xxx[33] = rk_eta[225];
acadoWorkspace.rk_xxx[34] = rk_eta[226];
acadoWorkspace.rk_xxx[35] = rk_eta[227];
acadoWorkspace.rk_xxx[36] = rk_eta[228];
acadoWorkspace.rk_xxx[37] = rk_eta[229];
acadoWorkspace.rk_xxx[38] = rk_eta[230];
acadoWorkspace.rk_xxx[39] = rk_eta[231];
acadoWorkspace.rk_xxx[40] = rk_eta[232];
acadoWorkspace.rk_xxx[41] = rk_eta[233];
acadoWorkspace.rk_xxx[42] = rk_eta[234];
acadoWorkspace.rk_xxx[43] = rk_eta[235];
acadoWorkspace.rk_xxx[44] = rk_eta[236];
acadoWorkspace.rk_xxx[45] = rk_eta[237];
acadoWorkspace.rk_xxx[46] = rk_eta[238];
acadoWorkspace.rk_xxx[47] = rk_eta[239];
acadoWorkspace.rk_xxx[48] = rk_eta[240];
acadoWorkspace.rk_xxx[49] = rk_eta[241];
acadoWorkspace.rk_xxx[50] = rk_eta[242];
acadoWorkspace.rk_xxx[51] = rk_eta[243];
acadoWorkspace.rk_xxx[52] = rk_eta[244];
acadoWorkspace.rk_xxx[53] = rk_eta[245];
acadoWorkspace.rk_xxx[54] = rk_eta[246];


for (run = 0; run < 2; ++run)
{
acadoWorkspace.rk_xxx[0] = rk_eta[0];
acadoWorkspace.rk_xxx[1] = rk_eta[1];
acadoWorkspace.rk_xxx[2] = rk_eta[2];
acadoWorkspace.rk_xxx[3] = rk_eta[3];
acadoWorkspace.rk_xxx[4] = rk_eta[4];
acadoWorkspace.rk_xxx[5] = rk_eta[5];
acadoWorkspace.rk_xxx[6] = rk_eta[6];
acadoWorkspace.rk_xxx[7] = rk_eta[7];
acadoWorkspace.rk_xxx[8] = rk_eta[8];
acadoWorkspace.rk_xxx[9] = rk_eta[9];
acadoWorkspace.rk_xxx[10] = rk_eta[10];
acadoWorkspace.rk_xxx[11] = rk_eta[11];
if( run > 0 ) {
for (i = 0; i < 12; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 16] = rk_eta[i * 12 + 12];
acadoWorkspace.rk_diffsPrev2[i * 16 + 1] = rk_eta[i * 12 + 13];
acadoWorkspace.rk_diffsPrev2[i * 16 + 2] = rk_eta[i * 12 + 14];
acadoWorkspace.rk_diffsPrev2[i * 16 + 3] = rk_eta[i * 12 + 15];
acadoWorkspace.rk_diffsPrev2[i * 16 + 4] = rk_eta[i * 12 + 16];
acadoWorkspace.rk_diffsPrev2[i * 16 + 5] = rk_eta[i * 12 + 17];
acadoWorkspace.rk_diffsPrev2[i * 16 + 6] = rk_eta[i * 12 + 18];
acadoWorkspace.rk_diffsPrev2[i * 16 + 7] = rk_eta[i * 12 + 19];
acadoWorkspace.rk_diffsPrev2[i * 16 + 8] = rk_eta[i * 12 + 20];
acadoWorkspace.rk_diffsPrev2[i * 16 + 9] = rk_eta[i * 12 + 21];
acadoWorkspace.rk_diffsPrev2[i * 16 + 10] = rk_eta[i * 12 + 22];
acadoWorkspace.rk_diffsPrev2[i * 16 + 11] = rk_eta[i * 12 + 23];
acadoWorkspace.rk_diffsPrev2[i * 16 + 12] = rk_eta[i * 4 + 156];
acadoWorkspace.rk_diffsPrev2[i * 16 + 13] = rk_eta[i * 4 + 157];
acadoWorkspace.rk_diffsPrev2[i * 16 + 14] = rk_eta[i * 4 + 158];
acadoWorkspace.rk_diffsPrev2[i * 16 + 15] = rk_eta[i * 4 + 159];
}
}
acado_rhs( acadoWorkspace.rk_xxx, rk_eta );
acado_diffs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_diffsNew2 );
if( run == 0 ) {
for (i = 0; i < 12; ++i)
{
for (j = 0; j < 12; ++j)
{
tmp_index = (j) + (i * 12);
rk_eta[tmp_index + 12] = acadoWorkspace.rk_diffsNew2[(i * 16) + (j)];
}
for (j = 0; j < 4; ++j)
{
tmp_index = (j) + (i * 4);
rk_eta[tmp_index + 156] = acadoWorkspace.rk_diffsNew2[(i * 16) + (j + 12)];
}
}
}
else {
for (i = 0; i < 12; ++i)
{
for (j = 0; j < 12; ++j)
{
tmp_index = (j) + (i * 12);
rk_eta[tmp_index + 12] = + acadoWorkspace.rk_diffsNew2[i * 16]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index + 12] += + acadoWorkspace.rk_diffsNew2[i * 16 + 1]*acadoWorkspace.rk_diffsPrev2[j + 16];
rk_eta[tmp_index + 12] += + acadoWorkspace.rk_diffsNew2[i * 16 + 2]*acadoWorkspace.rk_diffsPrev2[j + 32];
rk_eta[tmp_index + 12] += + acadoWorkspace.rk_diffsNew2[i * 16 + 3]*acadoWorkspace.rk_diffsPrev2[j + 48];
rk_eta[tmp_index + 12] += + acadoWorkspace.rk_diffsNew2[i * 16 + 4]*acadoWorkspace.rk_diffsPrev2[j + 64];
rk_eta[tmp_index + 12] += + acadoWorkspace.rk_diffsNew2[i * 16 + 5]*acadoWorkspace.rk_diffsPrev2[j + 80];
rk_eta[tmp_index + 12] += + acadoWorkspace.rk_diffsNew2[i * 16 + 6]*acadoWorkspace.rk_diffsPrev2[j + 96];
rk_eta[tmp_index + 12] += + acadoWorkspace.rk_diffsNew2[i * 16 + 7]*acadoWorkspace.rk_diffsPrev2[j + 112];
rk_eta[tmp_index + 12] += + acadoWorkspace.rk_diffsNew2[i * 16 + 8]*acadoWorkspace.rk_diffsPrev2[j + 128];
rk_eta[tmp_index + 12] += + acadoWorkspace.rk_diffsNew2[i * 16 + 9]*acadoWorkspace.rk_diffsPrev2[j + 144];
rk_eta[tmp_index + 12] += + acadoWorkspace.rk_diffsNew2[i * 16 + 10]*acadoWorkspace.rk_diffsPrev2[j + 160];
rk_eta[tmp_index + 12] += + acadoWorkspace.rk_diffsNew2[i * 16 + 11]*acadoWorkspace.rk_diffsPrev2[j + 176];
}
for (j = 0; j < 4; ++j)
{
tmp_index = (j) + (i * 4);
rk_eta[tmp_index + 156] = acadoWorkspace.rk_diffsNew2[(i * 16) + (j + 12)];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16]*acadoWorkspace.rk_diffsPrev2[j + 12];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16 + 1]*acadoWorkspace.rk_diffsPrev2[j + 28];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16 + 2]*acadoWorkspace.rk_diffsPrev2[j + 44];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16 + 3]*acadoWorkspace.rk_diffsPrev2[j + 60];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16 + 4]*acadoWorkspace.rk_diffsPrev2[j + 76];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16 + 5]*acadoWorkspace.rk_diffsPrev2[j + 92];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16 + 6]*acadoWorkspace.rk_diffsPrev2[j + 108];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16 + 7]*acadoWorkspace.rk_diffsPrev2[j + 124];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16 + 8]*acadoWorkspace.rk_diffsPrev2[j + 140];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16 + 9]*acadoWorkspace.rk_diffsPrev2[j + 156];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16 + 10]*acadoWorkspace.rk_diffsPrev2[j + 172];
rk_eta[tmp_index + 156] += + acadoWorkspace.rk_diffsNew2[i * 16 + 11]*acadoWorkspace.rk_diffsPrev2[j + 188];
}
}
}
}
for (i = 0; i < 12; ++i)
{
}
return error;
}

