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
for (lRun1 = 0; lRun1 < 37; ++lRun1)
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

acadoWorkspace.state[9] = acadoVariables.mu[lRun1 * 9];
acadoWorkspace.state[10] = acadoVariables.mu[lRun1 * 9 + 1];
acadoWorkspace.state[11] = acadoVariables.mu[lRun1 * 9 + 2];
acadoWorkspace.state[12] = acadoVariables.mu[lRun1 * 9 + 3];
acadoWorkspace.state[13] = acadoVariables.mu[lRun1 * 9 + 4];
acadoWorkspace.state[14] = acadoVariables.mu[lRun1 * 9 + 5];
acadoWorkspace.state[15] = acadoVariables.mu[lRun1 * 9 + 6];
acadoWorkspace.state[16] = acadoVariables.mu[lRun1 * 9 + 7];
acadoWorkspace.state[17] = acadoVariables.mu[lRun1 * 9 + 8];
acadoWorkspace.state[204] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.state[205] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.state[206] = acadoVariables.u[lRun1 * 3 + 2];

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

acadoWorkspace.evGx[lRun1 * 81] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 81 + 1] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 81 + 2] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 81 + 3] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 81 + 4] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 81 + 5] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 81 + 6] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 81 + 7] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 81 + 8] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 81 + 9] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 81 + 10] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 81 + 11] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 81 + 12] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 81 + 13] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 81 + 14] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 81 + 15] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 81 + 16] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 81 + 17] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 81 + 18] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 81 + 19] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 81 + 20] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 81 + 21] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 81 + 22] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 81 + 23] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 81 + 24] = acadoWorkspace.state[42];
acadoWorkspace.evGx[lRun1 * 81 + 25] = acadoWorkspace.state[43];
acadoWorkspace.evGx[lRun1 * 81 + 26] = acadoWorkspace.state[44];
acadoWorkspace.evGx[lRun1 * 81 + 27] = acadoWorkspace.state[45];
acadoWorkspace.evGx[lRun1 * 81 + 28] = acadoWorkspace.state[46];
acadoWorkspace.evGx[lRun1 * 81 + 29] = acadoWorkspace.state[47];
acadoWorkspace.evGx[lRun1 * 81 + 30] = acadoWorkspace.state[48];
acadoWorkspace.evGx[lRun1 * 81 + 31] = acadoWorkspace.state[49];
acadoWorkspace.evGx[lRun1 * 81 + 32] = acadoWorkspace.state[50];
acadoWorkspace.evGx[lRun1 * 81 + 33] = acadoWorkspace.state[51];
acadoWorkspace.evGx[lRun1 * 81 + 34] = acadoWorkspace.state[52];
acadoWorkspace.evGx[lRun1 * 81 + 35] = acadoWorkspace.state[53];
acadoWorkspace.evGx[lRun1 * 81 + 36] = acadoWorkspace.state[54];
acadoWorkspace.evGx[lRun1 * 81 + 37] = acadoWorkspace.state[55];
acadoWorkspace.evGx[lRun1 * 81 + 38] = acadoWorkspace.state[56];
acadoWorkspace.evGx[lRun1 * 81 + 39] = acadoWorkspace.state[57];
acadoWorkspace.evGx[lRun1 * 81 + 40] = acadoWorkspace.state[58];
acadoWorkspace.evGx[lRun1 * 81 + 41] = acadoWorkspace.state[59];
acadoWorkspace.evGx[lRun1 * 81 + 42] = acadoWorkspace.state[60];
acadoWorkspace.evGx[lRun1 * 81 + 43] = acadoWorkspace.state[61];
acadoWorkspace.evGx[lRun1 * 81 + 44] = acadoWorkspace.state[62];
acadoWorkspace.evGx[lRun1 * 81 + 45] = acadoWorkspace.state[63];
acadoWorkspace.evGx[lRun1 * 81 + 46] = acadoWorkspace.state[64];
acadoWorkspace.evGx[lRun1 * 81 + 47] = acadoWorkspace.state[65];
acadoWorkspace.evGx[lRun1 * 81 + 48] = acadoWorkspace.state[66];
acadoWorkspace.evGx[lRun1 * 81 + 49] = acadoWorkspace.state[67];
acadoWorkspace.evGx[lRun1 * 81 + 50] = acadoWorkspace.state[68];
acadoWorkspace.evGx[lRun1 * 81 + 51] = acadoWorkspace.state[69];
acadoWorkspace.evGx[lRun1 * 81 + 52] = acadoWorkspace.state[70];
acadoWorkspace.evGx[lRun1 * 81 + 53] = acadoWorkspace.state[71];
acadoWorkspace.evGx[lRun1 * 81 + 54] = acadoWorkspace.state[72];
acadoWorkspace.evGx[lRun1 * 81 + 55] = acadoWorkspace.state[73];
acadoWorkspace.evGx[lRun1 * 81 + 56] = acadoWorkspace.state[74];
acadoWorkspace.evGx[lRun1 * 81 + 57] = acadoWorkspace.state[75];
acadoWorkspace.evGx[lRun1 * 81 + 58] = acadoWorkspace.state[76];
acadoWorkspace.evGx[lRun1 * 81 + 59] = acadoWorkspace.state[77];
acadoWorkspace.evGx[lRun1 * 81 + 60] = acadoWorkspace.state[78];
acadoWorkspace.evGx[lRun1 * 81 + 61] = acadoWorkspace.state[79];
acadoWorkspace.evGx[lRun1 * 81 + 62] = acadoWorkspace.state[80];
acadoWorkspace.evGx[lRun1 * 81 + 63] = acadoWorkspace.state[81];
acadoWorkspace.evGx[lRun1 * 81 + 64] = acadoWorkspace.state[82];
acadoWorkspace.evGx[lRun1 * 81 + 65] = acadoWorkspace.state[83];
acadoWorkspace.evGx[lRun1 * 81 + 66] = acadoWorkspace.state[84];
acadoWorkspace.evGx[lRun1 * 81 + 67] = acadoWorkspace.state[85];
acadoWorkspace.evGx[lRun1 * 81 + 68] = acadoWorkspace.state[86];
acadoWorkspace.evGx[lRun1 * 81 + 69] = acadoWorkspace.state[87];
acadoWorkspace.evGx[lRun1 * 81 + 70] = acadoWorkspace.state[88];
acadoWorkspace.evGx[lRun1 * 81 + 71] = acadoWorkspace.state[89];
acadoWorkspace.evGx[lRun1 * 81 + 72] = acadoWorkspace.state[90];
acadoWorkspace.evGx[lRun1 * 81 + 73] = acadoWorkspace.state[91];
acadoWorkspace.evGx[lRun1 * 81 + 74] = acadoWorkspace.state[92];
acadoWorkspace.evGx[lRun1 * 81 + 75] = acadoWorkspace.state[93];
acadoWorkspace.evGx[lRun1 * 81 + 76] = acadoWorkspace.state[94];
acadoWorkspace.evGx[lRun1 * 81 + 77] = acadoWorkspace.state[95];
acadoWorkspace.evGx[lRun1 * 81 + 78] = acadoWorkspace.state[96];
acadoWorkspace.evGx[lRun1 * 81 + 79] = acadoWorkspace.state[97];
acadoWorkspace.evGx[lRun1 * 81 + 80] = acadoWorkspace.state[98];

acadoWorkspace.evGu[lRun1 * 27] = acadoWorkspace.state[99];
acadoWorkspace.evGu[lRun1 * 27 + 1] = acadoWorkspace.state[100];
acadoWorkspace.evGu[lRun1 * 27 + 2] = acadoWorkspace.state[101];
acadoWorkspace.evGu[lRun1 * 27 + 3] = acadoWorkspace.state[102];
acadoWorkspace.evGu[lRun1 * 27 + 4] = acadoWorkspace.state[103];
acadoWorkspace.evGu[lRun1 * 27 + 5] = acadoWorkspace.state[104];
acadoWorkspace.evGu[lRun1 * 27 + 6] = acadoWorkspace.state[105];
acadoWorkspace.evGu[lRun1 * 27 + 7] = acadoWorkspace.state[106];
acadoWorkspace.evGu[lRun1 * 27 + 8] = acadoWorkspace.state[107];
acadoWorkspace.evGu[lRun1 * 27 + 9] = acadoWorkspace.state[108];
acadoWorkspace.evGu[lRun1 * 27 + 10] = acadoWorkspace.state[109];
acadoWorkspace.evGu[lRun1 * 27 + 11] = acadoWorkspace.state[110];
acadoWorkspace.evGu[lRun1 * 27 + 12] = acadoWorkspace.state[111];
acadoWorkspace.evGu[lRun1 * 27 + 13] = acadoWorkspace.state[112];
acadoWorkspace.evGu[lRun1 * 27 + 14] = acadoWorkspace.state[113];
acadoWorkspace.evGu[lRun1 * 27 + 15] = acadoWorkspace.state[114];
acadoWorkspace.evGu[lRun1 * 27 + 16] = acadoWorkspace.state[115];
acadoWorkspace.evGu[lRun1 * 27 + 17] = acadoWorkspace.state[116];
acadoWorkspace.evGu[lRun1 * 27 + 18] = acadoWorkspace.state[117];
acadoWorkspace.evGu[lRun1 * 27 + 19] = acadoWorkspace.state[118];
acadoWorkspace.evGu[lRun1 * 27 + 20] = acadoWorkspace.state[119];
acadoWorkspace.evGu[lRun1 * 27 + 21] = acadoWorkspace.state[120];
acadoWorkspace.evGu[lRun1 * 27 + 22] = acadoWorkspace.state[121];
acadoWorkspace.evGu[lRun1 * 27 + 23] = acadoWorkspace.state[122];
acadoWorkspace.evGu[lRun1 * 27 + 24] = acadoWorkspace.state[123];
acadoWorkspace.evGu[lRun1 * 27 + 25] = acadoWorkspace.state[124];
acadoWorkspace.evGu[lRun1 * 27 + 26] = acadoWorkspace.state[125];
acadoWorkspace.EH[lRun1 * 144] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[126];
acadoWorkspace.EH[lRun1 * 144 + 12] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[127];
acadoWorkspace.EH[lRun1 * 144 + 1] = acadoWorkspace.EH[lRun1 * 144 + 12];
acadoWorkspace.EH[lRun1 * 144 + 13] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[128];
acadoWorkspace.EH[lRun1 * 144 + 24] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[129];
acadoWorkspace.EH[lRun1 * 144 + 2] = acadoWorkspace.EH[lRun1 * 144 + 24];
acadoWorkspace.EH[lRun1 * 144 + 25] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[130];
acadoWorkspace.EH[lRun1 * 144 + 14] = acadoWorkspace.EH[lRun1 * 144 + 25];
acadoWorkspace.EH[lRun1 * 144 + 26] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[131];
acadoWorkspace.EH[lRun1 * 144 + 36] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[132];
acadoWorkspace.EH[lRun1 * 144 + 3] = acadoWorkspace.EH[lRun1 * 144 + 36];
acadoWorkspace.EH[lRun1 * 144 + 37] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[133];
acadoWorkspace.EH[lRun1 * 144 + 15] = acadoWorkspace.EH[lRun1 * 144 + 37];
acadoWorkspace.EH[lRun1 * 144 + 38] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[134];
acadoWorkspace.EH[lRun1 * 144 + 27] = acadoWorkspace.EH[lRun1 * 144 + 38];
acadoWorkspace.EH[lRun1 * 144 + 39] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[135];
acadoWorkspace.EH[lRun1 * 144 + 48] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[136];
acadoWorkspace.EH[lRun1 * 144 + 4] = acadoWorkspace.EH[lRun1 * 144 + 48];
acadoWorkspace.EH[lRun1 * 144 + 49] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[137];
acadoWorkspace.EH[lRun1 * 144 + 16] = acadoWorkspace.EH[lRun1 * 144 + 49];
acadoWorkspace.EH[lRun1 * 144 + 50] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[138];
acadoWorkspace.EH[lRun1 * 144 + 28] = acadoWorkspace.EH[lRun1 * 144 + 50];
acadoWorkspace.EH[lRun1 * 144 + 51] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[139];
acadoWorkspace.EH[lRun1 * 144 + 40] = acadoWorkspace.EH[lRun1 * 144 + 51];
acadoWorkspace.EH[lRun1 * 144 + 52] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[140];
acadoWorkspace.EH[lRun1 * 144 + 60] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[141];
acadoWorkspace.EH[lRun1 * 144 + 5] = acadoWorkspace.EH[lRun1 * 144 + 60];
acadoWorkspace.EH[lRun1 * 144 + 61] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[142];
acadoWorkspace.EH[lRun1 * 144 + 17] = acadoWorkspace.EH[lRun1 * 144 + 61];
acadoWorkspace.EH[lRun1 * 144 + 62] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[143];
acadoWorkspace.EH[lRun1 * 144 + 29] = acadoWorkspace.EH[lRun1 * 144 + 62];
acadoWorkspace.EH[lRun1 * 144 + 63] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[144];
acadoWorkspace.EH[lRun1 * 144 + 41] = acadoWorkspace.EH[lRun1 * 144 + 63];
acadoWorkspace.EH[lRun1 * 144 + 64] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[145];
acadoWorkspace.EH[lRun1 * 144 + 53] = acadoWorkspace.EH[lRun1 * 144 + 64];
acadoWorkspace.EH[lRun1 * 144 + 65] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[146];
acadoWorkspace.EH[lRun1 * 144 + 72] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[147];
acadoWorkspace.EH[lRun1 * 144 + 6] = acadoWorkspace.EH[lRun1 * 144 + 72];
acadoWorkspace.EH[lRun1 * 144 + 73] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[148];
acadoWorkspace.EH[lRun1 * 144 + 18] = acadoWorkspace.EH[lRun1 * 144 + 73];
acadoWorkspace.EH[lRun1 * 144 + 74] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[149];
acadoWorkspace.EH[lRun1 * 144 + 30] = acadoWorkspace.EH[lRun1 * 144 + 74];
acadoWorkspace.EH[lRun1 * 144 + 75] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[150];
acadoWorkspace.EH[lRun1 * 144 + 42] = acadoWorkspace.EH[lRun1 * 144 + 75];
acadoWorkspace.EH[lRun1 * 144 + 76] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[151];
acadoWorkspace.EH[lRun1 * 144 + 54] = acadoWorkspace.EH[lRun1 * 144 + 76];
acadoWorkspace.EH[lRun1 * 144 + 77] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[152];
acadoWorkspace.EH[lRun1 * 144 + 66] = acadoWorkspace.EH[lRun1 * 144 + 77];
acadoWorkspace.EH[lRun1 * 144 + 78] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[153];
acadoWorkspace.EH[lRun1 * 144 + 84] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[154];
acadoWorkspace.EH[lRun1 * 144 + 7] = acadoWorkspace.EH[lRun1 * 144 + 84];
acadoWorkspace.EH[lRun1 * 144 + 85] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[155];
acadoWorkspace.EH[lRun1 * 144 + 19] = acadoWorkspace.EH[lRun1 * 144 + 85];
acadoWorkspace.EH[lRun1 * 144 + 86] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[156];
acadoWorkspace.EH[lRun1 * 144 + 31] = acadoWorkspace.EH[lRun1 * 144 + 86];
acadoWorkspace.EH[lRun1 * 144 + 87] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[157];
acadoWorkspace.EH[lRun1 * 144 + 43] = acadoWorkspace.EH[lRun1 * 144 + 87];
acadoWorkspace.EH[lRun1 * 144 + 88] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[158];
acadoWorkspace.EH[lRun1 * 144 + 55] = acadoWorkspace.EH[lRun1 * 144 + 88];
acadoWorkspace.EH[lRun1 * 144 + 89] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[159];
acadoWorkspace.EH[lRun1 * 144 + 67] = acadoWorkspace.EH[lRun1 * 144 + 89];
acadoWorkspace.EH[lRun1 * 144 + 90] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[160];
acadoWorkspace.EH[lRun1 * 144 + 79] = acadoWorkspace.EH[lRun1 * 144 + 90];
acadoWorkspace.EH[lRun1 * 144 + 91] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[161];
acadoWorkspace.EH[lRun1 * 144 + 96] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[162];
acadoWorkspace.EH[lRun1 * 144 + 8] = acadoWorkspace.EH[lRun1 * 144 + 96];
acadoWorkspace.EH[lRun1 * 144 + 97] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[163];
acadoWorkspace.EH[lRun1 * 144 + 20] = acadoWorkspace.EH[lRun1 * 144 + 97];
acadoWorkspace.EH[lRun1 * 144 + 98] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[164];
acadoWorkspace.EH[lRun1 * 144 + 32] = acadoWorkspace.EH[lRun1 * 144 + 98];
acadoWorkspace.EH[lRun1 * 144 + 99] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[165];
acadoWorkspace.EH[lRun1 * 144 + 44] = acadoWorkspace.EH[lRun1 * 144 + 99];
acadoWorkspace.EH[lRun1 * 144 + 100] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[166];
acadoWorkspace.EH[lRun1 * 144 + 56] = acadoWorkspace.EH[lRun1 * 144 + 100];
acadoWorkspace.EH[lRun1 * 144 + 101] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[167];
acadoWorkspace.EH[lRun1 * 144 + 68] = acadoWorkspace.EH[lRun1 * 144 + 101];
acadoWorkspace.EH[lRun1 * 144 + 102] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[168];
acadoWorkspace.EH[lRun1 * 144 + 80] = acadoWorkspace.EH[lRun1 * 144 + 102];
acadoWorkspace.EH[lRun1 * 144 + 103] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[169];
acadoWorkspace.EH[lRun1 * 144 + 92] = acadoWorkspace.EH[lRun1 * 144 + 103];
acadoWorkspace.EH[lRun1 * 144 + 104] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[170];
acadoWorkspace.EH[lRun1 * 144 + 108] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[171];
acadoWorkspace.EH[lRun1 * 144 + 9] = acadoWorkspace.EH[lRun1 * 144 + 108];
acadoWorkspace.EH[lRun1 * 144 + 109] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[172];
acadoWorkspace.EH[lRun1 * 144 + 21] = acadoWorkspace.EH[lRun1 * 144 + 109];
acadoWorkspace.EH[lRun1 * 144 + 110] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[173];
acadoWorkspace.EH[lRun1 * 144 + 33] = acadoWorkspace.EH[lRun1 * 144 + 110];
acadoWorkspace.EH[lRun1 * 144 + 111] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[174];
acadoWorkspace.EH[lRun1 * 144 + 45] = acadoWorkspace.EH[lRun1 * 144 + 111];
acadoWorkspace.EH[lRun1 * 144 + 112] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[175];
acadoWorkspace.EH[lRun1 * 144 + 57] = acadoWorkspace.EH[lRun1 * 144 + 112];
acadoWorkspace.EH[lRun1 * 144 + 113] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[176];
acadoWorkspace.EH[lRun1 * 144 + 69] = acadoWorkspace.EH[lRun1 * 144 + 113];
acadoWorkspace.EH[lRun1 * 144 + 114] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[177];
acadoWorkspace.EH[lRun1 * 144 + 81] = acadoWorkspace.EH[lRun1 * 144 + 114];
acadoWorkspace.EH[lRun1 * 144 + 115] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[178];
acadoWorkspace.EH[lRun1 * 144 + 93] = acadoWorkspace.EH[lRun1 * 144 + 115];
acadoWorkspace.EH[lRun1 * 144 + 116] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[179];
acadoWorkspace.EH[lRun1 * 144 + 105] = acadoWorkspace.EH[lRun1 * 144 + 116];
acadoWorkspace.EH[lRun1 * 144 + 117] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[180];
acadoWorkspace.EH[lRun1 * 144 + 120] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[181];
acadoWorkspace.EH[lRun1 * 144 + 10] = acadoWorkspace.EH[lRun1 * 144 + 120];
acadoWorkspace.EH[lRun1 * 144 + 121] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[182];
acadoWorkspace.EH[lRun1 * 144 + 22] = acadoWorkspace.EH[lRun1 * 144 + 121];
acadoWorkspace.EH[lRun1 * 144 + 122] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[183];
acadoWorkspace.EH[lRun1 * 144 + 34] = acadoWorkspace.EH[lRun1 * 144 + 122];
acadoWorkspace.EH[lRun1 * 144 + 123] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[184];
acadoWorkspace.EH[lRun1 * 144 + 46] = acadoWorkspace.EH[lRun1 * 144 + 123];
acadoWorkspace.EH[lRun1 * 144 + 124] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[185];
acadoWorkspace.EH[lRun1 * 144 + 58] = acadoWorkspace.EH[lRun1 * 144 + 124];
acadoWorkspace.EH[lRun1 * 144 + 125] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[186];
acadoWorkspace.EH[lRun1 * 144 + 70] = acadoWorkspace.EH[lRun1 * 144 + 125];
acadoWorkspace.EH[lRun1 * 144 + 126] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[187];
acadoWorkspace.EH[lRun1 * 144 + 82] = acadoWorkspace.EH[lRun1 * 144 + 126];
acadoWorkspace.EH[lRun1 * 144 + 127] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[188];
acadoWorkspace.EH[lRun1 * 144 + 94] = acadoWorkspace.EH[lRun1 * 144 + 127];
acadoWorkspace.EH[lRun1 * 144 + 128] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[189];
acadoWorkspace.EH[lRun1 * 144 + 106] = acadoWorkspace.EH[lRun1 * 144 + 128];
acadoWorkspace.EH[lRun1 * 144 + 129] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[190];
acadoWorkspace.EH[lRun1 * 144 + 118] = acadoWorkspace.EH[lRun1 * 144 + 129];
acadoWorkspace.EH[lRun1 * 144 + 130] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[191];
acadoWorkspace.EH[lRun1 * 144 + 132] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[192];
acadoWorkspace.EH[lRun1 * 144 + 11] = acadoWorkspace.EH[lRun1 * 144 + 132];
acadoWorkspace.EH[lRun1 * 144 + 133] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[193];
acadoWorkspace.EH[lRun1 * 144 + 23] = acadoWorkspace.EH[lRun1 * 144 + 133];
acadoWorkspace.EH[lRun1 * 144 + 134] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[194];
acadoWorkspace.EH[lRun1 * 144 + 35] = acadoWorkspace.EH[lRun1 * 144 + 134];
acadoWorkspace.EH[lRun1 * 144 + 135] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[195];
acadoWorkspace.EH[lRun1 * 144 + 47] = acadoWorkspace.EH[lRun1 * 144 + 135];
acadoWorkspace.EH[lRun1 * 144 + 136] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[196];
acadoWorkspace.EH[lRun1 * 144 + 59] = acadoWorkspace.EH[lRun1 * 144 + 136];
acadoWorkspace.EH[lRun1 * 144 + 137] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[197];
acadoWorkspace.EH[lRun1 * 144 + 71] = acadoWorkspace.EH[lRun1 * 144 + 137];
acadoWorkspace.EH[lRun1 * 144 + 138] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[198];
acadoWorkspace.EH[lRun1 * 144 + 83] = acadoWorkspace.EH[lRun1 * 144 + 138];
acadoWorkspace.EH[lRun1 * 144 + 139] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[199];
acadoWorkspace.EH[lRun1 * 144 + 95] = acadoWorkspace.EH[lRun1 * 144 + 139];
acadoWorkspace.EH[lRun1 * 144 + 140] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[200];
acadoWorkspace.EH[lRun1 * 144 + 107] = acadoWorkspace.EH[lRun1 * 144 + 140];
acadoWorkspace.EH[lRun1 * 144 + 141] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[201];
acadoWorkspace.EH[lRun1 * 144 + 119] = acadoWorkspace.EH[lRun1 * 144 + 141];
acadoWorkspace.EH[lRun1 * 144 + 142] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[202];
acadoWorkspace.EH[lRun1 * 144 + 131] = acadoWorkspace.EH[lRun1 * 144 + 142];
acadoWorkspace.EH[lRun1 * 144 + 143] = + (real_t)-1.0000000000000000e+00*acadoWorkspace.state[203];
}
return ret;
}

void acado_evaluateLagrange(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 90. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (xd[0]+xd[0]);
a[1] = (xd[1]+xd[1]);
a[2] = (xd[2]+xd[2]);
a[3] = (xd[3]+xd[3]);
a[4] = (xd[4]+xd[4]);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (real_t)(0.0000000000000000e+00);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(2.0000000000000000e+00);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(2.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (real_t)(0.0000000000000000e+00);
a[27] = (real_t)(0.0000000000000000e+00);
a[28] = (real_t)(0.0000000000000000e+00);
a[29] = (real_t)(2.0000000000000000e+00);
a[30] = (real_t)(0.0000000000000000e+00);
a[31] = (real_t)(0.0000000000000000e+00);
a[32] = (real_t)(0.0000000000000000e+00);
a[33] = (real_t)(0.0000000000000000e+00);
a[34] = (real_t)(0.0000000000000000e+00);
a[35] = (real_t)(0.0000000000000000e+00);
a[36] = (real_t)(2.0000000000000000e+00);
a[37] = (real_t)(0.0000000000000000e+00);
a[38] = (real_t)(0.0000000000000000e+00);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (real_t)(0.0000000000000000e+00);
a[42] = (real_t)(2.0000000000000000e+00);
a[43] = (real_t)(0.0000000000000000e+00);
a[44] = (real_t)(0.0000000000000000e+00);
a[45] = (real_t)(0.0000000000000000e+00);
a[46] = (real_t)(0.0000000000000000e+00);
a[47] = (real_t)(0.0000000000000000e+00);
a[48] = (real_t)(0.0000000000000000e+00);
a[49] = (real_t)(0.0000000000000000e+00);
a[50] = (real_t)(0.0000000000000000e+00);
a[51] = (real_t)(0.0000000000000000e+00);
a[52] = (real_t)(0.0000000000000000e+00);
a[53] = (real_t)(0.0000000000000000e+00);
a[54] = (real_t)(0.0000000000000000e+00);
a[55] = (real_t)(0.0000000000000000e+00);
a[56] = (real_t)(0.0000000000000000e+00);
a[57] = (real_t)(0.0000000000000000e+00);
a[58] = (real_t)(0.0000000000000000e+00);
a[59] = (real_t)(0.0000000000000000e+00);
a[60] = (real_t)(0.0000000000000000e+00);
a[61] = (real_t)(0.0000000000000000e+00);
a[62] = (real_t)(0.0000000000000000e+00);
a[63] = (real_t)(0.0000000000000000e+00);
a[64] = (real_t)(0.0000000000000000e+00);
a[65] = (real_t)(0.0000000000000000e+00);
a[66] = (real_t)(0.0000000000000000e+00);
a[67] = (real_t)(0.0000000000000000e+00);
a[68] = (real_t)(0.0000000000000000e+00);
a[69] = (real_t)(0.0000000000000000e+00);
a[70] = (real_t)(0.0000000000000000e+00);
a[71] = (real_t)(0.0000000000000000e+00);
a[72] = (real_t)(0.0000000000000000e+00);
a[73] = (real_t)(0.0000000000000000e+00);
a[74] = (real_t)(0.0000000000000000e+00);
a[75] = (real_t)(0.0000000000000000e+00);
a[76] = (real_t)(0.0000000000000000e+00);
a[77] = (real_t)(0.0000000000000000e+00);
a[78] = (real_t)(0.0000000000000000e+00);
a[79] = (real_t)(0.0000000000000000e+00);
a[80] = (real_t)(0.0000000000000000e+00);
a[81] = (real_t)(0.0000000000000000e+00);
a[82] = (real_t)(0.0000000000000000e+00);
a[83] = (real_t)(0.0000000000000000e+00);
a[84] = (real_t)(0.0000000000000000e+00);
a[85] = (real_t)(0.0000000000000000e+00);
a[86] = (real_t)(0.0000000000000000e+00);
a[87] = (real_t)(0.0000000000000000e+00);
a[88] = (real_t)(0.0000000000000000e+00);
a[89] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = (((((xd[0]*xd[0])+(xd[1]*xd[1]))+(xd[2]*xd[2]))+(xd[3]*xd[3]))+(xd[4]*xd[4]));
out[1] = a[0];
out[2] = a[1];
out[3] = a[2];
out[4] = a[3];
out[5] = a[4];
out[6] = a[5];
out[7] = a[6];
out[8] = a[7];
out[9] = a[8];
out[10] = a[9];
out[11] = a[10];
out[12] = a[11];
out[13] = a[12];
out[14] = a[13];
out[15] = a[14];
out[16] = a[15];
out[17] = a[16];
out[18] = a[17];
out[19] = a[18];
out[20] = a[19];
out[21] = a[20];
out[22] = a[13];
out[23] = a[21];
out[24] = a[22];
out[25] = a[23];
out[26] = a[24];
out[27] = a[25];
out[28] = a[26];
out[29] = a[27];
out[30] = a[28];
out[31] = a[14];
out[32] = a[22];
out[33] = a[29];
out[34] = a[30];
out[35] = a[31];
out[36] = a[32];
out[37] = a[33];
out[38] = a[34];
out[39] = a[35];
out[40] = a[15];
out[41] = a[23];
out[42] = a[30];
out[43] = a[36];
out[44] = a[37];
out[45] = a[38];
out[46] = a[39];
out[47] = a[40];
out[48] = a[41];
out[49] = a[16];
out[50] = a[24];
out[51] = a[31];
out[52] = a[37];
out[53] = a[42];
out[54] = a[43];
out[55] = a[44];
out[56] = a[45];
out[57] = a[46];
out[58] = a[17];
out[59] = a[25];
out[60] = a[32];
out[61] = a[38];
out[62] = a[43];
out[63] = a[47];
out[64] = a[48];
out[65] = a[49];
out[66] = a[50];
out[67] = a[18];
out[68] = a[26];
out[69] = a[33];
out[70] = a[39];
out[71] = a[44];
out[72] = a[48];
out[73] = a[51];
out[74] = a[52];
out[75] = a[53];
out[76] = a[19];
out[77] = a[27];
out[78] = a[34];
out[79] = a[40];
out[80] = a[45];
out[81] = a[49];
out[82] = a[52];
out[83] = a[54];
out[84] = a[55];
out[85] = a[20];
out[86] = a[28];
out[87] = a[35];
out[88] = a[41];
out[89] = a[46];
out[90] = a[50];
out[91] = a[53];
out[92] = a[55];
out[93] = a[56];
out[94] = a[57];
out[95] = a[58];
out[96] = a[59];
out[97] = a[60];
out[98] = a[61];
out[99] = a[62];
out[100] = a[63];
out[101] = a[64];
out[102] = a[65];
out[103] = a[66];
out[104] = a[67];
out[105] = a[68];
out[106] = a[69];
out[107] = a[70];
out[108] = a[71];
out[109] = a[72];
out[110] = a[73];
out[111] = a[74];
out[112] = a[75];
out[113] = a[76];
out[114] = a[77];
out[115] = a[78];
out[116] = a[79];
out[117] = a[80];
out[118] = a[81];
out[119] = a[82];
out[120] = a[83];
out[121] = a[84];
out[122] = a[85];
out[123] = a[86];
out[124] = a[85];
out[125] = a[87];
out[126] = a[88];
out[127] = a[86];
out[128] = a[88];
out[129] = a[89];
}

void acado_addObjTerm( real_t* const tmpFxx, real_t* const tmpFxu, real_t* const tmpFuu, real_t* const tmpEH )
{
tmpEH[0] += tmpFxx[0];
tmpEH[1] += tmpFxx[1];
tmpEH[2] += tmpFxx[2];
tmpEH[3] += tmpFxx[3];
tmpEH[4] += tmpFxx[4];
tmpEH[5] += tmpFxx[5];
tmpEH[6] += tmpFxx[6];
tmpEH[7] += tmpFxx[7];
tmpEH[8] += tmpFxx[8];
tmpEH[12] += tmpFxx[9];
tmpEH[13] += tmpFxx[10];
tmpEH[14] += tmpFxx[11];
tmpEH[15] += tmpFxx[12];
tmpEH[16] += tmpFxx[13];
tmpEH[17] += tmpFxx[14];
tmpEH[18] += tmpFxx[15];
tmpEH[19] += tmpFxx[16];
tmpEH[20] += tmpFxx[17];
tmpEH[24] += tmpFxx[18];
tmpEH[25] += tmpFxx[19];
tmpEH[26] += tmpFxx[20];
tmpEH[27] += tmpFxx[21];
tmpEH[28] += tmpFxx[22];
tmpEH[29] += tmpFxx[23];
tmpEH[30] += tmpFxx[24];
tmpEH[31] += tmpFxx[25];
tmpEH[32] += tmpFxx[26];
tmpEH[36] += tmpFxx[27];
tmpEH[37] += tmpFxx[28];
tmpEH[38] += tmpFxx[29];
tmpEH[39] += tmpFxx[30];
tmpEH[40] += tmpFxx[31];
tmpEH[41] += tmpFxx[32];
tmpEH[42] += tmpFxx[33];
tmpEH[43] += tmpFxx[34];
tmpEH[44] += tmpFxx[35];
tmpEH[48] += tmpFxx[36];
tmpEH[49] += tmpFxx[37];
tmpEH[50] += tmpFxx[38];
tmpEH[51] += tmpFxx[39];
tmpEH[52] += tmpFxx[40];
tmpEH[53] += tmpFxx[41];
tmpEH[54] += tmpFxx[42];
tmpEH[55] += tmpFxx[43];
tmpEH[56] += tmpFxx[44];
tmpEH[60] += tmpFxx[45];
tmpEH[61] += tmpFxx[46];
tmpEH[62] += tmpFxx[47];
tmpEH[63] += tmpFxx[48];
tmpEH[64] += tmpFxx[49];
tmpEH[65] += tmpFxx[50];
tmpEH[66] += tmpFxx[51];
tmpEH[67] += tmpFxx[52];
tmpEH[68] += tmpFxx[53];
tmpEH[72] += tmpFxx[54];
tmpEH[73] += tmpFxx[55];
tmpEH[74] += tmpFxx[56];
tmpEH[75] += tmpFxx[57];
tmpEH[76] += tmpFxx[58];
tmpEH[77] += tmpFxx[59];
tmpEH[78] += tmpFxx[60];
tmpEH[79] += tmpFxx[61];
tmpEH[80] += tmpFxx[62];
tmpEH[84] += tmpFxx[63];
tmpEH[85] += tmpFxx[64];
tmpEH[86] += tmpFxx[65];
tmpEH[87] += tmpFxx[66];
tmpEH[88] += tmpFxx[67];
tmpEH[89] += tmpFxx[68];
tmpEH[90] += tmpFxx[69];
tmpEH[91] += tmpFxx[70];
tmpEH[92] += tmpFxx[71];
tmpEH[96] += tmpFxx[72];
tmpEH[97] += tmpFxx[73];
tmpEH[98] += tmpFxx[74];
tmpEH[99] += tmpFxx[75];
tmpEH[100] += tmpFxx[76];
tmpEH[101] += tmpFxx[77];
tmpEH[102] += tmpFxx[78];
tmpEH[103] += tmpFxx[79];
tmpEH[104] += tmpFxx[80];
tmpEH[9] += tmpFxu[0];
tmpEH[10] += tmpFxu[1];
tmpEH[11] += tmpFxu[2];
tmpEH[21] += tmpFxu[3];
tmpEH[22] += tmpFxu[4];
tmpEH[23] += tmpFxu[5];
tmpEH[33] += tmpFxu[6];
tmpEH[34] += tmpFxu[7];
tmpEH[35] += tmpFxu[8];
tmpEH[45] += tmpFxu[9];
tmpEH[46] += tmpFxu[10];
tmpEH[47] += tmpFxu[11];
tmpEH[57] += tmpFxu[12];
tmpEH[58] += tmpFxu[13];
tmpEH[59] += tmpFxu[14];
tmpEH[69] += tmpFxu[15];
tmpEH[70] += tmpFxu[16];
tmpEH[71] += tmpFxu[17];
tmpEH[81] += tmpFxu[18];
tmpEH[82] += tmpFxu[19];
tmpEH[83] += tmpFxu[20];
tmpEH[93] += tmpFxu[21];
tmpEH[94] += tmpFxu[22];
tmpEH[95] += tmpFxu[23];
tmpEH[105] += tmpFxu[24];
tmpEH[106] += tmpFxu[25];
tmpEH[107] += tmpFxu[26];
tmpEH[108] += tmpFxu[0];
tmpEH[109] += tmpFxu[3];
tmpEH[110] += tmpFxu[6];
tmpEH[111] += tmpFxu[9];
tmpEH[112] += tmpFxu[12];
tmpEH[113] += tmpFxu[15];
tmpEH[114] += tmpFxu[18];
tmpEH[115] += tmpFxu[21];
tmpEH[116] += tmpFxu[24];
tmpEH[120] += tmpFxu[1];
tmpEH[121] += tmpFxu[4];
tmpEH[122] += tmpFxu[7];
tmpEH[123] += tmpFxu[10];
tmpEH[124] += tmpFxu[13];
tmpEH[125] += tmpFxu[16];
tmpEH[126] += tmpFxu[19];
tmpEH[127] += tmpFxu[22];
tmpEH[128] += tmpFxu[25];
tmpEH[132] += tmpFxu[2];
tmpEH[133] += tmpFxu[5];
tmpEH[134] += tmpFxu[8];
tmpEH[135] += tmpFxu[11];
tmpEH[136] += tmpFxu[14];
tmpEH[137] += tmpFxu[17];
tmpEH[138] += tmpFxu[20];
tmpEH[139] += tmpFxu[23];
tmpEH[140] += tmpFxu[26];
tmpEH[117] += tmpFuu[0];
tmpEH[118] += tmpFuu[1];
tmpEH[119] += tmpFuu[2];
tmpEH[129] += tmpFuu[3];
tmpEH[130] += tmpFuu[4];
tmpEH[131] += tmpFuu[5];
tmpEH[141] += tmpFuu[6];
tmpEH[142] += tmpFuu[7];
tmpEH[143] += tmpFuu[8];
}

void acado_addObjLinearTerm( real_t* const tmpDx, real_t* const tmpDu, real_t* const tmpDF )
{
tmpDx[0] = tmpDF[0];
tmpDx[1] = tmpDF[1];
tmpDx[2] = tmpDF[2];
tmpDx[3] = tmpDF[3];
tmpDx[4] = tmpDF[4];
tmpDx[5] = tmpDF[5];
tmpDx[6] = tmpDF[6];
tmpDx[7] = tmpDF[7];
tmpDx[8] = tmpDF[8];
tmpDu[0] = tmpDF[9];
tmpDu[1] = tmpDF[10];
tmpDu[2] = tmpDF[11];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 37; ++runObj)
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

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acado_addObjTerm( &(acadoWorkspace.objValueOut[ 13 ]), &(acadoWorkspace.objValueOut[ 94 ]), &(acadoWorkspace.objValueOut[ 121 ]), &(acadoWorkspace.EH[ runObj * 144 ]) );
acado_addObjLinearTerm( &(acadoWorkspace.QDy[ runObj * 9 ]), &(acadoWorkspace.g[ runObj * 3 ]), &(acadoWorkspace.objValueOut[ 1 ]) );

}
acadoWorkspace.QDy[333] = 0.0000000000000000e+00;
acadoWorkspace.QDy[334] = 0.0000000000000000e+00;
acadoWorkspace.QDy[335] = 0.0000000000000000e+00;
acadoWorkspace.QDy[336] = 0.0000000000000000e+00;
acadoWorkspace.QDy[337] = 0.0000000000000000e+00;
acadoWorkspace.QDy[338] = 0.0000000000000000e+00;
acadoWorkspace.QDy[339] = 0.0000000000000000e+00;
acadoWorkspace.QDy[340] = 0.0000000000000000e+00;
acadoWorkspace.QDy[341] = 0.0000000000000000e+00;
}

void acado_regularizeHessian(  )
{
int lRun1;
for (lRun1 = 0; lRun1 < 37; ++lRun1)
{
acado_regularize( &(acadoWorkspace.EH[ lRun1 * 144 ]) );
acadoWorkspace.Q1[lRun1 * 81] = acadoWorkspace.EH[lRun1 * 144];
acadoWorkspace.Q1[lRun1 * 81 + 1] = acadoWorkspace.EH[lRun1 * 144 + 1];
acadoWorkspace.Q1[lRun1 * 81 + 2] = acadoWorkspace.EH[lRun1 * 144 + 2];
acadoWorkspace.Q1[lRun1 * 81 + 3] = acadoWorkspace.EH[lRun1 * 144 + 3];
acadoWorkspace.Q1[lRun1 * 81 + 4] = acadoWorkspace.EH[lRun1 * 144 + 4];
acadoWorkspace.Q1[lRun1 * 81 + 5] = acadoWorkspace.EH[lRun1 * 144 + 5];
acadoWorkspace.Q1[lRun1 * 81 + 6] = acadoWorkspace.EH[lRun1 * 144 + 6];
acadoWorkspace.Q1[lRun1 * 81 + 7] = acadoWorkspace.EH[lRun1 * 144 + 7];
acadoWorkspace.Q1[lRun1 * 81 + 8] = acadoWorkspace.EH[lRun1 * 144 + 8];
acadoWorkspace.Q1[lRun1 * 81 + 9] = acadoWorkspace.EH[lRun1 * 144 + 12];
acadoWorkspace.Q1[lRun1 * 81 + 10] = acadoWorkspace.EH[lRun1 * 144 + 13];
acadoWorkspace.Q1[lRun1 * 81 + 11] = acadoWorkspace.EH[lRun1 * 144 + 14];
acadoWorkspace.Q1[lRun1 * 81 + 12] = acadoWorkspace.EH[lRun1 * 144 + 15];
acadoWorkspace.Q1[lRun1 * 81 + 13] = acadoWorkspace.EH[lRun1 * 144 + 16];
acadoWorkspace.Q1[lRun1 * 81 + 14] = acadoWorkspace.EH[lRun1 * 144 + 17];
acadoWorkspace.Q1[lRun1 * 81 + 15] = acadoWorkspace.EH[lRun1 * 144 + 18];
acadoWorkspace.Q1[lRun1 * 81 + 16] = acadoWorkspace.EH[lRun1 * 144 + 19];
acadoWorkspace.Q1[lRun1 * 81 + 17] = acadoWorkspace.EH[lRun1 * 144 + 20];
acadoWorkspace.Q1[lRun1 * 81 + 18] = acadoWorkspace.EH[lRun1 * 144 + 24];
acadoWorkspace.Q1[lRun1 * 81 + 19] = acadoWorkspace.EH[lRun1 * 144 + 25];
acadoWorkspace.Q1[lRun1 * 81 + 20] = acadoWorkspace.EH[lRun1 * 144 + 26];
acadoWorkspace.Q1[lRun1 * 81 + 21] = acadoWorkspace.EH[lRun1 * 144 + 27];
acadoWorkspace.Q1[lRun1 * 81 + 22] = acadoWorkspace.EH[lRun1 * 144 + 28];
acadoWorkspace.Q1[lRun1 * 81 + 23] = acadoWorkspace.EH[lRun1 * 144 + 29];
acadoWorkspace.Q1[lRun1 * 81 + 24] = acadoWorkspace.EH[lRun1 * 144 + 30];
acadoWorkspace.Q1[lRun1 * 81 + 25] = acadoWorkspace.EH[lRun1 * 144 + 31];
acadoWorkspace.Q1[lRun1 * 81 + 26] = acadoWorkspace.EH[lRun1 * 144 + 32];
acadoWorkspace.Q1[lRun1 * 81 + 27] = acadoWorkspace.EH[lRun1 * 144 + 36];
acadoWorkspace.Q1[lRun1 * 81 + 28] = acadoWorkspace.EH[lRun1 * 144 + 37];
acadoWorkspace.Q1[lRun1 * 81 + 29] = acadoWorkspace.EH[lRun1 * 144 + 38];
acadoWorkspace.Q1[lRun1 * 81 + 30] = acadoWorkspace.EH[lRun1 * 144 + 39];
acadoWorkspace.Q1[lRun1 * 81 + 31] = acadoWorkspace.EH[lRun1 * 144 + 40];
acadoWorkspace.Q1[lRun1 * 81 + 32] = acadoWorkspace.EH[lRun1 * 144 + 41];
acadoWorkspace.Q1[lRun1 * 81 + 33] = acadoWorkspace.EH[lRun1 * 144 + 42];
acadoWorkspace.Q1[lRun1 * 81 + 34] = acadoWorkspace.EH[lRun1 * 144 + 43];
acadoWorkspace.Q1[lRun1 * 81 + 35] = acadoWorkspace.EH[lRun1 * 144 + 44];
acadoWorkspace.Q1[lRun1 * 81 + 36] = acadoWorkspace.EH[lRun1 * 144 + 48];
acadoWorkspace.Q1[lRun1 * 81 + 37] = acadoWorkspace.EH[lRun1 * 144 + 49];
acadoWorkspace.Q1[lRun1 * 81 + 38] = acadoWorkspace.EH[lRun1 * 144 + 50];
acadoWorkspace.Q1[lRun1 * 81 + 39] = acadoWorkspace.EH[lRun1 * 144 + 51];
acadoWorkspace.Q1[lRun1 * 81 + 40] = acadoWorkspace.EH[lRun1 * 144 + 52];
acadoWorkspace.Q1[lRun1 * 81 + 41] = acadoWorkspace.EH[lRun1 * 144 + 53];
acadoWorkspace.Q1[lRun1 * 81 + 42] = acadoWorkspace.EH[lRun1 * 144 + 54];
acadoWorkspace.Q1[lRun1 * 81 + 43] = acadoWorkspace.EH[lRun1 * 144 + 55];
acadoWorkspace.Q1[lRun1 * 81 + 44] = acadoWorkspace.EH[lRun1 * 144 + 56];
acadoWorkspace.Q1[lRun1 * 81 + 45] = acadoWorkspace.EH[lRun1 * 144 + 60];
acadoWorkspace.Q1[lRun1 * 81 + 46] = acadoWorkspace.EH[lRun1 * 144 + 61];
acadoWorkspace.Q1[lRun1 * 81 + 47] = acadoWorkspace.EH[lRun1 * 144 + 62];
acadoWorkspace.Q1[lRun1 * 81 + 48] = acadoWorkspace.EH[lRun1 * 144 + 63];
acadoWorkspace.Q1[lRun1 * 81 + 49] = acadoWorkspace.EH[lRun1 * 144 + 64];
acadoWorkspace.Q1[lRun1 * 81 + 50] = acadoWorkspace.EH[lRun1 * 144 + 65];
acadoWorkspace.Q1[lRun1 * 81 + 51] = acadoWorkspace.EH[lRun1 * 144 + 66];
acadoWorkspace.Q1[lRun1 * 81 + 52] = acadoWorkspace.EH[lRun1 * 144 + 67];
acadoWorkspace.Q1[lRun1 * 81 + 53] = acadoWorkspace.EH[lRun1 * 144 + 68];
acadoWorkspace.Q1[lRun1 * 81 + 54] = acadoWorkspace.EH[lRun1 * 144 + 72];
acadoWorkspace.Q1[lRun1 * 81 + 55] = acadoWorkspace.EH[lRun1 * 144 + 73];
acadoWorkspace.Q1[lRun1 * 81 + 56] = acadoWorkspace.EH[lRun1 * 144 + 74];
acadoWorkspace.Q1[lRun1 * 81 + 57] = acadoWorkspace.EH[lRun1 * 144 + 75];
acadoWorkspace.Q1[lRun1 * 81 + 58] = acadoWorkspace.EH[lRun1 * 144 + 76];
acadoWorkspace.Q1[lRun1 * 81 + 59] = acadoWorkspace.EH[lRun1 * 144 + 77];
acadoWorkspace.Q1[lRun1 * 81 + 60] = acadoWorkspace.EH[lRun1 * 144 + 78];
acadoWorkspace.Q1[lRun1 * 81 + 61] = acadoWorkspace.EH[lRun1 * 144 + 79];
acadoWorkspace.Q1[lRun1 * 81 + 62] = acadoWorkspace.EH[lRun1 * 144 + 80];
acadoWorkspace.Q1[lRun1 * 81 + 63] = acadoWorkspace.EH[lRun1 * 144 + 84];
acadoWorkspace.Q1[lRun1 * 81 + 64] = acadoWorkspace.EH[lRun1 * 144 + 85];
acadoWorkspace.Q1[lRun1 * 81 + 65] = acadoWorkspace.EH[lRun1 * 144 + 86];
acadoWorkspace.Q1[lRun1 * 81 + 66] = acadoWorkspace.EH[lRun1 * 144 + 87];
acadoWorkspace.Q1[lRun1 * 81 + 67] = acadoWorkspace.EH[lRun1 * 144 + 88];
acadoWorkspace.Q1[lRun1 * 81 + 68] = acadoWorkspace.EH[lRun1 * 144 + 89];
acadoWorkspace.Q1[lRun1 * 81 + 69] = acadoWorkspace.EH[lRun1 * 144 + 90];
acadoWorkspace.Q1[lRun1 * 81 + 70] = acadoWorkspace.EH[lRun1 * 144 + 91];
acadoWorkspace.Q1[lRun1 * 81 + 71] = acadoWorkspace.EH[lRun1 * 144 + 92];
acadoWorkspace.Q1[lRun1 * 81 + 72] = acadoWorkspace.EH[lRun1 * 144 + 96];
acadoWorkspace.Q1[lRun1 * 81 + 73] = acadoWorkspace.EH[lRun1 * 144 + 97];
acadoWorkspace.Q1[lRun1 * 81 + 74] = acadoWorkspace.EH[lRun1 * 144 + 98];
acadoWorkspace.Q1[lRun1 * 81 + 75] = acadoWorkspace.EH[lRun1 * 144 + 99];
acadoWorkspace.Q1[lRun1 * 81 + 76] = acadoWorkspace.EH[lRun1 * 144 + 100];
acadoWorkspace.Q1[lRun1 * 81 + 77] = acadoWorkspace.EH[lRun1 * 144 + 101];
acadoWorkspace.Q1[lRun1 * 81 + 78] = acadoWorkspace.EH[lRun1 * 144 + 102];
acadoWorkspace.Q1[lRun1 * 81 + 79] = acadoWorkspace.EH[lRun1 * 144 + 103];
acadoWorkspace.Q1[lRun1 * 81 + 80] = acadoWorkspace.EH[lRun1 * 144 + 104];
acadoWorkspace.S1[lRun1 * 27] = acadoWorkspace.EH[lRun1 * 144 + 9];
acadoWorkspace.S1[lRun1 * 27 + 1] = acadoWorkspace.EH[lRun1 * 144 + 10];
acadoWorkspace.S1[lRun1 * 27 + 2] = acadoWorkspace.EH[lRun1 * 144 + 11];
acadoWorkspace.S1[lRun1 * 27 + 3] = acadoWorkspace.EH[lRun1 * 144 + 21];
acadoWorkspace.S1[lRun1 * 27 + 4] = acadoWorkspace.EH[lRun1 * 144 + 22];
acadoWorkspace.S1[lRun1 * 27 + 5] = acadoWorkspace.EH[lRun1 * 144 + 23];
acadoWorkspace.S1[lRun1 * 27 + 6] = acadoWorkspace.EH[lRun1 * 144 + 33];
acadoWorkspace.S1[lRun1 * 27 + 7] = acadoWorkspace.EH[lRun1 * 144 + 34];
acadoWorkspace.S1[lRun1 * 27 + 8] = acadoWorkspace.EH[lRun1 * 144 + 35];
acadoWorkspace.S1[lRun1 * 27 + 9] = acadoWorkspace.EH[lRun1 * 144 + 45];
acadoWorkspace.S1[lRun1 * 27 + 10] = acadoWorkspace.EH[lRun1 * 144 + 46];
acadoWorkspace.S1[lRun1 * 27 + 11] = acadoWorkspace.EH[lRun1 * 144 + 47];
acadoWorkspace.S1[lRun1 * 27 + 12] = acadoWorkspace.EH[lRun1 * 144 + 57];
acadoWorkspace.S1[lRun1 * 27 + 13] = acadoWorkspace.EH[lRun1 * 144 + 58];
acadoWorkspace.S1[lRun1 * 27 + 14] = acadoWorkspace.EH[lRun1 * 144 + 59];
acadoWorkspace.S1[lRun1 * 27 + 15] = acadoWorkspace.EH[lRun1 * 144 + 69];
acadoWorkspace.S1[lRun1 * 27 + 16] = acadoWorkspace.EH[lRun1 * 144 + 70];
acadoWorkspace.S1[lRun1 * 27 + 17] = acadoWorkspace.EH[lRun1 * 144 + 71];
acadoWorkspace.S1[lRun1 * 27 + 18] = acadoWorkspace.EH[lRun1 * 144 + 81];
acadoWorkspace.S1[lRun1 * 27 + 19] = acadoWorkspace.EH[lRun1 * 144 + 82];
acadoWorkspace.S1[lRun1 * 27 + 20] = acadoWorkspace.EH[lRun1 * 144 + 83];
acadoWorkspace.S1[lRun1 * 27 + 21] = acadoWorkspace.EH[lRun1 * 144 + 93];
acadoWorkspace.S1[lRun1 * 27 + 22] = acadoWorkspace.EH[lRun1 * 144 + 94];
acadoWorkspace.S1[lRun1 * 27 + 23] = acadoWorkspace.EH[lRun1 * 144 + 95];
acadoWorkspace.S1[lRun1 * 27 + 24] = acadoWorkspace.EH[lRun1 * 144 + 105];
acadoWorkspace.S1[lRun1 * 27 + 25] = acadoWorkspace.EH[lRun1 * 144 + 106];
acadoWorkspace.S1[lRun1 * 27 + 26] = acadoWorkspace.EH[lRun1 * 144 + 107];
acadoWorkspace.R1[lRun1 * 9] = acadoWorkspace.EH[lRun1 * 144 + 117];
acadoWorkspace.R1[lRun1 * 9 + 1] = acadoWorkspace.EH[lRun1 * 144 + 118];
acadoWorkspace.R1[lRun1 * 9 + 2] = acadoWorkspace.EH[lRun1 * 144 + 119];
acadoWorkspace.R1[lRun1 * 9 + 3] = acadoWorkspace.EH[lRun1 * 144 + 129];
acadoWorkspace.R1[lRun1 * 9 + 4] = acadoWorkspace.EH[lRun1 * 144 + 130];
acadoWorkspace.R1[lRun1 * 9 + 5] = acadoWorkspace.EH[lRun1 * 144 + 131];
acadoWorkspace.R1[lRun1 * 9 + 6] = acadoWorkspace.EH[lRun1 * 144 + 141];
acadoWorkspace.R1[lRun1 * 9 + 7] = acadoWorkspace.EH[lRun1 * 144 + 142];
acadoWorkspace.R1[lRun1 * 9 + 8] = acadoWorkspace.EH[lRun1 * 144 + 143];
}
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
acadoWorkspace.H[(iRow * 333) + (iCol * 3)] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24];
acadoWorkspace.H[(iRow * 333) + (iCol * 3 + 1)] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25];
acadoWorkspace.H[(iRow * 333) + (iCol * 3 + 2)] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26];
acadoWorkspace.H[(iRow * 333 + 111) + (iCol * 3)] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24];
acadoWorkspace.H[(iRow * 333 + 111) + (iCol * 3 + 1)] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25];
acadoWorkspace.H[(iRow * 333 + 111) + (iCol * 3 + 2)] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26];
acadoWorkspace.H[(iRow * 333 + 222) + (iCol * 3)] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24];
acadoWorkspace.H[(iRow * 333 + 222) + (iCol * 3 + 1)] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25];
acadoWorkspace.H[(iRow * 333 + 222) + (iCol * 3 + 2)] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26];
}

void acado_mac_S1T_E( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 333) + (iCol * 3)] += + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24];
acadoWorkspace.H[(iRow * 333) + (iCol * 3 + 1)] += + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25];
acadoWorkspace.H[(iRow * 333) + (iCol * 3 + 2)] += + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26];
acadoWorkspace.H[(iRow * 333 + 111) + (iCol * 3)] += + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24];
acadoWorkspace.H[(iRow * 333 + 111) + (iCol * 3 + 1)] += + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25];
acadoWorkspace.H[(iRow * 333 + 111) + (iCol * 3 + 2)] += + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26];
acadoWorkspace.H[(iRow * 333 + 222) + (iCol * 3)] += + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24];
acadoWorkspace.H[(iRow * 333 + 222) + (iCol * 3 + 1)] += + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25];
acadoWorkspace.H[(iRow * 333 + 222) + (iCol * 3 + 2)] += + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 336] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + R11[0];
acadoWorkspace.H[iRow * 336 + 1] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + R11[1];
acadoWorkspace.H[iRow * 336 + 2] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + R11[2];
acadoWorkspace.H[iRow * 336 + 111] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + R11[3];
acadoWorkspace.H[iRow * 336 + 112] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + R11[4];
acadoWorkspace.H[iRow * 336 + 113] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + R11[5];
acadoWorkspace.H[iRow * 336 + 222] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + R11[6];
acadoWorkspace.H[iRow * 336 + 223] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + R11[7];
acadoWorkspace.H[iRow * 336 + 224] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + R11[8];
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

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[3] + Q11[2]*Gu1[6] + Q11[3]*Gu1[9] + Q11[4]*Gu1[12] + Q11[5]*Gu1[15] + Q11[6]*Gu1[18] + Q11[7]*Gu1[21] + Q11[8]*Gu1[24] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[4] + Q11[2]*Gu1[7] + Q11[3]*Gu1[10] + Q11[4]*Gu1[13] + Q11[5]*Gu1[16] + Q11[6]*Gu1[19] + Q11[7]*Gu1[22] + Q11[8]*Gu1[25] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[5] + Q11[2]*Gu1[8] + Q11[3]*Gu1[11] + Q11[4]*Gu1[14] + Q11[5]*Gu1[17] + Q11[6]*Gu1[20] + Q11[7]*Gu1[23] + Q11[8]*Gu1[26] + Gu2[2];
Gu3[3] = + Q11[9]*Gu1[0] + Q11[10]*Gu1[3] + Q11[11]*Gu1[6] + Q11[12]*Gu1[9] + Q11[13]*Gu1[12] + Q11[14]*Gu1[15] + Q11[15]*Gu1[18] + Q11[16]*Gu1[21] + Q11[17]*Gu1[24] + Gu2[3];
Gu3[4] = + Q11[9]*Gu1[1] + Q11[10]*Gu1[4] + Q11[11]*Gu1[7] + Q11[12]*Gu1[10] + Q11[13]*Gu1[13] + Q11[14]*Gu1[16] + Q11[15]*Gu1[19] + Q11[16]*Gu1[22] + Q11[17]*Gu1[25] + Gu2[4];
Gu3[5] = + Q11[9]*Gu1[2] + Q11[10]*Gu1[5] + Q11[11]*Gu1[8] + Q11[12]*Gu1[11] + Q11[13]*Gu1[14] + Q11[14]*Gu1[17] + Q11[15]*Gu1[20] + Q11[16]*Gu1[23] + Q11[17]*Gu1[26] + Gu2[5];
Gu3[6] = + Q11[18]*Gu1[0] + Q11[19]*Gu1[3] + Q11[20]*Gu1[6] + Q11[21]*Gu1[9] + Q11[22]*Gu1[12] + Q11[23]*Gu1[15] + Q11[24]*Gu1[18] + Q11[25]*Gu1[21] + Q11[26]*Gu1[24] + Gu2[6];
Gu3[7] = + Q11[18]*Gu1[1] + Q11[19]*Gu1[4] + Q11[20]*Gu1[7] + Q11[21]*Gu1[10] + Q11[22]*Gu1[13] + Q11[23]*Gu1[16] + Q11[24]*Gu1[19] + Q11[25]*Gu1[22] + Q11[26]*Gu1[25] + Gu2[7];
Gu3[8] = + Q11[18]*Gu1[2] + Q11[19]*Gu1[5] + Q11[20]*Gu1[8] + Q11[21]*Gu1[11] + Q11[22]*Gu1[14] + Q11[23]*Gu1[17] + Q11[24]*Gu1[20] + Q11[25]*Gu1[23] + Q11[26]*Gu1[26] + Gu2[8];
Gu3[9] = + Q11[27]*Gu1[0] + Q11[28]*Gu1[3] + Q11[29]*Gu1[6] + Q11[30]*Gu1[9] + Q11[31]*Gu1[12] + Q11[32]*Gu1[15] + Q11[33]*Gu1[18] + Q11[34]*Gu1[21] + Q11[35]*Gu1[24] + Gu2[9];
Gu3[10] = + Q11[27]*Gu1[1] + Q11[28]*Gu1[4] + Q11[29]*Gu1[7] + Q11[30]*Gu1[10] + Q11[31]*Gu1[13] + Q11[32]*Gu1[16] + Q11[33]*Gu1[19] + Q11[34]*Gu1[22] + Q11[35]*Gu1[25] + Gu2[10];
Gu3[11] = + Q11[27]*Gu1[2] + Q11[28]*Gu1[5] + Q11[29]*Gu1[8] + Q11[30]*Gu1[11] + Q11[31]*Gu1[14] + Q11[32]*Gu1[17] + Q11[33]*Gu1[20] + Q11[34]*Gu1[23] + Q11[35]*Gu1[26] + Gu2[11];
Gu3[12] = + Q11[36]*Gu1[0] + Q11[37]*Gu1[3] + Q11[38]*Gu1[6] + Q11[39]*Gu1[9] + Q11[40]*Gu1[12] + Q11[41]*Gu1[15] + Q11[42]*Gu1[18] + Q11[43]*Gu1[21] + Q11[44]*Gu1[24] + Gu2[12];
Gu3[13] = + Q11[36]*Gu1[1] + Q11[37]*Gu1[4] + Q11[38]*Gu1[7] + Q11[39]*Gu1[10] + Q11[40]*Gu1[13] + Q11[41]*Gu1[16] + Q11[42]*Gu1[19] + Q11[43]*Gu1[22] + Q11[44]*Gu1[25] + Gu2[13];
Gu3[14] = + Q11[36]*Gu1[2] + Q11[37]*Gu1[5] + Q11[38]*Gu1[8] + Q11[39]*Gu1[11] + Q11[40]*Gu1[14] + Q11[41]*Gu1[17] + Q11[42]*Gu1[20] + Q11[43]*Gu1[23] + Q11[44]*Gu1[26] + Gu2[14];
Gu3[15] = + Q11[45]*Gu1[0] + Q11[46]*Gu1[3] + Q11[47]*Gu1[6] + Q11[48]*Gu1[9] + Q11[49]*Gu1[12] + Q11[50]*Gu1[15] + Q11[51]*Gu1[18] + Q11[52]*Gu1[21] + Q11[53]*Gu1[24] + Gu2[15];
Gu3[16] = + Q11[45]*Gu1[1] + Q11[46]*Gu1[4] + Q11[47]*Gu1[7] + Q11[48]*Gu1[10] + Q11[49]*Gu1[13] + Q11[50]*Gu1[16] + Q11[51]*Gu1[19] + Q11[52]*Gu1[22] + Q11[53]*Gu1[25] + Gu2[16];
Gu3[17] = + Q11[45]*Gu1[2] + Q11[46]*Gu1[5] + Q11[47]*Gu1[8] + Q11[48]*Gu1[11] + Q11[49]*Gu1[14] + Q11[50]*Gu1[17] + Q11[51]*Gu1[20] + Q11[52]*Gu1[23] + Q11[53]*Gu1[26] + Gu2[17];
Gu3[18] = + Q11[54]*Gu1[0] + Q11[55]*Gu1[3] + Q11[56]*Gu1[6] + Q11[57]*Gu1[9] + Q11[58]*Gu1[12] + Q11[59]*Gu1[15] + Q11[60]*Gu1[18] + Q11[61]*Gu1[21] + Q11[62]*Gu1[24] + Gu2[18];
Gu3[19] = + Q11[54]*Gu1[1] + Q11[55]*Gu1[4] + Q11[56]*Gu1[7] + Q11[57]*Gu1[10] + Q11[58]*Gu1[13] + Q11[59]*Gu1[16] + Q11[60]*Gu1[19] + Q11[61]*Gu1[22] + Q11[62]*Gu1[25] + Gu2[19];
Gu3[20] = + Q11[54]*Gu1[2] + Q11[55]*Gu1[5] + Q11[56]*Gu1[8] + Q11[57]*Gu1[11] + Q11[58]*Gu1[14] + Q11[59]*Gu1[17] + Q11[60]*Gu1[20] + Q11[61]*Gu1[23] + Q11[62]*Gu1[26] + Gu2[20];
Gu3[21] = + Q11[63]*Gu1[0] + Q11[64]*Gu1[3] + Q11[65]*Gu1[6] + Q11[66]*Gu1[9] + Q11[67]*Gu1[12] + Q11[68]*Gu1[15] + Q11[69]*Gu1[18] + Q11[70]*Gu1[21] + Q11[71]*Gu1[24] + Gu2[21];
Gu3[22] = + Q11[63]*Gu1[1] + Q11[64]*Gu1[4] + Q11[65]*Gu1[7] + Q11[66]*Gu1[10] + Q11[67]*Gu1[13] + Q11[68]*Gu1[16] + Q11[69]*Gu1[19] + Q11[70]*Gu1[22] + Q11[71]*Gu1[25] + Gu2[22];
Gu3[23] = + Q11[63]*Gu1[2] + Q11[64]*Gu1[5] + Q11[65]*Gu1[8] + Q11[66]*Gu1[11] + Q11[67]*Gu1[14] + Q11[68]*Gu1[17] + Q11[69]*Gu1[20] + Q11[70]*Gu1[23] + Q11[71]*Gu1[26] + Gu2[23];
Gu3[24] = + Q11[72]*Gu1[0] + Q11[73]*Gu1[3] + Q11[74]*Gu1[6] + Q11[75]*Gu1[9] + Q11[76]*Gu1[12] + Q11[77]*Gu1[15] + Q11[78]*Gu1[18] + Q11[79]*Gu1[21] + Q11[80]*Gu1[24] + Gu2[24];
Gu3[25] = + Q11[72]*Gu1[1] + Q11[73]*Gu1[4] + Q11[74]*Gu1[7] + Q11[75]*Gu1[10] + Q11[76]*Gu1[13] + Q11[77]*Gu1[16] + Q11[78]*Gu1[19] + Q11[79]*Gu1[22] + Q11[80]*Gu1[25] + Gu2[25];
Gu3[26] = + Q11[72]*Gu1[2] + Q11[73]*Gu1[5] + Q11[74]*Gu1[8] + Q11[75]*Gu1[11] + Q11[76]*Gu1[14] + Q11[77]*Gu1[17] + Q11[78]*Gu1[20] + Q11[79]*Gu1[23] + Q11[80]*Gu1[26] + Gu2[26];
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

void acado_macS1TSbar( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5] + Gu1[18]*w11[6] + Gu1[21]*w11[7] + Gu1[24]*w11[8];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5] + Gu1[19]*w11[6] + Gu1[22]*w11[7] + Gu1[25]*w11[8];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5] + Gu1[20]*w11[6] + Gu1[23]*w11[7] + Gu1[26]*w11[8];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + Q11[8]*w11[8] + w12[0];
w13[1] = + Q11[9]*w11[0] + Q11[10]*w11[1] + Q11[11]*w11[2] + Q11[12]*w11[3] + Q11[13]*w11[4] + Q11[14]*w11[5] + Q11[15]*w11[6] + Q11[16]*w11[7] + Q11[17]*w11[8] + w12[1];
w13[2] = + Q11[18]*w11[0] + Q11[19]*w11[1] + Q11[20]*w11[2] + Q11[21]*w11[3] + Q11[22]*w11[4] + Q11[23]*w11[5] + Q11[24]*w11[6] + Q11[25]*w11[7] + Q11[26]*w11[8] + w12[2];
w13[3] = + Q11[27]*w11[0] + Q11[28]*w11[1] + Q11[29]*w11[2] + Q11[30]*w11[3] + Q11[31]*w11[4] + Q11[32]*w11[5] + Q11[33]*w11[6] + Q11[34]*w11[7] + Q11[35]*w11[8] + w12[3];
w13[4] = + Q11[36]*w11[0] + Q11[37]*w11[1] + Q11[38]*w11[2] + Q11[39]*w11[3] + Q11[40]*w11[4] + Q11[41]*w11[5] + Q11[42]*w11[6] + Q11[43]*w11[7] + Q11[44]*w11[8] + w12[4];
w13[5] = + Q11[45]*w11[0] + Q11[46]*w11[1] + Q11[47]*w11[2] + Q11[48]*w11[3] + Q11[49]*w11[4] + Q11[50]*w11[5] + Q11[51]*w11[6] + Q11[52]*w11[7] + Q11[53]*w11[8] + w12[5];
w13[6] = + Q11[54]*w11[0] + Q11[55]*w11[1] + Q11[56]*w11[2] + Q11[57]*w11[3] + Q11[58]*w11[4] + Q11[59]*w11[5] + Q11[60]*w11[6] + Q11[61]*w11[7] + Q11[62]*w11[8] + w12[6];
w13[7] = + Q11[63]*w11[0] + Q11[64]*w11[1] + Q11[65]*w11[2] + Q11[66]*w11[3] + Q11[67]*w11[4] + Q11[68]*w11[5] + Q11[69]*w11[6] + Q11[70]*w11[7] + Q11[71]*w11[8] + w12[7];
w13[8] = + Q11[72]*w11[0] + Q11[73]*w11[1] + Q11[74]*w11[2] + Q11[75]*w11[3] + Q11[76]*w11[4] + Q11[77]*w11[5] + Q11[78]*w11[6] + Q11[79]*w11[7] + Q11[80]*w11[8] + w12[8];
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

void acado_expansionStep2( real_t* const QDy1, real_t* const Q11, real_t* const w11, real_t* const Gu1, real_t* const U1, real_t* const Gx1, real_t* const mu1, real_t* const mu2 )
{
mu1[0] += QDy1[0];
mu1[1] += QDy1[1];
mu1[2] += QDy1[2];
mu1[3] += QDy1[3];
mu1[4] += QDy1[4];
mu1[5] += QDy1[5];
mu1[6] += QDy1[6];
mu1[7] += QDy1[7];
mu1[8] += QDy1[8];
mu1[0] += + w11[0]*Q11[0] + w11[1]*Q11[1] + w11[2]*Q11[2] + w11[3]*Q11[3] + w11[4]*Q11[4] + w11[5]*Q11[5] + w11[6]*Q11[6] + w11[7]*Q11[7] + w11[8]*Q11[8];
mu1[1] += + w11[0]*Q11[9] + w11[1]*Q11[10] + w11[2]*Q11[11] + w11[3]*Q11[12] + w11[4]*Q11[13] + w11[5]*Q11[14] + w11[6]*Q11[15] + w11[7]*Q11[16] + w11[8]*Q11[17];
mu1[2] += + w11[0]*Q11[18] + w11[1]*Q11[19] + w11[2]*Q11[20] + w11[3]*Q11[21] + w11[4]*Q11[22] + w11[5]*Q11[23] + w11[6]*Q11[24] + w11[7]*Q11[25] + w11[8]*Q11[26];
mu1[3] += + w11[0]*Q11[27] + w11[1]*Q11[28] + w11[2]*Q11[29] + w11[3]*Q11[30] + w11[4]*Q11[31] + w11[5]*Q11[32] + w11[6]*Q11[33] + w11[7]*Q11[34] + w11[8]*Q11[35];
mu1[4] += + w11[0]*Q11[36] + w11[1]*Q11[37] + w11[2]*Q11[38] + w11[3]*Q11[39] + w11[4]*Q11[40] + w11[5]*Q11[41] + w11[6]*Q11[42] + w11[7]*Q11[43] + w11[8]*Q11[44];
mu1[5] += + w11[0]*Q11[45] + w11[1]*Q11[46] + w11[2]*Q11[47] + w11[3]*Q11[48] + w11[4]*Q11[49] + w11[5]*Q11[50] + w11[6]*Q11[51] + w11[7]*Q11[52] + w11[8]*Q11[53];
mu1[6] += + w11[0]*Q11[54] + w11[1]*Q11[55] + w11[2]*Q11[56] + w11[3]*Q11[57] + w11[4]*Q11[58] + w11[5]*Q11[59] + w11[6]*Q11[60] + w11[7]*Q11[61] + w11[8]*Q11[62];
mu1[7] += + w11[0]*Q11[63] + w11[1]*Q11[64] + w11[2]*Q11[65] + w11[3]*Q11[66] + w11[4]*Q11[67] + w11[5]*Q11[68] + w11[6]*Q11[69] + w11[7]*Q11[70] + w11[8]*Q11[71];
mu1[8] += + w11[0]*Q11[72] + w11[1]*Q11[73] + w11[2]*Q11[74] + w11[3]*Q11[75] + w11[4]*Q11[76] + w11[5]*Q11[77] + w11[6]*Q11[78] + w11[7]*Q11[79] + w11[8]*Q11[80];
mu1[0] += + U1[0]*Gu1[0] + U1[1]*Gu1[1] + U1[2]*Gu1[2];
mu1[1] += + U1[0]*Gu1[3] + U1[1]*Gu1[4] + U1[2]*Gu1[5];
mu1[2] += + U1[0]*Gu1[6] + U1[1]*Gu1[7] + U1[2]*Gu1[8];
mu1[3] += + U1[0]*Gu1[9] + U1[1]*Gu1[10] + U1[2]*Gu1[11];
mu1[4] += + U1[0]*Gu1[12] + U1[1]*Gu1[13] + U1[2]*Gu1[14];
mu1[5] += + U1[0]*Gu1[15] + U1[1]*Gu1[16] + U1[2]*Gu1[17];
mu1[6] += + U1[0]*Gu1[18] + U1[1]*Gu1[19] + U1[2]*Gu1[20];
mu1[7] += + U1[0]*Gu1[21] + U1[1]*Gu1[22] + U1[2]*Gu1[23];
mu1[8] += + U1[0]*Gu1[24] + U1[1]*Gu1[25] + U1[2]*Gu1[26];
mu1[0] += + mu2[0]*Gx1[0] + mu2[1]*Gx1[9] + mu2[2]*Gx1[18] + mu2[3]*Gx1[27] + mu2[4]*Gx1[36] + mu2[5]*Gx1[45] + mu2[6]*Gx1[54] + mu2[7]*Gx1[63] + mu2[8]*Gx1[72];
mu1[1] += + mu2[0]*Gx1[1] + mu2[1]*Gx1[10] + mu2[2]*Gx1[19] + mu2[3]*Gx1[28] + mu2[4]*Gx1[37] + mu2[5]*Gx1[46] + mu2[6]*Gx1[55] + mu2[7]*Gx1[64] + mu2[8]*Gx1[73];
mu1[2] += + mu2[0]*Gx1[2] + mu2[1]*Gx1[11] + mu2[2]*Gx1[20] + mu2[3]*Gx1[29] + mu2[4]*Gx1[38] + mu2[5]*Gx1[47] + mu2[6]*Gx1[56] + mu2[7]*Gx1[65] + mu2[8]*Gx1[74];
mu1[3] += + mu2[0]*Gx1[3] + mu2[1]*Gx1[12] + mu2[2]*Gx1[21] + mu2[3]*Gx1[30] + mu2[4]*Gx1[39] + mu2[5]*Gx1[48] + mu2[6]*Gx1[57] + mu2[7]*Gx1[66] + mu2[8]*Gx1[75];
mu1[4] += + mu2[0]*Gx1[4] + mu2[1]*Gx1[13] + mu2[2]*Gx1[22] + mu2[3]*Gx1[31] + mu2[4]*Gx1[40] + mu2[5]*Gx1[49] + mu2[6]*Gx1[58] + mu2[7]*Gx1[67] + mu2[8]*Gx1[76];
mu1[5] += + mu2[0]*Gx1[5] + mu2[1]*Gx1[14] + mu2[2]*Gx1[23] + mu2[3]*Gx1[32] + mu2[4]*Gx1[41] + mu2[5]*Gx1[50] + mu2[6]*Gx1[59] + mu2[7]*Gx1[68] + mu2[8]*Gx1[77];
mu1[6] += + mu2[0]*Gx1[6] + mu2[1]*Gx1[15] + mu2[2]*Gx1[24] + mu2[3]*Gx1[33] + mu2[4]*Gx1[42] + mu2[5]*Gx1[51] + mu2[6]*Gx1[60] + mu2[7]*Gx1[69] + mu2[8]*Gx1[78];
mu1[7] += + mu2[0]*Gx1[7] + mu2[1]*Gx1[16] + mu2[2]*Gx1[25] + mu2[3]*Gx1[34] + mu2[4]*Gx1[43] + mu2[5]*Gx1[52] + mu2[6]*Gx1[61] + mu2[7]*Gx1[70] + mu2[8]*Gx1[79];
mu1[8] += + mu2[0]*Gx1[8] + mu2[1]*Gx1[17] + mu2[2]*Gx1[26] + mu2[3]*Gx1[35] + mu2[4]*Gx1[44] + mu2[5]*Gx1[53] + mu2[6]*Gx1[62] + mu2[7]*Gx1[71] + mu2[8]*Gx1[80];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 333) + (iCol * 3)] = acadoWorkspace.H[(iCol * 333) + (iRow * 3)];
acadoWorkspace.H[(iRow * 333) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 333 + 111) + (iRow * 3)];
acadoWorkspace.H[(iRow * 333) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 333 + 222) + (iRow * 3)];
acadoWorkspace.H[(iRow * 333 + 111) + (iCol * 3)] = acadoWorkspace.H[(iCol * 333) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 333 + 111) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 333 + 111) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 333 + 111) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 333 + 222) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 333 + 222) + (iCol * 3)] = acadoWorkspace.H[(iCol * 333) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 333 + 222) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 333 + 111) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 333 + 222) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 333 + 222) + (iRow * 3 + 2)];
}

void acado_multRDy( real_t* const RDy1 )
{
}

void acado_multQDy( real_t* const QDy1 )
{
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun2 = 0; lRun2 < 37; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 75)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 27 ]), &(acadoWorkspace.E[ lRun3 * 27 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 37; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (9)) * (9)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (9)) * (3)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (9)) * (3)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (37)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 36; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 27 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_mac_S1T_E( &(acadoWorkspace.S1[ lRun1 * 27 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (9)) * (3)) + (0) ]), lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 81 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 81 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 9 ]), &(acadoWorkspace.evGu[ lRun2 * 27 ]), acadoWorkspace.W1, lRun2 );
}

for (lRun1 = 0; lRun1 < 37; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
acado_copyHTH( lRun2, lRun1 );
}
}

for (lRun1 = 0; lRun1 < 333; ++lRun1)
acadoWorkspace.sbar[lRun1 + 9] = acadoWorkspace.d[lRun1];


}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];

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
acado_macASbar( &(acadoWorkspace.evGx[ 2430 ]), &(acadoWorkspace.sbar[ 270 ]), &(acadoWorkspace.sbar[ 279 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2511 ]), &(acadoWorkspace.sbar[ 279 ]), &(acadoWorkspace.sbar[ 288 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.sbar[ 288 ]), &(acadoWorkspace.sbar[ 297 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2673 ]), &(acadoWorkspace.sbar[ 297 ]), &(acadoWorkspace.sbar[ 306 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2754 ]), &(acadoWorkspace.sbar[ 306 ]), &(acadoWorkspace.sbar[ 315 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2835 ]), &(acadoWorkspace.sbar[ 315 ]), &(acadoWorkspace.sbar[ 324 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.sbar[ 324 ]), &(acadoWorkspace.sbar[ 333 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[333] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[334] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[335] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[336] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[337] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[338] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[339] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[340] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[341] + acadoWorkspace.QDy[333];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[333] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[334] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[335] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[336] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[337] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[338] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[339] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[340] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[341] + acadoWorkspace.QDy[334];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[333] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[334] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[335] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[336] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[337] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[338] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[339] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[340] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[341] + acadoWorkspace.QDy[335];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[333] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[334] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[335] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[336] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[337] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[338] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[339] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[340] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[341] + acadoWorkspace.QDy[336];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[333] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[334] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[335] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[336] + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[337] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[338] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[339] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[340] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[341] + acadoWorkspace.QDy[337];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[333] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[334] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[335] + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[336] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[337] + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[338] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[339] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[340] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[341] + acadoWorkspace.QDy[338];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[333] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[334] + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[335] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[336] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[337] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[338] + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[339] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[340] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[341] + acadoWorkspace.QDy[339];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[333] + acadoWorkspace.QN1[64]*acadoWorkspace.sbar[334] + acadoWorkspace.QN1[65]*acadoWorkspace.sbar[335] + acadoWorkspace.QN1[66]*acadoWorkspace.sbar[336] + acadoWorkspace.QN1[67]*acadoWorkspace.sbar[337] + acadoWorkspace.QN1[68]*acadoWorkspace.sbar[338] + acadoWorkspace.QN1[69]*acadoWorkspace.sbar[339] + acadoWorkspace.QN1[70]*acadoWorkspace.sbar[340] + acadoWorkspace.QN1[71]*acadoWorkspace.sbar[341] + acadoWorkspace.QDy[340];
acadoWorkspace.w1[8] = + acadoWorkspace.QN1[72]*acadoWorkspace.sbar[333] + acadoWorkspace.QN1[73]*acadoWorkspace.sbar[334] + acadoWorkspace.QN1[74]*acadoWorkspace.sbar[335] + acadoWorkspace.QN1[75]*acadoWorkspace.sbar[336] + acadoWorkspace.QN1[76]*acadoWorkspace.sbar[337] + acadoWorkspace.QN1[77]*acadoWorkspace.sbar[338] + acadoWorkspace.QN1[78]*acadoWorkspace.sbar[339] + acadoWorkspace.QN1[79]*acadoWorkspace.sbar[340] + acadoWorkspace.QN1[80]*acadoWorkspace.sbar[341] + acadoWorkspace.QDy[341];
acado_macBTw1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 108 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 972 ]), &(acadoWorkspace.sbar[ 324 ]), &(acadoWorkspace.g[ 108 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 324 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.sbar[ 324 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 945 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 105 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 945 ]), &(acadoWorkspace.sbar[ 315 ]), &(acadoWorkspace.g[ 105 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2835 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 315 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2835 ]), &(acadoWorkspace.sbar[ 315 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 918 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 102 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 918 ]), &(acadoWorkspace.sbar[ 306 ]), &(acadoWorkspace.g[ 102 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2754 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 306 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2754 ]), &(acadoWorkspace.sbar[ 306 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 891 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 99 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 891 ]), &(acadoWorkspace.sbar[ 297 ]), &(acadoWorkspace.g[ 99 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2673 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 297 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2673 ]), &(acadoWorkspace.sbar[ 297 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 96 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 864 ]), &(acadoWorkspace.sbar[ 288 ]), &(acadoWorkspace.g[ 96 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 288 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.sbar[ 288 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 837 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 93 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 837 ]), &(acadoWorkspace.sbar[ 279 ]), &(acadoWorkspace.g[ 93 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2511 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 279 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2511 ]), &(acadoWorkspace.sbar[ 279 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 810 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 90 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 810 ]), &(acadoWorkspace.sbar[ 270 ]), &(acadoWorkspace.g[ 90 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2430 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 270 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2430 ]), &(acadoWorkspace.sbar[ 270 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 783 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 87 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 783 ]), &(acadoWorkspace.sbar[ 261 ]), &(acadoWorkspace.g[ 87 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2349 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 261 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2349 ]), &(acadoWorkspace.sbar[ 261 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 84 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 756 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.g[ 84 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2268 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 252 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2268 ]), &(acadoWorkspace.sbar[ 252 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 729 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 81 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 729 ]), &(acadoWorkspace.sbar[ 243 ]), &(acadoWorkspace.g[ 81 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2187 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 243 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2187 ]), &(acadoWorkspace.sbar[ 243 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 702 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 78 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 702 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.g[ 78 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2106 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 234 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2106 ]), &(acadoWorkspace.sbar[ 234 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 675 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 75 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 675 ]), &(acadoWorkspace.sbar[ 225 ]), &(acadoWorkspace.g[ 75 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2025 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 225 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2025 ]), &(acadoWorkspace.sbar[ 225 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 648 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1944 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 216 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1944 ]), &(acadoWorkspace.sbar[ 216 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 621 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 69 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 621 ]), &(acadoWorkspace.sbar[ 207 ]), &(acadoWorkspace.g[ 69 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1863 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 207 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1863 ]), &(acadoWorkspace.sbar[ 207 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 594 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 66 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 594 ]), &(acadoWorkspace.sbar[ 198 ]), &(acadoWorkspace.g[ 66 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1782 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 198 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1782 ]), &(acadoWorkspace.sbar[ 198 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 567 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 63 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 567 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.g[ 63 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1701 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 189 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1701 ]), &(acadoWorkspace.sbar[ 189 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 540 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 540 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1620 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 180 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1620 ]), &(acadoWorkspace.sbar[ 180 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 513 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 57 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 513 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.g[ 57 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1539 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 171 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1539 ]), &(acadoWorkspace.sbar[ 171 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 486 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1458 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 162 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1458 ]), &(acadoWorkspace.sbar[ 162 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 459 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 51 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 459 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.g[ 51 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1377 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 153 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1377 ]), &(acadoWorkspace.sbar[ 153 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 432 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 45 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 405 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.g[ 45 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1215 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 135 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1215 ]), &(acadoWorkspace.sbar[ 135 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 378 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 378 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1134 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1134 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 351 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 351 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1053 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1053 ]), &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 324 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 972 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 972 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 297 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 297 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 891 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 99 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 891 ]), &(acadoWorkspace.sbar[ 99 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 270 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 810 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 810 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 243 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 729 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 81 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.sbar[ 81 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 216 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 189 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 567 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.sbar[ 63 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 162 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 135 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 81 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 54 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macS1TSbar( &(acadoWorkspace.S1[ 27 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );
acado_macS1TSbar( acadoWorkspace.S1, acadoWorkspace.sbar, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.lb[60] = acadoVariables.lbValues[60] - acadoVariables.u[60];
acadoWorkspace.lb[61] = acadoVariables.lbValues[61] - acadoVariables.u[61];
acadoWorkspace.lb[62] = acadoVariables.lbValues[62] - acadoVariables.u[62];
acadoWorkspace.lb[63] = acadoVariables.lbValues[63] - acadoVariables.u[63];
acadoWorkspace.lb[64] = acadoVariables.lbValues[64] - acadoVariables.u[64];
acadoWorkspace.lb[65] = acadoVariables.lbValues[65] - acadoVariables.u[65];
acadoWorkspace.lb[66] = acadoVariables.lbValues[66] - acadoVariables.u[66];
acadoWorkspace.lb[67] = acadoVariables.lbValues[67] - acadoVariables.u[67];
acadoWorkspace.lb[68] = acadoVariables.lbValues[68] - acadoVariables.u[68];
acadoWorkspace.lb[69] = acadoVariables.lbValues[69] - acadoVariables.u[69];
acadoWorkspace.lb[70] = acadoVariables.lbValues[70] - acadoVariables.u[70];
acadoWorkspace.lb[71] = acadoVariables.lbValues[71] - acadoVariables.u[71];
acadoWorkspace.lb[72] = acadoVariables.lbValues[72] - acadoVariables.u[72];
acadoWorkspace.lb[73] = acadoVariables.lbValues[73] - acadoVariables.u[73];
acadoWorkspace.lb[74] = acadoVariables.lbValues[74] - acadoVariables.u[74];
acadoWorkspace.lb[75] = acadoVariables.lbValues[75] - acadoVariables.u[75];
acadoWorkspace.lb[76] = acadoVariables.lbValues[76] - acadoVariables.u[76];
acadoWorkspace.lb[77] = acadoVariables.lbValues[77] - acadoVariables.u[77];
acadoWorkspace.lb[78] = acadoVariables.lbValues[78] - acadoVariables.u[78];
acadoWorkspace.lb[79] = acadoVariables.lbValues[79] - acadoVariables.u[79];
acadoWorkspace.lb[80] = acadoVariables.lbValues[80] - acadoVariables.u[80];
acadoWorkspace.lb[81] = acadoVariables.lbValues[81] - acadoVariables.u[81];
acadoWorkspace.lb[82] = acadoVariables.lbValues[82] - acadoVariables.u[82];
acadoWorkspace.lb[83] = acadoVariables.lbValues[83] - acadoVariables.u[83];
acadoWorkspace.lb[84] = acadoVariables.lbValues[84] - acadoVariables.u[84];
acadoWorkspace.lb[85] = acadoVariables.lbValues[85] - acadoVariables.u[85];
acadoWorkspace.lb[86] = acadoVariables.lbValues[86] - acadoVariables.u[86];
acadoWorkspace.lb[87] = acadoVariables.lbValues[87] - acadoVariables.u[87];
acadoWorkspace.lb[88] = acadoVariables.lbValues[88] - acadoVariables.u[88];
acadoWorkspace.lb[89] = acadoVariables.lbValues[89] - acadoVariables.u[89];
acadoWorkspace.lb[90] = acadoVariables.lbValues[90] - acadoVariables.u[90];
acadoWorkspace.lb[91] = acadoVariables.lbValues[91] - acadoVariables.u[91];
acadoWorkspace.lb[92] = acadoVariables.lbValues[92] - acadoVariables.u[92];
acadoWorkspace.lb[93] = acadoVariables.lbValues[93] - acadoVariables.u[93];
acadoWorkspace.lb[94] = acadoVariables.lbValues[94] - acadoVariables.u[94];
acadoWorkspace.lb[95] = acadoVariables.lbValues[95] - acadoVariables.u[95];
acadoWorkspace.lb[96] = acadoVariables.lbValues[96] - acadoVariables.u[96];
acadoWorkspace.lb[97] = acadoVariables.lbValues[97] - acadoVariables.u[97];
acadoWorkspace.lb[98] = acadoVariables.lbValues[98] - acadoVariables.u[98];
acadoWorkspace.lb[99] = acadoVariables.lbValues[99] - acadoVariables.u[99];
acadoWorkspace.lb[100] = acadoVariables.lbValues[100] - acadoVariables.u[100];
acadoWorkspace.lb[101] = acadoVariables.lbValues[101] - acadoVariables.u[101];
acadoWorkspace.lb[102] = acadoVariables.lbValues[102] - acadoVariables.u[102];
acadoWorkspace.lb[103] = acadoVariables.lbValues[103] - acadoVariables.u[103];
acadoWorkspace.lb[104] = acadoVariables.lbValues[104] - acadoVariables.u[104];
acadoWorkspace.lb[105] = acadoVariables.lbValues[105] - acadoVariables.u[105];
acadoWorkspace.lb[106] = acadoVariables.lbValues[106] - acadoVariables.u[106];
acadoWorkspace.lb[107] = acadoVariables.lbValues[107] - acadoVariables.u[107];
acadoWorkspace.lb[108] = acadoVariables.lbValues[108] - acadoVariables.u[108];
acadoWorkspace.lb[109] = acadoVariables.lbValues[109] - acadoVariables.u[109];
acadoWorkspace.lb[110] = acadoVariables.lbValues[110] - acadoVariables.u[110];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[60] = acadoVariables.ubValues[60] - acadoVariables.u[60];
acadoWorkspace.ub[61] = acadoVariables.ubValues[61] - acadoVariables.u[61];
acadoWorkspace.ub[62] = acadoVariables.ubValues[62] - acadoVariables.u[62];
acadoWorkspace.ub[63] = acadoVariables.ubValues[63] - acadoVariables.u[63];
acadoWorkspace.ub[64] = acadoVariables.ubValues[64] - acadoVariables.u[64];
acadoWorkspace.ub[65] = acadoVariables.ubValues[65] - acadoVariables.u[65];
acadoWorkspace.ub[66] = acadoVariables.ubValues[66] - acadoVariables.u[66];
acadoWorkspace.ub[67] = acadoVariables.ubValues[67] - acadoVariables.u[67];
acadoWorkspace.ub[68] = acadoVariables.ubValues[68] - acadoVariables.u[68];
acadoWorkspace.ub[69] = acadoVariables.ubValues[69] - acadoVariables.u[69];
acadoWorkspace.ub[70] = acadoVariables.ubValues[70] - acadoVariables.u[70];
acadoWorkspace.ub[71] = acadoVariables.ubValues[71] - acadoVariables.u[71];
acadoWorkspace.ub[72] = acadoVariables.ubValues[72] - acadoVariables.u[72];
acadoWorkspace.ub[73] = acadoVariables.ubValues[73] - acadoVariables.u[73];
acadoWorkspace.ub[74] = acadoVariables.ubValues[74] - acadoVariables.u[74];
acadoWorkspace.ub[75] = acadoVariables.ubValues[75] - acadoVariables.u[75];
acadoWorkspace.ub[76] = acadoVariables.ubValues[76] - acadoVariables.u[76];
acadoWorkspace.ub[77] = acadoVariables.ubValues[77] - acadoVariables.u[77];
acadoWorkspace.ub[78] = acadoVariables.ubValues[78] - acadoVariables.u[78];
acadoWorkspace.ub[79] = acadoVariables.ubValues[79] - acadoVariables.u[79];
acadoWorkspace.ub[80] = acadoVariables.ubValues[80] - acadoVariables.u[80];
acadoWorkspace.ub[81] = acadoVariables.ubValues[81] - acadoVariables.u[81];
acadoWorkspace.ub[82] = acadoVariables.ubValues[82] - acadoVariables.u[82];
acadoWorkspace.ub[83] = acadoVariables.ubValues[83] - acadoVariables.u[83];
acadoWorkspace.ub[84] = acadoVariables.ubValues[84] - acadoVariables.u[84];
acadoWorkspace.ub[85] = acadoVariables.ubValues[85] - acadoVariables.u[85];
acadoWorkspace.ub[86] = acadoVariables.ubValues[86] - acadoVariables.u[86];
acadoWorkspace.ub[87] = acadoVariables.ubValues[87] - acadoVariables.u[87];
acadoWorkspace.ub[88] = acadoVariables.ubValues[88] - acadoVariables.u[88];
acadoWorkspace.ub[89] = acadoVariables.ubValues[89] - acadoVariables.u[89];
acadoWorkspace.ub[90] = acadoVariables.ubValues[90] - acadoVariables.u[90];
acadoWorkspace.ub[91] = acadoVariables.ubValues[91] - acadoVariables.u[91];
acadoWorkspace.ub[92] = acadoVariables.ubValues[92] - acadoVariables.u[92];
acadoWorkspace.ub[93] = acadoVariables.ubValues[93] - acadoVariables.u[93];
acadoWorkspace.ub[94] = acadoVariables.ubValues[94] - acadoVariables.u[94];
acadoWorkspace.ub[95] = acadoVariables.ubValues[95] - acadoVariables.u[95];
acadoWorkspace.ub[96] = acadoVariables.ubValues[96] - acadoVariables.u[96];
acadoWorkspace.ub[97] = acadoVariables.ubValues[97] - acadoVariables.u[97];
acadoWorkspace.ub[98] = acadoVariables.ubValues[98] - acadoVariables.u[98];
acadoWorkspace.ub[99] = acadoVariables.ubValues[99] - acadoVariables.u[99];
acadoWorkspace.ub[100] = acadoVariables.ubValues[100] - acadoVariables.u[100];
acadoWorkspace.ub[101] = acadoVariables.ubValues[101] - acadoVariables.u[101];
acadoWorkspace.ub[102] = acadoVariables.ubValues[102] - acadoVariables.u[102];
acadoWorkspace.ub[103] = acadoVariables.ubValues[103] - acadoVariables.u[103];
acadoWorkspace.ub[104] = acadoVariables.ubValues[104] - acadoVariables.u[104];
acadoWorkspace.ub[105] = acadoVariables.ubValues[105] - acadoVariables.u[105];
acadoWorkspace.ub[106] = acadoVariables.ubValues[106] - acadoVariables.u[106];
acadoWorkspace.ub[107] = acadoVariables.ubValues[107] - acadoVariables.u[107];
acadoWorkspace.ub[108] = acadoVariables.ubValues[108] - acadoVariables.u[108];
acadoWorkspace.ub[109] = acadoVariables.ubValues[109] - acadoVariables.u[109];
acadoWorkspace.ub[110] = acadoVariables.ubValues[110] - acadoVariables.u[110];

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
acadoVariables.u[90] += acadoWorkspace.x[90];
acadoVariables.u[91] += acadoWorkspace.x[91];
acadoVariables.u[92] += acadoWorkspace.x[92];
acadoVariables.u[93] += acadoWorkspace.x[93];
acadoVariables.u[94] += acadoWorkspace.x[94];
acadoVariables.u[95] += acadoWorkspace.x[95];
acadoVariables.u[96] += acadoWorkspace.x[96];
acadoVariables.u[97] += acadoWorkspace.x[97];
acadoVariables.u[98] += acadoWorkspace.x[98];
acadoVariables.u[99] += acadoWorkspace.x[99];
acadoVariables.u[100] += acadoWorkspace.x[100];
acadoVariables.u[101] += acadoWorkspace.x[101];
acadoVariables.u[102] += acadoWorkspace.x[102];
acadoVariables.u[103] += acadoWorkspace.x[103];
acadoVariables.u[104] += acadoWorkspace.x[104];
acadoVariables.u[105] += acadoWorkspace.x[105];
acadoVariables.u[106] += acadoWorkspace.x[106];
acadoVariables.u[107] += acadoWorkspace.x[107];
acadoVariables.u[108] += acadoWorkspace.x[108];
acadoVariables.u[109] += acadoWorkspace.x[109];
acadoVariables.u[110] += acadoWorkspace.x[110];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
for (lRun1 = 0; lRun1 < 333; ++lRun1)
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
acado_expansionStep( &(acadoWorkspace.evGx[ 2430 ]), &(acadoWorkspace.evGu[ 810 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.sbar[ 270 ]), &(acadoWorkspace.sbar[ 279 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2511 ]), &(acadoWorkspace.evGu[ 837 ]), &(acadoWorkspace.x[ 93 ]), &(acadoWorkspace.sbar[ 279 ]), &(acadoWorkspace.sbar[ 288 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.evGu[ 864 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.sbar[ 288 ]), &(acadoWorkspace.sbar[ 297 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2673 ]), &(acadoWorkspace.evGu[ 891 ]), &(acadoWorkspace.x[ 99 ]), &(acadoWorkspace.sbar[ 297 ]), &(acadoWorkspace.sbar[ 306 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2754 ]), &(acadoWorkspace.evGu[ 918 ]), &(acadoWorkspace.x[ 102 ]), &(acadoWorkspace.sbar[ 306 ]), &(acadoWorkspace.sbar[ 315 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2835 ]), &(acadoWorkspace.evGu[ 945 ]), &(acadoWorkspace.x[ 105 ]), &(acadoWorkspace.sbar[ 315 ]), &(acadoWorkspace.sbar[ 324 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.evGu[ 972 ]), &(acadoWorkspace.x[ 108 ]), &(acadoWorkspace.sbar[ 324 ]), &(acadoWorkspace.sbar[ 333 ]) );
for (lRun1 = 0; lRun1 < 342; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

acadoVariables.mu[324] = 0.0000000000000000e+00;
acadoVariables.mu[325] = 0.0000000000000000e+00;
acadoVariables.mu[326] = 0.0000000000000000e+00;
acadoVariables.mu[327] = 0.0000000000000000e+00;
acadoVariables.mu[328] = 0.0000000000000000e+00;
acadoVariables.mu[329] = 0.0000000000000000e+00;
acadoVariables.mu[330] = 0.0000000000000000e+00;
acadoVariables.mu[331] = 0.0000000000000000e+00;
acadoVariables.mu[332] = 0.0000000000000000e+00;
acadoVariables.mu[324] += + acadoWorkspace.sbar[333]*acadoWorkspace.QN1[0] + acadoWorkspace.sbar[334]*acadoWorkspace.QN1[9] + acadoWorkspace.sbar[335]*acadoWorkspace.QN1[18] + acadoWorkspace.sbar[336]*acadoWorkspace.QN1[27] + acadoWorkspace.sbar[337]*acadoWorkspace.QN1[36] + acadoWorkspace.sbar[338]*acadoWorkspace.QN1[45] + acadoWorkspace.sbar[339]*acadoWorkspace.QN1[54] + acadoWorkspace.sbar[340]*acadoWorkspace.QN1[63] + acadoWorkspace.sbar[341]*acadoWorkspace.QN1[72];
acadoVariables.mu[325] += + acadoWorkspace.sbar[333]*acadoWorkspace.QN1[1] + acadoWorkspace.sbar[334]*acadoWorkspace.QN1[10] + acadoWorkspace.sbar[335]*acadoWorkspace.QN1[19] + acadoWorkspace.sbar[336]*acadoWorkspace.QN1[28] + acadoWorkspace.sbar[337]*acadoWorkspace.QN1[37] + acadoWorkspace.sbar[338]*acadoWorkspace.QN1[46] + acadoWorkspace.sbar[339]*acadoWorkspace.QN1[55] + acadoWorkspace.sbar[340]*acadoWorkspace.QN1[64] + acadoWorkspace.sbar[341]*acadoWorkspace.QN1[73];
acadoVariables.mu[326] += + acadoWorkspace.sbar[333]*acadoWorkspace.QN1[2] + acadoWorkspace.sbar[334]*acadoWorkspace.QN1[11] + acadoWorkspace.sbar[335]*acadoWorkspace.QN1[20] + acadoWorkspace.sbar[336]*acadoWorkspace.QN1[29] + acadoWorkspace.sbar[337]*acadoWorkspace.QN1[38] + acadoWorkspace.sbar[338]*acadoWorkspace.QN1[47] + acadoWorkspace.sbar[339]*acadoWorkspace.QN1[56] + acadoWorkspace.sbar[340]*acadoWorkspace.QN1[65] + acadoWorkspace.sbar[341]*acadoWorkspace.QN1[74];
acadoVariables.mu[327] += + acadoWorkspace.sbar[333]*acadoWorkspace.QN1[3] + acadoWorkspace.sbar[334]*acadoWorkspace.QN1[12] + acadoWorkspace.sbar[335]*acadoWorkspace.QN1[21] + acadoWorkspace.sbar[336]*acadoWorkspace.QN1[30] + acadoWorkspace.sbar[337]*acadoWorkspace.QN1[39] + acadoWorkspace.sbar[338]*acadoWorkspace.QN1[48] + acadoWorkspace.sbar[339]*acadoWorkspace.QN1[57] + acadoWorkspace.sbar[340]*acadoWorkspace.QN1[66] + acadoWorkspace.sbar[341]*acadoWorkspace.QN1[75];
acadoVariables.mu[328] += + acadoWorkspace.sbar[333]*acadoWorkspace.QN1[4] + acadoWorkspace.sbar[334]*acadoWorkspace.QN1[13] + acadoWorkspace.sbar[335]*acadoWorkspace.QN1[22] + acadoWorkspace.sbar[336]*acadoWorkspace.QN1[31] + acadoWorkspace.sbar[337]*acadoWorkspace.QN1[40] + acadoWorkspace.sbar[338]*acadoWorkspace.QN1[49] + acadoWorkspace.sbar[339]*acadoWorkspace.QN1[58] + acadoWorkspace.sbar[340]*acadoWorkspace.QN1[67] + acadoWorkspace.sbar[341]*acadoWorkspace.QN1[76];
acadoVariables.mu[329] += + acadoWorkspace.sbar[333]*acadoWorkspace.QN1[5] + acadoWorkspace.sbar[334]*acadoWorkspace.QN1[14] + acadoWorkspace.sbar[335]*acadoWorkspace.QN1[23] + acadoWorkspace.sbar[336]*acadoWorkspace.QN1[32] + acadoWorkspace.sbar[337]*acadoWorkspace.QN1[41] + acadoWorkspace.sbar[338]*acadoWorkspace.QN1[50] + acadoWorkspace.sbar[339]*acadoWorkspace.QN1[59] + acadoWorkspace.sbar[340]*acadoWorkspace.QN1[68] + acadoWorkspace.sbar[341]*acadoWorkspace.QN1[77];
acadoVariables.mu[330] += + acadoWorkspace.sbar[333]*acadoWorkspace.QN1[6] + acadoWorkspace.sbar[334]*acadoWorkspace.QN1[15] + acadoWorkspace.sbar[335]*acadoWorkspace.QN1[24] + acadoWorkspace.sbar[336]*acadoWorkspace.QN1[33] + acadoWorkspace.sbar[337]*acadoWorkspace.QN1[42] + acadoWorkspace.sbar[338]*acadoWorkspace.QN1[51] + acadoWorkspace.sbar[339]*acadoWorkspace.QN1[60] + acadoWorkspace.sbar[340]*acadoWorkspace.QN1[69] + acadoWorkspace.sbar[341]*acadoWorkspace.QN1[78];
acadoVariables.mu[331] += + acadoWorkspace.sbar[333]*acadoWorkspace.QN1[7] + acadoWorkspace.sbar[334]*acadoWorkspace.QN1[16] + acadoWorkspace.sbar[335]*acadoWorkspace.QN1[25] + acadoWorkspace.sbar[336]*acadoWorkspace.QN1[34] + acadoWorkspace.sbar[337]*acadoWorkspace.QN1[43] + acadoWorkspace.sbar[338]*acadoWorkspace.QN1[52] + acadoWorkspace.sbar[339]*acadoWorkspace.QN1[61] + acadoWorkspace.sbar[340]*acadoWorkspace.QN1[70] + acadoWorkspace.sbar[341]*acadoWorkspace.QN1[79];
acadoVariables.mu[332] += + acadoWorkspace.sbar[333]*acadoWorkspace.QN1[8] + acadoWorkspace.sbar[334]*acadoWorkspace.QN1[17] + acadoWorkspace.sbar[335]*acadoWorkspace.QN1[26] + acadoWorkspace.sbar[336]*acadoWorkspace.QN1[35] + acadoWorkspace.sbar[337]*acadoWorkspace.QN1[44] + acadoWorkspace.sbar[338]*acadoWorkspace.QN1[53] + acadoWorkspace.sbar[339]*acadoWorkspace.QN1[62] + acadoWorkspace.sbar[340]*acadoWorkspace.QN1[71] + acadoWorkspace.sbar[341]*acadoWorkspace.QN1[80];
acadoVariables.mu[324] += acadoWorkspace.QDy[333];
acadoVariables.mu[325] += acadoWorkspace.QDy[334];
acadoVariables.mu[326] += acadoWorkspace.QDy[335];
acadoVariables.mu[327] += acadoWorkspace.QDy[336];
acadoVariables.mu[328] += acadoWorkspace.QDy[337];
acadoVariables.mu[329] += acadoWorkspace.QDy[338];
acadoVariables.mu[330] += acadoWorkspace.QDy[339];
acadoVariables.mu[331] += acadoWorkspace.QDy[340];
acadoVariables.mu[332] += acadoWorkspace.QDy[341];
acadoVariables.mu[315] = 0.0000000000000000e+00;
acadoVariables.mu[316] = 0.0000000000000000e+00;
acadoVariables.mu[317] = 0.0000000000000000e+00;
acadoVariables.mu[318] = 0.0000000000000000e+00;
acadoVariables.mu[319] = 0.0000000000000000e+00;
acadoVariables.mu[320] = 0.0000000000000000e+00;
acadoVariables.mu[321] = 0.0000000000000000e+00;
acadoVariables.mu[322] = 0.0000000000000000e+00;
acadoVariables.mu[323] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 324 ]), &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.sbar[ 324 ]), &(acadoWorkspace.S1[ 972 ]), &(acadoWorkspace.x[ 108 ]), &(acadoWorkspace.evGx[ 2916 ]), &(acadoVariables.mu[ 315 ]), &(acadoVariables.mu[ 324 ]) );
acadoVariables.mu[306] = 0.0000000000000000e+00;
acadoVariables.mu[307] = 0.0000000000000000e+00;
acadoVariables.mu[308] = 0.0000000000000000e+00;
acadoVariables.mu[309] = 0.0000000000000000e+00;
acadoVariables.mu[310] = 0.0000000000000000e+00;
acadoVariables.mu[311] = 0.0000000000000000e+00;
acadoVariables.mu[312] = 0.0000000000000000e+00;
acadoVariables.mu[313] = 0.0000000000000000e+00;
acadoVariables.mu[314] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 315 ]), &(acadoWorkspace.Q1[ 2835 ]), &(acadoWorkspace.sbar[ 315 ]), &(acadoWorkspace.S1[ 945 ]), &(acadoWorkspace.x[ 105 ]), &(acadoWorkspace.evGx[ 2835 ]), &(acadoVariables.mu[ 306 ]), &(acadoVariables.mu[ 315 ]) );
acadoVariables.mu[297] = 0.0000000000000000e+00;
acadoVariables.mu[298] = 0.0000000000000000e+00;
acadoVariables.mu[299] = 0.0000000000000000e+00;
acadoVariables.mu[300] = 0.0000000000000000e+00;
acadoVariables.mu[301] = 0.0000000000000000e+00;
acadoVariables.mu[302] = 0.0000000000000000e+00;
acadoVariables.mu[303] = 0.0000000000000000e+00;
acadoVariables.mu[304] = 0.0000000000000000e+00;
acadoVariables.mu[305] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 306 ]), &(acadoWorkspace.Q1[ 2754 ]), &(acadoWorkspace.sbar[ 306 ]), &(acadoWorkspace.S1[ 918 ]), &(acadoWorkspace.x[ 102 ]), &(acadoWorkspace.evGx[ 2754 ]), &(acadoVariables.mu[ 297 ]), &(acadoVariables.mu[ 306 ]) );
acadoVariables.mu[288] = 0.0000000000000000e+00;
acadoVariables.mu[289] = 0.0000000000000000e+00;
acadoVariables.mu[290] = 0.0000000000000000e+00;
acadoVariables.mu[291] = 0.0000000000000000e+00;
acadoVariables.mu[292] = 0.0000000000000000e+00;
acadoVariables.mu[293] = 0.0000000000000000e+00;
acadoVariables.mu[294] = 0.0000000000000000e+00;
acadoVariables.mu[295] = 0.0000000000000000e+00;
acadoVariables.mu[296] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 297 ]), &(acadoWorkspace.Q1[ 2673 ]), &(acadoWorkspace.sbar[ 297 ]), &(acadoWorkspace.S1[ 891 ]), &(acadoWorkspace.x[ 99 ]), &(acadoWorkspace.evGx[ 2673 ]), &(acadoVariables.mu[ 288 ]), &(acadoVariables.mu[ 297 ]) );
acadoVariables.mu[279] = 0.0000000000000000e+00;
acadoVariables.mu[280] = 0.0000000000000000e+00;
acadoVariables.mu[281] = 0.0000000000000000e+00;
acadoVariables.mu[282] = 0.0000000000000000e+00;
acadoVariables.mu[283] = 0.0000000000000000e+00;
acadoVariables.mu[284] = 0.0000000000000000e+00;
acadoVariables.mu[285] = 0.0000000000000000e+00;
acadoVariables.mu[286] = 0.0000000000000000e+00;
acadoVariables.mu[287] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 288 ]), &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.sbar[ 288 ]), &(acadoWorkspace.S1[ 864 ]), &(acadoWorkspace.x[ 96 ]), &(acadoWorkspace.evGx[ 2592 ]), &(acadoVariables.mu[ 279 ]), &(acadoVariables.mu[ 288 ]) );
acadoVariables.mu[270] = 0.0000000000000000e+00;
acadoVariables.mu[271] = 0.0000000000000000e+00;
acadoVariables.mu[272] = 0.0000000000000000e+00;
acadoVariables.mu[273] = 0.0000000000000000e+00;
acadoVariables.mu[274] = 0.0000000000000000e+00;
acadoVariables.mu[275] = 0.0000000000000000e+00;
acadoVariables.mu[276] = 0.0000000000000000e+00;
acadoVariables.mu[277] = 0.0000000000000000e+00;
acadoVariables.mu[278] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 279 ]), &(acadoWorkspace.Q1[ 2511 ]), &(acadoWorkspace.sbar[ 279 ]), &(acadoWorkspace.S1[ 837 ]), &(acadoWorkspace.x[ 93 ]), &(acadoWorkspace.evGx[ 2511 ]), &(acadoVariables.mu[ 270 ]), &(acadoVariables.mu[ 279 ]) );
acadoVariables.mu[261] = 0.0000000000000000e+00;
acadoVariables.mu[262] = 0.0000000000000000e+00;
acadoVariables.mu[263] = 0.0000000000000000e+00;
acadoVariables.mu[264] = 0.0000000000000000e+00;
acadoVariables.mu[265] = 0.0000000000000000e+00;
acadoVariables.mu[266] = 0.0000000000000000e+00;
acadoVariables.mu[267] = 0.0000000000000000e+00;
acadoVariables.mu[268] = 0.0000000000000000e+00;
acadoVariables.mu[269] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 270 ]), &(acadoWorkspace.Q1[ 2430 ]), &(acadoWorkspace.sbar[ 270 ]), &(acadoWorkspace.S1[ 810 ]), &(acadoWorkspace.x[ 90 ]), &(acadoWorkspace.evGx[ 2430 ]), &(acadoVariables.mu[ 261 ]), &(acadoVariables.mu[ 270 ]) );
acadoVariables.mu[252] = 0.0000000000000000e+00;
acadoVariables.mu[253] = 0.0000000000000000e+00;
acadoVariables.mu[254] = 0.0000000000000000e+00;
acadoVariables.mu[255] = 0.0000000000000000e+00;
acadoVariables.mu[256] = 0.0000000000000000e+00;
acadoVariables.mu[257] = 0.0000000000000000e+00;
acadoVariables.mu[258] = 0.0000000000000000e+00;
acadoVariables.mu[259] = 0.0000000000000000e+00;
acadoVariables.mu[260] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 261 ]), &(acadoWorkspace.Q1[ 2349 ]), &(acadoWorkspace.sbar[ 261 ]), &(acadoWorkspace.S1[ 783 ]), &(acadoWorkspace.x[ 87 ]), &(acadoWorkspace.evGx[ 2349 ]), &(acadoVariables.mu[ 252 ]), &(acadoVariables.mu[ 261 ]) );
acadoVariables.mu[243] = 0.0000000000000000e+00;
acadoVariables.mu[244] = 0.0000000000000000e+00;
acadoVariables.mu[245] = 0.0000000000000000e+00;
acadoVariables.mu[246] = 0.0000000000000000e+00;
acadoVariables.mu[247] = 0.0000000000000000e+00;
acadoVariables.mu[248] = 0.0000000000000000e+00;
acadoVariables.mu[249] = 0.0000000000000000e+00;
acadoVariables.mu[250] = 0.0000000000000000e+00;
acadoVariables.mu[251] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 252 ]), &(acadoWorkspace.Q1[ 2268 ]), &(acadoWorkspace.sbar[ 252 ]), &(acadoWorkspace.S1[ 756 ]), &(acadoWorkspace.x[ 84 ]), &(acadoWorkspace.evGx[ 2268 ]), &(acadoVariables.mu[ 243 ]), &(acadoVariables.mu[ 252 ]) );
acadoVariables.mu[234] = 0.0000000000000000e+00;
acadoVariables.mu[235] = 0.0000000000000000e+00;
acadoVariables.mu[236] = 0.0000000000000000e+00;
acadoVariables.mu[237] = 0.0000000000000000e+00;
acadoVariables.mu[238] = 0.0000000000000000e+00;
acadoVariables.mu[239] = 0.0000000000000000e+00;
acadoVariables.mu[240] = 0.0000000000000000e+00;
acadoVariables.mu[241] = 0.0000000000000000e+00;
acadoVariables.mu[242] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 243 ]), &(acadoWorkspace.Q1[ 2187 ]), &(acadoWorkspace.sbar[ 243 ]), &(acadoWorkspace.S1[ 729 ]), &(acadoWorkspace.x[ 81 ]), &(acadoWorkspace.evGx[ 2187 ]), &(acadoVariables.mu[ 234 ]), &(acadoVariables.mu[ 243 ]) );
acadoVariables.mu[225] = 0.0000000000000000e+00;
acadoVariables.mu[226] = 0.0000000000000000e+00;
acadoVariables.mu[227] = 0.0000000000000000e+00;
acadoVariables.mu[228] = 0.0000000000000000e+00;
acadoVariables.mu[229] = 0.0000000000000000e+00;
acadoVariables.mu[230] = 0.0000000000000000e+00;
acadoVariables.mu[231] = 0.0000000000000000e+00;
acadoVariables.mu[232] = 0.0000000000000000e+00;
acadoVariables.mu[233] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 234 ]), &(acadoWorkspace.Q1[ 2106 ]), &(acadoWorkspace.sbar[ 234 ]), &(acadoWorkspace.S1[ 702 ]), &(acadoWorkspace.x[ 78 ]), &(acadoWorkspace.evGx[ 2106 ]), &(acadoVariables.mu[ 225 ]), &(acadoVariables.mu[ 234 ]) );
acadoVariables.mu[216] = 0.0000000000000000e+00;
acadoVariables.mu[217] = 0.0000000000000000e+00;
acadoVariables.mu[218] = 0.0000000000000000e+00;
acadoVariables.mu[219] = 0.0000000000000000e+00;
acadoVariables.mu[220] = 0.0000000000000000e+00;
acadoVariables.mu[221] = 0.0000000000000000e+00;
acadoVariables.mu[222] = 0.0000000000000000e+00;
acadoVariables.mu[223] = 0.0000000000000000e+00;
acadoVariables.mu[224] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 225 ]), &(acadoWorkspace.Q1[ 2025 ]), &(acadoWorkspace.sbar[ 225 ]), &(acadoWorkspace.S1[ 675 ]), &(acadoWorkspace.x[ 75 ]), &(acadoWorkspace.evGx[ 2025 ]), &(acadoVariables.mu[ 216 ]), &(acadoVariables.mu[ 225 ]) );
acadoVariables.mu[207] = 0.0000000000000000e+00;
acadoVariables.mu[208] = 0.0000000000000000e+00;
acadoVariables.mu[209] = 0.0000000000000000e+00;
acadoVariables.mu[210] = 0.0000000000000000e+00;
acadoVariables.mu[211] = 0.0000000000000000e+00;
acadoVariables.mu[212] = 0.0000000000000000e+00;
acadoVariables.mu[213] = 0.0000000000000000e+00;
acadoVariables.mu[214] = 0.0000000000000000e+00;
acadoVariables.mu[215] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 216 ]), &(acadoWorkspace.Q1[ 1944 ]), &(acadoWorkspace.sbar[ 216 ]), &(acadoWorkspace.S1[ 648 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.evGx[ 1944 ]), &(acadoVariables.mu[ 207 ]), &(acadoVariables.mu[ 216 ]) );
acadoVariables.mu[198] = 0.0000000000000000e+00;
acadoVariables.mu[199] = 0.0000000000000000e+00;
acadoVariables.mu[200] = 0.0000000000000000e+00;
acadoVariables.mu[201] = 0.0000000000000000e+00;
acadoVariables.mu[202] = 0.0000000000000000e+00;
acadoVariables.mu[203] = 0.0000000000000000e+00;
acadoVariables.mu[204] = 0.0000000000000000e+00;
acadoVariables.mu[205] = 0.0000000000000000e+00;
acadoVariables.mu[206] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 207 ]), &(acadoWorkspace.Q1[ 1863 ]), &(acadoWorkspace.sbar[ 207 ]), &(acadoWorkspace.S1[ 621 ]), &(acadoWorkspace.x[ 69 ]), &(acadoWorkspace.evGx[ 1863 ]), &(acadoVariables.mu[ 198 ]), &(acadoVariables.mu[ 207 ]) );
acadoVariables.mu[189] = 0.0000000000000000e+00;
acadoVariables.mu[190] = 0.0000000000000000e+00;
acadoVariables.mu[191] = 0.0000000000000000e+00;
acadoVariables.mu[192] = 0.0000000000000000e+00;
acadoVariables.mu[193] = 0.0000000000000000e+00;
acadoVariables.mu[194] = 0.0000000000000000e+00;
acadoVariables.mu[195] = 0.0000000000000000e+00;
acadoVariables.mu[196] = 0.0000000000000000e+00;
acadoVariables.mu[197] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 198 ]), &(acadoWorkspace.Q1[ 1782 ]), &(acadoWorkspace.sbar[ 198 ]), &(acadoWorkspace.S1[ 594 ]), &(acadoWorkspace.x[ 66 ]), &(acadoWorkspace.evGx[ 1782 ]), &(acadoVariables.mu[ 189 ]), &(acadoVariables.mu[ 198 ]) );
acadoVariables.mu[180] = 0.0000000000000000e+00;
acadoVariables.mu[181] = 0.0000000000000000e+00;
acadoVariables.mu[182] = 0.0000000000000000e+00;
acadoVariables.mu[183] = 0.0000000000000000e+00;
acadoVariables.mu[184] = 0.0000000000000000e+00;
acadoVariables.mu[185] = 0.0000000000000000e+00;
acadoVariables.mu[186] = 0.0000000000000000e+00;
acadoVariables.mu[187] = 0.0000000000000000e+00;
acadoVariables.mu[188] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 189 ]), &(acadoWorkspace.Q1[ 1701 ]), &(acadoWorkspace.sbar[ 189 ]), &(acadoWorkspace.S1[ 567 ]), &(acadoWorkspace.x[ 63 ]), &(acadoWorkspace.evGx[ 1701 ]), &(acadoVariables.mu[ 180 ]), &(acadoVariables.mu[ 189 ]) );
acadoVariables.mu[171] = 0.0000000000000000e+00;
acadoVariables.mu[172] = 0.0000000000000000e+00;
acadoVariables.mu[173] = 0.0000000000000000e+00;
acadoVariables.mu[174] = 0.0000000000000000e+00;
acadoVariables.mu[175] = 0.0000000000000000e+00;
acadoVariables.mu[176] = 0.0000000000000000e+00;
acadoVariables.mu[177] = 0.0000000000000000e+00;
acadoVariables.mu[178] = 0.0000000000000000e+00;
acadoVariables.mu[179] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 180 ]), &(acadoWorkspace.Q1[ 1620 ]), &(acadoWorkspace.sbar[ 180 ]), &(acadoWorkspace.S1[ 540 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.evGx[ 1620 ]), &(acadoVariables.mu[ 171 ]), &(acadoVariables.mu[ 180 ]) );
acadoVariables.mu[162] = 0.0000000000000000e+00;
acadoVariables.mu[163] = 0.0000000000000000e+00;
acadoVariables.mu[164] = 0.0000000000000000e+00;
acadoVariables.mu[165] = 0.0000000000000000e+00;
acadoVariables.mu[166] = 0.0000000000000000e+00;
acadoVariables.mu[167] = 0.0000000000000000e+00;
acadoVariables.mu[168] = 0.0000000000000000e+00;
acadoVariables.mu[169] = 0.0000000000000000e+00;
acadoVariables.mu[170] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 171 ]), &(acadoWorkspace.Q1[ 1539 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.S1[ 513 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.evGx[ 1539 ]), &(acadoVariables.mu[ 162 ]), &(acadoVariables.mu[ 171 ]) );
acadoVariables.mu[153] = 0.0000000000000000e+00;
acadoVariables.mu[154] = 0.0000000000000000e+00;
acadoVariables.mu[155] = 0.0000000000000000e+00;
acadoVariables.mu[156] = 0.0000000000000000e+00;
acadoVariables.mu[157] = 0.0000000000000000e+00;
acadoVariables.mu[158] = 0.0000000000000000e+00;
acadoVariables.mu[159] = 0.0000000000000000e+00;
acadoVariables.mu[160] = 0.0000000000000000e+00;
acadoVariables.mu[161] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 162 ]), &(acadoWorkspace.Q1[ 1458 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.S1[ 486 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.evGx[ 1458 ]), &(acadoVariables.mu[ 153 ]), &(acadoVariables.mu[ 162 ]) );
acadoVariables.mu[144] = 0.0000000000000000e+00;
acadoVariables.mu[145] = 0.0000000000000000e+00;
acadoVariables.mu[146] = 0.0000000000000000e+00;
acadoVariables.mu[147] = 0.0000000000000000e+00;
acadoVariables.mu[148] = 0.0000000000000000e+00;
acadoVariables.mu[149] = 0.0000000000000000e+00;
acadoVariables.mu[150] = 0.0000000000000000e+00;
acadoVariables.mu[151] = 0.0000000000000000e+00;
acadoVariables.mu[152] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 153 ]), &(acadoWorkspace.Q1[ 1377 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.S1[ 459 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.evGx[ 1377 ]), &(acadoVariables.mu[ 144 ]), &(acadoVariables.mu[ 153 ]) );
acadoVariables.mu[135] = 0.0000000000000000e+00;
acadoVariables.mu[136] = 0.0000000000000000e+00;
acadoVariables.mu[137] = 0.0000000000000000e+00;
acadoVariables.mu[138] = 0.0000000000000000e+00;
acadoVariables.mu[139] = 0.0000000000000000e+00;
acadoVariables.mu[140] = 0.0000000000000000e+00;
acadoVariables.mu[141] = 0.0000000000000000e+00;
acadoVariables.mu[142] = 0.0000000000000000e+00;
acadoVariables.mu[143] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 144 ]), &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.S1[ 432 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.evGx[ 1296 ]), &(acadoVariables.mu[ 135 ]), &(acadoVariables.mu[ 144 ]) );
acadoVariables.mu[126] = 0.0000000000000000e+00;
acadoVariables.mu[127] = 0.0000000000000000e+00;
acadoVariables.mu[128] = 0.0000000000000000e+00;
acadoVariables.mu[129] = 0.0000000000000000e+00;
acadoVariables.mu[130] = 0.0000000000000000e+00;
acadoVariables.mu[131] = 0.0000000000000000e+00;
acadoVariables.mu[132] = 0.0000000000000000e+00;
acadoVariables.mu[133] = 0.0000000000000000e+00;
acadoVariables.mu[134] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 135 ]), &(acadoWorkspace.Q1[ 1215 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.S1[ 405 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.evGx[ 1215 ]), &(acadoVariables.mu[ 126 ]), &(acadoVariables.mu[ 135 ]) );
acadoVariables.mu[117] = 0.0000000000000000e+00;
acadoVariables.mu[118] = 0.0000000000000000e+00;
acadoVariables.mu[119] = 0.0000000000000000e+00;
acadoVariables.mu[120] = 0.0000000000000000e+00;
acadoVariables.mu[121] = 0.0000000000000000e+00;
acadoVariables.mu[122] = 0.0000000000000000e+00;
acadoVariables.mu[123] = 0.0000000000000000e+00;
acadoVariables.mu[124] = 0.0000000000000000e+00;
acadoVariables.mu[125] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 126 ]), &(acadoWorkspace.Q1[ 1134 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.S1[ 378 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.evGx[ 1134 ]), &(acadoVariables.mu[ 117 ]), &(acadoVariables.mu[ 126 ]) );
acadoVariables.mu[108] = 0.0000000000000000e+00;
acadoVariables.mu[109] = 0.0000000000000000e+00;
acadoVariables.mu[110] = 0.0000000000000000e+00;
acadoVariables.mu[111] = 0.0000000000000000e+00;
acadoVariables.mu[112] = 0.0000000000000000e+00;
acadoVariables.mu[113] = 0.0000000000000000e+00;
acadoVariables.mu[114] = 0.0000000000000000e+00;
acadoVariables.mu[115] = 0.0000000000000000e+00;
acadoVariables.mu[116] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 117 ]), &(acadoWorkspace.Q1[ 1053 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.S1[ 351 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.evGx[ 1053 ]), &(acadoVariables.mu[ 108 ]), &(acadoVariables.mu[ 117 ]) );
acadoVariables.mu[99] = 0.0000000000000000e+00;
acadoVariables.mu[100] = 0.0000000000000000e+00;
acadoVariables.mu[101] = 0.0000000000000000e+00;
acadoVariables.mu[102] = 0.0000000000000000e+00;
acadoVariables.mu[103] = 0.0000000000000000e+00;
acadoVariables.mu[104] = 0.0000000000000000e+00;
acadoVariables.mu[105] = 0.0000000000000000e+00;
acadoVariables.mu[106] = 0.0000000000000000e+00;
acadoVariables.mu[107] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 108 ]), &(acadoWorkspace.Q1[ 972 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.S1[ 324 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.evGx[ 972 ]), &(acadoVariables.mu[ 99 ]), &(acadoVariables.mu[ 108 ]) );
acadoVariables.mu[90] = 0.0000000000000000e+00;
acadoVariables.mu[91] = 0.0000000000000000e+00;
acadoVariables.mu[92] = 0.0000000000000000e+00;
acadoVariables.mu[93] = 0.0000000000000000e+00;
acadoVariables.mu[94] = 0.0000000000000000e+00;
acadoVariables.mu[95] = 0.0000000000000000e+00;
acadoVariables.mu[96] = 0.0000000000000000e+00;
acadoVariables.mu[97] = 0.0000000000000000e+00;
acadoVariables.mu[98] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 99 ]), &(acadoWorkspace.Q1[ 891 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.S1[ 297 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.evGx[ 891 ]), &(acadoVariables.mu[ 90 ]), &(acadoVariables.mu[ 99 ]) );
acadoVariables.mu[81] = 0.0000000000000000e+00;
acadoVariables.mu[82] = 0.0000000000000000e+00;
acadoVariables.mu[83] = 0.0000000000000000e+00;
acadoVariables.mu[84] = 0.0000000000000000e+00;
acadoVariables.mu[85] = 0.0000000000000000e+00;
acadoVariables.mu[86] = 0.0000000000000000e+00;
acadoVariables.mu[87] = 0.0000000000000000e+00;
acadoVariables.mu[88] = 0.0000000000000000e+00;
acadoVariables.mu[89] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.Q1[ 810 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.S1[ 270 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.evGx[ 810 ]), &(acadoVariables.mu[ 81 ]), &(acadoVariables.mu[ 90 ]) );
acadoVariables.mu[72] = 0.0000000000000000e+00;
acadoVariables.mu[73] = 0.0000000000000000e+00;
acadoVariables.mu[74] = 0.0000000000000000e+00;
acadoVariables.mu[75] = 0.0000000000000000e+00;
acadoVariables.mu[76] = 0.0000000000000000e+00;
acadoVariables.mu[77] = 0.0000000000000000e+00;
acadoVariables.mu[78] = 0.0000000000000000e+00;
acadoVariables.mu[79] = 0.0000000000000000e+00;
acadoVariables.mu[80] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 81 ]), &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.S1[ 243 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.evGx[ 729 ]), &(acadoVariables.mu[ 72 ]), &(acadoVariables.mu[ 81 ]) );
acadoVariables.mu[63] = 0.0000000000000000e+00;
acadoVariables.mu[64] = 0.0000000000000000e+00;
acadoVariables.mu[65] = 0.0000000000000000e+00;
acadoVariables.mu[66] = 0.0000000000000000e+00;
acadoVariables.mu[67] = 0.0000000000000000e+00;
acadoVariables.mu[68] = 0.0000000000000000e+00;
acadoVariables.mu[69] = 0.0000000000000000e+00;
acadoVariables.mu[70] = 0.0000000000000000e+00;
acadoVariables.mu[71] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.S1[ 216 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoVariables.mu[ 63 ]), &(acadoVariables.mu[ 72 ]) );
acadoVariables.mu[54] = 0.0000000000000000e+00;
acadoVariables.mu[55] = 0.0000000000000000e+00;
acadoVariables.mu[56] = 0.0000000000000000e+00;
acadoVariables.mu[57] = 0.0000000000000000e+00;
acadoVariables.mu[58] = 0.0000000000000000e+00;
acadoVariables.mu[59] = 0.0000000000000000e+00;
acadoVariables.mu[60] = 0.0000000000000000e+00;
acadoVariables.mu[61] = 0.0000000000000000e+00;
acadoVariables.mu[62] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 63 ]), &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.S1[ 189 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.evGx[ 567 ]), &(acadoVariables.mu[ 54 ]), &(acadoVariables.mu[ 63 ]) );
acadoVariables.mu[45] = 0.0000000000000000e+00;
acadoVariables.mu[46] = 0.0000000000000000e+00;
acadoVariables.mu[47] = 0.0000000000000000e+00;
acadoVariables.mu[48] = 0.0000000000000000e+00;
acadoVariables.mu[49] = 0.0000000000000000e+00;
acadoVariables.mu[50] = 0.0000000000000000e+00;
acadoVariables.mu[51] = 0.0000000000000000e+00;
acadoVariables.mu[52] = 0.0000000000000000e+00;
acadoVariables.mu[53] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.S1[ 162 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.evGx[ 486 ]), &(acadoVariables.mu[ 45 ]), &(acadoVariables.mu[ 54 ]) );
acadoVariables.mu[36] = 0.0000000000000000e+00;
acadoVariables.mu[37] = 0.0000000000000000e+00;
acadoVariables.mu[38] = 0.0000000000000000e+00;
acadoVariables.mu[39] = 0.0000000000000000e+00;
acadoVariables.mu[40] = 0.0000000000000000e+00;
acadoVariables.mu[41] = 0.0000000000000000e+00;
acadoVariables.mu[42] = 0.0000000000000000e+00;
acadoVariables.mu[43] = 0.0000000000000000e+00;
acadoVariables.mu[44] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.S1[ 135 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.evGx[ 405 ]), &(acadoVariables.mu[ 36 ]), &(acadoVariables.mu[ 45 ]) );
acadoVariables.mu[27] = 0.0000000000000000e+00;
acadoVariables.mu[28] = 0.0000000000000000e+00;
acadoVariables.mu[29] = 0.0000000000000000e+00;
acadoVariables.mu[30] = 0.0000000000000000e+00;
acadoVariables.mu[31] = 0.0000000000000000e+00;
acadoVariables.mu[32] = 0.0000000000000000e+00;
acadoVariables.mu[33] = 0.0000000000000000e+00;
acadoVariables.mu[34] = 0.0000000000000000e+00;
acadoVariables.mu[35] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.S1[ 108 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoVariables.mu[ 27 ]), &(acadoVariables.mu[ 36 ]) );
acadoVariables.mu[18] = 0.0000000000000000e+00;
acadoVariables.mu[19] = 0.0000000000000000e+00;
acadoVariables.mu[20] = 0.0000000000000000e+00;
acadoVariables.mu[21] = 0.0000000000000000e+00;
acadoVariables.mu[22] = 0.0000000000000000e+00;
acadoVariables.mu[23] = 0.0000000000000000e+00;
acadoVariables.mu[24] = 0.0000000000000000e+00;
acadoVariables.mu[25] = 0.0000000000000000e+00;
acadoVariables.mu[26] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.S1[ 81 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.evGx[ 243 ]), &(acadoVariables.mu[ 18 ]), &(acadoVariables.mu[ 27 ]) );
acadoVariables.mu[9] = 0.0000000000000000e+00;
acadoVariables.mu[10] = 0.0000000000000000e+00;
acadoVariables.mu[11] = 0.0000000000000000e+00;
acadoVariables.mu[12] = 0.0000000000000000e+00;
acadoVariables.mu[13] = 0.0000000000000000e+00;
acadoVariables.mu[14] = 0.0000000000000000e+00;
acadoVariables.mu[15] = 0.0000000000000000e+00;
acadoVariables.mu[16] = 0.0000000000000000e+00;
acadoVariables.mu[17] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.S1[ 54 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoVariables.mu[ 9 ]), &(acadoVariables.mu[ 18 ]) );
acadoVariables.mu[0] = 0.0000000000000000e+00;
acadoVariables.mu[1] = 0.0000000000000000e+00;
acadoVariables.mu[2] = 0.0000000000000000e+00;
acadoVariables.mu[3] = 0.0000000000000000e+00;
acadoVariables.mu[4] = 0.0000000000000000e+00;
acadoVariables.mu[5] = 0.0000000000000000e+00;
acadoVariables.mu[6] = 0.0000000000000000e+00;
acadoVariables.mu[7] = 0.0000000000000000e+00;
acadoVariables.mu[8] = 0.0000000000000000e+00;
acado_expansionStep2( &(acadoWorkspace.QDy[ 9 ]), &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.S1[ 27 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.evGx[ 81 ]), acadoVariables.mu, &(acadoVariables.mu[ 9 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_regularizeHessian(  );
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
acadoVariables.lbValues[0] = -8.7266467511653900e-02;
acadoVariables.lbValues[1] = -8.7266467511653900e-02;
acadoVariables.lbValues[2] = -1.0000000000000000e+00;
acadoVariables.lbValues[3] = -8.7266467511653900e-02;
acadoVariables.lbValues[4] = -8.7266467511653900e-02;
acadoVariables.lbValues[5] = -1.0000000000000000e+00;
acadoVariables.lbValues[6] = -8.7266467511653900e-02;
acadoVariables.lbValues[7] = -8.7266467511653900e-02;
acadoVariables.lbValues[8] = -1.0000000000000000e+00;
acadoVariables.lbValues[9] = -8.7266467511653900e-02;
acadoVariables.lbValues[10] = -8.7266467511653900e-02;
acadoVariables.lbValues[11] = -1.0000000000000000e+00;
acadoVariables.lbValues[12] = -8.7266467511653900e-02;
acadoVariables.lbValues[13] = -8.7266467511653900e-02;
acadoVariables.lbValues[14] = -1.0000000000000000e+00;
acadoVariables.lbValues[15] = -8.7266467511653900e-02;
acadoVariables.lbValues[16] = -8.7266467511653900e-02;
acadoVariables.lbValues[17] = -1.0000000000000000e+00;
acadoVariables.lbValues[18] = -8.7266467511653900e-02;
acadoVariables.lbValues[19] = -8.7266467511653900e-02;
acadoVariables.lbValues[20] = -1.0000000000000000e+00;
acadoVariables.lbValues[21] = -8.7266467511653900e-02;
acadoVariables.lbValues[22] = -8.7266467511653900e-02;
acadoVariables.lbValues[23] = -1.0000000000000000e+00;
acadoVariables.lbValues[24] = -8.7266467511653900e-02;
acadoVariables.lbValues[25] = -8.7266467511653900e-02;
acadoVariables.lbValues[26] = -1.0000000000000000e+00;
acadoVariables.lbValues[27] = -8.7266467511653900e-02;
acadoVariables.lbValues[28] = -8.7266467511653900e-02;
acadoVariables.lbValues[29] = -1.0000000000000000e+00;
acadoVariables.lbValues[30] = -8.7266467511653900e-02;
acadoVariables.lbValues[31] = -8.7266467511653900e-02;
acadoVariables.lbValues[32] = -1.0000000000000000e+00;
acadoVariables.lbValues[33] = -8.7266467511653900e-02;
acadoVariables.lbValues[34] = -8.7266467511653900e-02;
acadoVariables.lbValues[35] = -1.0000000000000000e+00;
acadoVariables.lbValues[36] = -8.7266467511653900e-02;
acadoVariables.lbValues[37] = -8.7266467511653900e-02;
acadoVariables.lbValues[38] = -1.0000000000000000e+00;
acadoVariables.lbValues[39] = -8.7266467511653900e-02;
acadoVariables.lbValues[40] = -8.7266467511653900e-02;
acadoVariables.lbValues[41] = -1.0000000000000000e+00;
acadoVariables.lbValues[42] = -8.7266467511653900e-02;
acadoVariables.lbValues[43] = -8.7266467511653900e-02;
acadoVariables.lbValues[44] = -1.0000000000000000e+00;
acadoVariables.lbValues[45] = -8.7266467511653900e-02;
acadoVariables.lbValues[46] = -8.7266467511653900e-02;
acadoVariables.lbValues[47] = -1.0000000000000000e+00;
acadoVariables.lbValues[48] = -8.7266467511653900e-02;
acadoVariables.lbValues[49] = -8.7266467511653900e-02;
acadoVariables.lbValues[50] = -1.0000000000000000e+00;
acadoVariables.lbValues[51] = -8.7266467511653900e-02;
acadoVariables.lbValues[52] = -8.7266467511653900e-02;
acadoVariables.lbValues[53] = -1.0000000000000000e+00;
acadoVariables.lbValues[54] = -8.7266467511653900e-02;
acadoVariables.lbValues[55] = -8.7266467511653900e-02;
acadoVariables.lbValues[56] = -1.0000000000000000e+00;
acadoVariables.lbValues[57] = -8.7266467511653900e-02;
acadoVariables.lbValues[58] = -8.7266467511653900e-02;
acadoVariables.lbValues[59] = -1.0000000000000000e+00;
acadoVariables.lbValues[60] = -8.7266467511653900e-02;
acadoVariables.lbValues[61] = -8.7266467511653900e-02;
acadoVariables.lbValues[62] = -1.0000000000000000e+00;
acadoVariables.lbValues[63] = -8.7266467511653900e-02;
acadoVariables.lbValues[64] = -8.7266467511653900e-02;
acadoVariables.lbValues[65] = -1.0000000000000000e+00;
acadoVariables.lbValues[66] = -8.7266467511653900e-02;
acadoVariables.lbValues[67] = -8.7266467511653900e-02;
acadoVariables.lbValues[68] = -1.0000000000000000e+00;
acadoVariables.lbValues[69] = -8.7266467511653900e-02;
acadoVariables.lbValues[70] = -8.7266467511653900e-02;
acadoVariables.lbValues[71] = -1.0000000000000000e+00;
acadoVariables.lbValues[72] = -8.7266467511653900e-02;
acadoVariables.lbValues[73] = -8.7266467511653900e-02;
acadoVariables.lbValues[74] = -1.0000000000000000e+00;
acadoVariables.lbValues[75] = -8.7266467511653900e-02;
acadoVariables.lbValues[76] = -8.7266467511653900e-02;
acadoVariables.lbValues[77] = -1.0000000000000000e+00;
acadoVariables.lbValues[78] = -8.7266467511653900e-02;
acadoVariables.lbValues[79] = -8.7266467511653900e-02;
acadoVariables.lbValues[80] = -1.0000000000000000e+00;
acadoVariables.lbValues[81] = -8.7266467511653900e-02;
acadoVariables.lbValues[82] = -8.7266467511653900e-02;
acadoVariables.lbValues[83] = -1.0000000000000000e+00;
acadoVariables.lbValues[84] = -8.7266467511653900e-02;
acadoVariables.lbValues[85] = -8.7266467511653900e-02;
acadoVariables.lbValues[86] = -1.0000000000000000e+00;
acadoVariables.lbValues[87] = -8.7266467511653900e-02;
acadoVariables.lbValues[88] = -8.7266467511653900e-02;
acadoVariables.lbValues[89] = -1.0000000000000000e+00;
acadoVariables.lbValues[90] = -8.7266467511653900e-02;
acadoVariables.lbValues[91] = -8.7266467511653900e-02;
acadoVariables.lbValues[92] = -1.0000000000000000e+00;
acadoVariables.lbValues[93] = -8.7266467511653900e-02;
acadoVariables.lbValues[94] = -8.7266467511653900e-02;
acadoVariables.lbValues[95] = -1.0000000000000000e+00;
acadoVariables.lbValues[96] = -8.7266467511653900e-02;
acadoVariables.lbValues[97] = -8.7266467511653900e-02;
acadoVariables.lbValues[98] = -1.0000000000000000e+00;
acadoVariables.lbValues[99] = -8.7266467511653900e-02;
acadoVariables.lbValues[100] = -8.7266467511653900e-02;
acadoVariables.lbValues[101] = -1.0000000000000000e+00;
acadoVariables.lbValues[102] = -8.7266467511653900e-02;
acadoVariables.lbValues[103] = -8.7266467511653900e-02;
acadoVariables.lbValues[104] = -1.0000000000000000e+00;
acadoVariables.lbValues[105] = -8.7266467511653900e-02;
acadoVariables.lbValues[106] = -8.7266467511653900e-02;
acadoVariables.lbValues[107] = -1.0000000000000000e+00;
acadoVariables.lbValues[108] = -8.7266467511653900e-02;
acadoVariables.lbValues[109] = -8.7266467511653900e-02;
acadoVariables.lbValues[110] = -1.0000000000000000e+00;
acadoVariables.ubValues[0] = 8.7266467511653900e-02;
acadoVariables.ubValues[1] = 8.7266467511653900e-02;
acadoVariables.ubValues[2] = 1.0000000000000000e+00;
acadoVariables.ubValues[3] = 8.7266467511653900e-02;
acadoVariables.ubValues[4] = 8.7266467511653900e-02;
acadoVariables.ubValues[5] = 1.0000000000000000e+00;
acadoVariables.ubValues[6] = 8.7266467511653900e-02;
acadoVariables.ubValues[7] = 8.7266467511653900e-02;
acadoVariables.ubValues[8] = 1.0000000000000000e+00;
acadoVariables.ubValues[9] = 8.7266467511653900e-02;
acadoVariables.ubValues[10] = 8.7266467511653900e-02;
acadoVariables.ubValues[11] = 1.0000000000000000e+00;
acadoVariables.ubValues[12] = 8.7266467511653900e-02;
acadoVariables.ubValues[13] = 8.7266467511653900e-02;
acadoVariables.ubValues[14] = 1.0000000000000000e+00;
acadoVariables.ubValues[15] = 8.7266467511653900e-02;
acadoVariables.ubValues[16] = 8.7266467511653900e-02;
acadoVariables.ubValues[17] = 1.0000000000000000e+00;
acadoVariables.ubValues[18] = 8.7266467511653900e-02;
acadoVariables.ubValues[19] = 8.7266467511653900e-02;
acadoVariables.ubValues[20] = 1.0000000000000000e+00;
acadoVariables.ubValues[21] = 8.7266467511653900e-02;
acadoVariables.ubValues[22] = 8.7266467511653900e-02;
acadoVariables.ubValues[23] = 1.0000000000000000e+00;
acadoVariables.ubValues[24] = 8.7266467511653900e-02;
acadoVariables.ubValues[25] = 8.7266467511653900e-02;
acadoVariables.ubValues[26] = 1.0000000000000000e+00;
acadoVariables.ubValues[27] = 8.7266467511653900e-02;
acadoVariables.ubValues[28] = 8.7266467511653900e-02;
acadoVariables.ubValues[29] = 1.0000000000000000e+00;
acadoVariables.ubValues[30] = 8.7266467511653900e-02;
acadoVariables.ubValues[31] = 8.7266467511653900e-02;
acadoVariables.ubValues[32] = 1.0000000000000000e+00;
acadoVariables.ubValues[33] = 8.7266467511653900e-02;
acadoVariables.ubValues[34] = 8.7266467511653900e-02;
acadoVariables.ubValues[35] = 1.0000000000000000e+00;
acadoVariables.ubValues[36] = 8.7266467511653900e-02;
acadoVariables.ubValues[37] = 8.7266467511653900e-02;
acadoVariables.ubValues[38] = 1.0000000000000000e+00;
acadoVariables.ubValues[39] = 8.7266467511653900e-02;
acadoVariables.ubValues[40] = 8.7266467511653900e-02;
acadoVariables.ubValues[41] = 1.0000000000000000e+00;
acadoVariables.ubValues[42] = 8.7266467511653900e-02;
acadoVariables.ubValues[43] = 8.7266467511653900e-02;
acadoVariables.ubValues[44] = 1.0000000000000000e+00;
acadoVariables.ubValues[45] = 8.7266467511653900e-02;
acadoVariables.ubValues[46] = 8.7266467511653900e-02;
acadoVariables.ubValues[47] = 1.0000000000000000e+00;
acadoVariables.ubValues[48] = 8.7266467511653900e-02;
acadoVariables.ubValues[49] = 8.7266467511653900e-02;
acadoVariables.ubValues[50] = 1.0000000000000000e+00;
acadoVariables.ubValues[51] = 8.7266467511653900e-02;
acadoVariables.ubValues[52] = 8.7266467511653900e-02;
acadoVariables.ubValues[53] = 1.0000000000000000e+00;
acadoVariables.ubValues[54] = 8.7266467511653900e-02;
acadoVariables.ubValues[55] = 8.7266467511653900e-02;
acadoVariables.ubValues[56] = 1.0000000000000000e+00;
acadoVariables.ubValues[57] = 8.7266467511653900e-02;
acadoVariables.ubValues[58] = 8.7266467511653900e-02;
acadoVariables.ubValues[59] = 1.0000000000000000e+00;
acadoVariables.ubValues[60] = 8.7266467511653900e-02;
acadoVariables.ubValues[61] = 8.7266467511653900e-02;
acadoVariables.ubValues[62] = 1.0000000000000000e+00;
acadoVariables.ubValues[63] = 8.7266467511653900e-02;
acadoVariables.ubValues[64] = 8.7266467511653900e-02;
acadoVariables.ubValues[65] = 1.0000000000000000e+00;
acadoVariables.ubValues[66] = 8.7266467511653900e-02;
acadoVariables.ubValues[67] = 8.7266467511653900e-02;
acadoVariables.ubValues[68] = 1.0000000000000000e+00;
acadoVariables.ubValues[69] = 8.7266467511653900e-02;
acadoVariables.ubValues[70] = 8.7266467511653900e-02;
acadoVariables.ubValues[71] = 1.0000000000000000e+00;
acadoVariables.ubValues[72] = 8.7266467511653900e-02;
acadoVariables.ubValues[73] = 8.7266467511653900e-02;
acadoVariables.ubValues[74] = 1.0000000000000000e+00;
acadoVariables.ubValues[75] = 8.7266467511653900e-02;
acadoVariables.ubValues[76] = 8.7266467511653900e-02;
acadoVariables.ubValues[77] = 1.0000000000000000e+00;
acadoVariables.ubValues[78] = 8.7266467511653900e-02;
acadoVariables.ubValues[79] = 8.7266467511653900e-02;
acadoVariables.ubValues[80] = 1.0000000000000000e+00;
acadoVariables.ubValues[81] = 8.7266467511653900e-02;
acadoVariables.ubValues[82] = 8.7266467511653900e-02;
acadoVariables.ubValues[83] = 1.0000000000000000e+00;
acadoVariables.ubValues[84] = 8.7266467511653900e-02;
acadoVariables.ubValues[85] = 8.7266467511653900e-02;
acadoVariables.ubValues[86] = 1.0000000000000000e+00;
acadoVariables.ubValues[87] = 8.7266467511653900e-02;
acadoVariables.ubValues[88] = 8.7266467511653900e-02;
acadoVariables.ubValues[89] = 1.0000000000000000e+00;
acadoVariables.ubValues[90] = 8.7266467511653900e-02;
acadoVariables.ubValues[91] = 8.7266467511653900e-02;
acadoVariables.ubValues[92] = 1.0000000000000000e+00;
acadoVariables.ubValues[93] = 8.7266467511653900e-02;
acadoVariables.ubValues[94] = 8.7266467511653900e-02;
acadoVariables.ubValues[95] = 1.0000000000000000e+00;
acadoVariables.ubValues[96] = 8.7266467511653900e-02;
acadoVariables.ubValues[97] = 8.7266467511653900e-02;
acadoVariables.ubValues[98] = 1.0000000000000000e+00;
acadoVariables.ubValues[99] = 8.7266467511653900e-02;
acadoVariables.ubValues[100] = 8.7266467511653900e-02;
acadoVariables.ubValues[101] = 1.0000000000000000e+00;
acadoVariables.ubValues[102] = 8.7266467511653900e-02;
acadoVariables.ubValues[103] = 8.7266467511653900e-02;
acadoVariables.ubValues[104] = 1.0000000000000000e+00;
acadoVariables.ubValues[105] = 8.7266467511653900e-02;
acadoVariables.ubValues[106] = 8.7266467511653900e-02;
acadoVariables.ubValues[107] = 1.0000000000000000e+00;
acadoVariables.ubValues[108] = 8.7266467511653900e-02;
acadoVariables.ubValues[109] = 8.7266467511653900e-02;
acadoVariables.ubValues[110] = 1.0000000000000000e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 37; ++index)
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
acadoWorkspace.state[204] = acadoVariables.u[index * 3];
acadoWorkspace.state[205] = acadoVariables.u[index * 3 + 1];
acadoWorkspace.state[206] = acadoVariables.u[index * 3 + 2];

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
for (index = 0; index < 37; ++index)
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
acadoVariables.x[333] = xEnd[0];
acadoVariables.x[334] = xEnd[1];
acadoVariables.x[335] = xEnd[2];
acadoVariables.x[336] = xEnd[3];
acadoVariables.x[337] = xEnd[4];
acadoVariables.x[338] = xEnd[5];
acadoVariables.x[339] = xEnd[6];
acadoVariables.x[340] = xEnd[7];
acadoVariables.x[341] = xEnd[8];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[333];
acadoWorkspace.state[1] = acadoVariables.x[334];
acadoWorkspace.state[2] = acadoVariables.x[335];
acadoWorkspace.state[3] = acadoVariables.x[336];
acadoWorkspace.state[4] = acadoVariables.x[337];
acadoWorkspace.state[5] = acadoVariables.x[338];
acadoWorkspace.state[6] = acadoVariables.x[339];
acadoWorkspace.state[7] = acadoVariables.x[340];
acadoWorkspace.state[8] = acadoVariables.x[341];
if (uEnd != 0)
{
acadoWorkspace.state[204] = uEnd[0];
acadoWorkspace.state[205] = uEnd[1];
acadoWorkspace.state[206] = uEnd[2];
}
else
{
acadoWorkspace.state[204] = acadoVariables.u[108];
acadoWorkspace.state[205] = acadoVariables.u[109];
acadoWorkspace.state[206] = acadoVariables.u[110];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[333] = acadoWorkspace.state[0];
acadoVariables.x[334] = acadoWorkspace.state[1];
acadoVariables.x[335] = acadoWorkspace.state[2];
acadoVariables.x[336] = acadoWorkspace.state[3];
acadoVariables.x[337] = acadoWorkspace.state[4];
acadoVariables.x[338] = acadoWorkspace.state[5];
acadoVariables.x[339] = acadoWorkspace.state[6];
acadoVariables.x[340] = acadoWorkspace.state[7];
acadoVariables.x[341] = acadoWorkspace.state[8];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 36; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[108] = uEnd[0];
acadoVariables.u[109] = uEnd[1];
acadoVariables.u[110] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79] + acadoWorkspace.g[80]*acadoWorkspace.x[80] + acadoWorkspace.g[81]*acadoWorkspace.x[81] + acadoWorkspace.g[82]*acadoWorkspace.x[82] + acadoWorkspace.g[83]*acadoWorkspace.x[83] + acadoWorkspace.g[84]*acadoWorkspace.x[84] + acadoWorkspace.g[85]*acadoWorkspace.x[85] + acadoWorkspace.g[86]*acadoWorkspace.x[86] + acadoWorkspace.g[87]*acadoWorkspace.x[87] + acadoWorkspace.g[88]*acadoWorkspace.x[88] + acadoWorkspace.g[89]*acadoWorkspace.x[89] + acadoWorkspace.g[90]*acadoWorkspace.x[90] + acadoWorkspace.g[91]*acadoWorkspace.x[91] + acadoWorkspace.g[92]*acadoWorkspace.x[92] + acadoWorkspace.g[93]*acadoWorkspace.x[93] + acadoWorkspace.g[94]*acadoWorkspace.x[94] + acadoWorkspace.g[95]*acadoWorkspace.x[95] + acadoWorkspace.g[96]*acadoWorkspace.x[96] + acadoWorkspace.g[97]*acadoWorkspace.x[97] + acadoWorkspace.g[98]*acadoWorkspace.x[98] + acadoWorkspace.g[99]*acadoWorkspace.x[99] + acadoWorkspace.g[100]*acadoWorkspace.x[100] + acadoWorkspace.g[101]*acadoWorkspace.x[101] + acadoWorkspace.g[102]*acadoWorkspace.x[102] + acadoWorkspace.g[103]*acadoWorkspace.x[103] + acadoWorkspace.g[104]*acadoWorkspace.x[104] + acadoWorkspace.g[105]*acadoWorkspace.x[105] + acadoWorkspace.g[106]*acadoWorkspace.x[106] + acadoWorkspace.g[107]*acadoWorkspace.x[107] + acadoWorkspace.g[108]*acadoWorkspace.x[108] + acadoWorkspace.g[109]*acadoWorkspace.x[109] + acadoWorkspace.g[110]*acadoWorkspace.x[110];
kkt = fabs( kkt );
for (index = 0; index < 111; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 37; ++lRun1)
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

acado_evaluateLagrange( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
objVal += acadoWorkspace.objValueOut[0];
}
return objVal;
}

