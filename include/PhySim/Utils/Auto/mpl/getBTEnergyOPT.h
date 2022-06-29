#include <math.h>

double getBTEnergyOPT (
  double *vshear,
  double *vyoung,
  double *vradw,
  double *vradh,
  double L0,
  double L1,
  const double *K0,
  double twist0,
  const double *nRefIni1,
  const double *tRefIni1,
  const double *nRefIni2,
  const double *tRefIni2,
  double rtx,
  const double *e1,
  const double *e2,
  double theta1,
  double theta2)
{
  double t10;
  double t11;
  double t111;
  double t114;
  double t12;
  double t126;
  double t13;
  double t136;
  double t137;
  double t14;
  double t147;
  double t15;
  double t159;
  double t16;
  double t160;
  double t168;
  double t17;
  double t170;
  double t172;
  double t174;
  double t176;
  double t178;
  double t18;
  double t180;
  double t181;
  double t182;
  double t188;
  double t189;
  double t195;
  double t196;
  double t20;
  double t204;
  double t209;
  double t21;
  double t211;
  double t213;
  double t218;
  double t22;
  double t225;
  double t228;
  double t238;
  double t24;
  double t241;
  double t249;
  double t25;
  double t253;
  double t26;
  double t264;
  double t268;
  double t28;
  double t286;
  double t29;
  double t291;
  double t297;
  double t3;
  double t30;
  double t302;
  double t303;
  double t32;
  double t324;
  double t326;
  double t336;
  double t34;
  double t348;
  double t349;
  double t357;
  double t359;
  double t361;
  double t366;
  double t373;
  double t386;
  double t389;
  double t39;
  double t4;
  double t40;
  double t400;
  double t41;
  double t411;
  double t418;
  double t420;
  double t43;
  double t45;
  double t47;
  double t49;
  double t5;
  double t51;
  double t53;
  double t54;
  double t55;
  double t6;
  double t61;
  double t62;
  double t68;
  double t69;
  double t7;
  double t77;
  double t8;
  double t82;
  double t84;
  double t86;
  double t91;
  double t98;
  t3 = e1[0];
  t4 = t3 * t3;
  t5 = e1[1];
  t6 = t5 * t5;
  t7 = e1[2];
  t8 = t7 * t7;
  t10 = sqrt(t4 + t6 + t8);
  t11 = 0.1e1 / t10;
  t12 = t11 * t3;
  t13 = e2[0];
  t14 = t13 * t13;
  t15 = e2[1];
  t16 = t15 * t15;
  t17 = e2[2];
  t18 = t17 * t17;
  t20 = sqrt(t14 + t16 + t18);
  t21 = 0.1e1 / t20;
  t22 = t21 * t13;
  t24 = 0.1e1 * t12 * t22;
  t25 = t11 * t5;
  t26 = t21 * t15;
  t28 = 0.1e1 * t25 * t26;
  t29 = t11 * t7;
  t30 = t21 * t17;
  t32 = 0.1e1 * t29 * t30;
  t34 = 0.1e1 / (0.1e1 + t24 + t28 + t32);
  t39 = 0.1e1 * t25 * t30 - 0.1e1 * t29 * t26;
  t40 = t34 * t39;
  t41 = cos(theta1);
  t43 = tRefIni1[0] * t11;
  t45 = 0.1e1 * t43 * t3;
  t47 = tRefIni1[1] * t11;
  t49 = 0.1e1 * t47 * t5;
  t51 = tRefIni1[2] * t11;
  t53 = 0.1e1 * t51 * t7;
  t54 = t45 + t49 + t53;
  t55 = nRefIni1[2];
  t61 = 0.1e1 * t47 * t7 - 0.1e1 * t51 * t5;
  t62 = nRefIni1[1];
  t68 = 0.1e1 * t51 * t3 - 0.1e1 * t43 * t7;
  t69 = nRefIni1[0];
  t77 = 0.1e1 * t43 * t5 - 0.1e1 * t47 * t3;
  t82 = (t61 * t69 + t68 * t62 + t77 * t55) / (0.1e1 + t45 + t49 + t53);
  t84 = t54 * t55 + t61 * t62 - t68 * t69 + t82 * t77;
  t86 = sin(theta1);
  t91 = t54 * t62 + t77 * t69 - t61 * t55 + t82 * t68;
  t98 = t54 * t69 + t68 * t55 - t77 * t62 + t82 * t61;
  t111 = (0.1e1 * t12 * t98 + 0.1e1 * t25 * t91 + 0.1e1 * t29 * t84) * (0.1e1 - t41);
  t114 = t41 * t84 + t86 * (0.1e1 * t12 * t91 - 0.1e1 * t25 * t98) + 0.1e1 * t111 * t29;
  t126 = t41 * t91 + t86 * (0.1e1 * t29 * t98 - 0.1e1 * t12 * t84) + 0.1e1 * t111 * t25;
  t136 = 0.1e1 * t29 * t22 - 0.1e1 * t12 * t30;
  t137 = t34 * t136;
  t147 = t41 * t98 + t86 * (0.1e1 * t25 * t84 - 0.1e1 * t29 * t91) + 0.1e1 * t111 * t12;
  t159 = 0.1e1 * t12 * t26 - 0.1e1 * t25 * t22;
  t160 = t34 * t159;
  t168 = cos(theta2);
  t170 = tRefIni2[0] * t21;
  t172 = 0.1e1 * t170 * t13;
  t174 = tRefIni2[1] * t21;
  t176 = 0.1e1 * t174 * t15;
  t178 = tRefIni2[2] * t21;
  t180 = 0.1e1 * t178 * t17;
  t181 = t172 + t176 + t180;
  t182 = nRefIni2[2];
  t188 = 0.1e1 * t174 * t17 - 0.1e1 * t178 * t15;
  t189 = nRefIni2[1];
  t195 = 0.1e1 * t178 * t13 - 0.1e1 * t170 * t17;
  t196 = nRefIni2[0];
  t204 = 0.1e1 * t170 * t15 - 0.1e1 * t174 * t13;
  t209 = (t188 * t196 + t195 * t189 + t204 * t182) / (0.1e1 + t172 + t176 + t180);
  t211 = t181 * t182 + t188 * t189 - t195 * t196 + t209 * t204;
  t213 = sin(theta2);
  t218 = t181 * t189 + t204 * t196 - t188 * t182 + t209 * t195;
  t225 = t181 * t196 + t195 * t182 - t204 * t189 + t209 * t188;
  t228 = 0.1e1 * t22 * t218 - 0.1e1 * t26 * t225;
  t238 = (0.1e1 * t22 * t225 + 0.1e1 * t26 * t218 + 0.1e1 * t30 * t211) * (0.1e1 - t168);
  t241 = t168 * t211 + t213 * t228 + 0.1e1 * t238 * t30;
  t249 = 0.1e1 * t30 * t225 - 0.1e1 * t22 * t211;
  t253 = t168 * t218 + t213 * t249 + 0.1e1 * t238 * t26;
  t264 = 0.1e1 * t26 * t211 - 0.1e1 * t30 * t218;
  t268 = t168 * t225 + t213 * t264 + 0.1e1 * t238 * t22;
  t286 = pow(0.1000000000e1 * t40 * (0.1e1 * t25 * t114 - 0.1e1 * t29 * t126) + 0.1000000000e1 * t137 * (0.1e1 * t29 * t147 - 0.1e1 * t12 * t114) + 0.1000000000e1 * t160 * (0.1e1 * t12 * t126 - 0.1e1 * t25 * t147) + 0.1000000000e1 * t40 * (0.1e1 * t26 * t241 - 0.1e1 * t30 * t253) + 0.1000000000e1 * t137 * (0.1e1 * t30 * t268 - 0.1e1 * t22 * t241) + 0.1000000000e1 * t160 * (0.1e1 * t22 * t253 - 0.1e1 * t26 * t268) - 0.1e1 * K0[0], 0.2e1);
  t291 = 0.5e0 * vyoung[0] + 0.5e0 * vyoung[1];
  t297 = 0.5e0 * vradw[0] + 0.5e0 * vradw[1];
  t302 = 0.5e0 * vradh[0] + 0.5e0 * vradh[1];
  t303 = t302 * t302;
  t324 = pow(-0.1000000000e1 * t40 * t147 - 0.1000000000e1 * t137 * t126 - 0.1000000000e1 * t160 * t114 - 0.1000000000e1 * t40 * t268 - 0.1000000000e1 * t137 * t253 - 0.1000000000e1 * t160 * t241 - 0.1e1 * K0[1], 0.2e1);
  t326 = t297 * t297;
  t336 = 0.1e1 / (0.5e0 * L0 + 0.5e0 * L1);
  t348 = cos(rtx);
  t349 = t24 + t28 + t32;
  t357 = (t39 * t98 + t136 * t91 + t159 * t84) * t34;
  t359 = t349 * t98 + t136 * t84 - t159 * t91 + t357 * t39;
  t361 = sin(rtx);
  t366 = t349 * t84 + t39 * t91 - t136 * t98 + t357 * t159;
  t373 = t349 * t91 + t159 * t98 - t39 * t84 + t357 * t136;
  t386 = (0.1e1 * t22 * t359 + 0.1e1 * t26 * t373 + 0.1e1 * t30 * t366) * (0.1e1 - t348);
  t389 = t348 * t359 + t361 * (0.1e1 * t26 * t366 - 0.1e1 * t30 * t373) + 0.1e1 * t386 * t22;
  t400 = t348 * t373 + t361 * (0.1e1 * t30 * t359 - 0.1e1 * t22 * t366) + 0.1e1 * t386 * t26;
  t411 = t348 * t366 + t361 * (0.1e1 * t22 * t373 - 0.1e1 * t26 * t359) + 0.1e1 * t386 * t30;
  t418 = atan2(t264 * t389 + t249 * t400 + t228 * t411, t225 * t389 + t218 * t400 + t211 * t411);
  t420 = pow(theta2 - theta1 + rtx - t418 - twist0, 0.2e1);
  return(0.5000000000e0 * (0.25e0 * t286 * t291 * t297 * t303 * t302 * M_PI + 0.25e0 * t324 * t291 * t326 * t297 * t302 * M_PI) * t336 + 0.1250000000e0 * (0.5e0 * vshear[0] + 0.5e0 * vshear[1]) * t297 * t302 * M_PI * (t326 + t303) * t420 * t336);
}
