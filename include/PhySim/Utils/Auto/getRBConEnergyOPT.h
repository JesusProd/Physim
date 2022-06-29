#include <math.h>

double getRBConEnergyOPT(double young,
                         double shear,
                         double radw,
                         double radh,
                         double hL0,
                         const double* iniMatTanRB,
                         const double* iniMatNorRB,
                         const double* iniRefTanRO,
                         const double* iniRefNorRO,
                         double rtx,
                         const double* rotationAnglesRB,
                         const double* ex,
                         double thetax) {
  double t10;
  double t101;
  double t104;
  double t108;
  double t11;
  double t111;
  double t113;
  double t115;
  double t117;
  double t12;
  double t120;
  double t122;
  double t123;
  double t124;
  double t126;
  double t128;
  double t130;
  double t136;
  double t14;
  double t140;
  double t146;
  double t15;
  double t150;
  double t156;
  double t16;
  double t162;
  double t163;
  double t165;
  double t167;
  double t172;
  double t179;
  double t18;
  double t184;
  double t193;
  double t196;
  double t20;
  double t202;
  double t207;
  double t217;
  double t22;
  double t223;
  double t233;
  double t24;
  double t240;
  double t242;
  double t243;
  double t251;
  double t252;
  double t254;
  double t26;
  double t264;
  double t267;
  double t274;
  double t28;
  double t280;
  double t285;
  double t29;
  double t293;
  double t3;
  double t30;
  double t302;
  double t316;
  double t36;
  double t37;
  double t4;
  double t43;
  double t44;
  double t52;
  double t57;
  double t59;
  double t62;
  double t67;
  double t7;
  double t70;
  double t71;
  double t72;
  double t73;
  double t74;
  double t75;
  double t76;
  double t77;
  double t79;
  double t8;
  double t80;
  double t81;
  double t82;
  double t84;
  double t85;
  double t87;
  double t88;
  double t9;
  double t92;
  double t93;
  double t96;
  double t98;
  double t99;
  t3 = radw * radw;
  t4 = radh * radh;
  t7 = ex[0];
  t8 = t7 * t7;
  t9 = ex[1];
  t10 = t9 * t9;
  t11 = ex[2];
  t12 = t11 * t11;
  t14 = sqrt(t8 + t10 + t12);
  t15 = 0.1e1 / t14;
  t16 = t15 * t9;
  t18 = iniRefTanRO[0] * t15;
  t20 = 0.1e1 * t18 * t7;
  t22 = iniRefTanRO[1] * t15;
  t24 = 0.1e1 * t22 * t9;
  t26 = iniRefTanRO[2] * t15;
  t28 = 0.1e1 * t26 * t11;
  t29 = t20 + t24 + t28;
  t30 = iniRefNorRO[2];
  t36 = 0.1e1 * t22 * t11 - 0.1e1 * t26 * t9;
  t37 = iniRefNorRO[1];
  t43 = 0.1e1 * t26 * t7 - 0.1e1 * t18 * t11;
  t44 = iniRefNorRO[0];
  t52 = 0.1e1 * t18 * t9 - 0.1e1 * t22 * t7;
  t57 = (t36 * t44 + t43 * t37 + t52 * t30) / (0.1e1 + t20 + t24 + t28);
  t59 = t29 * t30 + t36 * t37 - t43 * t44 + t57 * t52;
  t62 = t15 * t11;
  t67 = t29 * t37 + t52 * t44 - t36 * t30 + t57 * t43;
  t70 = 0.1e1 * t16 * t59 - 0.1e1 * t62 * t67;
  t71 = cos(rtx);
  t72 = rotationAnglesRB[2];
  t73 = cos(t72);
  t74 = rotationAnglesRB[1];
  t75 = cos(t74);
  t76 = t73 * t75;
  t77 = iniMatTanRB[0];
  t79 = sin(t74);
  t80 = t73 * t79;
  t81 = rotationAnglesRB[0];
  t82 = sin(t81);
  t84 = sin(t72);
  t85 = cos(t81);
  t87 = t80 * t82 - t84 * t85;
  t88 = iniMatTanRB[1];
  t92 = t80 * t85 + t84 * t82;
  t93 = iniMatTanRB[2];
  t96 = (t76 * t77 + t87 * t88 + t92 * t93) * t15;
  t98 = 0.1e1 * t96 * t7;
  t99 = t84 * t75;
  t101 = t84 * t79;
  t104 = t101 * t82 + t73 * t85;
  t108 = t101 * t85 - t73 * t82;
  t111 = (t99 * t77 + t104 * t88 + t108 * t93) * t15;
  t113 = 0.1e1 * t111 * t9;
  t115 = t75 * t82;
  t117 = t75 * t85;
  t120 = (-t79 * t77 + t115 * t88 + t117 * t93) * t15;
  t122 = 0.1e1 * t120 * t11;
  t123 = t98 + t113 + t122;
  t124 = iniMatNorRB[0];
  t126 = iniMatNorRB[1];
  t128 = iniMatNorRB[2];
  t130 = t76 * t124 + t87 * t126 + t92 * t128;
  t136 = 0.1e1 * t120 * t7 - 0.1e1 * t96 * t11;
  t140 = -t79 * t124 + t115 * t126 + t117 * t128;
  t146 = 0.1e1 * t96 * t9 - 0.1e1 * t111 * t7;
  t150 = t99 * t124 + t104 * t126 + t108 * t128;
  t156 = 0.1e1 * t111 * t11 - 0.1e1 * t120 * t9;
  t162 = 0.1e1 / (0.1e1 + t98 + t113 + t122);
  t163 = (t156 * t130 + t136 * t150 + t146 * t140) * t162;
  t165 = t123 * t130 + t136 * t140 - t146 * t150 + t163 * t156;
  t167 = sin(rtx);
  t172 = t123 * t140 + t156 * t150 - t136 * t130 + t163 * t146;
  t179 = t123 * t150 + t146 * t130 - t156 * t140 + t163 * t136;
  t184 = t15 * t7;
  t193 = (0.1e1 * t184 * t165 + 0.1e1 * t16 * t179 + 0.1e1 * t62 * t172) *
         (0.1e1 - t71);
  t196 = t71 * t165 + t167 * (0.1e1 * t16 * t172 - 0.1e1 * t62 * t179) +
         0.1e1 * t193 * t184;
  t202 = t29 * t44 + t43 * t30 - t52 * t37 + t57 * t36;
  t207 = 0.1e1 * t62 * t202 - 0.1e1 * t184 * t59;
  t217 = t71 * t179 + t167 * (0.1e1 * t62 * t165 - 0.1e1 * t184 * t172) +
         0.1e1 * t193 * t16;
  t223 = 0.1e1 * t184 * t67 - 0.1e1 * t16 * t202;
  t233 = t71 * t172 + t167 * (0.1e1 * t184 * t179 - 0.1e1 * t16 * t165) +
         0.1e1 * t193 * t62;
  t240 = atan2(t70 * t196 + t207 * t217 + t223 * t233,
               t202 * t196 + t67 * t217 + t59 * t233);
  t242 = pow(thetax + rtx - t240, 0.2e1);
  t243 = 0.1e1 / hL0;
  t251 = t162 * t156;
  t252 = cos(thetax);
  t254 = sin(thetax);
  t264 = (0.1e1 * t184 * t202 + 0.1e1 * t16 * t67 + 0.1e1 * t62 * t59) *
         (0.1e1 - t252);
  t267 = t252 * t59 + t254 * t223 + 0.1e1 * t264 * t62;
  t274 = t252 * t67 + t254 * t207 + 0.1e1 * t264 * t16;
  t280 = t162 * t136;
  t285 = t252 * t202 + t254 * t70 + 0.1e1 * t264 * t184;
  t293 = t162 * t146;
  t302 = pow(0.2e1 * t251 * (0.1e1 * t16 * t267 - 0.1e1 * t62 * t274) +
                 0.2e1 * t280 * (0.1e1 * t62 * t285 - 0.1e1 * t184 * t267) +
                 0.2e1 * t293 * (0.1e1 * t184 * t274 - 0.1e1 * t16 * t285),
             0.2e1);
  t316 = pow(-0.2e1 * t251 * t285 - 0.2e1 * t280 * t274 - 0.2e1 * t293 * t267,
             0.2e1);
  return (0.1250000000e0 * shear * radw * radh * M_PI * (t3 + t4) * t242 *
              t243 +
          (0.25e0 * young * radw * t4 * radh * M_PI * t302 +
           0.25e0 * young * t3 * radw * radh * M_PI * t316) *
              t243 / 0.2e1);
}
