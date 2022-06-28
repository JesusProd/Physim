#ifndef COMPUTE_REFERENCE_TWIST_CHANGE_H
#define COMPUTE_REFERENCE_TWIST_CHANGE_H

inline double computeReferenceTwistChange (
  double *refTan1,
  double *refNor1,
  double *refTan2,
  double *refNor2,
  double refTwistPrev)
{
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t23;
  double t24;
  double t28;
  double t29;
  double t3;
  double t33;
  double t4;
  double t40;
  double t42;
  double t44;
  double t49;
  double t55;
  double t6;
  double t64;
  double t66;
  double t68;
  double t7;
  double t78;
  double t89;
  double t96;
  t3 = refTan2[1];
  t4 = refNor2[2];
  t6 = refTan2[2];
  t7 = refNor2[1];
  t10 = cos(refTwistPrev);
  t11 = refTan1[0];
  t12 = refTan2[0];
  t13 = t11 * t12;
  t14 = refTan1[1];
  t15 = t14 * t3;
  t16 = refTan1[2];
  t17 = t16 * t6;
  t18 = t13 + t15 + t17;
  t19 = refNor1[0];
  t23 = t16 * t12 - t11 * t6;
  t24 = refNor1[2];
  t28 = t11 * t3 - t14 * t12;
  t29 = refNor1[1];
  t33 = t14 * t6 - t16 * t3;
  t40 = (t33 * t19 + t23 * t29 + t28 * t24) / (0.1e1 + t13 + t15 + t17);
  t42 = t19 * t18 + t23 * t24 - t28 * t29 + t40 * t33;
  t44 = sin(refTwistPrev);
  t49 = t18 * t24 + t33 * t29 - t23 * t19 + t40 * t28;
  t55 = t18 * t29 + t28 * t19 - t33 * t24 + t40 * t23;
  t64 = (t12 * t42 + t3 * t55 + t6 * t49) * (0.1e1 - t10);
  t66 = t10 * t42 + t44 * (t3 * t49 - t6 * t55) + t64 * t12;
  t68 = refNor2[0];
  t78 = t10 * t55 + t44 * (t6 * t42 - t12 * t49) + t64 * t3;
  t89 = t10 * t49 + t44 * (t12 * t55 - t3 * t42) + t64 * t6;
  t96 = atan2((t3 * t4 - t6 * t7) * t66 + (t6 * t68 - t12 * t4) * t78 + (t12 * t7 - t3 * t68) * t89, t68 * t66 + t7 * t78 + t4 * t89);
  return(-t96);
}

#endif