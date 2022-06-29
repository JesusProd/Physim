void rigidBodyTransformQuat (
  double *vec,
  double *cen,
  double rot,
  double *pos)
{
  pos[0] = cen[0] + vec[0];
  pos[1] = cen[1] + vec[1];
  pos[2] = cen[2] + vec[2];
}
