// SPDX-FileCopyrightText: 2011 H. Strasdat
//
// SPDX-License-Identifier: BSD-2-Clause

Matrix3D skew(const Vector3D& v) {
  Matrix3D m;
  m.fill(0.);
  m(0, 1) = -v(2);
  m(0, 2) = v(1);
  m(1, 2) = -v(0);
  m(1, 0) = v(2);
  m(2, 0) = -v(1);
  m(2, 1) = v(0);
  return m;
}

Vector3D deltaR(const Matrix3D& R) {
  Vector3D v;
  v(0) = R(2, 1) - R(1, 2);
  v(1) = R(0, 2) - R(2, 0);
  v(2) = R(1, 0) - R(0, 1);
  return v;
}

Vector2D project(const Vector3D& v) {
  Vector2D res;
  res(0) = v(0) / v(2);
  res(1) = v(1) / v(2);
  return res;
}

Vector3D project(const Vector4D& v) {
  Vector3D res;
  res(0) = v(0) / v(3);
  res(1) = v(1) / v(3);
  res(2) = v(2) / v(3);
  return res;
}

Vector3D unproject(const Vector2D& v) {
  Vector3D res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

Vector4D unproject(const Vector3D& v) {
  Vector4D res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = v(2);
  res(3) = 1;
  return res;
}
