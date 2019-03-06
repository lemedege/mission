/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@oersted.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

//#include "ucommon.h"
//#include "umatrix.h"
#include "u2dline.h"



/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
// global functions

inline float sqr(float v)
{
  return v*v;
}

/**
same as fabs, but for float variables. */
inline float absf(float v)
{
  if (v >= 0.0)
    return v;
  else
    return -v;
}

/**
Tests if point (m,n) is within the square surrounded
by (x1,y1), (x2,y2), (x3,y3) and (x4,y4) taken
either clockwise or counter clockwise.
Returns true if within. */
bool isWithinSquare(float m, float n,
             float x1, float y1, float x2, float y2,
             float x3, float y3, float x4, float y4)
{
  bool result;
  U2Dline ln;
  int pos = 0;
  int neg = 0;
  //
  ln.set2P(x1, y1, x2, y2);
  if (ln.distanceSigned(m,n) > 0.0)
    pos++;
  else
    neg++;
  ln.set2P(x2, y2, x3, y3);
  if (ln.distanceSigned(m,n) > 0.0)
    pos++;
  else
    neg++;
  ln.set2P(x3, y3, x4, y4);
  if (ln.distanceSigned(m,n) > 0.0)
    pos++;
  else
    neg++;
  ln.set2P(x4, y4, x1, y1);
  if (ln.distanceSigned(m,n) > 0.0)
    pos++;
  else
    neg++;
  result = (pos == 4) or (neg == 4);
  return result;
}


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////


U2Dline::U2Dline()
{ // default line x = 0
  lA = 1.0;
  lB = 0.0;
  lC = 0.0;
}

//////////////////////////////////////////////////////////////////////////

U2Dline::~U2Dline()
{}

//////////////////////////////////////////////////////////////////////////

// bool U2Dline::setXline(double sX, double sY,
//                       double sX2, double sXY,
//                       int count)
// {
//   bool result;
//   double a, b, l, cnt;
//   //
//   result = (count > 0) and (absf(sX2 - sqr(sX)) > 1e-10);
//   if (result)
//   {
//     cnt = double(count);
//     b = (cnt * sXY - sX * sY)/(cnt * sX2 - sqr(sX));
//     a = (sY - b * sX)/cnt;
//     lA = -b;
//     lB = 1.0;
//     lC = -a;
//     // normalize
//     l = hypot(lA, lB);
//     lA /= l;
//     lB /= l;
//     lC /= l;
//   }
//   //
//   return result;
// }

//////////////////////////////////////////////////////////////////////////

// bool U2Dline::setYline(double sX, double sY,
//                       double sY2, double sXY,
//                       int count)
// {
//   bool result;
//   double a, b, l, cnt;
//   //
//   result = (count > 0) and (absf(sY2 - sqr(sY)) > 1e-10);
//   {
//     cnt = double(count);
//     b = (cnt * sXY - sX * sY)/(cnt * sY2 - sqr(sY));
//     a = (sX - b * sY)/cnt;
//     lA = 1.0;
//     lB = -b;
//     lC = -a;
//     // normalize
//     l = hypot(lA, lB);
//     lA /= l;
//     lB /= l;
//     lC /= l;
//   }
//   //
//   return result;
// }

//////////////////////////////////////////////////////////////////////////

int U2Dline::set(float fx[], float fy[], int count, float * V)
{ // find best line equation Ax + By + C = 0; from points in fx[i],fy[i], i=0..count-1.
  // scaled so that sqrt(A*A + B*B) = 1.0 (unit vector)
  // returns err = 0 for success
  int err = 0;
  int i;
  //
  float minX = 1e10, maxX = -1e10; // east
  float minY = 1e10, maxY = -1e10; // north
  float y, x;
  //
  float sX = 0.0, sY = 0.0, sXY = 0.0, sX2 = 0.0, sY2 = 0.0;
  float cntD;
  float a, b;
  float l; // vector length
  // Method:
  // use least square distance parallel with
  // one of the axes, base equation is: y = a + bx
  // square distance is (y - (a + bx))^2 summed for all (x,y),
  // partial derivative must be 0 for both a and b
  // a and b are then found.
  // if slope is more than approx. 45 deg then X and Y are
  // exchanged to base equation x = a + by
  if (count > 1)
  { // OK to calculate statistics
    // loop all detections in cluster
    for (i = 0; i < count; i++)
    { // get statistics
      x = fx[i];
      y = fy[i];
      //
      if (y > maxY) maxY = y;
      if (y < minY) minY = y;
      if (x > maxX) maxX = x;
      if (x < minX) minX = x;
      // statistics for line extimation in horizontal plane
      sX += x;
      sY += y;
      sXY += x * y;
      sX2 += sqr(x);
      sY2 += sqr(y);
    }
    cntD = float(count);
    if ((maxX - minX) > (maxY - minY))
    { // more x oriented. A may be zero so find A and C
      // find y = a + bx, so B = 1; A=-b; C =-a;
      if (absf(sX2 - sqr(sX)) < 1e-10)
        err = -2; // all detections in same point in this plane
      else
      {
        b = (cntD * sXY - sX * sY)/(cntD * sX2 - sqr(sX));
        a = (sY - b * sX)/cntD;
        lA = -b;
        lB = 1.0;
        lC = -a;
      }
    }
    else
    { // more y oriented. B may be zero, so find B and C
      // find e = a + bn, so A = 1; B = -b; C = -a
      if (absf(sY2 - sqr(sY)) < 1e-10)
        err = -2; // all detections in same point in this plane
      else
      {
        b = (cntD * sXY - sX * sY)/(cntD * sY2 - sqr(sY));
        a = (sX - b * sY)/cntD;
        lA = 1.0;
        lB = -b;
        lC = -a;
      }
    }
    // normalize
    l = hypot(lA, lB);
    lA /= l;
    lB /= l;
    lC /= l;
  }
  else
    err = -1;
  if (V != NULL)
    *V = variance(fx, fy, count);
  return err;
}

int U2Dline::set(std::vector<cv::Point> &p, float * V)
{ // find best line equation Ax + By + C = 0; from points in fx[i],fy[i], i=0..count-1.
  // scaled so that sqrt(A*A + B*B) = 1.0 (unit vector)
  // returns err = 0 for success
  int err = 0;
  int i;
  int count = p.size();
  //
  float minX = 1e10, maxX = -1e10; // east
  float minY = 1e10, maxY = -1e10; // north
//   float y, x;
  //
  float sX = 0.0, sY = 0.0, sXY = 0.0, sX2 = 0.0, sY2 = 0.0;
  float cntD;
  float a, b;
  float l; // vector length
  // Method:
  // use least square distance parallel with
  // one of the axes, base equation is: y = a + bx
  // square distance is (y - (a + bx))^2 summed for all (x,y),
  // partial derivative must be 0 for both a and b
  // a and b are then found.
  // if slope is more than approx. 45 deg then X and Y are
  // exchanged to base equation x = a + by
  if (count > 1)
  { // OK to calculate statistics
    // loop all detections in cluster
    for (i = 0; i < count; i++)
    { // get statistics
      cv::Point v = p.at(i);
      float x = v.x;
      float y = v.y;
      //
      if (y > maxY) maxY = y;
      if (y < minY) minY = y;
      if (x > maxX) maxX = x;
      if (x < minX) minX = x;
      // statistics for line extimation in horizontal plane
      sX += x;
      sY += y;
      sXY += x * y;
      sX2 += sqr(x);
      sY2 += sqr(y);
    }
    cntD = float(count);
    if ((maxX - minX) > (maxY - minY))
    { // more x oriented. A may be zero so find A and C
      // find y = a + bx, so B = 1; A=-b; C =-a;
      if (absf(sX2 - sqr(sX)) < 1e-10)
        err = -2; // all detections in same point in this plane
        else
        {
          b = (cntD * sXY - sX * sY)/(cntD * sX2 - sqr(sX));
          a = (sY - b * sX)/cntD;
          lA = -b;
          lB = 1.0;
          lC = -a;
        }
    }
    else
    { // more y oriented. B may be zero, so find B and C
      // find e = a + bn, so A = 1; B = -b; C = -a
      if (absf(sY2 - sqr(sY)) < 1e-10)
        err = -2; // all detections in same point in this plane
        else
        {
          b = (cntD * sXY - sX * sY)/(cntD * sY2 - sqr(sY));
          a = (sX - b * sY)/cntD;
          lA = 1.0;
          lB = -b;
          lC = -a;
        }
    }
    // normalize
    l = hypot(lA, lB);
    lA /= l;
    lB /= l;
    lC /= l;
  }
  else
    err = -1;
  if (V != NULL)
    *V = variance(p);
  return err;
}

/////////////////////////////////////////////////////////////////

bool U2Dline::fit(float fx[], float fy[], int count, float * V)
{ // find best line equation Ax + By + C = 0; from points in fx[i],fy[i], i=0..count-1.
  // scaled so that sqrt(A*A + B*B) = 1.0 (unit vector)
  // returns err = 0 for success
  int err = 0;
  int i;
  float y, x;
  float sX = 0.0, sY = 0.0, sXY = 0.0, sX2 = 0.0, sY2 = 0.0;
  float cntD;
  float a, b;
  float l; // vector length
  float v1, v2;
  float A1 = 0.0, B1 = 0.0, C1 = 0.0;
  float A2, B2, C2;
  // Method:
  // use least square distance parallel with
  // one of the axes, base equation is: y = a + bx
  // square distance is (y - (a + bx))^2 summed for all (x,y),
  // partial derivative must be 0 for both a and b
  // a and b are then found.
  // if slope is more than approx. 45 deg then X and Y are
  // exchanged to base equation x = a + by
  if (count > 1)
  { // OK to calculate statistics
    // loop all detections in cluster
    for (i = 0; i < count; i++)
    { // get statistics
      x = fx[i];
      y = fy[i];
      // statistics for line extimation in horizontal plane
      sX += x;
      sY += y;
      sXY += x * y;
      sX2 += sqr(x);
      sY2 += sqr(y);
    }
    cntD = float(count);
    // find y = a + bx, so B = 1; A=-b; C =-a;
    if (absf(sX2 - sqr(sX)) < 1e-10)
    {
      err -= 1; // all detections in same point in this plane
      v1 = 1e100;
    }
    else
    {
      b = (cntD * sXY - sX * sY)/(cntD * sX2 - sqr(sX));
      a = (sY - b * sX)/cntD;
      A1 = -b;
      B1 = 1.0;
      C1 = -a;
      // normalize and store in temp-line A, B and C
      l = hypot(A1, B1);
      A1 /= l;
      B1 /= l;
      C1 /= l;
      // get variance for method 1 first
      lA = A1;
      lB = B1;
      lC = C1;
      v1 = variance(fx, fy, count);
    }
    // find e = a + bn, so A = 1; B = -b; C = -a
    if (absf(sY2 - sqr(sY)) < 1e-10)
    {
      err -= 1; // all detections in same point in this plane
      v2 = 1e100;
    }
    else
    {
      b = (cntD * sXY - sX * sY)/(cntD * sY2 - sqr(sY));
      a = (sX - b * sY)/cntD;
      A2 = 1.0;
      B2 = -b;
      C2 = -a;
      // normalize
      l = hypot(A2, B2);
      A2 /= l;
      B2 /= l;
      C2 /= l;
      //
      // ... then for method 2
      lA = A2;
      lB = B2;
      lC = C2;
      v2 = variance(fx, fy, count);
    }
    // best fit wins
    if ((err != -2) and (v1 < v2))
    {
      lA = A1;
      lB = B1;
      lC = C1;
      //printf("U2Dline::fit %fv1, %fv2 (result=%s)\n", v1, v2, bool2str(err == 0));
      v2 = v1;
    }
    if ((err == 0) and (V != NULL))
      *V = v2;
  }
  else
    err = -1;
  return (err == 0);
}

/////////////////////////////////////////////////////////////

bool U2Dline::fitV(std::vector<cv::Point> &p, float * V)
{ // find best line equation Ax + By + C = 0; from points in fx[i],fy[i], i=0..count-1.
  // scaled so that sqrt(A*A + B*B) = 1.0 (unit vector)
  // returns err = 0 for success
  int err = 0;
  int i;
  float y, x;
  float sX = 0.0, sY = 0.0, sXY = 0.0, sX2 = 0.0, sY2 = 0.0;
  float cntD;
  float a, b;
  float l; // vector length
  float v1, v2;
  float A1 = 0.0, B1 = 0.0, C1 = 0.0;
  float A2, B2, C2;
  int count = p.size();
  // Method:
  // use least square distance parallel with
  // one of the axes, base equation is: y = a + bx
  // square distance is (y - (a + bx))^2 summed for all (x,y),
  // partial derivative must be 0 for both a and b
  // a and b are then found.
  // if slope is more than approx. 45 deg then X and Y are
  // exchanged to base equation x = a + by
  if (count > 1)
  { // OK to calculate statistics
    // loop all detections in cluster
    for (i = 0; i < count; i++)
    { // get statistics
      x = p.at(i).x;
      y = p.at(i).y;
      // statistics for line extimation in horizontal plane
      sX += x;
      sY += y;
      sXY += x * y;
      sX2 += sqr(x);
      sY2 += sqr(y);
    }
    cntD = float(count);
    // find y = a + bx, so B = 1; A=-b; C =-a;
    if (absf(sX2 - sqr(sX)) < 1e-10)
    {
      err -= 1; // all detections in same point in this plane
      v1 = 1e100;
    }
    else
    {
      b = (cntD * sXY - sX * sY)/(cntD * sX2 - sqr(sX));
      a = (sY - b * sX)/cntD;
      A1 = -b;
      B1 = 1.0;
      C1 = -a;
      // normalize and store in temp-line A, B and C
      l = hypot(A1, B1);
      A1 /= l;
      B1 /= l;
      C1 /= l;
      // get variance for method 1 first
      lA = A1;
      lB = B1;
      lC = C1;
      v1 = variance(p);
    }
    // find e = a + bn, so A = 1; B = -b; C = -a
    if (absf(sY2 - sqr(sY)) < 1e-10)
    {
      err -= 1; // all detections in same point in this plane
      v2 = 1e100;
    }
    else
    {
      b = (cntD * sXY - sX * sY)/(cntD * sY2 - sqr(sY));
      a = (sX - b * sY)/cntD;
      A2 = 1.0;
      B2 = -b;
      C2 = -a;
      // normalize
      l = hypot(A2, B2);
      A2 /= l;
      B2 /= l;
      C2 /= l;
      //
      // ... then for method 2
      lA = A2;
      lB = B2;
      lC = C2;
      v2 = variance(p);
    }
    // best fit wins
    if ((err != -2) and (v1 < v2))
    {
      lA = A1;
      lB = B1;
      lC = C1;
      //printf("U2Dline::fit %fv1, %fv2 (result=%s)\n", v1, v2, bool2str(err == 0));
      v2 = v1;
    }
    if ((err == 0) and (V != NULL))
      *V = v2;
  }
  else
    err = -1;
  return (err == 0);
}

/////////////////////////////////////////////////////////////////

float U2Dline::variance(float fx[], float fy[], int count, float * E)
{ // Returns variance and mean distance for these points from the line
  float result;
  float sd = 0.0;
  float s2 = 0.0;
  float d;
  int i;
  if (count > 1)
  {
    for (i = 0; i < count; i++)
    {
      d = distanceSigned(fx[i], fy[i]);
      sd += d;
      s2 += sqr(d);
    }
    sd /= count;
    if (E != NULL)
      *E = sd;
    result = s2/(float(count)-1.0);// - sqr(sd);
  }
  else
    result = -1.0;
  return result;
}

float U2Dline::variance(std::vector<cv::Point> &p, float * E)
{ // Returns variance and mean distance for these points from the line
  float result;
  float sd = 0.0;
  float s2 = 0.0;
  float d;
  int i;
  int count = p.size();
  if (count > 1)
  {
    for (i = 0; i < count; i++)
    {
      cv::Point v = p.at(i);
      d = distanceSigned(v.x, v.y);
      sd += d;
      s2 += sqr(d);
    }
    sd /= count;
    if (E != NULL)
      *E = sd;
    result = s2/(float(count)-1.0);// - sqr(sd);
  }
  else
    result = -1.0;
  return result;
}

//////////////////////////////////////////////////////////////////////


void U2Dline::getPV(float * Px, float * Py, float * Vx, float * Vy)
{ // returns line equation in point - vector format
  // [x,y] = [Px,Py] + [Vx,Vy]*t
  *Vx = lB;
  *Vy = -lA;
  if (absf(lA) < absf(lB))
  {
    *Px = 0.0;
    *Py = -lC/lB;
  }
  else
  {
    *Px = -lC/lA;
    *Py = 0.0;
  }
}


//////////////////////////////////////////////////////////////////
/**
Set line from point-vector format */
int U2Dline::setPV(const float Px, const float Py, const float Vx, const float Vy)
{
  float C = Vy * Px - Vx * Py;
  return set(-Vy, Vx, C);
}

//////////////////////////////////////////////////////////////////

/**
Set from two points * /
int U2Dline::set2P(const float x1, const float y1, const float x2, const float y2)
{
  return setPV(x1, y1, x2 - x1, y2 - y1);
} */

/////////////////////////////////////////////////////////////////

bool U2Dline::getCrossing(U2Dline L2, float * x, float * y)
{ // get crossing with L2 and this line
  bool result = false;
  const float zero = 1e-27;
  float N;
  //
  if (absf(A()) > absf(B()))
  {
    N = L2.B() - L2.A() * B() / A();
    result = (absf(N) > zero);
    if (result)
    {
      *y = (L2.A() * C() / A() - L2.C())/N;
      *x = -B()/A()* *y - C()/A();
    }
  }
  else
  {
    N = L2.A() - L2.B() * A() / B();
    result = (absf(N) > zero);
    if (result)
    {
      *x = (L2.B() * C() / B() - L2.C())/N;
      *y = -A()/B() * *x - C()/B();
    }
  }
  //
  return result;
}

/////////////////////////////////////////////////////////////////

void U2Dline::getOnLine(const float px, const float py, float * lx, float * ly)
{ // Get nearest point on line to this point.
  float w;
  //
  if (lA < lB)
  { // do not divide by A
    w = lB * lB * px - lA * lC - lA * lB * py;
    *lx = w;
    *ly = (-lC - lA * w)/lB;
  }
  else
  { // do not divide by B
    w = lA * lA * py - lB * lC - lA * lB * px;
    *lx = (-lC - lB * w)/lA;
    *ly = w;
  }
  //
  //return 0;
}

/////////////////////////////////////////////////////////////////

void U2Dline::print(char * prestring)
{
  printf("%s A: %f, B: %f, C: %f\n", prestring, lA, lB, lC);
}

/////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

/*
U2Dlined::U2Dlined()
{ // default line x = 0
  lA = 1.0;
  lB = 0.0;
  lC = 0.0;
}

/////////////////////////////////////////////////////////////////////


bool U2Dlined::setXline(double sX, double sY,
                       double sX2, double sXY,
                       int count)
{
  bool result;
  double a, b, l, cnt;
  //
  result = (count > 0) and (absf(sX2 - sqr(sX)) > 1e-10);
  if (result)
  {
    cnt = double(count);
    b = (cnt * sXY - sX * sY)/(cnt * sX2 - sqr(sX));
    a = (sY - b * sX)/cnt;
    lA = -b;
    lB = 1.0;
    lC = -a;
    // normalize
    l = hypot(lA, lB);
    lA /= l;
    lB /= l;
    lC /= l;
  }
  //
  return result;
}

//////////////////////////////////////////////////////////////////////////

bool U2Dlined::setYline(double sX, double sY,
                       double sY2, double sXY,
                       int count)
{
  bool result;
  double a, b, l, cnt;
  //
  result = (count > 0) and (absf(sY2 - sqr(sY)) > 1e-10);
  {
    cnt = double(count);
    b = (cnt * sXY - sX * sY)/(cnt * sY2 - sqr(sY));
    a = (sX - b * sY)/cnt;
    lA = 1.0;
    lB = -b;
    lC = -a;
    // normalize
    l = hypot(lA, lB);
    lA /= l;
    lB /= l;
    lC /= l;
  }
  //
  return result;
}

//////////////////////////////////////////////////////////////////////////

int U2Dlined::set(double fx[], double fy[], int count, double * V)
{ // find best line equation Ax + By + C = 0; from points in fx[i],fy[i], i=0..count-1.
  // scaled so that sqrt(A*A + B*B) = 1.0 (unit vector)
  // returns err = 0 for success
  int err = 0;
  int i;
  //
  double minX = 1e10, maxX = -1e10; // east
  double minY = 1e10, maxY = -1e10; // north
  double y, x;
  //
  double sX = 0.0, sY = 0.0, sXY = 0.0, sX2 = 0.0, sY2 = 0.0;
  double cntD;
  double a, b;
  double l; // vector length
  // Method:
  // use least square distance parallel with
  // one of the axes, base equation is: y = a + bx
  // square distance is (y - (a + bx))^2 summed for all (x,y),
  // partial derivative must be 0 for both a and b
  // a and b are then found.
  // if slope is more than approx. 45 deg then X and Y are
  // exchanged to base equation x = a + by
  if (count > 1)
  { // OK to calculate statistics
    // loop all detections in cluster
    for (i = 0; i < count; i++)
    { // get statistics
      x = fx[i];
      y = fy[i];
      //
      if (y > maxY) maxY = y;
      if (y < minY) minY = y;
      if (x > maxX) maxX = x;
      if (x < minX) minX = x;
      // statistics for line extimation in horizontal plane
      sX += x;
      sY += y;
      sXY += x * y;
      sX2 += sqr(x);
      sY2 += sqr(y);
    }
    cntD = double(count);
    if ((maxX - minX) > (maxY - minY))
    { // more x oriented. A may be zero so find A and C
      // find y = a + bx, so B = 1; A=-b; C =-a;
      if (absf(sX2 - sqr(sX)) < 1e-10)
        err = -2; // all detections in same point in this plane
      else
      {
        b = (cntD * sXY - sX * sY)/(cntD * sX2 - sqr(sX));
        a = (sY - b * sX)/cntD;
        lA = -b;
        lB = 1.0;
        lC = -a;
      }
    }
    else
    { // more y oriented. B may be zero, so find B and C
      // find e = a + bn, so A = 1; B = -b; C = -a
      if (absf(sY2 - sqr(sY)) < 1e-10)
        err = -2; // all detections in same point in this plane
      else
      {
        b = (cntD * sXY - sX * sY)/(cntD * sY2 - sqr(sY));
        a = (sX - b * sY)/cntD;
        lA = 1.0;
        lB = -b;
        lC = -a;
      }
    }
    // normalize
    l = hypot(lA, lB);
    lA /= l;
    lB /= l;
    lC /= l;
  }
  else
    err = -1;
  if (V != NULL)
    *V = variance(fx, fy, count);
  return err;
}

/////////////////////////////////////////////////////////////////

bool U2Dlined::fit(double fx[], double fy[], int count, double * V)
{ // find best line equation Ax + By + C = 0; from points in fx[i],fy[i], i=0..count-1.
  // scaled so that sqrt(A*A + B*B) = 1.0 (unit vector)
  // returns err = 0 for success
  int err = 0;
  int i;
  double y, x;
  double sX = 0.0, sY = 0.0, sXY = 0.0, sX2 = 0.0, sY2 = 0.0;
  double cntD;
  double a, b;
  double l; // vector length
  double v1, v2;
  double A1 = 0.0, B1 = 0.0, C1 = 0.0;
  double A2, B2, C2;
  // Method:
  // use least square distance parallel with
  // one of the axes, base equation is: y = a + bx
  // square distance is (y - (a + bx))^2 summed for all (x,y),
  // partial derivative must be 0 for both a and b
  // a and b are then found.
  // if slope is more than approx. 45 deg then X and Y are
  // exchanged to base equation x = a + by
  if (count > 1)
  { // OK to calculate statistics
    // loop all detections in cluster
    for (i = 0; i < count; i++)
    { // get statistics
      x = fx[i];
      y = fy[i];
      // statistics for line extimation in horizontal plane
      sX += x;
      sY += y;
      sXY += x * y;
      sX2 += sqr(x);
      sY2 += sqr(y);
    }
    cntD = double(count);
    // find y = a + bx, so B = 1; A=-b; C =-a;
    if (absf(sX2/count - sqr(sX/count)) < 1e-10)
    {
      err -= 1; // all detections in same point in this plane
      v1 = 1e100;
    }
    else
    {
      b = (cntD * sXY - sX * sY)/(cntD * sX2 - sqr(sX));
      a = (sY - b * sX)/cntD;
      A1 = -b;
      B1 = 1.0;
      C1 = -a;
      // normalize and store in temp-line A, B and C
      l = hypot(A1, B1);
      A1 /= l;
      B1 /= l;
      C1 /= l;
      // get variance for method 1 first
      lA = A1;
      lB = B1;
      lC = C1;
      v1 = variance(fx, fy, count);
    }
    // find e = a + bn, so A = 1; B = -b; C = -a
    if (absf(sY2/count - sqr(sY/count)) < 1e-10)
    {
      err -= 1; // all detections in same point in this plane
      v2 = 1e100;
    }
    else
    {
      b = (cntD * sXY - sX * sY)/(cntD * sY2 - sqr(sY));
      a = (sX - b * sY)/cntD;
      A2 = 1.0;
      B2 = -b;
      C2 = -a;
      // normalize
      l = hypot(A2, B2);
      A2 /= l;
      B2 /= l;
      C2 /= l;
      //
      // ... then for method 2
      lA = A2;
      lB = B2;
      lC = C2;
      v2 = variance(fx, fy, count);
    }
    // best fit wins
    if ((err != -2) and (v1 < v2))
    {
      lA = A1;
      lB = B1;
      lC = C1;
      //printf("U2Dlined::fit %fv1, %fv2 (result=%s)\n", v1, v2, bool2str(err == 0));
      v2 = v1;
    }
    if ((err == 0) and (V != NULL))
      *V = v2;
  }
  else
    err = -1;
  return (err == 0);
}

/////////////////////////////////////////////////////////////////

bool U2Dlined::fitV(double fx[], double fy[], int count, double * V)
{ // find best line equation Ax + By + C = 0; from points in fx[i],fy[i], i=0..count-1.
  // scaled so that sqrt(A*A + B*B) = 1.0 (unit vector)
  // returns err = 0 for success
  int err = 0;
  int i;
  double y, x;
  double sX = 0.0, sY = 0.0, sXY = 0.0, sX2 = 0.0, sY2 = 0.0;
  double cntD;
  double a, b;
  double l; // vector length
  double v1, v2;
  double A1 = 0.0, B1 = 0.0, C1 = 0.0;
  double A2, B2, C2;
  double v3,v4;
  // Method:
  // use least square distance parallel with
  // one of the axes, base equation is: y = a + bx
  // square distance is (y - (a + bx))^2 summed for all (x,y),
  // partial derivative must be 0 for both a and b
  // a and b are then found.
  // if slope is more than approx. 45 deg then X and Y are
  // exchanged to base equation x = a + by
  if (count > 1)
  { // OK to calculate statistics
    // loop all detections in cluster
    for (i = 0; i < count; i++)
    { // get statistics
      x = fx[i];
      y = fy[i];
      // statistics for line extimation in horizontal plane
      sX += x;
      sY += y;
      sXY += x * y;
      sX2 += sqr(x);
      sY2 += sqr(y);
    }
    cntD = double(count);
    // find y = a + bx, so B = 1; A=-b; C =-a;
    if (absf(sX2/count - sqr(sX/count)) < 1e-10)
    {
      err -= 1; // all detections in same point in this plane
      v1 = 1e100;
    }
    else
    {
      b = (cntD * sXY - sX * sY)/(cntD * sX2 - sqr(sX));
      a = (sY - b * sX)/cntD;
      A1 = -b;
      B1 = 1.0;
      C1 = -a;
      // normalize and store in temp-line A, B and C
      l = hypot(A1, B1);
      A1 /= l;
      B1 /= l;
      C1 /= l;
      // get variance for method 1 first
      lA = A1;
      lB = B1;
      lC = C1;
      v1 = variance(fx, fy, count);
      // debug
      v3 = (sXY - sX * sY/cntD - b * (sY2 - sqr(sY)/cntD))/(cntD - 2);
      printf("Line X: a=%f, b=%f, c=%f, v=%f\n", A1, B1, C1, v3);
      // debug end
    }
    // find e = a + bn, so A = 1; B = -b; C = -a
    if (absf(sY2/count - sqr(sY/count)) < 1e-10)
    {
      err -= 1; // all detections in same point in this plane
      v2 = 1e100;
    }
    else
    {
      b = (cntD * sXY - sX * sY)/(cntD * sY2 - sqr(sY));
      a = (sX - b * sY)/cntD;
      A2 = 1.0;
      B2 = -b;
      C2 = -a;
      // normalize
      l = hypot(A2, B2);
      A2 /= l;
      B2 /= l;
      C2 /= l;
      //
      // ... then for method 2
      lA = A2;
      lB = B2;
      lC = C2;
      v2 = variance(fx, fy, count);
      // debug
      v4 = (sXY - sX * sY/cntD - b * (sX2 - sqr(sX)/cntD))/(cntD - 2);
      printf("Line Y: a=%f, b=%f, c=%f - v=%f\n", A2, B2, C2, v4);
      // debug end
    }
    // best fit wins
    if ((err != -2) and (v1 < v2))
    {
      lA = A1;
      lB = B1;
      lC = C1;
      //printf("U2Dlined::fit %fv1, %fv2 (result=%s)\n", v1, v2, bool2str(err == 0));
      v2 = v1;
    }
    if ((err == 0) and (V != NULL))
      *V = v2;
  }
  else
    err = -1;
  return (err == 0);
}

/////////////////////////////////////////////////////////////////

double U2Dlined::variance(double fx[], double fy[], int count, double * E)
{ // Returns variance and mean distance for these points from the line
  double result;
  double sd = 0.0;
  double s2 = 0.0;
  double d;
  int i;
  if (count > 0)
  {
    for (i = 0; i < count; i++)
    {
      d = distanceSigned(fx[i], fy[i]);
      sd += d;
      s2 += sqr(d);
    }
    sd /= count;
    if (E != NULL)
      *E = sd;
    result = s2/count;// - sqr(sd);
  }
  else
    result = -1.0;
  return result;
}

//////////////////////////////////////////////////////////////////////


void U2Dlined::getPV(double * Px, double * Py, double * Vx, double * Vy)
{ // returns line equation in point - vector format
  // [x,y] = [Px,Py] + [Vx,Vy]*t
  *Vx = lB;
  *Vy = -lA;
  if (absf(lA) < absf(lB))
  {
    *Px = 0.0;
    *Py = -lC/lB;
  }
  else
  {
    *Px = -lC/lA;
    *Py = 0.0;
  }
}


//////////////////////////////////////////////////////////////////

int U2Dlined::setPV(const double Px, const double Py, const double Vx, const double Vy)
{
  double C = Vy * Px - Vx * Py;
  return set(-Vy, Vx, C);
}

//////////////////////////////////////////////////////////////////

int U2Dlined::setPH(const double Px, const double Py, const double h)
{
  double Vx = cos(h);
  double Vy = sin(h);
  double C = Vy * Px - Vx * Py;
  return set(-Vy, Vx, C);
}

/////////////////////////////////////////////////////////////////

bool U2Dlined::getCrossing(U2Dlined L2, double * x, double * y)
{ // get crossing with L2 and this line
  bool result = false;
  const double zero = 1e-27;
  double N;
  //
  if (fabs(A()) > fabs(B()))
  {
    N = L2.B() - L2.A() * B() / A();
    result = (fabs(N) > zero);
    if (result)
    {
      *y = (L2.A() * C() / A() - L2.C())/N;
      *x = -B()/A()* *y - C()/A();
    }
  }
  else if (fabs(L2.B()) > zero)
  {
    N = L2.A() - L2.B() * A() / B();
    result = (fabs(N) > zero);
    if (result)
    {
      *x = (L2.B() * C() / B() - L2.C())/N;
      *y = -A()/B() * *x - C()/B();
    }
  }
  else if (fabs(L2.A()) > zero)
  { // L2.B == 0
    result = (fabs(B()) > zero);
    *y = (A() * L2.C() / L2.A() - C()) / B();
    *x = -L2.C() / L2.A();
  }
  else
    result = false;
  //
  return result;
}

/////////////////////////////////////////////////////////////////

void U2Dlined::getOnLine(const double px, const double py, double * lx, double * ly)
{ // Get nearest point on line to this point.
  double w;
  //
  if (lA < lB)
  { // do not divide by A
    w = lB * lB * px - lA * lC - lA * lB * py;
    *lx = w;
    *ly = (-lC - lA * w)/lB;
  }
  else
  { // do not divide by B
    w = lA * lA * py - lB * lC - lA * lB * px;
    *lx = (-lC - lB * w)/lA;
    *ly = w;
  }
  //
  //return 0;
}

/////////////////////////////////////////////////////////////////

void U2Dlined::print(const char * prestring)
{
  printf("%s A: %f, B: %f, C: %f\n", prestring, lA, lB, lC);
}


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

U2Dpos U2Dpos::operator = (U2Dseg seg)
{
  x = seg.x; y=seg.y;
  return *this;
}

////////////////////////

UMatrix4 U2Dpos::asRow3()
{
  UMatrix4 m(1,3);
  m.set(x, y, 1.0);
  return m;
}

/////////////////

UMatrix4 U2Dpos::asCol3()
{
  UMatrix4 m(3, 1);
  m.set(x, y, 1.0);
  return m;
}

/////////////////

UMatrix4 U2Dpos::asRow2()
{
  UMatrix4 m(1,2);
  m.set(x, y);
  return m;
}

/////////////////

UMatrix4 U2Dpos::asCol2()
{
  UMatrix4 m(2, 1);
  m.set(x, y);
  return m;
}


/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////

void U2Dseg::clear()
{
  lA = 1.0;
  lB = 0.0;
  lC = 0.0;
  length = 0.0;
  x = 0.0;
  y = 0.0;
}

///////////////////////////////////////////////////////////

void U2Dseg::setFromPoints(double x1, double y1, double x2, double y2)
{
  x = x1;
  y = y1;
  setPH(x, y, atan2(y2 - y1, x2 - x1));
  length = hypot(y2 - y1, x2 - x1);
}

///////////////////////////////////////////////////////////
*/
// void U2Dseg::setFromPose(double x1, double y1, double h1, double len /*= 1.0 */
// {
//   x = x1;
//   y = y1;
//   setPH(x, y, h1);
//   length = len;
// }

///////////////////////////////////////////////////////////

// const char * U2Dseg::print(const char * prestring, char * buff, const int buffCnt)
// {
//   char *p1 = buff;
//   int n = 0;
//   snprintf(p1, buffCnt - n, "%s start (%g,%g) A=%g B=%g, C=%g, length=%g\n",
//          prestring, x, y, lA, lB, lC, length);
//   return buff;
// }
// 
// ///////////////////////////////////////////////////////////
// 
// double U2Dseg::getDistanceSigned(double px, double py, int * where)
// {
//   U2Dseg linxy;
//   U2Dpos p1, p2;
//   double dist;
//   double t;
//   int w;
//   //
//   dist = distanceSigned(px, py);
//   w = 0;
//   // get closest point on line as line parameter t
//   t = getPositionOnLine(px, py);
//   if (t <= 0.0)
//   { // closest to ref point
//     dist = hypot(x - px, y - py) * signofd(dist);
//     w = 1; // to end 1
//   }
//   else if (t > length)
//   { // closest to other end
//     p2 = getOtherEnd();
//     dist = hypot(p2.x - px, p2.y - py) * signofd(dist);
//     w = 2; // to end 2
//   }
//   if (where != NULL)
//     *where = w;
//   return dist;
// }
// 
// /////////////////////////////////////////////////////
// 
// bool U2Dseg::isCrossing(U2Dseg * other, U2Dpos * crossing)
// {
//   bool result;
//   //double t;
//   U2Dpos b;
//   //
//   result = getCrossing(*other, &crossing->x, &crossing->y);
//   if (result)
//   { // crossing is inside this x,y line segment
//     b = getOtherEnd();
//     result = crossing->x >= fmin(b.x, x) and crossing->x <= fmax(b.x, x) and
//         crossing->y >= fmin(b.y, y) and crossing->y <= fmax(b.y, y);
// /*    t = getPositionOnLine(crossing);
//     result = (t >= 0) and (t <= length);*/
//   }
//   if (result)
//   { // is crossing inside other x,y segment
//     b = other->getOtherEnd();
//     result = crossing->x >= fmin(b.x, other->x) and crossing->x <= fmax(b.x, other->x) and
//         crossing->y >= fmin(b.y, other->y) and crossing->y <= fmax(b.y, other->y);
// /*    t = other->getPositionOnLine(crossing);
//     result = (t >= 0) and (t <= other->length);*/
//   }
//   return result;
// }
// 
// ///////////////////////////////////////////////////////
// 
// double U2Dseg::getPositionOnLine(U2Dpos * point)
// { // Get get parameter value for nearest position on
//   // line to this point.
//   // parameter is the 't' in line equation [x,y] = vec * t + pos.
//   // assumed that 'vec' is a unit vector i.e. [A,B]
//   // Returns t value
//   return -lB * (x - point->x) + lA * (y - point->y);
// }
// 
// ///////////////////////////////////////////////////////
// 
// double U2Dseg::getPositionOnLine(U2Dpos point)
// { // Get get parameter value for nearest position on
//   // line to this point.
//   // parameter is the 't' in line equation [x,y] = vec * t + pos.
//   // assumed that 'vec' is a unit vector i.e. [A,B]
//   // Returns t value
//   return -lB * (x - point.x) + lA * (y - point.y);
// }
// 
// ///////////////////////////////////////////////////////
// 
// double U2Dseg::getPositionOnLine(double px, double py)
// { // Get get parameter value for nearest position on
//   // line to this point.
//   // parameter is the 't' in line equation [x,y] = vec * t + pos.
//   // assumes that [lA,lB] is a unit vector
//   // Returns t value
//   return -lB * (x - px) + lA * (y - py);
// }
// 
// ////////////////////////////////////////////////////////
// 
// U2Dpos U2Dseg::getPositionOnLine(const double t)
// { // Get position with this parameter value.
//   // Given a 't' value for a parametized line [x,y] = vec * t + pos,
//   // The [x,y] position is returned.
//   U2Dpos result;
//   //
//   result.x = lB * t + x;
//   result.y = -lA * t + y;
//   //
//   return result;
// }
// 
// ///////////////////////////////////////////////////////
// 
// int U2Dseg::getCircleCrossings(double cx, double cy, double r,
//                                 double * t1, double * t2)
// { // calculated from
//   // ((Px + Vx * t) - Cx)^2 + ((Px + Vx * t) - Cx)^2 = r^2
//   // and solve this 2nd order equation
//   // (Vx� + Vy�)t� + 2*((Px-Cx)Vx + (Py-Cy)Vy)t + (Px-Cx)� + (Py - Cy)� - r� = 0
//   double bb, cc;
//   double kx, ky; // difference from line origin to circle center
//   double d; // determinant
//   int result;
//   //
//   kx = x - cx;
//   ky = y - cy;
//   bb = 2.0 * (kx * lB - ky * lA);
//   cc = sqr(kx) + sqr(ky) - sqr(r);
//   d = sqr(bb) - 4.0 * cc;
//   if (d < 0.0)
//     result = 0;
//   else
//   {
//     result = 2;
//     d = sqrt(d);
//     *t1 = (-bb + d)/(2.0);
//     *t2 = (-bb - d)/(2.0);
//   }
//   return result;
// }
// 
// /////////////////////////////////////////
// 
// void U2Dseg::shiftLeft(double dist)
// {
//   x += dist * lA;
//   y += dist * lB;
//   U2Dlined::shiftLeft(dist);
// }
// 
// /////////////////////////////////////////
// 
// void U2Dseg::shiftRight(double dist)
// {
//   x -= dist * lA;
//   y -= dist * lB;
//   U2Dlined::shiftRight(dist);
// }
// 
// /////////////////////////////////////////
// 
// const char * U2Dseg::codeXml(const char * name, char * buff, const int buffCnt, const char * extraAtt)
// {
//   U2Dpos end;
//   char * p1 = buff;
//   int n = 0;
//   //
//   snprintf(p1, buffCnt, "<line x=\"%.5f\" y=\"%.5f\" th=\"%.6f\" l=\"%.4f\"",
//            x, y, heading(), length);
//   n += strlen(p1);
//   p1 = &buff[n];
//   if (name != NULL)
//   {
//     snprintf(p1, buffCnt - n, " name=\"%s\"", name);
//     n += strlen(p1);
//     p1 = &buff[n];
//   }
//   if (extraAtt != NULL)
//   {
//     snprintf(p1, buffCnt - n, " %s", extraAtt);
//     n += strlen(p1);
//     p1 = &buff[n];
//   }
//   snprintf(p1, buffCnt - n, "/>\n");
//   return buff;
// }

