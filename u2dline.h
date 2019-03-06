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

#ifndef U2DLINE_H
#define U2DLINE_H

#include <stdio.h>
#include <math.h> // hypot(x,y)
#include <opencv2/opencv.hpp>

//#include "u3d.h"

/**
Tests if point (m,n) is within the square surrounded
by (x1,y1), (x2,y2), (x3,y3) and (x4,y4) taken
either clockwise or counter clockwise.
Returns true if within. */
bool isWithinSquare(float m, float n,
             float x1, float y1, float x2, float y2,
             float x3, float y3, float x4, float y4);


///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////


/**Functions related with 2D lines
   based on line equation Ax + By + C = 0. <br>
   sqr(A) + sqr(B) is always 1.0.
   Calculations are in float precition.
   - you should probably rather use U2Dlined that is in double precition, and
   is maintained with more functionality.
  *@author Christian Andersen
  */

class U2Dline
{
protected:
  /**
  Parameters in line equation Ax + By + C = 0 */
  float lA, lB, lC;
public:
  //
  /**
  Constructor (A=1.0, B=0.0, C=0.0 line */
  U2Dline();
  /**
  Construct from A, B, C) */
  U2Dline(const float A, const float B, const float C)
       {set(A, B, C);};
  /**
  Construct from 2 points */
  U2Dline(const float x1, const float y1, const float x2, const float y2)
       {set2P(x1, y1, x2, y2);};
  /**
  Destructor */
  ~U2Dline();
  /**
  Set line from known A, B, C values in the line equation Ax + By + C = 0 */
  inline int set(const float A, const float B, const float C)
  {
    int err = 0;
    float d = hypot(A,B);
    if (d > 1e-50)
    {
      lA = A/d;
      lB = B/d;
      lC = C/d;
    }
    else
    { // not a proper line
      lA = A;
      lB = B;
      lC = C;
      err = -1;
    }
    return err;
  };
  /**
  Set as best fit line through a series of points.
  if V != NULL then the variance is returned here */
  int set(float fx[], float fy[], int count, float * V = NULL);
  /**
   * same as above, but with a vector input */
  int set(std::vector< cv::Point >& P, float* V)
;

  /**
  Set as best fit line through a series of points.
  if V != NULL then the variance is returned here */
  bool fit(float fx[], float fy[], int count, float * V = NULL);
  /**
   * same as above, but with a vector input */
  bool fitV(std::vector<cv::Point> &p, float * V = NULL);
  
  /**
  Set line parameters, assuming that the line is more or less parallel with
  the x-axis */
//   bool setXline(double sX, double sY,
//                 double sX2, double sXY,
//                 int count);
//   /**
//   Set line parameters, assuming that the line is more or less parallel with
//   the y-axis */
//   bool setYline(double sX, double sY,
//                 double sY2, double sXY,
//                 int count);
  /**
  Get mean and variance for the distance from these points to the line
  if E == NULL then mean is not returned.
  if no points are given, then -1.0 is returned */
  float variance(float fx[], float fy[], int count, float * E = NULL);
  /**
   * same as above, but with a vector input */
  float variance(std::vector<cv::Point> &p, float * E = NULL);
  
  /**
  Get A parameter in Ax + By + C = 0 */
  inline float A() {return lA;};
  /**
  Get B parameter in Ax + By + C = 0 */
  inline float B() {return lB;};
  /**
  Get C parameter in Ax + By + C = 0 */
  inline float C() {return lC;};
  /**
  Get heading of the line in radians */
  inline double heading()
  { return atan2(-lA, lB); };
  /**
  Get line as in position - vector format on the form
    [x,y] = [Px,Py] + [Vx,Vy]*t
  Either Px or Py will be zero. */
  void getPV(float * Px, float * Py, float * Vx, float * Vy);
  /**
  Set line from point-vector format.
  returns -1 if zero vector. */
  int setPV(const float Px, const float Py, const float Vx, const float Vy);
  /**
  Set from two points.
  returns -1 if too close. */
  inline int set2P(const float x1, const float y1, const float x2, const float y2)
    {
      return setPV(x1, y1, x2 - x1, y2 - y1);
    };
  /**
  Returns distance from point to line, signed
  so that distance on one side is negative. */
  inline float distanceSigned(const float x, const float y)
    {
      return (lA*x + lB*y + lC); // sqrt(sqr(lA) + sqr(lB)) == 1;
    };
  // get other coordinate
  inline float getY(float x)
  {
    float y;
    if (fabs(lB) > 1e-10)
      y = (-lA*x - lC)/lB;
    else
      y = 0.0;
    return y;
  }
  // get other coordinate
  inline float getX(float y)
  {
    float x;
    if (fabs(lA) > 1e-10)
      x = (-lB*y - lC)/lA;
    else
      x = 0.0;
    return x;
  }
  /**
  Find crossing between two lines.
  Returns -1 if lines are parallel.
  using linear algebra solve */
  //int getCrossingA(U2Dline L2, float * x, float * y);
  /**
  Find corssing between this line and line L2.
  Returns the result in x and y if the lines are not
  parallel. If so the function returns false. */
  bool getCrossing(U2Dline L2, float * x, float * y);
  /**
  \brief Get nearest point on line to this point.
  Found by minimizing distance <br>
  \f$ d = \sqrt{(x_l - x)^2+(y_l - y)^2} \f$. <br>
  where \f$(x_l, y_l)\f$ is point on line expressed by parameter t. <br>
  \f$ [x_l,y_l] = [P_x,P_y] + [V_x,V_y] * t \f$.
  Insert into first equation. <br>
  \f$ d^2 = (P_x + V_x * t - x)^2 + (P_y + V_y * t - y)^2 \f$. <br>
  find \f$ \frac{d d^2}{dt} = 0\f$ gives <br>
  \f$ t = (-P_x * V_x + V_x * x  - P_y * V_y + V_y * y)/((V_x^2 + V_y^2) \f$.
  Denominator is equal to 1.0 and can be ignored.
  Setting t into point-vector line expression gives nearest (lx,ly) point on line.
  Px, Py, Vx, Vy can then be replaced by A,B,C to simplify expression.
  \param x,y is the input x,y position
  \param lx,ly is the found position on the line.*/
  void getOnLine(const float x, const float y, float * lx, float * ly);
  /**
  Print A, B, C to console */
  void print(char * prestring);
  /**
  Get line described as alpha,r, where r is distance to line from origo and
  alpha is the angle to the thpoint on the line closest to the origo.
  \param a is where the alpha angle is returned.
  \param r is where the distance to the line is returned */
  inline void getARLine(float * a, float * r)
  {
    *a = atan2(lB, lA);
    *r = -lC;
    if (*r < 0.0)
    {
      *r = -*r;
      if (*a < 0.0)
        *a = *a + M_PI;
      else
        *a = *a - M_PI;
    }
  };
  
  inline U2Dline operator = (U2Dline src)
  {
    set(src.A(), src.B(), src.C());
    return *this;
  };
  
};

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

// class U2Dseg;
// /**
//  * A simple class for a 2D position in double precition. */
// class U2Dpos
// {
// public:
//   /**
//    * initialized constructor */
//   U2Dpos(double ix, double iy)
//   { x= ix; y = iy; };
// 
//   /**
//    * uninitialized constructor */
//   U2Dpos() {};
//   /**
//    * clear 2D value */
//   inline void clear()
//   {
//     x = 0.0;
//     y = 0.0;
//   }
//   /**
//    * Length of vector */
//   inline double length()
//   { return hypot(x, y); };
//   /**
//    * Length of vector */
//   inline double heading()
//   { return atan2(y, x); };
//   /**
//    * set from 3D position */
//   inline U2Dpos operator = (UPosition pos)
//   {
//     x = pos.x; y=pos.y;
//     return *this;
//   };
//   /**
//   * set from 2D line segment position */
//   U2Dpos operator = (U2Dseg seg);
//   /**
//    * Set 2D value */
//   void set(double ix, double iy)
//   {
//     x = ix; y = iy;
//   };
//   /**
//    * get distance or length of this 2D vector */
//   inline double dist()
//   {
//     return hypot(x, y);
//   };
//   /**
//    * get distance from this to other position (2D)
//    \param b is the other position.
//    \returns length between the two points. */
//   inline double dist(UPosition b)
//   {
//     return hypot(b.x - x, b.y - y);
//   };
//   /**
//    * get distance from this to other position (2D)
//    \param b is the other position.
//    \returns length between the two points. */
//   inline double dist(U2Dpos b)
//   {
//     return hypot(b.x - x, b.y - y);
//   };
//   /**
//   \Returns position as homogenous [x,y,1] row. */
//   UMatrix4 asRow3();
//   /**
//   \Returns position as homogenous [x,y,1] column. */
//   UMatrix4 asCol3();
//   /**
//   \Returns position as [x,y] row. */
//   UMatrix4 asRow2();
//   /**
//   \Returns position as [x,y] column. */
//   UMatrix4 asCol2();
// 
// public:
//   /**
//    * positiin */
//     double x;
//     double y;
// };
// 
// ////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////
// 
// /**
// This is a 2D line with parameters in double precition
//    based on line equation Ax + By + C = 0. \n
//    sqr(A) + sqr(B) is always 1.0.
//    and should be kept that way to simplify some calculations.
// */
// 
// class U2Dlined
// {
//   public:
//     //
//   /**
//   Constructor (A=1.0, B=0.0, C=0.0 line */
//   U2Dlined();
//   /**
//   Construct from A, B, C) */
//   U2Dlined(const double A, const double B, const double C)
//   { set(A, B, C); };
//   /**
//   Construct from A, B, C) */
//   U2Dlined(const double a, const double r)
//   { setAR(a, r); };
//   /**
//   Construct from 2 points */
//   U2Dlined(const double x1, const double y1, const double x2, const double y2)
//   { set2P(x1, y1, x2, y2); };
//   /**
//   Destructor */
//   virtual ~U2Dlined() {};
//   /**
//   Set line from known A, B, C values in the line equation Ax + By + C = 0 */
//   inline int set(const double A, const double B, const double C)
//   {
//     int err = 0;
//     double d = hypot(A,B);
//     if (d > 1e-50)
//     {
//       lA = A/d;
//       lB = B/d;
//       lC = C/d;
//     }
//     else
//     { // not a proper line
//       lA = A;
//       lB = B;
//       lC = C;
//       err = -1;
//     }
//     return err;
//   };
//   /**
//     Set as best fit line through a series of points.
//     if V != NULL then the variance is returned here */
//     int set(double fx[], double fy[], int count, double * V = NULL);
//   /**
//     Set as best fit line through a series of points.
//     if V != NULL then the variance is returned here */
//     bool fit(double fx[], double fy[], int count, double * V = NULL);
//     bool fitV(double fx[], double fy[], int count, double * V);
//   /**
//     Set line parameters, assuming that the line is more or less parallel with
//     the x-axis */
//     bool setXline(double sX, double sY,
//                   double sX2, double sXY,
//                   int count);
//   /**
//   Set line parameters, assuming that the line is more or less parallel with
//   the y-axis */
//   bool setYline(double sX, double sY,
//                 double sY2, double sXY,
//                 int count);
//   /**
//   Get mean and variance for the distance from these points to the line
//   if E == NULL then mean is not returned.
//   if no points are given, then -1.0 is returned */
//   double variance(double fx[], double fy[], int count, double * E = NULL);
//   /**
//   Get A parameter in Ax + By + C = 0 */
//   inline double A() {return lA;};
//   /**
//   Get B parameter in Ax + By + C = 0 */
//   inline double B() {return lB;};
//   /**
//   Get C parameter in Ax + By + C = 0 */
//   inline double C() {return lC;};
//   /**
//    * Get line vextor X component */
//   inline double vx() {return lB;};
//   /**
//    * Get line vextor Y component */
//   inline double vy() {return -lA;};
//   /**
//   Get line heading in radians */
//   inline double heading()
//   { return atan2(-lA, lB); };
//   /**
//   Get line as in position - vector format on the form
//   [x,y] = [Px,Py] + [Vx,Vy]*t
//   Either Px or Py will be zero. */
//   void getPV(double * Px, double * Py,
//              double * Vx, double * Vy);
//   /**
//    * Set line from point-vector format.
//    * the resulting line is normalized, so that the A,B vector is a unit vector.
//    * \param Px is X-position (in meter)
//    * \param Py is Y position (in meter)
//    * \param Vx is X-component of heading vector (in meter)
//    * \param Vy is Y component of heading vector (in meter)
//   returns -1 if zero vector. */
//   int setPV(const double Px, const double Py,
//             const double Vx, const double Vy);
//   /**
//    * Set line from point-heading format.
//    * the resulting line is normalized, so that the A,B vector is a unit vector.
//    * \param Px is X-position (in meter)
//    * \param Py is Y position (in meter)
//    * \param h is heading of line in radians.
//    * \returns 0. */
//   int setPH(const double Px, const double Py,
//             const double h);
//   /**
//   Set from two points.
//   returns -1 if too close. */
//   inline int set2P(const double x1, const double y1,
//                     const double x2, const double y2)
//   {
//     return setPV(x1, y1, x2 - x1, y2 - y1);
//   };
//   /**
//   Returns distance from point to line.
//   To the left in vector p1->p2 direction (if set from points or point-vector),
//   the distance will be positive. */
//   inline double distanceSigned(const double x, const double y)
//   {
//     return (lA*x + lB*y + lC); // sqrt(sqr(lA) + sqr(lB)) == 1;
//   };
//   /**
//   Find crossing between this line and line L2.
//   Returns the result in x and y if the lines are not
//   parallel.
//    * \param line2 is the other line
//    * \param x,y is the crossing point, unchanged if not crossing.
//    * \returns false if lines are parallel. */
//   bool getCrossing(U2Dlined L2, double * x, double * y);
//   /**
//   Get nearest point on line to this point.
//   Found by minimizing distance <br>
//   \f$ d = \sqrt{(x_l - x)^2+(y_l - y)^2} \f$. <br>
//   where \f$(x_l, y_l)\f$ is point on line expressed by parameter t. <br>
//   \f$ [x_l,y_l] = [P_x,P_y] + [V_x,V_y] * t \f$.
//   Insert into first equation. <br>
//   \f$ d^2 = (P_x + V_x * t - x)^2 + (P_y + V_y * t - y)^2 \f$. <br>
//   find \f$ \frac{d d^2}{dt} = 0\f$ gives <br>
//   \f$ t = (-P_x * V_x + V_x * x  - P_y * V_y + V_y * y)/((V_x^2 + V_y^2) \f$.
//   Denominator is equal to 1.0 and can be ignored.
//   Setting t into point-vector line expression gives nearest (lx,ly) point on line.
//   Px, Py, Vx, Vy can then be replaced by A,B,C to simplify expression.
//   \param x,y is the input x,y position
//   \param lx,ly is the found position on the line.*/
//   void getOnLine(const double x, const double y, double * lx, double * ly);
//   /**
//   Print A, B, C to console */
//   void print(const char * prestring);
//   /**
//    * Shift the line this distance to the Right
//    * This moves the C parameter and the start position.
//    * \param dist is the distance shifted. */
//   virtual inline void shiftRight(double dist)
//   { lC += dist; };
//   /**
//    * Shift the line this distance to the Left
//    * This moves the C parameter and the start position.
//    * \param dist is the distance shifted. */
//   virtual inline void shiftLeft(double dist)
//   { lC -= dist; };
//   /**
//   Get line described as alpha,r, where r is distance to line from origo and
//   alpha is the angle to the thpoint on the line closest to the origo.
//   \param a is where the alpha angle is returned.
//   \param r is where the distance to the line is returned */
//   inline void getARLine(double * a, double * r)
//   {
//     *a = atan2(lB, lA);
//     *r = -lC;
//     if (*r < 0.0)
//     {
//       *r = -*r;
//       if (*a < 0.0)
//         *a = *a + M_PI;
//       else
//         *a = *a - M_PI;
//     }
//   }
//   /**
//   Sets the line from an alpha (a) and r notation.
//   like A = cos(a), B = sin(a), C = -r
//   \param a is the angle to the closest point on the line - from origo.
//   \param r is the distance to the closest point (from origo) to the line. */
//   inline void setAR(const double a, const double r)
//   {
//     lA = cos(a);
//     lB = sin(a);
//     lC = -r;
//   }
//   
// protected:
//   /**
//   Parameters in line equation Ax + By + C = 0 */
//   double lA, lB, lC;
// };
// 
// /**
// This is a 2D line with parameters in double precition */
// 
// class U2Dseg : public U2Dlined, public U2Dpos, public UDataBase
// {
// public:
//   /**
//    * Constructor */
//   U2Dseg()  {};
//   /**
//    * destructor */
//   virtual ~U2Dseg()  {};
//   /**
//   Get (end) type string for this structure */
//   virtual const char * getDataType()
//   {
//     return "2Dseg";
//   };
//   /**
//   Clear */
//   virtual void clear();
//   /**
//   Set line from two points.
//   Reference position is set from pos1, and other end is
//   set to pos2 so that "pos1 + vec * length" ends at pos2.  */
//   inline void setFromPoints(const UPosition * pos1, const UPosition * pos2)
//   { setFromPoints(pos1->x, pos1->y, pos2->x, pos2->y); };
//   /**
//   Set line from two points.
//   Reference position is set from pos1, and other end is
//   set to pos2 so that "pos1 + vec * length" ends at pos2.  */
//   inline void setFromPoints(UPosition pos1, UPosition pos2)
//   { setFromPoints(pos1.x, pos1.y, pos2.x, pos2.y); };
//   /**
//   Set line from two points.
//   Reference position is set from pos1, and other end is
//   set to pos2 so that "pos1 + vec * length" ends at pos2.  */
//   inline void setFromPoints(U2Dpos pos1, U2Dpos pos2)
//   { setFromPoints(pos1.x, pos1.y, pos2.x, pos2.y); };
//   /**
//    * Set line segment from points. a line segment is a start point, a direction vector and a length.
//    * \param x1,y1,z1 defines the first (start) point
//    * \param x2,y2,z2 defines the end point */
//   void setFromPoints(double x1, double y1,
//                        double x2, double y2);
//   /**
//    * Set 2D line segment from a pose (x1, y1, h1) with an optional length.
//    * \param x,y,h is the pose
//    * \param len is the optiomal length (default is 1). */
//   void setFromPose(double x1, double y1, double h1, double len = 1.0);
//   /**
//     Same as abowe */
//   const char * print(const char * prestring, char * buff, const int buffCnt);
//   /**
//   get heading of the line segment */
//   inline double heading()
//   { return U2Dlined::heading(); };
//   /**
//     Get distance from a 3D point to a line segemnt, i.e.
//     distance to the closest end or if closer to a point on the
//     line segment, then this distance. the distance is squared.
//     The optional second parameter returns the closest point (if != NULL)
//     where = 0 if on line segment, 1 if closest to 'pos', and 2 if
//     closest to 'other end' */
//   double getDistanceSigned(double px, double py, int * where);
//   /**
//     Get position of other end of line segment */
//   inline U2Dpos getOtherEnd()
//   { return getPositionOnLine(length); };
//   /**
//   Get position of first end of line segment as a 2D position */
//   inline U2Dpos getFirstEnd()
//   {
//     U2Dpos result(x, y);
//     return result;
//   };
//   /**
//     Is this line segment crossing the other segment,
//     \Returns true if crossing (within ends), and
//     The crossing is returned in 'crossing'
//     If no crossing is found, 'crossing' may be changed. */
//   bool isCrossing(U2Dseg * other, U2Dpos * crossing);
//   /**
//    * Get position on line this many meters from start position
//    * in direction of vector [A,B];
//    * \returns a positiuon on the line (but not neccesary on segment) */
//   U2Dpos getPositionOnLine(const double t);
//   /**
//    * Get position parameter on the line that is closest to this point
//    * \param point the point anywhere in space.
//    * \returns a parameter on the line as a parameter. this is the distance
//    * from the closest point oin the line to the line start, signed in the direction of the segment. */
//   double getPositionOnLine(U2Dpos * point);
//   /**
//    * Get position parameter on the line that is closest to this point
//    * \param point the point anywhere in space.
//    * \returns a parameter on the line as a parameter. this is the distance
//    * from the closest point oin the line to the line start, signed in the direction of the segment. */
//   double getPositionOnLine(U2Dpos point);
//   /**
//    * Get position parameter on the line that is closest to this point
//    * \param px,py the point anywhere in space.
//    * \returns a parameter on the line as a parameter. this is the distance
//    * from the closest point oin the line to the line start, signed in the direction of the segment. */
//   double getPositionOnLine(double px, double py);
//   /**
//    * Get heading along line */
//   double getHeading()
//   { return atan2(-lA, lB); };
//   /**
//    * Get crossings with a circle
//    * \param cx,cy is the center of the circle
//    * \param r is the circle radius
//    * \param t1,t2 is the distances from the line origin to the crossings, if no crossings, then these values are unchanged.
//    * \returns number of crossings, and can be either 0 or 2 */
//   int getCircleCrossings(double cx, double cy, double r,
//                           double * t1, double * t2);
//   /**
//   * Shift the line this distance to the right
//   * This moves the C parameter and the start position.
//   * \param dist is the distance shifted. */
//   virtual void shiftRight(double dist);
//   /**
//    * Shift the line this distance to the left
//    * This moves the C parameter and the start position.
//    * \param dist is the distance shifted. */
//   virtual void shiftLeft(double dist);
//   /**
//   Code this line as an XML tag into this string buffer.
//   like <line name="" x="" y="" th="" length="" [extraAtt]/>\n
//   \param name is an optional value to a name attribute .
//   \param buff is the buffer to write into.
//   \param buffCnt is the length of the buffer.
//   \param extraAtt is an optional string with extra attributes to put into the tag.
//   \returns a pointer to the buffer. */
//   const char * codeXml(const char * name, char * buff, const int buffCnt, const char * extraAtt);
//     /**
//   Set from two points.
//   returns -1 if too close. */
//   inline int set2P(const double x1, const double y1,
//                     const double x2, const double y2)
//   {
//     setFromPoints(x1, y1, x2, y2);
//     return 0;
//   };
// 
// public:
//   /**
//   Length of line segment */
//   double length;
// };
// 

#endif
