/*--------------------------------------------------------------------------*\
 |                                                                          |
 |         , __                 , __                                        |
 |        /|/  \               /|/  \                                       |
 |         | __/ _   ,_         | __/ _   ,_                                |
 |         |   \|/  /  |  |   | |   \|/  /  |  |   |                        |
 |         |(__/|__/   |_/ \_/|/|(__/|__/   |_/ \_/|/                       |
 |                           /|                   /|                        |
 |                           \|                   \|                        |
 |                                                                          |
 |      Enrico Bertolazzi                                                   |
 |      Dipartimento di Ingegneria Industriale                              |
 |      Universita` degli Studi di Trento                                   |
 |      email: enrico.bertolazzi@unitn.it                                   |
 |                                                                          |
\*--------------------------------------------------------------------------*/
/*
Copyright (c) 2014, Enrico Bertolazzi
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

///
/// file: Clothoid.hh
///

#ifndef CLOTHOID_HH
#define CLOTHOID_HH

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <cmath>

template <typename T = double>
std::vector<T> linspace(T a, T b, size_t N) {
  T h = (b - a) / static_cast<T>(N-1);
  std::vector<T> xs(N);
  typename std::vector<T>::iterator x;
  T val;
  for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
    *x = val;
  return xs;
};


//! Clothoid computations routine
namespace Clothoid {

  typedef double valueType ;

  //! Compute Fresnel integrals
  /*!
   * \f[ C(x) = \int_0^x \cos\left(\frac{\pi}{2}t^2\right) dt, \qquad
   *     S(x) = \int_0^x \sin\left(\frac{\pi}{2}t^2\right) dt \f]
   * \param x the input abscissa
   * \param S the value of \f$ S(x) \f$
   * \param C the value of \f$ C(x) \f$
   */
  void
  FresnelCS( valueType   x,
             valueType & C,
             valueType & S ) ;

  //! Compute Fresnel integrals and its derivatives
  /*!
   * \f[ C(x) = \int_0^x \cos\left(\frac{\pi}{2}t^2\right) dt, \qquad
   *     S(x) = \int_0^x \sin\left(\frac{\pi}{2}t^2\right) dt \f]
   * \param x the input abscissa
   * \param S S[0]=\f$ S(x) \f$, S[1]=\f$ S'(x) \f$, S[2]=\f$ S''(x) \f$
   * \param C C[0]=\f$ C(x) \f$, C[1]=\f$ C'(x) \f$, C[2]=\f$ C''(x) \f$
   */
  void
  FresnelCS( int       nk,
             valueType x,
             valueType C[],
             valueType S[] ) ;

  /*! \brief Compute the Fresnel integrals
   * \f[
   *   \int_0^1 t^k \cos\left(a\frac{t^2}{2} + b t + c\right) dt,\qquad
   *   \int_0^1 t^k \sin\left(a\frac{t^2}{2} + b t + c\right) dt
   * \f]
   * \param nk   number of momentae to compute
   * \param a    parameter \f$ a \f$
   * \param b    parameter \f$ b \f$
   * \param c    parameter \f$ c \f$
   * \param intC cosine integrals,
   * \param intS sine integrals
   */
  void
  GeneralizedFresnelCS( int       nk,
                        valueType a,
                        valueType b,
                        valueType c,
                        valueType intC[],
                        valueType intS[] ) ;

  /*! \brief Compute the Fresnel integrals
   * \f[
   *   \int_0^1 t^k \cos\left(a\frac{t^2}{2} + b t + c\right) dt,\qquad
   *   \int_0^1 t^k \sin\left(a\frac{t^2}{2} + b t + c\right) dt
   * \f]
   * \param a      parameter \f$ a \f$
   * \param b      parameter \f$ b \f$
   * \param c      parameter \f$ c \f$
   * \param intC   cosine integrals,
   * \param intS   sine integrals
   */
  void
  GeneralizedFresnelCS( valueType   a,
                        valueType   b,
                        valueType   c,
                        valueType & intC,
                        valueType & intS ) ;

  /*! \brief Compute the clothoid by Hemite data
   *
   * \param x0     initial x position            \f$ x_0      \f$
   * \param y0     initial y position            \f$ y_0      \f$
   * \param theta0 initial angle                 \f$ \theta_0 \f$
   * \param x1     final x position              \f$ x_1      \f$
   * \param y1     final y position              \f$ y_1      \f$
   * \param theta1 final angle                   \f$ \theta_1 \f$
   * \param k      computed curvature            \f$ K        \f$
   * \param dk     computed curvature derivative \f$ K'       \f$
   * \param L      computed length of the curve
   */
  int
  buildClothoid( valueType   x0,
                 valueType   y0,
                 valueType   theta0,
                 valueType   x1,
                 valueType   y1,
                 valueType   theta1,
                 valueType & k,
                 valueType & dk,
                 valueType & L ) ;


  int
  pointsOnClothoid( double x0,
                    double y0,
                    double theta0,
                    double kappa,
                    double dkappa,
                    double L,
                    int npts,
                    std::vector<double> & X,
                    std::vector<double> & Y );

}

#endif

///
/// eof: Clothoid.hh
///
