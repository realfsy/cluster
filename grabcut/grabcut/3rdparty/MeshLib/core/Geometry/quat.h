/*!
*      \file Quat.h
*      \brief Quaternion
*	   \author David Gu
*      \date 10/07/2010
*
*/

#ifndef _QUAT_H_
#define _QUAT_H_

#include <math.h>
#include "Point.h"

typedef float mat44[4][4];

namespace MeshLib
{

/*!
 *	\brief CQrot quaternion class
 */
class CQrot
{
public:
	/*!
	*	CQrot default constructor 
	*/
  CQrot(){m_w=1; m_x=m_y=m_z=0;};

	/*!
	*	CQrot default constructor 
	* \param w w-value
	* \param x x-value
	* \param y y-value
	* \param z z-value
	*/

  CQrot( float w, float x, float y, float z)
  {
    m_w = w;
    m_x = x;
    m_y = y;
    m_z = z;
  };

  /*!	Copy operator of CQrot
  *	\param q a quaternion
  */
  CQrot( const CQrot & q )
  {
    m_w = q.m_w; 
    m_x =  q.m_x; 
    m_y =  q.m_y; 
    m_z =  q.m_z;
  }

  /*!	Copy operator of CQrot
  *	\param q a quaternion
  */
	CQrot & operator=(const CQrot & q)
  {
      m_w = q.m_w;
      m_x  = q.m_x;
      m_y  = q.m_y;
      m_z  = q.m_z;
      return *this;
  }

  /*! CQrot destructor
  */
  ~CQrot(){};
  
  /*! power of quaternion
  *	\param p power
  * \return quoternion power p
  */

  CQrot& operator^(float p)
 {
  normalize();
  float theta = 2 * acos( m_w );
  if( theta < 1e-10 ) return (*this); 

  CPoint axis( m_x,m_y,m_z );
  axis /= axis.norm();

  theta *= p;
  m_w   = cos( theta * 0.5 );
  axis *= sin( theta * 0.5 );

  m_x = axis[0];
  m_y = axis[1];
  m_z = axis[2];

  return (*this);
};


  /*!	Convert quoternion to OpenGL matrix
  *     \param m pointer to the matrix
  */
  void convert( float * m)
 {
  CQrot q = *this;
  float l = q^q;
  float s = 2.0 / l;
  float xs = q.m_x*s;
  float ys = q.m_y*s;
  float zs = q.m_z*s;
  
  float wx = q.m_w*xs;
  float wy = q.m_w*ys;
  float wz = q.m_w*zs;
  
  float xx = q.m_x*xs;
  float xy = q.m_x*ys;
  float xz = q.m_x*zs;
  
  float yy = q.m_y*ys;
  float yz = q.m_y*zs;
  float zz = q.m_z*zs;
  

  m[0*4+0] = 1.0 - (yy + zz);
  m[1*4+0] = xy - wz;
  m[2*4+0] = xz + wy;
  m[0*4+1] = xy + wz;
  m[1*4+1] = 1.0 - (xx + zz);
  m[2*4+1] = yz - wx;
  m[0*4+2] = xz - wy;
  m[1*4+2] = yz + wx;
  m[2*4+2] = 1.0 - (xx + yy);

  
  m[0*4+3] = 0.0;
  m[1*4+3] = 0.0;
  m[2*4+3] = 0.0;
  m[3*4+0] = m[3*4+1] = m[3*4+2] = 0.0;
  m[3*4+3] = 1.0;
  };

  /*!	Multiply a point with the current quoternion
  *    \param  p input point
  *    \return product of the current quoternion with v
  */
  CPoint operator*( const CPoint & p )
 {
    CQrot   q(m_w,m_x,m_y,m_z);
    CQrot pq( 0, p[0],p[1], p[2]);
    CQrot iq(m_w,m_x,m_y,m_z);
	iq^(-1);
    CQrot r =     q *  pq * iq ;
    return CPoint(r.m_x,r.m_y, r.m_z);
 };


public:
	/*!
	*	(x,y,z,w) value of the current quoternion
	*/
  float m_w,m_x,m_y,m_z;
  /*!
   *	normalization	
   */
  void normalize()
  {
	float l = m_w * m_w + m_x * m_x + m_y * m_y + m_z * m_z;
    if( l < 1e-20 ) {  
        m_x = 0; m_y = 0; m_z = 0; m_w = 1;
        return;
    }

    l = sqrt(l);

    m_w /= l;
    m_x /= l;
    m_y /= l;
    m_z /= l;

  };

  /*!
   *	product of two quoternions
   *    \param q input quoternion
   *    \return product of the current quoternion and q
   */
   CQrot operator*( const CQrot & q ) const
  {

    float   sp = m_w;
    CPoint vp( m_x, m_y, m_z );

    float   sq = q.m_w;
    CPoint vq( q.m_x, q.m_y, q.m_z );
 
    float   sr = sp * sq - vp * vq;

    CPoint vr;
    CPoint dp;
    dp = vp^vq;
    vp = vp * sq;
    vq = vq * sp;
    vr = vp   + vq + dp;

    return CQrot( sr, vr[0], vr[1], vr[2]);
};
 

  /*!
   *	inner product of two quoternions
   *    \param q input quoternion
   *    \return inner product of the current quoternion and q
   */
   float operator^( const CQrot & q ) const
  {
	return ( m_w * q.m_w + m_x * q.m_x + m_y * q.m_y + m_z * q.m_z);
  };

};


  /*! power of quaternion
  * \param r input quaternion
  *	\param p power
  * \return quoternion power p
  */

inline CQrot operator^(const CQrot & r, float p)
{
  CQrot q = r;
  q.normalize();
  float theta = 2 * acos( q.m_w );
  if( theta < 1e-10 ) return q; 

  CPoint axis( q.m_x, q.m_y, q.m_z );
  axis /= axis.norm();
  theta *= p;
  q.m_w   = cos( theta * 0.5 );
  axis *= sin( theta * 0.5 );

  q.m_x = axis[0];
  q.m_y = axis[1];
  q.m_z = axis[2];

  return q;
};


}
#endif