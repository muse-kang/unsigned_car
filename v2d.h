#pragma once

#ifndef _V2D
#define _V2D 1

class v2d {
public:
	/* constructors */
	v2d() {}
	v2d(const v2d &src) { this->x = src.x; this->y = src.y; }
	v2d(float x, float y) { this->x = x; this->y = y; }

	/* operators */
	v2d& operator=(const v2d &src);         /* assignment */
	v2d operator+(const v2d &src) const;    /* addition */
	v2d operator-(void) const;              /* negation */
	v2d operator-(const v2d &src) const;    /* subtraction */
	v2d operator*(const float s) const;     /* multiply with scalar */
	float operator*(const v2d &src) const;  /* dot product */
	friend v2d operator*(const float s, const v2d & src);

	/* methods */
	float len(void) const;
	void normalize(void);
	float dist(const v2d &p) const;
	float cosalpha(const v2d &p2, const v2d &center) const;
	v2d rotate(const v2d &c, float arc) const;

	/* data */
	float x;
	float y;
};

/* assignment */
inline v2d& v2d::operator=(const v2d &src)
{
	x = src.x; y = src.y; return *this;
}
/* add *this + src (vector addition) */
inline v2d v2d::operator+(const v2d &src) const
{
	return v2d(x + src.x, y + src.y);
}
/* negation of *this */
inline v2d v2d::operator-(void) const
{
	return v2d(-x, -y);
}
/* compute *this - src (vector subtraction) */
inline v2d v2d::operator-(const v2d &src) const
{
	return v2d(x - src.x, y - src.y);
}
/* scalar product */
inline float v2d::operator*(const v2d &src) const
{
	return src.x*x + src.y*y;
}
/* multiply vector with scalar (v2d*float) */
inline v2d v2d::operator*(const float s) const
{
	return v2d(s*x, s*y);
}
/* multiply scalar with vector (float*v2d) */
inline v2d operator*(const float s, const v2d & src)
{
	return v2d(s*src.x, s*src.y);
}
/* compute cosine of the angle between vectors *this-c and p2-c */
inline float v2d::cosalpha(const v2d &p2, const v2d &c) const
{
	v2d l1 = *this - c;
	v2d l2 = p2 - c;
	return (l1*l2) / (l1.len()*l2.len());
}
/* rotate vector arc radians around center c */
inline v2d v2d::rotate(const v2d &c, float arc) const
{
	v2d d = *this - c;
	float sina = sin(arc), cosa = cos(arc);
	return c + v2d(d.x*cosa - d.y*sina, d.x*sina + d.y*cosa);
}
/* compute the length of the vector */
inline float v2d::len(void) const
{
	return sqrt(x*x + y*y);
}
/* distance between *this and p */
inline float v2d::dist(const v2d &p) const
{
	return sqrt((p.x - x)*(p.x - x) + (p.y - y)*(p.y - y));
}
/* normalize the vector */
inline void v2d::normalize(void)
{
	float l = this->len();
	x /= l; y /= l;
}

#endif
