#include <pbd/engine/collider/BodyBodyCollision.hpp>

#include <pbd/engine/RigidBody.hpp>
#include <pbd/common/Utils.hpp>

#include <cmath>
#include <cassert>
#include <limits>

#include <cppitertools/itertools.hpp>

#include <glm/glm.hpp>
#include <glm/mat3x3.hpp>
#include <glm/vec3.hpp>

namespace pbd {
#define dDOTpq(a, b, p, q) ((a)[0] * (b)[0] + (a)[p] * (b)[q] + (a)[2 * (p)] * (b)[2 * (q)])

using real_t = real_t;
using vec3_t = glm::tvec3<real_t>;
using mat3_t = glm::tmat3x3<real_t>;

#define M__PI 3.141592653
#define SIMD_EPSILON std::numeric_limits<real_t>::epsilon()

static real_t dDOT(const real_t* a, const real_t* b) { return dDOTpq(a, b, 1, 1); }
static real_t dDOT44(const real_t* a, const real_t* b) { return dDOTpq(a, b, 4, 4); }
static real_t dDOT41(const real_t* a, const real_t* b) { return dDOTpq(a, b, 4, 1); }
static real_t dDOT14(const real_t* a, const real_t* b) { return dDOTpq(a, b, 1, 4); }

#define dMULTIPLYOP1_331(A, op, B, C)   \
	{                                   \
		(A)[0] op dDOT41((B), (C));     \
		(A)[1] op dDOT41((B + 1), (C)); \
		(A)[2] op dDOT41((B + 2), (C)); \
	}

#define dMULTIPLYOP0_331(A, op, B, C) \
	{                                 \
		(A)[0] op dDOT((B), (C));     \
		(A)[1] op dDOT((B + 4), (C)); \
		(A)[2] op dDOT((B + 8), (C)); \
	}

#define dMULTIPLY1_331(A, B, C) dMULTIPLYOP1_331(A, =, B, C)
#define dMULTIPLY0_331(A, B, C) dMULTIPLYOP0_331(A, =, B, C)

typedef real_t dMatrix3[4 * 3];


static int intersectRectQuad2(real_t h[2], real_t p[8], real_t ret[16]) {
	// q (and r) contain nq (and nr) coordinate points for the current (and
	// chopped) polygons
	int nq = 4, nr = 0;
	real_t buffer[16];
	real_t* q = p;
	real_t* r = ret;
	for (int dir = 0; dir <= 1; dir++)
	{
		// direction notation: xy[0] = x axis, xy[1] = y axis
		for (int sign = -1; sign <= 1; sign += 2)
		{
			// chop q along the line xy[dir] = sign*h[dir]
			real_t* pq = q;
			real_t* pr = r;
			nr = 0;
			for (int i = nq; i > 0; i--)
			{
				// go through all points in q and all lines between adjacent points
				if (sign * pq[dir] < h[dir])
				{
					// this point is inside the chopping line
					pr[0] = pq[0];
					pr[1] = pq[1];
					pr += 2;
					nr++;
					if (nr & 8)
					{
						q = r;
						goto done;
					}
				}
				real_t* nextq = (i > 1) ? pq + 2 : q;
				if ((sign * pq[dir] < h[dir]) ^ (sign * nextq[dir] < h[dir]))
				{
					// this line crosses the chopping line
					pr[1 - dir] = pq[1 - dir] + (nextq[1 - dir] - pq[1 - dir]) /
						(nextq[dir] - pq[dir]) * (sign * h[dir] - pq[dir]);
					pr[dir] = sign * h[dir];
					pr += 2;
					nr++;
					if (nr & 8)
					{
						q = r;
						goto done;
					}
				}
				pq += 2;
			}
			q = r;
			r = (q == ret) ? buffer : ret;
			nq = nr;
		}
	}
done:
	if (q != ret) memcpy(ret, q, nr * 2 * sizeof(real_t));
	return nr;
}

void dLineClosestApproach(const vec3_t& pa, const vec3_t& ua,
	const vec3_t& pb, const vec3_t& ub,
	real_t* alpha, real_t* beta)
{
	vec3_t p = pb - pa;

	real_t uaub = glm::dot(ua, ub);
	real_t q1 = glm::dot(ua, p);
	real_t q2 = -glm::dot(ub, p);
	real_t d = 1 - uaub * uaub;
	if (d <= real_t(0.0001f))
	{
		// @@@ this needs to be made more robust
		*alpha = 0;
		*beta = 0;
	}
	else
	{
		d = 1.f / d;
		*alpha = (q1 + uaub * q2) * d;
		*beta = (uaub * q1 + q2) * d;
	}
}

void cullPoints2(int n, real_t p[], int m, int i0, int iret[])
{
	// compute the centroid of the polygon in cx,cy
	int i, j;
	real_t a, cx, cy, q;
	if (n == 1)
	{
		cx = p[0];
		cy = p[1];
	}
	else if (n == 2)
	{
		cx = real_t(0.5) * (p[0] + p[2]);
		cy = real_t(0.5) * (p[1] + p[3]);
	}
	else
	{
		a = 0;
		cx = 0;
		cy = 0;
		for (i = 0; i < (n - 1); i++)
		{
			q = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1];
			a += q;
			cx += q * (p[i * 2] + p[i * 2 + 2]);
			cy += q * (p[i * 2 + 1] + p[i * 2 + 3]);
		}
		q = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1];
		if (std::abs(a + q) > SIMD_EPSILON)
		{
			a = 1.f / (real_t(3.0) * (a + q));
		}
		else
		{
			a = 1e9;
		}
		cx = a * (cx + q * (p[n * 2 - 2] + p[0]));
		cy = a * (cy + q * (p[n * 2 - 1] + p[1]));
	}

	// compute the angle of each point w.r.t. the centroid
	real_t A[8];
	for (i = 0; i < n; i++) A[i] = std::atan2(p[i * 2 + 1] - cy, p[i * 2] - cx);

	// search for points that have angles closest to A[i0] + i*(2*pi/m).
	int avail[8];
	for (i = 0; i < n; i++) avail[i] = 1;
	avail[i0] = 0;
	iret[0] = i0;
	iret++;
	for (j = 1; j < m; j++)
	{
		a = real_t(j) * (2 * M__PI / m) + A[i0];
		if (a > M__PI) a -= 2 * M__PI;
		real_t maxdiff = 1e9, diff;

		*iret = i0;  // iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0

		for (i = 0; i < n; i++)
		{
			if (avail[i])
			{
				diff = std::abs(A[i] - a);
				if (diff > M__PI) diff = 2 * M__PI - diff;
				if (diff < maxdiff)
				{
					maxdiff = diff;
					*iret = i;
				}
			}
		}
		assert(*iret != i0);  // ensure iret got set

		avail[*iret] = 0;
		iret++;
	}
}


	// This collision algorithm is mostly taken from bullet3
	std::optional<Collision> obb_obb_collide(const RigidBody& body1, const RigidBody& body2) {
		// Compare the 3 axes from p0, 3 from p1,
		// the 9 edge pairs
		// Edge pairs are from cross products

		// Matrices, aligned along 4*real_t boundary.
		// These are ROW major!
		dMatrix3 R1;
		dMatrix3 R2;
		{
			mat3_t tmp = glm::mat3_cast(body1.orientation);
			for (int i: iter::range(3)) {
				for (int j: iter::range(3)) {
					R1[i + j * 4] = tmp[i][j];
				}
			}

			tmp = glm::mat3_cast(body2.orientation);
			for (int i : iter::range(3)) {
				for (int j : iter::range(3)) {
					R2[i + j * 4] = tmp[i][j];
				}
			}
		}

		const real_t fudge_factor = real_t(1.05);
		vec3_t p, pp, normalC(0.f, 0.f, 0.f);
		const real_t* normalR = 0;
		real_t A[3], B[3], R11, R12, R13, R21, R22, R23, R31, R32, R33,
			Q11, Q12, Q13, Q21, Q22, Q23, Q31, Q32, Q33, s, s2, l;
		int i, j, invert_normal, code;

		// get vector from centers of box 1 to box 2, relative to box 1
		p = body1.position - body2.position;
		dMULTIPLY1_331(&pp[0], R1, &p[0]);  // get pp = p relative to body 1

		// get side lengths / 2
		A[0] = body1.dims[0];
		A[1] = body1.dims[1];
		A[2] = body1.dims[2];
		B[0] = body2.dims[0];
		B[1] = body2.dims[1];
		B[2] = body2.dims[2];

		// Rij is R1'*R2, i.e. the relative rotation between R1 and R2
		R11 = dDOT44(R1 + 0, R2 + 0);
		R12 = dDOT44(R1 + 0, R2 + 1);
		R13 = dDOT44(R1 + 0, R2 + 2);
		R21 = dDOT44(R1 + 1, R2 + 0);
		R22 = dDOT44(R1 + 1, R2 + 1);
		R23 = dDOT44(R1 + 1, R2 + 2);
		R31 = dDOT44(R1 + 2, R2 + 0);
		R32 = dDOT44(R1 + 2, R2 + 1);
		R33 = dDOT44(R1 + 2, R2 + 2);

		Q11 = std::abs(R11);
		Q12 = std::abs(R12);
		Q13 = std::abs(R13);
		Q21 = std::abs(R21);
		Q22 = std::abs(R22);
		Q23 = std::abs(R23);
		Q31 = std::abs(R31);
		Q32 = std::abs(R32);
		Q33 = std::abs(R33);

		// for all 15 possible separating axes:
		//   * see if the axis separates the boxes. if so, return 0.
		//   * find the depth of the penetration along the separating axis (s2)
		//   * if this is the largest depth so far, record it.
		// the normal vector will be set to the separating axis with the smallest
		// depth. note: normalR is set to point to a column of R1 or R2 if that is
		// the smallest depth normal so far. otherwise normalR is 0 and normalC is
		// set to a vector relative to body 1. invert_normal is 1 if the sign of
		// the normal should be flipped.

#define TST(expr1, expr2, norm, cc)    \
	s2 = std::abs(expr1) - (expr2);      \
	if (s2 > 0) return std::nullopt;              \
	if (s2 > s)                        \
	{                                  \
		s = s2;                        \
		normalR = norm;                \
		invert_normal = ((expr1) < 0); \
		code = (cc);                   \
	}

		s = -std::numeric_limits<real_t>::infinity();
		invert_normal = 0;
		code = 0;

		// separating axis = u1,u2,u3
		TST(pp[0], (A[0] + B[0] * Q11 + B[1] * Q12 + B[2] * Q13), R1 + 0, 1);
		TST(pp[1], (A[1] + B[0] * Q21 + B[1] * Q22 + B[2] * Q23), R1 + 1, 2);
		TST(pp[2], (A[2] + B[0] * Q31 + B[1] * Q32 + B[2] * Q33), R1 + 2, 3);

		// separating axis = v1,v2,v3
		TST(dDOT41(R2 + 0, &p[0]), (A[0] * Q11 + A[1] * Q21 + A[2] * Q31 + B[0]), R2 + 0, 4);
		TST(dDOT41(R2 + 1, &p[0]), (A[0] * Q12 + A[1] * Q22 + A[2] * Q32 + B[1]), R2 + 1, 5);
		TST(dDOT41(R2 + 2, &p[0]), (A[0] * Q13 + A[1] * Q23 + A[2] * Q33 + B[2]), R2 + 2, 6);

		// note: cross product axes need to be scaled when s is computed.
		// normal (n1,n2,n3) is relative to box 1.
#undef TST
#define TST(expr1, expr2, n1, n2, n3, cc)                \
	s2 = std::abs(expr1) - (expr2);                        \
	if (s2 > SIMD_EPSILON) return std::nullopt;                     \
	l = std::sqrt((n1) * (n1) + (n2) * (n2) + (n3) * (n3)); \
	if (l > SIMD_EPSILON)                                \
	{                                                    \
		s2 /= l;                                         \
		if (s2 * fudge_factor > s)                       \
		{                                                \
			s = s2;                                      \
			normalR = 0;                                 \
			normalC[0] = (n1) / l;                       \
			normalC[1] = (n2) / l;                       \
			normalC[2] = (n3) / l;                       \
			invert_normal = ((expr1) < 0);               \
			code = (cc);                                 \
		}                                                \
	}

		real_t fudge2(1.0e-5f);

		Q11 += fudge2;
		Q12 += fudge2;
		Q13 += fudge2;

		Q21 += fudge2;
		Q22 += fudge2;
		Q23 += fudge2;

		Q31 += fudge2;
		Q32 += fudge2;
		Q33 += fudge2;

		// separating axis = u1 x (v1,v2,v3)
		TST(pp[2] * R21 - pp[1] * R31, (A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12), 0, -R31, R21, 7);
		TST(pp[2] * R22 - pp[1] * R32, (A[1] * Q32 + A[2] * Q22 + B[0] * Q13 + B[2] * Q11), 0, -R32, R22, 8);
		TST(pp[2] * R23 - pp[1] * R33, (A[1] * Q33 + A[2] * Q23 + B[0] * Q12 + B[1] * Q11), 0, -R33, R23, 9);

		// separating axis = u2 x (v1,v2,v3)
		TST(pp[0] * R31 - pp[2] * R11, (A[0] * Q31 + A[2] * Q11 + B[1] * Q23 + B[2] * Q22), R31, 0, -R11, 10);
		TST(pp[0] * R32 - pp[2] * R12, (A[0] * Q32 + A[2] * Q12 + B[0] * Q23 + B[2] * Q21), R32, 0, -R12, 11);
		TST(pp[0] * R33 - pp[2] * R13, (A[0] * Q33 + A[2] * Q13 + B[0] * Q22 + B[1] * Q21), R33, 0, -R13, 12);

		// separating axis = u3 x (v1,v2,v3)
		TST(pp[1] * R11 - pp[0] * R21, (A[0] * Q21 + A[1] * Q11 + B[1] * Q33 + B[2] * Q32), -R21, R11, 0, 13);
		TST(pp[1] * R12 - pp[0] * R22, (A[0] * Q22 + A[1] * Q12 + B[0] * Q33 + B[2] * Q31), -R22, R12, 0, 14);
		TST(pp[1] * R13 - pp[0] * R23, (A[0] * Q23 + A[1] * Q13 + B[0] * Q32 + B[1] * Q31), -R23, R13, 0, 15);

#undef TST

		if (!code) return std::nullopt;

		Collision result;

		// if we get to this point, the boxes interpenetrate. compute the normal
		// in global coordinates.
		if (normalR)
		{
			result.normal[0] = normalR[0];
			result.normal[1] = normalR[4];
			result.normal[2] = normalR[8];
		}
		else
		{
			dMULTIPLY0_331(&result.normal[0], R1, &normalC[0]);
		}
		if (invert_normal)
		{
			result.normal = -result.normal;
		}
		real_t depth = -s;

		// compute contact point(s)

		if (code > 6)
		{
			// an edge from box 1 touches an edge from box 2.
			// find a point pa on the intersecting edge of box 1
			vec3_t pa = body1.position;
			real_t sign;
			for (j = 0; j < 3; j++)
			{
				sign = (dDOT14(&result.normal[0], R1 + j) > 0) ? real_t(1.0) : real_t(-1.0);
				for (i = 0; i < 3; i++) pa[i] += sign * A[j] * R1[i * 4 + j];
			}

			// find a point pb on the intersecting edge of box 2
			vec3_t pb = body2.position;
			for (j = 0; j < 3; j++)
			{
				sign = (dDOT14(&result.normal[0], R2 + j) > 0) ? real_t(-1.0) : real_t(1.0);
				for (i = 0; i < 3; i++) pb[i] += sign * B[j] * R2[i * 4 + j];
			}

			real_t alpha, beta;
			vec3_t ua, ub;
			for (i = 0; i < 3; i++) ua[i] = R1[((code)-7) / 3 + i * 4];
			for (i = 0; i < 3; i++) ub[i] = R2[((code)-7) % 3 + i * 4];

			dLineClosestApproach(pa, ua, pb, ub, &alpha, &beta);
			for (i = 0; i < 3; i++) pa[i] += ua[i] * alpha;
			for (i = 0; i < 3; i++) pb[i] += ub[i] * beta;
			
			result.contacts[0] = pb;
			result.contacts[1] = pb + result.normal * depth;	
		
			result.contacts[0] = body1.to_local(result.contacts[0]);
			result.contacts[1] = body2.to_local(result.contacts[1]);

			return result;
		}

		// okay, we have a face-something intersection (because the separating
		// axis is perpendicular to a face). define face 'a' to be the reference
		// face (i.e. the normal vector is perpendicular to this) and face 'b' to be
		// the incident face (the closest face of the other box).

		const real_t* Ra, * Rb, * pa, * pb, * Sa, * Sb;
		if (code <= 3)
		{
			Ra = R1;
			Rb = R2;
			pa = &body1.position[0];
			pb = &body2.position[0];
			Sa = A;
			Sb = B;
		}
		else
		{
			Ra = R2;
			Rb = R1;
			pa = &body2.position[0];
			pb = &body1.position[0];
			Sa = B;
			Sb = A;
		}

		// nr = normal vector of reference face dotted with axes of incident box.
		// anr = absolute values of nr.
		vec3_t normal2, nr, anr;
		if (code <= 3)
		{
			normal2 = result.normal;
		}
		else
		{
			normal2 = -result.normal;
		}
		dMULTIPLY1_331(&nr[0], Rb, &normal2[0]);
		anr[0] = std::abs(nr[0]);
		anr[1] = std::abs(nr[1]);
		anr[2] = std::abs(nr[2]);

		// find the largest compontent of anr: this corresponds to the normal
		// for the indident face. the other axis numbers of the indicent face
		// are stored in a1,a2.
		int lanr, a1, a2;
		if (anr[1] > anr[0])
		{
			if (anr[1] > anr[2])
			{
				a1 = 0;
				lanr = 1;
				a2 = 2;
			}
			else
			{
				a1 = 0;
				a2 = 1;
				lanr = 2;
			}
		}
		else
		{
			if (anr[0] > anr[2])
			{
				lanr = 0;
				a1 = 1;
				a2 = 2;
			}
			else
			{
				a1 = 0;
				a2 = 1;
				lanr = 2;
			}
		}

		// compute center point of incident face, in reference-face coordinates
		vec3_t center;
		if (nr[lanr] < 0)
		{
			for (i = 0; i < 3; i++) center[i] = pb[i] - pa[i] + Sb[lanr] * Rb[i * 4 + lanr];
		}
		else
		{
			for (i = 0; i < 3; i++) center[i] = pb[i] - pa[i] - Sb[lanr] * Rb[i * 4 + lanr];
		}

		// find the normal and non-normal axis numbers of the reference box
		int codeN, code1, code2;
		if (code <= 3)
			codeN = code - 1;
		else
			codeN = code - 4;
		if (codeN == 0)
		{
			code1 = 1;
			code2 = 2;
		}
		else if (codeN == 1)
		{
			code1 = 0;
			code2 = 2;
		}
		else
		{
			code1 = 0;
			code2 = 1;
		}

		// find the four corners of the incident face, in reference-face coordinates
		real_t quad[8];  // 2D coordinate of incident face (x,y pairs)
		real_t c1, c2, m11, m12, m21, m22;
		c1 = dDOT14(&center[0], Ra + code1); // Dot with column of Ra
		c2 = dDOT14(&center[0], Ra + code2); // Dot with column of Ra
		// optimize this? - we have already computed this data above, but it is not
		// stored in an easy-to-index format. for now it's quicker just to recompute
		// the four dot products.
		m11 = dDOT44(Ra + code1, Rb + a1); // Dot between columns
		m12 = dDOT44(Ra + code1, Rb + a2); // Dot between columns
		m21 = dDOT44(Ra + code2, Rb + a1); // Dot between columns
		m22 = dDOT44(Ra + code2, Rb + a2); // Dot between columns
		{
			real_t k1 = m11 * Sb[a1];
			real_t k2 = m21 * Sb[a1];
			real_t k3 = m12 * Sb[a2];
			real_t k4 = m22 * Sb[a2];
			quad[0] = c1 - k1 - k3;
			quad[1] = c2 - k2 - k4;
			quad[2] = c1 - k1 + k3;
			quad[3] = c2 - k2 + k4;
			quad[4] = c1 + k1 + k3;
			quad[5] = c2 + k2 + k4;
			quad[6] = c1 + k1 - k3;
			quad[7] = c2 + k2 - k4;
		}

		// find the size of the reference face
		real_t rect[2];
		rect[0] = Sa[code1];
		rect[1] = Sa[code2];

		// intersect the incident and reference faces
		real_t ret[16];
		int n = intersectRectQuad2(rect, quad, ret);
		if (n < 1) return std::nullopt;  // this should never happen

		// convert the intersection points into reference-face coordinates,
		// and compute the contact position and depth for each point. only keep
		// those points that have a positive (penetrating) depth. delete points in
		// the 'ret' array as necessary so that 'point' and 'ret' correspond.
		real_t point[3 * 8];  // penetrating contact points
		real_t dep[8];        // depths for those points
		real_t det1 = 1.f / (m11 * m22 - m12 * m21);
		m11 *= det1;
		m12 *= det1;
		m21 *= det1;
		m22 *= det1;
		int cnum = 0;  // number of penetrating contact points found
		for (j = 0; j < n; j++)
		{
			real_t k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
			real_t k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
			for (i = 0; i < 3; i++) point[cnum * 3 + i] =
				center[i] + k1 * Rb[i * 4 + a1] + k2 * Rb[i * 4 + a2];
			dep[cnum] = Sa[codeN] - dDOT(&normal2[0], point + cnum * 3);
			if (dep[cnum] >= 0)
			{
				ret[cnum * 2] = ret[j * 2];
				ret[cnum * 2 + 1] = ret[j * 2 + 1];
				cnum++;
			}
		}
		if (cnum < 1) return std::nullopt;  // this should never happen

		// Only 1 please.
		int maxc = 1;

		if (cnum <= 1)
		{
			if (code < 4)
			{
				// we have less contacts than we need, so we use them all
				for (j = 0; j < cnum; j++)
				{
					vec3_t pointInWorld;
					for (i = 0; i < 3; i++)
						pointInWorld[i] = point[j * 3 + i] + pa[i];

					result.contacts[0] = pointInWorld;
					result.contacts[1] = pointInWorld + result.normal * dep[j];
					
					result.contacts[0] = body1.to_local(result.contacts[0]);
					result.contacts[1] = body2.to_local(result.contacts[1]);
				}
			}
			else
			{
				// we have less contacts than we need, so we use them all
				for (j = 0; j < cnum; j++)
				{
					vec3_t pointInWorld;
					for (i = 0; i < 3; i++)
						pointInWorld[i] = point[j * 3 + i] + pa[i] - result.normal[i] * dep[j];

					result.contacts[0] = pointInWorld;
					result.contacts[1] = pointInWorld + result.normal * dep[j];

					result.contacts[0] = body1.to_local(result.contacts[0]);
					result.contacts[1] = body2.to_local(result.contacts[1]);
				}
			}
		}
		else
		{
			// we have more contacts than are wanted, some of them must be culled.
			// find the deepest point, it is always the first contact.
			int i1 = 0;
			real_t maxdepth = dep[0];
			for (i = 1; i < cnum; i++)
			{
				if (dep[i] > maxdepth)
				{
					maxdepth = dep[i];
					i1 = i;
				}
			}

			int iret[8];
			cullPoints2(cnum, ret, maxc, i1, iret);

			for (j = 0; j < maxc; j++)
			{
				//      dContactGeom *con = CONTACT(contact,skip*j);
				//    for (i=0; i<3; i++) con->pos[i] = point[iret[j]*3+i] + pa[i];
				//  con->depth = dep[iret[j]];

				vec3_t posInWorld;
				for (i = 0; i < 3; i++)
					posInWorld[i] = point[iret[j] * 3 + i] + pa[i];
				if (code < 4)
				{
					result.contacts[0] = posInWorld;
					result.contacts[1] = posInWorld + result.normal * dep[iret[j]];

					result.contacts[0] = body1.to_local(result.contacts[0]);
					result.contacts[1] = body2.to_local(result.contacts[1]);
				}
				else
				{
					result.contacts[0] = posInWorld - result.normal * dep[iret[j]];
					result.contacts[1] = result.contacts[0] + result.normal * dep[iret[j]];

					result.contacts[0] = body1.to_local(result.contacts[0]);
					result.contacts[1] = body2.to_local(result.contacts[1]);
				}
			}
			cnum = maxc;
		}

		return result;
	}
}