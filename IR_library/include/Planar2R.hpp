#ifndef IRLIBRARY_PLANAR2R_H
#define IRLIBRARY_PLANAR2R_H

#include <array>
#include <cmath>
#include "TypeDefs.hpp"

namespace IRlibrary {
	class Planar2R {
		protected:
			Vec2 l;
			Vec2 q; // Configuration space (angle1, angle2)
			Vec2 xy; // Task space Cartesian cordinates
			Vec2 base; // Cordinates of the base
		public:
			// Constructors
			Planar2R() : l({0, 0}), q({0, 0}), base({0, 0}) { zeroForwardKinematics(); };
			Planar2R(double l1_i, double l2_i) : l({l1_i, l2_i}), q({0, 0}), base({0, 0}) { zeroForwardKinematics(); };

			void setConfig(const Vec2 &q_i) { q = q_i; zeroForwardKinematics(); }
			void setXY(const Vec2 &xy_i) { xy = xy_i; }
			void setOrigin(const Vec2 &o_i) { base = o_i; }
			void setLinks(double l1_i, double l2_i) {l[0] = l1_i; l[1] = l2_i; zeroForwardKinematics();}

			Vec2 getConfig() { return q; }
			Vec2 getXY() { return xy; }
			double getAngle () { double ang = q[0] + q[1]; if(ang > 2 * M_PI) ang -= 2 * M_PI; if(ang < 2 * M_PI) ang += 2 * M_PI; return ang; }

			virtual void zeroForwardKinematics();

	};
} /* IRlibrary */ 

#endif /* ifndef IRLIBRARY_PLANAR2R_H */
