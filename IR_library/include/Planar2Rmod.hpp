#ifndef IRLIBRARY_PLANAR2Rmod_H
#define IRLIBRARY_PLANAR2Rmod_H

#include "TypeDefs.hpp"
#include "Planar2R.hpp"

namespace IRlibrary {
	class Planar2Rmod:public Planar2R {
		public:
			// Constructors
			Planar2Rmod() : Planar2R() { zeroForwardKinematics(); };
			Planar2Rmod(double l1_i, double l2_i) : Planar2R(l1_i, l2_i){ zeroForwardKinematics(); };

			void zeroForwardKinematics();

	};
} /* IRlibrary */ 

#endif /* ifndef IRLIBRARY_PLANAR2R_H */
