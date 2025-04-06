#ifndef MATHUTILS
#define MATHUTILS

#include <cmath>

namespace IRlibrary {
	/** Returns true if value close to zero **/
	bool nearZero (double val, double eps = 1e-10);

	/** Wraps angle between 0 to 2 pi **/
	double wrapTo2PI (double val);

	/** Wraps angle between -pi to pi **/
	double wrapToPI (double val);

	/** Change rad to deg **/
	double rad2deg (double val);

	/** Change deg to rad **/
	double deg2rad (double val);

} /* IRlibrary */
#endif /* ifndef MATHUTILS */
