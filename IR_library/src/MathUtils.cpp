#include <MathUtils.hpp>
namespace IRlibrary {

	/** Returns true if value close to zero **/
	bool nearZero (double val, double eps) {
	  if (fabs(val) < eps){
		return true;
	  }else{
	    return false;
	  }
	}

	/** Wraps angle (radians) between 0 to 2 pi **/
	double wrapTo2PI (double val) {
	  val = fmod(val, 2*M_PI);
	  if (val < 0)
	    val += 2*M_PI;
	  return val;
	}

	/** Wraps angle (radians) between -pi to pi **/
	double wrapToPI (double val) {
	  val = fmod(val+M_PI,  2*M_PI);
	  if (val < 0)
	    val += 2*M_PI;
	  return val-M_PI;
	}
  
        /** Converts degrees to radians **/
        double deg2rad(double val){
          return val/180.0;
        }
  
        /** Converts radians to degrees **/
        double rad2deg(double val){
	  return val*180;
	}
} /* IRlibrary */

