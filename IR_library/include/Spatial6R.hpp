#ifndef IRLIBRARY_SPATIAL6R_H
#define IRLIBRARY_SPATIAL6R_H

#include <array>
#include <cmath>
#include <vector>
#include <TypeDefs.hpp>
#include "Spatial3R.hpp"
#include <InverseKinematics.hpp>

namespace IRlibrary {
  class Spatial6R {
  protected:
    Vec6 l;
    Vec6 q; // Configuration space (angle1, angle2, angle3, ...)
    SE3Mat X; // EndEffector Transformation matrix in fixed frame
    Vec3 base; // Cordinates of the base
    SE3Mat M; // EndEffector zero position transformation matrix in fixed frame
    std::vector <ScrewAxis> Slist; // list of screw axis in fixed frame
    std::vector<double> wsRad;
    AxisAngle axisAngle;
    //Spatial3R spatial3r;
  public:
    // Constructors
    Spatial6R() :base({0, 0, 0}) {
      l << 0, 0, 0, 0, 0, 0;
      q << 0, 0, 0, 0, 0, 0;
      setM(); setSlist(); wsCalc(); zeroForwardKinematics();
    };
    Spatial6R(double l1_i, double l2_i, double l3_i) : base({0, 0, 0}) {
      l << l1_i, l2_i, l3_i, 0, 0, 0;
      q << 0, 0, 0, 0, 0, 0;
      setM();
      setSlist();
      wsCalc();
      zeroForwardKinematics(); };
    void setConfig(const Vec6 &q_i) { q = q_i; zeroForwardKinematics(); }
    void setX(const SE3Mat X_i, bool numerical = false) {inverseKinematics_analytical(X_i); if(numerical){inverseKinematics_numerical(q, X_i);}}
    void setOrigin(const Vec3 &o_i) { base = o_i; }
    void setLinks(double l1_i, double l2_i, double l3_i) {l[0] = l1_i; l[1] = l2_i; l[2] = l3_i; l[3] = 0; l[4] = 0; l[5] = 0; setM(); setSlist(); wsCalc(); zeroForwardKinematics();}
    std::vector<double> getwsRad(){ return wsRad; }
    ScrewAxis getConfig() { return q; }
    SE3Mat getX() { return X;}
    Vec3 getPos(){ Vec3 p_e; p_e << X(0,3),X(1,3),X(2,3); return p_e;}
    // inline double getAngle () { return x[2]; }
    
    virtual void zeroForwardKinematics();
    inline void setM() {
      M = SE3Mat::Identity();
      M(0, 3) = l[1] + l[2];
      M(2, 3) = l[0];
    }
    inline SE3Mat getM(){return M;}
    inline void setSlist(){
      ScrewAxis S;
      Vec3 w, q_i;
      Slist.clear();
      // joint 1
      w << 0, 0, 1; q_i << 0, 0, 0;
      S << w, -w.cross(q_i);
      Slist.push_back(S);
      // joint 2
      w << 0, -1, 0; q_i << 0, 0, l[0];
      S << w, -w.cross(q_i);
      Slist.push_back(S);
      // joint 3
      w << 0, -1, 0; q_i << l[1], 0, l[0];
      S << w, -w.cross(q_i);
      Slist.push_back(S);
      // joint 4
      w << 0, 0, 1; q_i << l[1]+l[2], 0, l[0];
      S << w, -w.cross(q_i);
      Slist.push_back(S);
      // joint 5
      w << 0, 1, 0; q_i << l[1]+l[2], 0, l[0];
      S << w, -w.cross(q_i);
      Slist.push_back(S);
      // joint 6
      w << 1, 0, 0; q_i << l[1]+l[2], 0, l[0];
      S << w, -w.cross(q_i);
      Slist.push_back(S);
    }
    bool inverseKinematics_numerical(Eigen::VectorXd const &, SE3Mat const &);
    //bool inverseKinematics(Vec3 const &);
    inline void wsCalc() {
      wsRad.push_back(std::fabs(l[2]-l[1]));
      wsRad.push_back(l[2] + l[1]);
    }
    bool inverseKinematics_analytical(SE3Mat const &);
    bool checkInDexWs(SE3Mat const &);
    
  };
} /* IRlibrary */ 

#endif /* ifndef IRLIBRARY_PLANAR3R_H */
