#ifndef ADRC_BASE_H
#define ADRC_BASE_H
#include <math.h>
#include <iostream>
#include <float.h>
namespace adrc
{
    
int sign(double val);
double Fal(const double &error, const double &alpha, const double &delta);
double Fhan(const double &x1, const double &x2, const double &r, const double &h);

class Adrc_Base
{
    friend std::ostream &operator<<(std::ostream &os, const Adrc_Base &item);

public:
    Adrc_Base();
    virtual ~Adrc_Base();
    
    virtual void TD(const double &v0);
    // virtual void ESO(const double &y) = 0;
    // virtual void LESO(const double &y) = 0;
    // virtual double NLSEF() = 0;
    virtual double ComputeControl(const double &ref, const double &cur);
    void ConstraintControl(double &u);
    void SetLimit(const double &limit_up, const double &limit_down);
    virtual void PrintParameters(std::ostream &os) const;
    
    /**************通用参数*************/
    double _para_h;

    /**************TD参数*************/
    double _para_td_r;
    

protected:
    double _state_td_v1;
    double _state_td_v2;
    double _limit_up;
    double _limit_down;
};

std::ostream &operator<<(std::ostream &os, const Adrc_Base &item);
} // namespace adrc


#endif
