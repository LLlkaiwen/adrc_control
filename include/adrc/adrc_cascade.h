#ifndef ADRC_CASCADE_H
#define ADRC_CASCADE_H
#include "adrc/adrc_base.h"
namespace adrc
{

class Adrc_Cascade : public Adrc_Base
{

public:
    Adrc_Cascade(/* args */);
    ~Adrc_Cascade();
    // virtual void TD(const double &v0);
    void OuterESO(const double &y);
    void OuterNLSEF(const double &y);
    void InnerESO(const double &y);
    void InnerNLSEF();
    // virtual void LESO(const double &y) = 0;
    // virtual double NLSEF() = 0;
    double CascadeControl(const double &ref, const double &outer_cur, const double &inner_cur);
    void PrintParameters(std::ostream &os) const;
    double _para_b0;

    double _para_eso_beta1;
    double _para_eso_beta2;

    double _para_outer_nlsef_k;
    double _para_outer_nlsef_alpha;
    double _para_outer_nlsef_delta;

    double _para_inner_nlsef_k;
    double _para_inner_nlsef_alpha;
    double _para_inner_nlsef_delta;

private:
    /* data */
    double _state_outer_eso_z1;
    double _state_outer_eso_z2;
    double _state_outer_u;

    double _state_inner_eso_z1;
    double _state_inner_eso_z2;
    double _state_inner_u;
};

} // namespace adrc

#endif