#include "adrc/adrc_cascade.h"

namespace adrc
{
Adrc_Cascade::Adrc_Cascade()
    : Adrc_Base()
    , _state_outer_eso_z1(0.0)
    , _state_outer_eso_z2(0.0)
    , _state_outer_u(0.0)
    , _state_inner_eso_z1(0.0)
    , _state_inner_eso_z2(0.0)
    , _state_inner_u(0.0)
    , _para_b0(1.0)
    , _para_eso_beta1(200.0)
    , _para_eso_beta2(20000.0)
    , _para_outer_nlsef_k(1.0)
    , _para_outer_nlsef_alpha(0.5)
    , _para_outer_nlsef_delta(0.5)
    , _para_inner_nlsef_k(10.0)
    , _para_inner_nlsef_alpha(0.5)
    , _para_inner_nlsef_delta(0.5)
{
}

Adrc_Cascade::~Adrc_Cascade()
{
}

// void Adrc_Cascade::TD(const double &v0)
// {
//     //_state_td_v1 -= _para_h * _para_td_r * Fal(_state_td_v1 - v0, 0.5, _para_h);
//     double fh = -_para_td_r * _para_td_r * (_state_td_v1 - v0) + 2 * _para_td_r * _state_td_v2;
//     _state_td_v1 += _para_h * (_state_td_v2);
//     _state_td_v2 += _para_h * fh;
// }

void Adrc_Cascade::OuterESO(const double &y)
{
    double e = _state_outer_eso_z1 - y;
    // double fe = Fal(e, 0.5, _para_h);
    _state_outer_eso_z1 += _para_h * (_state_outer_eso_z2 - _para_eso_beta1 * e + _state_outer_u);
    _state_outer_eso_z2 += _para_h * (-_para_eso_beta2 * e);
}

void Adrc_Cascade::OuterNLSEF(const double &y)
{
    // double e = _state_td_v1 - _state_outer_eso_z1;
    double e = _state_td_v1 - y;
    // _state_outer_u = _para_outer_nlsef_k * Fal(e, _para_outer_nlsef_alpha, _para_outer_nlsef_delta);
    _state_outer_u = _para_outer_nlsef_k * e;
    // _state_outer_u -= _state_outer_eso_z2;
}

void Adrc_Cascade::InnerESO(const double &y)
{
    double e = _state_inner_eso_z1 - y;
    // double fe = Fal(e, 0.5, _para_h);
    _state_inner_eso_z1 += _para_h * (_state_inner_eso_z2 - _para_eso_beta1 * e + _para_b0 *_state_inner_u);
    _state_inner_eso_z2 += _para_h * (-_para_eso_beta2 * e);
}

void Adrc_Cascade::InnerNLSEF()
{
    double e = _state_outer_u - _state_inner_eso_z1;
    // _state_inner_u = _para_inner_nlsef_k * Fal(e, _para_inner_nlsef_alpha, _para_inner_nlsef_delta);
    _state_inner_u = _para_inner_nlsef_k * e;
    _state_inner_u -= _state_inner_eso_z2;
    _state_inner_u /= _para_b0;
}

double Adrc_Cascade::CascadeControl(const double &ref, const double &outer_cur, const double &inner_cur)
{
    TD(ref);
    // OuterESO(outer_cur);
    OuterNLSEF(outer_cur);
    InnerESO(inner_cur);
    InnerNLSEF();


    
    ConstraintControl(_state_inner_u);
    return _state_inner_u;
}
void Adrc_Cascade::PrintParameters(std::ostream &os) const
{
    Adrc_Base::PrintParameters(os);
    os << "b0: " << _para_b0 << " ";
    os << "eso beta1: " << _para_eso_beta1 << " beta2: " << _para_eso_beta2 << std::endl;
    os << "inner_nlsef k: " << _para_outer_nlsef_k << " alpha: " << _para_outer_nlsef_alpha << " delta： " << _para_outer_nlsef_delta << std::endl;
    os << "inner_nlsef k: " << _para_inner_nlsef_k << " alpha: " << _para_inner_nlsef_alpha << " delta： " << _para_inner_nlsef_delta << std::endl;
}
} // namespace adrc
