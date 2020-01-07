#include "adrc/adrc_firstorder.h"

namespace adrc
{

Adrc_FirstOrder::Adrc_FirstOrder()
    : Adrc_Base()
    , _state_leso_z1(0.0)
    , _state_leso_z2(0.0)
    , _state_cur_u(0.0)
    , _para_b0(1.0)
    , _para_leso_beta1(200.0)
    , _para_leso_beta2(13333.0)
    , _para_lsef_k1(10.0)
    , _para_lsef_k2(10.0)
{
}

Adrc_FirstOrder::~Adrc_FirstOrder()
{
}
// void Adrc_FirstOrder::TD(const double &v0)
// {
//     //_state_td_v1 -= _para_h * _para_td_r * Fal(_state_td_v1 - v0, 0.5, _para_h);
//     double fh = -_para_td_r * _para_td_r * (_state_td_v1 - v0) + 2 * _para_td_r * _state_td_v2;
//     _state_td_v1 += _para_h * (_state_td_v2);
//     _state_td_v2 += _para_h * fh;
//     _state_td_v1 = v0;
// }
void Adrc_FirstOrder::LESO(const double &y)
{
    double e = _state_leso_z1 - y;
    _state_leso_z1 += _para_h * (_state_leso_z2 - _para_leso_beta1 * e + _para_b0 * _state_cur_u);
    _state_leso_z2 += _para_h * (-_para_leso_beta2 * e);
}
void Adrc_FirstOrder::LSEF(const double &ref, const double &cur)
{
    double e = ref - _state_leso_z1;
    // double e = _state_td_v1 - _state_leso_z1;
    // double e1 = _state_td_v2;
    _state_cur_u = _para_lsef_k1 * e;// + _para_lsef_k2 * e1;
    _state_cur_u -= _state_leso_z2;
    _state_cur_u /= _para_b0;
}
double Adrc_FirstOrder::ComputeControl(const double &ref, const double &cur)
{
    //TD(ref-cur);
    LESO(cur);
    LSEF(ref, cur);
    ConstraintControl(_state_cur_u);
    return _state_cur_u;
}

void Adrc_FirstOrder::PrintParameters(std::ostream &os) const
{
    Adrc_Base::PrintParameters(os);
    os << "LESO: beta1: " << _para_leso_beta1 << " beta2: " << _para_leso_beta2 << std::endl;
    os << "LSEF: k1: " << _para_lsef_k1 << " k2: " << _para_lsef_k2 << " b0: " << _para_b0 << std::endl;
    os<<std::endl;
}
} // namespace adrc
