#include "adrc/adrc_secondorder.h"
namespace adrc
{
    Adrc_SecondOrder::Adrc_SecondOrder()
        : Adrc_Base()
        , _state_eso_z1(0.0)
        , _state_eso_z2(0.0)
        , _state_eso_z3(0.0)
        , _state_cur_u(0.0)
        , _para_b0(1.0)
        , _para_eso({mode : NLESO, beta1 : 200.0, beta2 : 1768.0, beta3 : 13420.0, alpha1 : 0.5, alpha2 : 0.25, delta : 0.05})
        , _para_sef({mode : NLSEF_1, k1 : 10.0, k2 : 10.0, alpha1 : 0.75, alpha2 : 1.25, delta : 0.5, r : 20000.0, c : 0.5, h1 : 0.025})
    {

    }

    Adrc_SecondOrder::~Adrc_SecondOrder()
    {}

    void Adrc_SecondOrder::ESO(const double &y)
    {
        double e = _state_eso_z1 - y;
        _state_eso_z1 += _para_h * (_state_eso_z2 - _para_eso.beta1 * e);
        
        switch (_para_eso.mode)
        {
        case LESO:
            {
                _state_eso_z2 += _para_h * (_state_eso_z3 - _para_eso.beta2 * e + _para_b0 * _state_cur_u);
                _state_eso_z3 += _para_h * (-_para_eso.beta3 * e);
            }
            break;
        case NLESO:
            {
                double fe = Fal(e, _para_eso.alpha1, _para_eso.delta);
                double fe1 = Fal(e, _para_eso.alpha2, _para_eso.delta);
                _state_eso_z2 += _para_h * (_state_eso_z3 - _para_eso.beta2 * fe + _para_b0 * _state_cur_u);
                _state_eso_z3 += _para_h * (-_para_eso.beta3 * fe1);
            }
            break;
        default:
            break;
        }
    }

    void Adrc_SecondOrder::SEF()
    {
        double e1 = _state_td_v1 - _state_eso_z1;
        double e2 = _state_td_v2 - _state_eso_z2;
        
        switch (_para_sef.mode)
        {
        case LSEF:
            _state_cur_u = _para_sef.k1 * e1 + _para_sef.k2 * e2;
            break;
        case NLSEF_1:
            _state_cur_u = _para_sef.k1*Fal(e1,_para_sef.alpha1,_para_sef.delta)
                         + _para_sef.k2*Fal(e2,_para_sef.alpha2,_para_sef.delta);
            break;
        case NLSEF_2:
            _state_cur_u = -Fhan(e1, e2, _para_sef.r, _para_sef.h1);
            break;
        case NLSEF_3:
            _state_cur_u = -Fhan(e1, _para_sef.c * e2, _para_sef.r, _para_sef.h1);
            break;

        default:
            break;
        }

        _state_cur_u -= _state_eso_z3;
        _state_cur_u /= _para_b0;
    }

    double Adrc_SecondOrder::ComputeControl(const double &ref, const double &cur)
    {
        TD(ref);
        ESO(cur);
        SEF();
        ConstraintControl(_state_cur_u);
        return _state_cur_u;
    }

    void Adrc_SecondOrder::PrintParameters(std::ostream &os) const
    {
        Adrc_Base::PrintParameters(os);
        os << "ESO: "<< "beta1: " << _para_eso.beta1 << " beta2: " << _para_eso.beta2 << " beta3: " << _para_eso.beta3 << std::endl;

        if(_para_eso.mode == NLESO)
        {
            os << "alpha1: "<<_para_eso.alpha1<<" alpha2: "<<_para_eso.alpha2<<" delta"<<_para_eso.delta << std::endl;
        }
        os << "SEF: b0: " <<_para_b0<< std::endl;

        switch (_para_sef.mode)
        {
        case LSEF:
            os << "k1: " << _para_sef.k1 <<" k2:"<<_para_sef.k2<< std::endl;
            break;
        case NLSEF_1:
            os << "k1: " << _para_sef.k1 << " k2:" << _para_sef.k2 << std::endl;
            os << "alpha1: " << _para_sef.alpha1 << " alpha2: " << _para_sef.alpha2 << " delta" << _para_sef.delta << std::endl;
            break;
        case NLSEF_2:
            os << "r1: " << _para_sef.r << " h1: " << _para_sef.h1 << std::endl;
            break;
        case NLSEF_3:
            os << "r1: " << _para_sef.r << " c: " << _para_sef.c << " h1: " << _para_sef.h1 << std::endl;
            break;

        default:
            break;
        }
    }
} // namespace adrc
