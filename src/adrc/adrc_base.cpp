#include "adrc/adrc_base.h"

namespace adrc
{

int sign(double val)
{
  if (val > DBL_EPSILON)
    return 1;
  else  if(val  < -DBL_EPSILON)
    return -1;
  else return 0;
}

double Fal(const double &error, const double &alpha, const double &delta)
{
  if (fabsf(error) > delta)
  {
    return sign(error) * pow(fabsf(error), alpha);
  }
  else
  {
    return error / (powf(delta, 1.0f - alpha));
  }
}

double Fhan(const double &x1, const double &x2, const double &r, const double &h)
{
  double d = r * h * h;
  double a0 = h * x2;
  double y = x1 + a0;
  double a1 = sqrtf(d * (d + 8.0f * fabs(y)));
  double a2 = a0 + sign(y) * (a1 - d) * 0.5f;
  double sy = (sign(y + d) - sign(y - d)) * 0.5f;
  double a = (a0 + y - a2) * sy + a2;
  double sa = (sign(a + d) - sign(a - d)) * 0.5f;

  return -r * (a / d - sign(a)) * sa - r * sign(a);
}

Adrc_Base::Adrc_Base()
    : _state_td_v1(0.0f), _state_td_v2(0.0f), _para_td_r(2.0f), _para_h(0.005f), _limit_up(INFINITY), _limit_down(-INFINITY)
{
}

Adrc_Base::~Adrc_Base()
{
}

void Adrc_Base::TD(const double &v0)
{
  double fh = Fhan(_state_td_v1 - v0, _state_td_v2, _para_td_r, _para_h);
  _state_td_v1 += _para_h * _state_td_v2;
  _state_td_v2 += _para_h * fh;
}

double Adrc_Base::ComputeControl(const double &ref, const double &cur)
{
  TD(ref);
  ConstraintControl(_state_td_v2);
  return _state_td_v2;
}

void Adrc_Base::SetLimit(const double &limit_up, const double &limit_down)
{
  _limit_up = fabs(limit_up);
  _limit_down = -fabs(limit_down);
}

void Adrc_Base::ConstraintControl(double &u)
{
  if (u > _limit_up)
  {
    u = _limit_up;
  }
  else if (u < _limit_down)
  {
    u = _limit_down;
  }
}
void Adrc_Base::PrintParameters(std::ostream &os) const
{
  // os << std::endl;
  os << "control limit: " << _limit_down << " - " << _limit_up << std::endl;
  os << "TD: r: " << _para_td_r << " h: " << _para_h << std::endl;
}

std::ostream &operator<<(std::ostream &os, const Adrc_Base &obj)
{
  obj.PrintParameters(os);
  return os;
}

} // namespace adrc
