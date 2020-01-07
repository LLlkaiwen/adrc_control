#ifndef ADRC_FIRSTORDER_H
#define ADRC_FIRSTORDER_H
#include "adrc/adrc_base.h"
namespace adrc
{

class Adrc_FirstOrder : public Adrc_Base
{

public:
    Adrc_FirstOrder(/* args */);

    ~Adrc_FirstOrder();
    // virtual void TD(const double &v0);
    void LESO(const double &y);
    void LSEF(const double &ref, const double &cur);
    virtual double ComputeControl(const double &ref, const double &cur);

    virtual void PrintParameters(std::ostream &os) const;
    double GetLESOZ1()
    {
        return _state_leso_z1;
    }

    double GetLESOZ2()
    {
        return _state_leso_z2;
    }

    /**************LESO参数*************/
    double _para_b0;
    double _para_leso_beta1;
    double _para_leso_beta2;
    /**************LSEF参数*************/
    double _para_lsef_k1;
    double _para_lsef_k2;

private:
    double _state_leso_z1;
    double _state_leso_z2;
    double _state_cur_u;
};

} // namespace adrc

#endif