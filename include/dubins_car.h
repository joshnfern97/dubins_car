#include "cartesian_v.h"

class dubins_car
{
    private:
        cartesian_v car_cartesian_v; 
    public:
        dubins_car();
        cartesian_v get_cartesian_v(float velocity, float theta);
        void SayHello();
};