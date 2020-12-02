#pragma once


namespace raptor_raytracer
{
template <class T>
class sobol_numbers_2d
{
    public :
        /* CTOR */
        sobol_numbers_2d(const int n) : _l(ceil(std::log(static_cast<T>(n)) / log(2.0f))), _cur_n(0), _n(n)
        {
            /* First dimension */
            _v.reset(new unsigned int [_l << 1]);
            for (int i = 0; i < _l; ++i)
            {
                _v[i << 1] = 1 << (31 - i);
            }

            /* Second dimension */
            _v[1] = 1 << 31; 
            for (int i = 1; i < _l; ++i)
            {
                const int idx = ((i - 1) << 1) + 1;
                _v[(i << 1) + 1] = _v[idx] ^ (_v[idx] >> 1);
            }
        }

        void reset()
        {
            _cur_n = 0;
        }
        
        /* Get the next number in all dimensions */
        void next(T &x, T &y)
        {
            /* Check for overrun */
            assert(_cur_n < _n);

            /* First vector is all 0 */
            if (_cur_n == 0)
            {
                _last_x = 0;
                _last_y = 0;
                x = static_cast<T>(_last_x) * _pow_2_32_inv;
                y = static_cast<T>(_last_y) * _pow_2_32_inv;
                ++_cur_n;
                return;
            }
            
            /* Only expect each number to be used once, find lowest 0 bit */
            int c = 0;
            while ((_cur_n - 1) & (0x1 << c))
            {
                ++c;
            }
            
            /* Create result and scale to -1.0 to 1.0 */
            _last_x ^= _v[c << 1];
            x = static_cast<T>(_last_x) * _pow_2_32_inv;

            _last_y ^= _v[(c << 1) + 1];
            y = static_cast<T>(_last_y) * _pow_2_32_inv;
            
            ++_cur_n;
            return;
        }

    private :
        const T                             _pow_2_32_inv = 1.0 / pow(2.0, 32);        
        std::shared_ptr<unsigned int []>    _v;
        int                                 _l;
        unsigned int                        _last_x;
        unsigned int                        _last_y;
        int                                 _cur_n;
        const int                           _n;
};
}; /* namespace raptor_raytracer */
