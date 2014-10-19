class LowPassFilter
{
public:
    // constructor
    LowPassFilter(float sample_freq, float cutoff_freq) :
        _cutoff_freq(cutoff_freq),
        _a1(0.0f),
        _a2(0.0f),
        _b0(0.0f),
        _b1(0.0f),
        _b2(0.0f),
        _delay_element_1(0.0f),
        _delay_element_2(0.0f)
    {
        // set initial parameters
        set_cutoff_frequency(sample_freq, cutoff_freq);
    }

    /**
     * Change filter parameters
     */
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);

    /**
     * Add a new raw value to the filter
     *
     * @return retrieve the filtered result
     */
    float apply(float sample);

    /**
     * Return the cutoff frequency
     */
    float get_cutoff_freq(void) const {
        return _cutoff_freq;
    }

    /**
     * Reset the filter state to this value
     */
    float reset(float sample);

private:
    float           _cutoff_freq;
    float           _a1;
    float           _a2;
    float           _b0;
    float           _b1;
    float           _b2;
    float           _delay_element_1;        // buffered sample -1
    float           _delay_element_2;        // buffered sample -2
};
