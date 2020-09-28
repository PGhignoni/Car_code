#ifndef FILTER_H_INCLUDED
#define FILTER_H_INCLUDED

class Filter{

    private:

        float m_filterState;			// state of the 1st order filter
        float m_filterSampling;			// sampling time of the discrete time filter [seconds]
        float m_filterCutoff;			// cut-off time constant of the filter [seconds]

    public:

        Filter(float dt, float Tf): m_filterState{0.0}, m_filterSampling{dt}, m_filterCutoff{Tf}
        {
        }
	
	// performs the filtering action
        void filter(float u);
	
	// get the filter output
	float getOutput();


};

#endif // FILTER_H_INCLUDED
