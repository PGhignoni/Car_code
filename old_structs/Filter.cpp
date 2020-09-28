#include "Filter.h"

void Filter::filter(float u){

	float y_k{this->m_filterState};
	float gg{this->m_filterSampling/this->m_filterCutoff};

	float y_kPlus{0.0};
	
	// update action of the filter
	y_kPlus=y_k*(1-gg)+gg*u;

	this->m_filterState=y_kPlus;
 	
}


float Filter::getOutput(){

	return this->m_filterState;
}
