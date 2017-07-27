#undef F_CPU
#if   defined(__AVR_ATmega16__) 
#include "configAtmega16.h"


#elif defined(__AVR_ATmega128A__) 
#include "configAtmega128.h"
#else 
#error cpu not specified 
#endif 

