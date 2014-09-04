/**
 * Tridaa pro diskretni filtr
 * 
 */

#ifndef F_H
#define F_H
#include <complex>
#include <valarray>


using namespace std;
namespace roadcheck {
	
	class Filter {
	public:
		Filter(float *b, int length); // nastaveni koeficientu a jejich delka
		~Filter();
		float filtering(float input);

	private:
		float *b;
		float *d;
		int length;
		int i;
	};

}
#endif
