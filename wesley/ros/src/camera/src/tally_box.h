/* TALLY_BOX.H
 *
 * written by Eric M Gonzalez
 * date of 26FEB14
 *
 * PURPOSE: This class contains methods to interface with a map
 * 			that holds a tally for each key given to it.
 *
 * 			key: the number of sides returned by approxPolyDP
 *  		value: the number of times the key has been given
 *
 * 			this class can return the mode and average of its
 * 			tallies.
 */

#ifndef TALLY_BOX
#define TALLY_BOX

#include <map>

class tally_box {
	private:
		std::map<unsigned short, unsigned int> array;
		double running_avg;
		unsigned long count;

	public:
		tally_box() {
			running_avg = 0.0f;
			count = 0;
		};
	   ~tally_box() { };

		void append(unsigned short input) {
			count += 1;
			array[input] += 1;

			if (array.size() == 1) {
				running_avg = input;
			} else {
				running_avg *= (count - 1);
				running_avg += input;
				running_avg /= count;
			}
					
		}

		void reset() {
			array.clear();
		}

		double mean() {
			return(running_avg);
		}

		unsigned short mode() {
			unsigned short key = 0;
			unsigned int value = 0;
			for (std::map<unsigned short, unsigned int>::iterator it = array.begin();
				 it != array.end(); it++) {
				if (it->second > value) {
					value = it->second;
					key = it->first;
				}
			}
			return(key);
		}
};

#endif // TALLY_BOX
