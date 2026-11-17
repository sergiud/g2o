// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_RAND_HH
#define G2O_RAND_HH

#include <cstdlib>
#include <cmath>
#include <ctime>

#include <g2o/examples/tutorial_slam2d/g2o_tutorial_slam2d_api.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  namespace tutorial {

    /**
     * \brief generate random numbers
     */
    class G2O_TUTORIAL_SLAM2D_API Rand
    {
      public:
        /**
         * Gaussian random with a mean and standard deviation. Uses the
         * Polar method of Marsaglia.
         */
        static double gauss_rand(double mean, double sigma)
        {
          double x, y, r2;
          do {
            x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
            y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
            r2 = x * x + y * y;
          } while (r2 > 1.0 || r2 == 0.0);
          return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
        }

        /**
         * sample a number from a uniform distribution
         */
        static double uniform_rand(double lowerBndr, double upperBndr)
        {
          return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
        }

        /**
         * default seed function using the current time in seconds
         */
        static void seed_rand()
        {
          seed_rand(static_cast<unsigned int>(std::time(nullptr)));
        }

        /** seed the random number generator */
        static void seed_rand(unsigned int seed)
        {
          std::srand(seed);
        }
    };

  } // namespace tutorial
G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
