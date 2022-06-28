//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>
#include <PhySim/Utils/IOUtils.h>

#ifdef WIN32
#include <windows.h>
#else 
#include <sys/time.h>
#endif

#undef min
#undef max

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class CustomTimer
	{
	private:

		double m_sd = 0;
		double m_av = 0;

	public:
		CustomTimer()
		{ 
			this->m_window = 10;
			this->m_identifier = string("");
			this->m_description = string("");
			this->Initialize();
		}

		// Default constructor (identifier and description)
		CustomTimer(int window, const string& iden = "", const string& desc = "")
		{
			assert(window >= 1);
			this->m_window = window;
			this->m_identifier = iden;
			this->m_description = desc;
			this->Initialize();
		}

		~CustomTimer() 
		{
			// Nothing to do here...
		}

		// Reinitialize
		void Initialize()
		{
#pragma omp critical(measurements)
			{
				this->m_measurements.clear();
			}

#pragma omp critical
			{
#ifdef WIN32
				QueryPerformanceFrequency(&m_frequency);
				m_startCount.QuadPart = 0;
				m_endCount.QuadPart = 0;
#else
				m_startCount.tv_sec = m_startCount.tv_usec = 0;
				m_endCount.tv_sec = m_endCount.tv_usec = 0;
#endif
				m_stopped = 0;
				m_staTimeInMicroSec = 0;
				m_endTimeInMicroSec = 0;
				m_cumTimeInMicroSec = 0;
			}
		}

		// Start timer
		void Restart()
		{
			this->Start();
		}

		// Do everything
		void StopStoreLog()
		{
			this->Stop();
			double time = this->getElapsedTimeInMilliSec();
			this->Store(time); // Store the time in millis
			this->Log();
		}

		// Store value to vector 
		void Store(double value)
		{
//#pragma omp critical(measurements)
//			{
				this->m_measurements.push_back(value);
				if ((int)this->m_measurements.size() > this->m_window)
					this->m_measurements.erase(this->m_measurements.begin());
			//}
		}

		// Compute mean value
		double ComputeMean() const
		{
//#pragma omp critical(measurements)
//			{
				int n = (int)this->m_measurements.size();

				double timeSum = 0.0;
				for (int i = 0; i < n; ++i)
					timeSum += this->m_measurements[i];

				return timeSum / (double)n;
			//}
		}

		// Standard deviation
		double ComputeSD() const
		{
//#pragma omp critical(measurements)
//			{
				int n = (int)this->m_measurements.size();

				double timeSum = 0.0;
				for (int i = 0; i < n; ++i)
					timeSum += this->m_measurements[i];

				double mean = timeSum / (double)n;

				double diffSum = 0.0;

				for (int i = 0; i < n; ++i)
				{
					double diff = this->m_measurements[i] - mean;
					diffSum += diff * diff; // SD: E[(X - XBar)^2]
				}

				return diffSum / (double)n;
			//}
		}

		Real GetSD() const
		{
			{
				return m_sd;
			}
		}

		Real GetMean() const
		{
			{
				return m_av;
			}
		}

		// Log state
		void Log()
		{
			double sd = this->ComputeSD();
			double av = this->ComputeMean();
			this->m_sd = sd;
			this->m_av = av;

			IOUtils::logTimes(Verbosity::V1_Default, "%s \t %.6e \t %.6e\n", this->m_identifier.c_str(), av, sd);
		}

		const string& GetIdentifier() const { return this->m_identifier; }
		const string& GetDescription() const { return this->m_description; }

		void Start()
		{
#pragma omp critical
			{
				m_cumTimeInMicroSec = 0; //Reset 
				m_stopped = 0; // Reset stop flag
#ifdef WIN32
				QueryPerformanceCounter(&m_startCount);
#else
				gettimeofday(&m_startCount, NULL);
#endif
			}
		}

		void Stop()
		{
#pragma omp critical
			{
				m_stopped = 1; // Set stop flag
#ifdef WIN32
				QueryPerformanceCounter(&m_endCount);
#else
				gettimeofday(&m_endCount, NULL);
#endif
			}
		}

		void Pause()
		{
			this->Stop();

			m_cumTimeInMicroSec += getElapsedTimeInMicroSec();
		}

		void Resume()
		{
			double accumulated = 0;

#pragma omp critical
			{
				accumulated = this->m_cumTimeInMicroSec;
			}

				this->Start(); // This set accumulated time to 0

#pragma omp critical
			{
				this->m_cumTimeInMicroSec = accumulated; // Reset
			}
		}

		// Seconds
		double getElapsedTime()
		{
			return this->getElapsedTimeInSec();
		}

		// Seconds
		double getElapsedTimeInSec()
		{
			return this->getElapsedTimeInMicroSec() * 0.000001;
		}

		// Milli-seconds
		double getElapsedTimeInMilliSec()
		{
			return this->getElapsedTimeInMicroSec() * 0.001;
		}

		// Micro-seconds
		double getElapsedTimeInMicroSec()
		{
#pragma omp critical
			{
#ifdef WIN32
				if (!m_stopped)
					QueryPerformanceCounter(&m_endCount);

				m_staTimeInMicroSec = m_startCount.QuadPart * (1000000.0 / m_frequency.QuadPart);
				m_endTimeInMicroSec = m_endCount.QuadPart * (1000000.0 / m_frequency.QuadPart);
#else
				if (!m_stopped)
					gettimeofday(&m_endCount, NULL);

				m_staTimeInMicroSec = (m_startCount.tv_sec * 1000000.0) + m_startCount.tv_usec;
				m_endTimeInMicroSec = (m_endCount.tv_sec * 1000000.0) + m_endCount.tv_usec;
#endif
			}

			return (m_endTimeInMicroSec - m_staTimeInMicroSec) + m_cumTimeInMicroSec;
		}

		int m_window;
		string m_identifier;
		string m_description;
		vector<double> m_measurements;


		double m_staTimeInMicroSec;	// Starting time in micro-seconds
		double m_endTimeInMicroSec;	// Ending time in micro-second
		double m_cumTimeInMicroSec;	// Storing cumulative time

		int  m_stopped; // Stop flag
#ifdef WIN32
		// Ticks per second
		LARGE_INTEGER m_frequency;
		LARGE_INTEGER m_startCount;
		LARGE_INTEGER m_endCount;
#else
		timeval m_startCount;
		timeval m_endCount;
#endif

	};
}