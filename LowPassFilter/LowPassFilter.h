/* Copyright (C) 2014 Digipen Institute of Technology */
class LowPassFilter
{
	public:
		LowPassFilter(int sampleSize); // constructor
		~LowPassFilter(); // destructor
		bool ApplyFilter(float data, float & XFilter);
	
	private:
		unsigned counter;
		float * _data;
};
