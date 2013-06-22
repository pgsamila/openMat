module ImuDataCommunication 
{ 
	sequence<float> MatrixSq;

	struct OpenMatImuData 
	{
		int openMatId;
		float timestamp;		
			
		float xAcc;
		float yAcc;
		float zAcc;
		float xGyro;
		float yGyro;
		float zGyro;
		float xMag;
		float yMag;
		float zMag;
		float wQuat;
		float xQuat;
		float yQuat;
		float zQuat;
		float xEuler;
		float yEuler;
		float zEuler;
		
		MatrixSq rotationM;
	};
	
	struct RawImuData 
	{
		float timestamp;
			
		float xAcc;
		float yAcc;
		float zAcc;
		float xGyro;
		float yGyro;
		float zGyro;
		float xMag;
		float yMag;
		float zMag;
	};	

	interface ImuMonitor
	{
		void updateImuData(OpenMatImuData id);
	};
	
	interface RawImuMonitor
	{
		void updateImuData(RawImuData id);
	};	
};
