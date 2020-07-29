#include "PNPSolver.h"


// �������ڿ��ٽ��PNP���⣬˳������ռ�������ת�Լ�ͼ��ϵ�����ϵ������ϵ��ϵ����ͶӰ����
// ����˳��
// 1.��ʼ������
// 2.����SetCameraMatrix(),SetDistortionCoefficients()���ú�����ڲ����뾵ͷ�������
// 3.��Points3D��Points2D�����һһ��Ӧ���������
// 4.����Solve()�������м���
// 5.��RoteM, TransM, W2CTheta��������������
//
// ԭ��μ���http://www.cnblogs.com/singlex/category/911880.html
// Author��VShawn
// Ver:2016.11.26.0
PNPSolver::PNPSolver()
{
	//��ʼ���������
	vector<double> rv(3), tv(3);
	cv::Mat rvec(rv), tvec(tv);
}
PNPSolver::PNPSolver(double fx, double fy, double u0, double v0, double k_1, double  k_2, double  p_1, double  p_2, double k_3)
{
	//��ʼ���������
	vector<double> rv(3), tv(3);
	cv::Mat rvec(rv), tvec(tv);
	SetCameraMatrix(fx, fy, u0, v0);
	SetDistortionCoefficients(k_1, k_2, p_1, p_2, k_3);
}

PNPSolver::~PNPSolver()
{
}

int PNPSolver::Solve(METHOD method)
{
	//����У��
	if (camera_matrix.cols == 0 || distortion_coefficients.cols == 0)
	{
		printf("ErrCode:-1,����ڲ�����������δ���ã�\r\n");
		return -1;
	}

	if (Points3D.size() != Points2D.size())
	{
		printf("ErrCode:-2��3D��������2D��������һ�£�\r\n");
		return -2;
	}
	if (method == METHOD::CV_P3P || method == METHOD::CV_ITERATIVE)
	{
		if (Points3D.size() != 4)
		{
			printf("ErrCode:-2,ʹ��CV_ITERATIVE��CV_P3P����ʱ���������������ӦΪ4��\r\n");
			return -2;
		}
	}
	else
	{
		if (Points3D.size() < 4)
		{
			printf("ErrCode:-2,���������������Ӧ����4��\r\n");
			return -2;
		}
	}

	////TODO::�����Ƿ��ǹ�����ĵ�
	//if ((method == METHOD::CV_ITERATIVE || method == METHOD::CV_EPNP) && Points2D.size() == 4)
	//{
	//	//ͨ������������˻�÷����������������Ƿ�ƽ��
	//}






	/*******************���PNP����*********************/
	//�����ַ������
	solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, method);	//ʵ��������ƺ�ֻ���ù�����������λ��
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_ITERATIVE);	//ʵ��������ƺ�ֻ���ù�����������λ��
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_P3P);		//Gao�ķ�������ʹ�������ĸ�������
	//solvePnP(Points3D, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false, CV_EPNP);


	/*******************��ȡ��ת����*********************/
	double rm[9];
	RoteM = cv::Mat(3, 3, CV_64FC1, rm);
	Rodrigues(rvec, RoteM);
	double r11 = RoteM.ptr<double>(0)[0];
	double r12 = RoteM.ptr<double>(0)[1];
	double r13 = RoteM.ptr<double>(0)[2];
	double r21 = RoteM.ptr<double>(1)[0];
	double r22 = RoteM.ptr<double>(1)[1];
	double r23 = RoteM.ptr<double>(1)[2];
	double r31 = RoteM.ptr<double>(2)[0];
	double r32 = RoteM.ptr<double>(2)[1];
	double r33 = RoteM.ptr<double>(2)[2];
	TransM = tvec;
	cout << rvec << endl;
	//������������ϵ��������תŷ���ǣ���ת�����ת����������ϵ��
	//��ת˳��Ϊz��y��x
	double thetaz = atan2(r21, r11) / CV_PI * 180;
	double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
	double thetax = atan2(r32, r33) / CV_PI * 180;
	//���ϵ������ϵ��������תŷ���ǣ��������ϵ�մ���ת���������������ϵ��ȫƽ�С�
	//��ת˳��Ϊz��y��x
	Theta_C2W.z = thetaz;
	Theta_C2W.y = thetay;
	Theta_C2W.x = thetax;

	//���������ϵ�����ϵ��������תŷ���ǣ�����ϵ�մ���ת�����ת���������ϵ��
	//��ת˳��Ϊx��y��z
	Theta_W2C.x = -1 * thetax;
	Theta_W2C.y = -1 * thetay;
	Theta_W2C.z = -1 * thetaz;
	

	/*************************************�˴�������������ϵԭ��Oc����������ϵ�е�λ��**********************************************/

	/***********************************************************************************/
	/* ��ԭʼ����ϵ������תz��y��x������ת������������ϵƽ�У�����OcOw�������ת */
	/* ��������֪��������������ϵ��ȫƽ��ʱ��OcOw��ֵ */
	/* ��ˣ�ԭʼ����ϵÿ����ת��ɺ󣬶�����OcOw����һ�η�����ת�����տ��Եõ���������ϵ��ȫƽ��ʱ��OcOw */
	/* ����������-1������������ϵ����������� */
	/***********************************************************************************/

	//���ƽ�ƾ��󣬱�ʾ���������ϵԭ�㣬��������(x,y,z)�ߣ��͵�����������ϵԭ��
	double tx = tvec.ptr<double>(0)[0];
	double ty = tvec.ptr<double>(0)[1];
	double tz = tvec.ptr<double>(0)[2];

	//x y z ΪΨһ���������ԭʼ����ϵ�µ�����ֵ
	//Ҳ��������OcOw���������ϵ�µ�ֵ
	double x = tx, y = ty, z = tz;
	Position_OwInC.x = x;
	Position_OwInC.y = y;
	Position_OwInC.z = z;
	//�������η�����ת
	CodeRotateByZ(x, y, -1 * thetaz, x, y);
	CodeRotateByY(x, z, -1 * thetay, x, z);
	CodeRotateByX(y, z, -1 * thetax, y, z);


	//����������������ϵ�µ�λ������
	//������OcOw����������ϵ�µ�ֵ
	Position_OcInW.x = x*-1;
	Position_OcInW.y = y*-1;
	Position_OcInW.z = z*-1;

	return 0;
}


//���ݼ�����Ľ��������������ͶӰ��ͼ�񣬷�����������㼯
//����Ϊ��������ϵ�ĵ����꼯��
//���Ϊ��ͶӰ��ͼ���ϵ�ͼ�����꼯��
vector<cv::Point2f> PNPSolver::WordFrame2ImageFrame(vector<cv::Point3f> WorldPoints)
{
	vector<cv::Point2f> projectedPoints;
	cv::projectPoints(WorldPoints, rvec, tvec, camera_matrix, distortion_coefficients, projectedPoints);
	return projectedPoints;
}



//��������Ĳ�����ͼ������ת�������������
//ʹ��ǰ��Ҫ����Solve()������λ��
//����Ϊͼ���ϵĵ�����
//double FΪ��ͷ����
//���Ϊ���ڽ���=Fʱ���������ϵ����
cv::Point3f PNPSolver::ImageFrame2CameraFrame(cv::Point2f p, double F)
{
	double fx;
	double fy; 
	double u0;
	double v0;

	fx = camera_matrix.ptr<double>(0)[0];
	u0 = camera_matrix.ptr<double>(0)[2];
	fy = camera_matrix.ptr<double>(1)[1];
	v0 = camera_matrix.ptr<double>(1)[2];
	double zc = F;
	double xc = (p.x - u0)*F / fx;
	double yc = (p.y - v0)*F / fy;
	return cv::Point3f(xc, yc, zc);
}
