


#include "alignment.h"
#include <fstream>


#ifdef USE_LAPACK
extern "C"
{
#include <third_party/numeric_stuff/CLAPACK/f2c.h>
#include <third_party/numeric_stuff/CLAPACK/clapack.h>
}
#endif

using namespace std;
using namespace Eigen;

//using namespace MM;

namespace TBasic {
	
	// initialize rotation matrix
	RSAlign::RSAlign(){
		m_R.resize(3, 3);
		m_t.resize(3);
		for(int i=0;i<3;++i){
			for(int j=0;j<3;++j){
				if(i==j){
					m_R(i,j)=1;
				} else {
					m_R(i,j)=0;
				}
			}
			m_t[i] = 0;
		}
	}
	

	
	bool RSAlign::compute_trans_triangle(){
		if (m_points1.size() != m_points2.size())
			return false;
		
		double n = m_points1.size();
		
		if (n!=3)
			return false;
		
		
		cout<<"Used the triangle version!"<<endl;
		
		Matrix3d R1, R2;
		// calculating two orthogonal vectors based on the face formed
		// by three neighbours
		{
			Vector3d v1 = m_points1[1] - m_points1[0];
			if(fabs(v1.norm()<1.0e-10))
			   cout<<"Problem"<<endl;
			   
			v1 /= v1.norm();
  
			Vector3d v2 = m_points1[2] - m_points1[0];
			if(fabs(v2.norm()<1.0e-10))
			   cout<<"Problem2"<<endl;
			   
			
			v2 /= v2.norm();
		
			Vector3d v3 = v1.cross(v2);
			if(fabs(v3.norm()<1.0e-10))
				cout<<"Problem3"<<endl;
			
			v3 /= v3.norm();
			
			v2 = v3.cross(v1);
			v2 /= v2.norm();
			
			for(int i=0;i<3;++i){
				R1(i,0) = v1[i];
				R1(i,1) = v2[i];
				R1(i,2) = v3[i];
			}
		}
		{
			Vector3d v1 = m_points2[1] - m_points2[0];
			if(fabs(v1.norm()<1.0e-10))
				cout<<"Problem4"<<endl;
			
			v1 /= v1.norm();
			Vector3d v2 = m_points2[2] - m_points2[0];
			if(fabs(v2.norm()<1.0e-10))
				cout<<"Problem5"<<endl;
			
			v2 /= v2.norm();
			
			Vector3d v3 = v1.cross(v2);
			if(fabs(v3.norm()<1.0e-10))
				cout<<"Problem6"<<endl;
			
			v3 /= v3.norm();
			
			v2 = v3.cross(v1);
			v2 /= v2.norm();
			
			for(int i=0;i<3;++i){
				R2(i,0) = v1[i];
				R2(i,1) = v2[i];
				R2(i,2) = v3[i];
			}
		}
		m_R = R2*R1.transpose();
		m_t = m_points2[0] - m_R * m_points1[0];
		
		// Scale should eventually change
		m_s = 1;
		

		return true;
		
	}
	
	
	bool RSAlign::compute_trans(){
#ifdef USE_LAPACK
		return compute_trans_LAPACK();
#else
		return compute_trans_EIGEN();
#endif
	}
	

	
	bool RSAlign::compute_trans_EIGEN(){
		int n = m_points1.size();
		if(m_points2.size()!=n)
			return false;
		
		
		Vector3d C1(0,0,0), C2(0,0,0);
		for (int i = 0; i < n; ++i){
			C1 = C1 + m_points1[i];
			C2 = C2 + m_points2[i];
		}
		
		C1 /= (double)n;
		C2 /= (double)n;
		
		MatrixXd M;
		M.resize(3, 3);
		for(int i=0;i<3;++i)
			for(int j=0;j<3;++j)
				M(i,j) = 0;
		
		
		for (int i = 0; i<m_points1.size(); ++i)
		{
			Vector3d eijp = m_points2[i] - C2;
			Vector3d eij = m_points1[i] - C1;
			
			M(0,0) += eij[0]*eijp[0];
			M(1,0) += eij[0]*eijp[1];
			M(2,0) += eij[0]*eijp[2];
			M(0,1) += eij[1]*eijp[0];
			M(1,1) += eij[1]*eijp[1];
			M(2,1) += eij[1]*eijp[2];
			M(0,2) += eij[2]*eijp[0];
			M(1,2) += eij[2]*eijp[1];
			M(2,2) += eij[2]*eijp[2];
		}
		
		JacobiSVD<MatrixXd> svd(M, ComputeThinU | ComputeThinV);
		
		MatrixXd U = svd.matrixU();
		MatrixXd V = svd.matrixV();
		
//        m_R = V*U.transpose();
	  /*  m_R = U*V.transpose();
		m_t = C2 - m_R * C1;*/

		double det = (U * V.transpose()).determinant();
		if (det > 0) det = 1.0;
		else det = -1.0;
		std::cout << "det: " << det << std::endl;
		Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
		I(2, 2) = det;
		m_R = U * I * V.transpose();
		//m_R = U * V.transpose();
		m_t = C2 - m_R * C1;


		return true;
	}
		
	bool RSAlign::compute_trans_LAPACK(){
#ifdef USE_LAPACK
		if (m_points1.size() != m_points2.size())
			return false;

		double n = m_points1.size();

		if (n<3)
			return false;

		if(n==3)
			return compute_trans_triangle();

		Vector3d C1(0,0,0), C2(0,0,0);
		for (int i = 0; i < n; ++i){
			C1 = C1 + m_points1[i];
			C2 = C2 + m_points2[i];
		}

		C1 /= (double)n;
		C2 /= (double)n;

		double a[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		

		for (int i = 0; i<m_points1.size(); ++i)
		{
			Vector3d eijp = m_points2[i] - C2;
			Vector3d eij = m_points1[i] - C1;

			a[0] += eij[0]*eijp[0] / n;
			a[1] += eij[0]*eijp[1] / n;
			a[2] += eij[0]*eijp[2] / n;
			a[3] += eij[1]*eijp[0] / n;
			a[4] += eij[1]*eijp[1] / n;
			a[5] += eij[1]*eijp[2] / n;
			a[6] += eij[2]*eijp[0] / n;
			a[7] += eij[2]*eijp[1] / n;
			a[8] += eij[2]*eijp[2] / n;
		}

		double s[3];
		double u[9];
		double vt[9];
		int m = 3;
		int nn = 3;
		int lda = 3;
		int ldu = 3;
		int ldvt = 3;
		int iwork[24];
		int lwork = -1;
		double work;
		char mode = 'A';

		int info;

		dgesdd_(&mode, &m, &nn, a, &lda, s, u, &ldu, vt, &ldvt, &work, &lwork, iwork, &info);
		lwork = (int)work;
		std::vector<double> workv(lwork);
		//double workv[lwork];
		dgesdd_(&mode, &m, &nn, a, &lda, s, u, &ldu, vt, &ldvt, &workv[0], &lwork, iwork, &info);

		/*double v_ut[9];
		
		v_ut[0] = vt[0] * u[0] + vt[1] * u[3] + vt[2] * u[6];
		v_ut[1] = vt[0] * u[1] + vt[1] * u[4] + vt[2] * u[7];
		v_ut[2] = vt[0] * u[2] + vt[1] * u[5] + vt[2] * u[8];

		v_ut[3] = vt[3] * u[0] + vt[4] * u[3] + vt[5] * u[6];
		v_ut[4] = vt[3] * u[1] + vt[4] * u[4] + vt[5] * u[7];
		v_ut[5] = vt[3] * u[2] + vt[4] * u[5] + vt[5] * u[8];

		v_ut[6] = vt[6] * u[0] + vt[7] * u[3] + vt[8] * u[6];
		v_ut[7] = vt[6] * u[1] + vt[7] * u[4] + vt[8] * u[7];
		v_ut[8] = vt[6] * u[2] + vt[7] * u[5] + vt[8] * u[8];*/

		
		Matrix3d v_utm;
		v_utm(0, 0) = vt[0] * u[0] + vt[1] * u[3] + vt[2] * u[6];
		v_utm(1, 0) = vt[0] * u[1] + vt[1] * u[4] + vt[2] * u[7];
		v_utm(2, 0) = vt[0] * u[2] + vt[1] * u[5] + vt[2] * u[8];

		v_utm(0, 1) = vt[3] * u[0] + vt[4] * u[3] + vt[5] * u[6];
		v_utm(1, 1) = vt[3] * u[1] + vt[4] * u[4] + vt[5] * u[7];
		v_utm(2, 1) = vt[3] * u[2] + vt[4] * u[5] + vt[5] * u[8];

		v_utm(0, 2) = vt[6] * u[0] + vt[7] * u[3] + vt[8] * u[6];
		v_utm(1, 2) = vt[6] * u[1] + vt[7] * u[4] + vt[8] * u[7];
		v_utm(2, 2) = vt[6] * u[2] + vt[7] * u[5] + vt[8] * u[8];
		
		
		/*m_R.resize(9);
		
		for (int i = 0; i < 0; ++i)
			m_R[i] = v_ut[i];*/

		m_R = v_utm;
		m_t = C2 - m_R * C1;

		// Scale should eventually change
		m_s = 1;

		return true;
#else
		cout<<"ERROR: NOT COMPILED WITH LAPACK"<<endl;
		return false;
#endif
	}

	
	// from web
	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
	
	
	Quaternion<double> RSAlign::export_quaternion(const Matrix3d& m){
		
		double qx,qy,qz,qw;
		qx = qy= qz= qw=0;
		

		double tr = m(0,0) + m(1,1) + m(2,2);
#if 1
		if (tr > 0) {
			double S = sqrt(tr+1.0) * 2; // S=4*qw
			qw = 0.25 * S;
			qx = (m(2,1) - m(1,2)) / S;
			qy = (m(0,2) - m(2,0)) / S;
			qz = (m(1,0) - m(0,1)) / S;
		} else if ((m(0,0) > m(1,1))&&(m(0,0) > m(2,2))) {
			double S = sqrt(1.0 + m(0,0) - m(1,1) - m(2,2)) * 2; // S=4*qx
			qw = (m(2,1) - m(1,2)) / S;
			qx = 0.25 * S;
			qy = (m(0,1) + m(1,0)) / S;
			qz = (m(0,2) + m(2,0)) / S;
		} else if (m(1,1) > m(2,2)) {
			double S = sqrt(1.0 + m(1,1) - m(0,0) - m(2,2)) * 2; // S=4*qy
			qw = (m(0,2) - m(2,0)) / S;
			qx = (m(0,1) + m(1,0)) / S;
			qy = 0.25 * S;
			qz = (m(1,2) + m(2,1)) / S;
		} else {
			double S = sqrt(1.0 + m(2,2) - m(0,0) - m(1,1)) * 2; // S=4*qz
			qw = (m(1,0) - m(0,1)) / S;
			qx = (m(0,2) + m(2,0)) / S;
			qy = (m(1,2) + m(2,1)) / S;
			qz = 0.25 * S;
		}
#endif
		
		return Quaternion<double>(qx, qy, qz, qw);
	}

	// from learn openCV .com
	
	// Checks if a matrix is a valid rotation matrix.
	bool RSAlign::isRotationMatrix(const Matrix3d &r)
	{
#if 0
		Matrix3d rt;
		transpose(R, Rt);
		Mat shouldBeIdentity = Rt * R;
		Mat I = Mat::eye(3,3, shouldBeIdentity.type());
		
		return  norm(I, shouldBeIdentity) < 1e-6;
#endif
		return true;
	}
	
	// Calculates rotation matrix to euler angles
	// The result is the same as MATLAB except the order
	// of the euler angles ( x and z are swapped ).
	Vector3d RSAlign::rotationMatrixToEulerAngles(const Matrix3d &r)
	{
		//assert(isRotationMatrix(R));
		
		double sy = sqrt(r(0,0) * r(0,0) +  r(1,0) * r(1,0) );
		
		bool singular = sy < 1e-6; // If
		
		double x, y, z;
		if (!singular)
		{
			x = atan2(r(2,1) , r(2,2));
			y = atan2(-r(2,0), sy);
			z = atan2(r(1,0), r(0,0));
		}
		else
		{
			x = atan2(-r(1,2), r(1,1));
			y = atan2(-r(2,0), sy);
			z = 0;
		}
		return Vector3d(x, y, z);
		
	}
	
	
	
	bool RSAlign::save(const std::string name){
		std::cout<<"file name: "<<name<<std::endl;
		ofstream f(name.c_str());
		if(!f){
			cout<<"Unable to open "<<name<<endl;
			return false;
		}
		
		f<<m_s<<endl;
		for(int i=0;i<3;++i){
			for(int j=0;j<3;++j)
				f<<m_R(i,j)<<" ";
			f<<endl;
		}
		
		for(int i=0;i<3;++i)
			f<<m_t[i]<<" ";
		f<<endl;
		std::cout<<"saved file: "<<name<<std::endl;
		return true;
	}

	bool RSAlign::save_feature_points(const std::string name)
	{
		std::cout<<"file name: "<<name<<std::endl;
		ofstream f(name.c_str());
		if (!f) {
			std::cout << "could not open file" << std::endl;
			return false;
		}

		f << m_points1.size() << std::endl;
		for (int i = 0; i < m_points1.size(); i++) {
			f << m_points1[i][0] << " " << m_points1[i][1] << " " <<
				m_points1[i][2] << std::endl;
		}

		f << m_points2.size() << std::endl;
		for (int i = 0; i < m_points2.size(); i++) {
			f << m_points2[i][0] << " " << m_points2[i][1] << " " << 
				m_points2[i][2] << std::endl;
		}


		return true;
	}
	
	bool RSAlign::load(const std::string name){
		ifstream f(name.c_str());
		if(!f){
			cout<<"Unable to open "<<name<<endl;
			return false;
		}
		
		//double d;
		f>>m_s;
		for(int i=0;i<3;++i){
			for(int j=0;j<3;++j)
				f>>m_R(i,j);
		}
		
		for(int i=0;i<3;++i)
			f>>m_t[i];
		
		return true;        
	}

	double RSAlign::getError(){
		double err = 0;
		for(int i=0;i<m_points1.size();++i){
			Vector3d P = m_R * m_points1[i]+m_t;
			err+=(P - m_points2[i]).norm();
		}
		err/=(double) m_points1.size();
		
		return err;
	}
	
	
	RSAlign RSAlign::inverse()const{
		RSAlign ret;

		ret.m_R = m_R.inverse();
		ret.m_t = (-1.0)*m_R.inverse()*m_t;
		
		return ret;
	}
	
	RSAlign RSAlign::operator*(const RSAlign& l){
		RSAlign ret;
		
		// calculate in this order. don't shuffle the lines
		ret.m_t = m_R* l.m_t + m_t;
		ret.m_s = m_s * l.m_s;
		ret.m_R = m_R * l.m_R;

		return ret;
	}
	
	
	 bool RSAlign::export_pts(const std::vector<Vector3d>& pts, std::string name){
		 ofstream f(name);
		 if(!f){
			 cout<<"Cannot open "<<name<<endl;
			 return false;
		 }
		 
		 for(int i=0;i<pts.size();++i){
			 f<<"v "<<pts[i][0]<<" "<<pts[i][1]<<" "<<pts[i][2]<<endl;
		 }
		 
		 return true;
	}

	
	Vector3d RSAlign::apply(Vector3d v){
		Vector3d r = m_R * v + m_t;
		return r;
	}
	
	
	void RSAlign::apply(const std::vector<Vector3d>& src, std::vector<Vector3d>& dst){
		dst.resize(src.size());
		
		for(int i=0;i<src.size();++i)
			dst[i] = apply(src[i]);
	}

	
}// namespace
