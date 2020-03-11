#ifndef _TB_ALIGNMENT_H_
#define _TB_ALIGNMENT_H_


/* 

   Some utilities to manipulate the scene graph

 */ 

#include <vector>
#include <string>
#include <iostream>
#include <Eigen/SparseCore>
#include <Eigen/Geometry>


//#define USE_LAPACK

namespace TBasic {

	class RSAlign {
	public:
        RSAlign();
        
        std::vector<Eigen::Vector3d> m_points1, m_points2;  //m_points2 contains points of reference frame, m_points1 contains points of 'needs to rotated frame'

		bool compute_trans();   // calls one of the compute_trans functions[in our cased compute_trans_EIGEN]
        // compute rotation matrix m_R and m_t
        bool compute_trans_triangle();
        // compute the scle factor keeping m_points2 as reference frame
        bool compute_scale();

            // internal
        // compute rotation matrix m_R
        bool compute_trans_LAPACK();
        // compute rotation matrix m_R
        bool compute_trans_EIGEN();
        
        
        Eigen::MatrixXd m_R;    // rotation matrix to align
        Eigen::VectorXd m_t;    // difference of positions between average of points or first points in different frames after aligning
		double m_s; // scale factor where m_points2 is reference frame
        
        // converts rotation matrix to a quaternion
        static Eigen::Quaternion<double> export_quaternion(const Eigen::Matrix3d& m);

        // Checks if a matrix is a valid rotation matrix.
        static bool isRotationMatrix(const Eigen::Matrix3d &m);
        
        // Calculates rotation matrix to euler angles
        // The result is the same as MATLAB except the order
        // of the euler angles ( x and z are swapped ).
        static Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d &R);
        
        // saves m_R and m_t to a file given the name
        bool save(const std::string name);
		bool save_feature_points(const std::string name);
        // loades m_R and m_t from a file given the name
        bool load(const std::string name);
        
        // returns the average error between the aligned frame and the original frame
        double getError();
        
        // inverse the rotational matrix and adjust the m_t so that 
        // now can be applied to trasform reference frame to the second frame
        RSAlign inverse()const;

        // oprator overload multiplication for the class
        // can be used for consecutive transformation
        // given a RSAlign which contains transformation matrix and m_t for a pair of frames,
        // it will combine the rotation matrices and total distance between frames and return
        // the combined rotation matrix and total distance between first frame and last frame
        RSAlign operator*(const RSAlign& l);
        
        // given the points of the frame and file name, saves the vertices of the frame in the file
        static bool export_pts(const std::vector<Eigen::Vector3d>& pts, std::string name);
        
        // transform a point to both rotate and translate
        Eigen::Vector3d apply(Eigen::Vector3d v);
        
        // transform all the points of the second frame
        void apply(const std::vector<Eigen::Vector3d>& src, std::vector<Eigen::Vector3d>& dst);

        double max_error()const;

        // deletes the points in both frames which produced most
        // error after transformation. returns false if there were
        // no error. otherwise returns true.
        bool prune(int n, int keep);
        
	};

}
 
#endif
