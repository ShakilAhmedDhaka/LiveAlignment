

#include "alignment.h"


using namespace std;
using namespace Eigen;

//using namespace MM;

namespace TBasic {
    
    
    // Unimplemented
    double RSAlign::max_error()const{
        return -1;
        
    }
    
    // needs to be called after compute_transf !!!!
    bool RSAlign::prune(int n, int keep){
        for(int i=0;i<n;++i){
            if(m_points1.size()<keep){
                return false;
            }
            
            // compute the error
            std::vector<Vector3d> dst;
            apply(m_points1, dst);
            

            // finds index for the pair of points who has most error-
            // rate after transformation
            double error = 0;
            int index = -1;
            for(int j=0;j<m_points1.size();++j){
                double d = (dst[j]-m_points2[j]).norm();
                if(index<0 || d>error){
                    index = j;
                    error = d;
                }
            }
            
            if(index<0){
                return false;
            }
            
            // remove index?? why replace with last point??
            m_points1[index] = m_points1[m_points1.size()-1];
            m_points1.pop_back();
            m_points2[index] = m_points2[m_points2.size()-1];
            m_points2.pop_back();
            
            // updating the new rotation matrix based on the pruned points
            if(!compute_trans()){ // in our case calls compute_trans_eigen function
                return false;
            }
        }
        
        return true;
    }

    
    bool RSAlign::compute_scale(){
        vector<double> scales;
        // compute first he scale base
        
        if (m_points1.size() != m_points2.size())
            return false;
        
        int n = m_points1.size();
        
        for(int i=0;i<n;++i){
            double l1 =(m_points1[i]-m_points1[(i+1)%n]).norm();
            double l2 =(m_points2[i]-m_points2[(i+1)%n]).norm();
            
            if(fabs(l1)<1.0e-8){
                cout<<"Points too close together!!! - Error"<<endl;
                continue;
            }
            
            scales.push_back(l2 / l1);
            
        }
        
        if(scales.size()!=0){
            m_s = 0;
            for(int i=0;i<scales.size();++i){
                //  cout<<"Debug scale ("<<i+1<<") out of "<<scales.size()<<" = "<<scales[i]<<endl;
                m_s+=scales[i];
            }
            m_s/=(double)scales.size();
        }
        
        return true;
    }

    
}// namespace
